#include "spi_master.h"
#include "master_cycle.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

static SPI_HandleTypeDef  *master_hspi  = NULL;
static UART_HandleTypeDef *master_huart = NULL;

/* The 200 Hz cycle is driven by the TIM2 hardware interrupt (master_cycle): one
 * cycle = poll all slaves then emit tele_robot_t. Rates come from the generated
 * MASTER_POLL_HZ. Status is a cycle divider off that (20 Hz). */
#define MASTER_STATUS_DIV (MASTER_POLL_HZ / 20u)   /* cycles per status emit (=10 @ 200 Hz) */

/* ── legacy USB-command path (unused, kept per spec / .h ABI) ────────────── */
uint8_t new_usb_packet_rx_flag = 0;
uint8_t buf_rx_jet2master[NUM_SLV * MAX_MOTORS_PER_SLAVE * USB_BYTES_PER_MOTOR];

/* ── per-slave command mailbox (latest cmd_chain_t from the host) ──────────
   The host sends one cmd_robot_t per tick; the USB ISR splits it into per-slave
   cmd_chain_t here (latest wins). Each SPI poll forwards the slave's chain
   (SPI_OP_ROBOT_CMD) or a keepalive (SPI_OP_NOP) if none has arrived. Mode
   requests are level-triggered, so re-sending the last chain is correct. */
static cmd_chain_t host_chain[NUM_SLAVES];
static uint8_t     host_chain_valid[NUM_SLAVES];
static uint16_t    host_cmd_seq      = CMD_SEQ_NONE;  /* last cmd_seq from the host   */
static uint16_t    master_cycle_id   = 0;             /* master cycle counter         */

/* ── master status counters ──────────────────────────────────────────────── */
uint32_t master_link_errors = 0;
uint32_t master_rx_frames   = 0;
uint32_t master_proto_ver_mismatch = 0;
uint32_t master_rx_resyncs  = 0;   /* USB RX resync events (contiguous discard runs) */
uint32_t master_rx_discarded = 0;  /* bytes dropped while resyncing                  */
uint32_t master_rx_overflows = 0;  /* RX ring overruns (producer outran consumer)    */

/* ── USB RX ring: CDC ISR produces, main loop consumes (SPSC, lock-free) ───── */
#define USB_RX_RING_SIZE 1024u                       /* power of two               */
#define USB_RX_RING_MASK (USB_RX_RING_SIZE - 1u)
static volatile uint8_t  usb_rx_ring[USB_RX_RING_SIZE];
static volatile uint16_t usb_rx_head = 0;            /* written by ISR only        */
static volatile uint16_t usb_rx_tail = 0;            /* written by main only       */

void MotorMaster_UsbRxFromISR(const uint8_t *buf, uint16_t len)
{
    uint16_t head = usb_rx_head;
    for (uint16_t i = 0; i < len; i++) {
        uint16_t next = (uint16_t)((head + 1u) & USB_RX_RING_MASK);
        if (next == usb_rx_tail) { master_rx_overflows++; break; }  /* full → drop rest */
        usb_rx_ring[head] = buf[i];
        head = next;
    }
    usb_rx_head = head;
}

/* ── per-slave runtime state ─────────────────────────────────────────────── */
static uint8_t      slave_alive[NUM_SLAVES];
static uint8_t      slave_motors_alive[NUM_SLAVES];
static tele_chain_t latest_tele[NUM_SLAVES];        /* last CRC-valid telemetry chain */
static uint32_t     slave_crc_errors[NUM_SLAVES];   /* telemetry frames failing CRC */
static uint8_t      spi_seq[NUM_SLAVES];             /* per-slave SPI link-health seq  */
/* seq/echo diagnostics and relayed slave-side command-CRC count. */
#define SEQ_STALL_POLLS 5u                           /* echo frozen this many polls = a gap */
static uint8_t      prev_echo[NUM_SLAVES];
static uint8_t      have_prev_echo[NUM_SLAVES];
static uint8_t      echo_stall[NUM_SLAVES];
static uint32_t     slave_seq_gaps[NUM_SLAVES];      /* command-link stalls (echo froze) */
static uint32_t     slave_cmd_crc_errors[NUM_SLAVES];/* relayed from tele_chain_t       */
static uint16_t     tx_seq = 0;

/* ── helpers ─────────────────────────────────────────────────────────────── */
/* usb_printf goes through the ring buffer so it no longer drops */
void usb_printf(const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) usb_tx_write((const uint8_t *)buf, (uint16_t)n);
}

static inline void CS_ALL_HIGH(void) {
    HAL_GPIO_WritePin(GPIOC,
        SLAVE_CS_0_Pin | SLAVE_CS_1_Pin | SLAVE_CS_2_Pin | SLAVE_CS_3_Pin,
        GPIO_PIN_SET);
}
static inline void CS_SELECT(SpiDevId dev) {
    CS_ALL_HIGH();
    switch (dev) {
        case DEV1: HAL_GPIO_WritePin(GPIOC, SLAVE_CS_0_Pin, GPIO_PIN_RESET); break;
        case DEV2: HAL_GPIO_WritePin(GPIOC, SLAVE_CS_1_Pin, GPIO_PIN_RESET); break;
        case DEV3: HAL_GPIO_WritePin(GPIOC, SLAVE_CS_2_Pin, GPIO_PIN_RESET); break;
        case DEV4: HAL_GPIO_WritePin(GPIOC, SLAVE_CS_3_Pin, GPIO_PIN_RESET); break;
        default: break;
    }
}

/* ── SPI exchange with one slave (fixed-size frames; one chain per slave) ──
   One full-duplex transfer: the command frame [opcode][spi_seq][cycle_id][cmd_seq]
   [cmd_chain_t][crc] rides in the TX prefix; the CRC-framed tele_chain_t comes
   back in RX. The frame CRC doubles as integrity AND presence check — an absent
   slave clocks back garbage that fails CRC. Returns HAL_OK only on a CRC-valid
   telemetry frame. */
static HAL_StatusTypeDef spi_exchange(SpiDevId dev, uint8_t opcode, uint8_t spi_seq_v,
                                       uint16_t cycle_id, uint16_t cmd_seq,
                                       const cmd_chain_t *chain,
                                       tele_chain_t *tele_out)
{
    uint8_t tx[SPI_XFER_SIZE];
    uint8_t rx[SPI_XFER_SIZE];
    memset(tx, 0, SPI_XFER_SIZE);
    tx[0] = opcode;
    tx[1] = spi_seq_v;
    tx[2] = (uint8_t)(cycle_id & 0xFFu);
    tx[3] = (uint8_t)(cycle_id >> 8);
    tx[4] = (uint8_t)(cmd_seq & 0xFFu);
    tx[5] = (uint8_t)(cmd_seq >> 8);
    if (chain != NULL) memcpy(&tx[SPI_CMD_HDR_BYTES], chain, sizeof(cmd_chain_t));
    uint16_t ccrc = proto_crc16(tx, SPI_CMD_CRC_OFF);
    tx[SPI_CMD_CRC_OFF]     = (uint8_t)(ccrc & 0xFFu);
    tx[SPI_CMD_CRC_OFF + 1] = (uint8_t)(ccrc >> 8);

    CS_SELECT(dev);
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(master_hspi, tx, rx,
                                                    SPI_XFER_SIZE, HAL_MAX_DELAY);
    CS_ALL_HIGH();
    if (st != HAL_OK) return st;

    uint16_t rx_crc = (uint16_t)(rx[SPI_TELE_CRC_OFF] |
                      ((uint16_t)rx[SPI_TELE_CRC_OFF + 1] << 8));
    uint16_t calc   = proto_crc16(rx, SPI_TELE_CRC_OFF);
    if (rx_crc != calc) {
        slave_crc_errors[(uint8_t)dev]++;
        return HAL_ERROR;              /* bad/absent frame → caller marks offline */
    }
    memcpy(tele_out, rx, sizeof(tele_chain_t));
    slave_cmd_crc_errors[(uint8_t)dev] = tele_out->cmd_crc_errors;   /* relayed */
    return HAL_OK;
}

/* ── emit protocol frames over USB TX ring ───────────────────────────────── */
/* Aggregate robot state + per-slave alive mask (shared by status + telemetry). */
static RobotState compute_robot_state(uint8_t *alive_mask_out)
{
    uint8_t alive_mask = 0u, any_alive = 0u, all_full = 1u;
    for (uint8_t s = 0; s < NUM_SLAVES; s++) {
        if (slave_alive[s]) {
            alive_mask |= (uint8_t)(1u << s);
            any_alive = 1u;
            uint8_t full = (uint8_t)((1u << slave_motor_counts[s]) - 1u);
            if (slave_motors_alive[s] != full) all_full = 0u;
        } else {
            all_full = 0u;
        }
    }
    if (alive_mask_out) *alive_mask_out = alive_mask;
    return !any_alive ? ROBOT_INIT : (all_full ? ROBOT_READY : ROBOT_DEGRADED);
}

static void emit_master_status(uint32_t now_ms)
{
    uint8_t alive_mask = 0u;
    RobotState rs = compute_robot_state(&alive_mask);

    MasterStatus pay = {0};
    pay.robot_state    = (uint8_t)rs;
    pay.slave_alive    = alive_mask;             /* bit s = slave s reachable */
    pay.uptime_ms      = now_ms;
    pay.link_errors    = master_link_errors;
    pay.rx_frames      = master_rx_frames;
    pay.master_poll_hz = MASTER_POLL_HZ;         /* configured rates (motor_config.h) */
    pay.telemetry_hz   = TELEMETRY_HZ;
    pay.slave_tick_hz  = SLAVE_TICK_HZ;
    pay.host_cmd_hz    = HOST_CMD_HZ;
    pay.rx_resyncs        = master_rx_resyncs;
    pay.rx_discarded_bytes = master_rx_discarded;

    uint8_t frame[MSG_HEADER_SIZE + sizeof(MasterStatus)];
    uint16_t n = proto_build(frame, sizeof(frame),
                              MSG_MASTER_STATUS, tx_seq++,
                              NODE_MASTER, NODE_JETSON,
                              now_ms,
                              (const uint8_t *)&pay, sizeof(pay));
    if (n > 0u) usb_tx_write(frame, n);
}

static void emit_slave_status(uint8_t s, uint32_t now_ms)
{
    SlaveStatus pay = {0};
    pay.slave_id       = s;
    pay.motors_alive   = slave_motors_alive[s];
    pay.uptime_ms      = now_ms;
    pay.crc_errors     = slave_crc_errors[s];
    pay.cmd_crc_errors = slave_cmd_crc_errors[s];   /* slave rejected these commands */
    pay.seq_gaps       = slave_seq_gaps[s];         /* master saw these echo gaps    */

    uint8_t frame[MSG_HEADER_SIZE + sizeof(SlaveStatus)];
    uint16_t n = proto_build(frame, sizeof(frame),
                              MSG_SLAVE_STATUS, tx_seq++,
                              NODE_MASTER, NODE_JETSON,
                              now_ms,
                              (const uint8_t *)&pay, sizeof(pay));
    if (n > 0u) usb_tx_write(frame, n);
}

static void emit_robot_tele(uint32_t master_time_us)
{
    /* Pass-through: assemble one tele_robot_t from every CURRENTLY-ALIVE slave's
       last CRC-valid tele_chain_t (emission-gated at chain granularity — a dead
       slave's chain is omitted, so the host sees it go silent). The master never
       touches motor data; the host decodes fixed-point raw→units.
       master_time_us is this cycle's real TIM2 µs timestamp (stamped by the caller);
       missed_deadlines now carries the hardware-cycle overrun count. */
    tele_robot_t pay;
    memset(&pay, 0, sizeof(pay));
    pay.cycle_id         = master_cycle_id;
    pay.master_time_us   = master_time_us;
    pay.last_cmd_seq_rx  = host_cmd_seq;
    pay.missed_deadlines = (uint16_t)master_cycle_overruns();
    pay.robot_state      = (uint8_t)compute_robot_state(NULL);

    uint8_t nc = 0u;
    for (uint8_t s = 0; s < NUM_SLAVES; s++) {
        if (slave_alive[s] && nc < MAX_CHAINS) {
            memcpy(&pay.chains[nc], &latest_tele[s], sizeof(tele_chain_t));
            nc++;
        }
    }
    pay.n_chains = nc;

    /* Transmit only the POPULATED prefix (header fields + nc chains), not the full
       fixed-size struct. The struct stays MAX_CHAINS-sized in memory, but sending a
       single 714 B frame per tick at 200 Hz (~146 KB/s) swamps the pure-Python host
       and builds a ~110 ms receive backlog; the prefix is ~129 B for one chain. The
       host parses by n_chains, so a truncated payload decodes identically. */
    uint16_t pay_len = (uint16_t)(offsetof(tele_robot_t, chains) +
                                  (uint16_t)nc * sizeof(tele_chain_t));
    uint8_t frame[MSG_HEADER_SIZE + sizeof(tele_robot_t)];
    uint16_t n = proto_build(frame, sizeof(frame),
                              MSG_ROBOT_TELE, tx_seq++,
                              NODE_MASTER, NODE_JETSON,
                              HAL_GetTick(),   /* header ts_ms (ms) — host live-detect uses this */
                              (const uint8_t *)&pay, pay_len);
    if (n > 0u) usb_tx_write(frame, n);
}

/* ── ROBOT_CMD handler (called from the USB frame decoder, ISR context) ────
   Split the host's cmd_robot_t into per-slave cmd_chain_t mailboxes (latest wins)
   by chain_id. Mode requests are level-triggered, so the poll re-sends the stored
   chain until a newer one arrives. No response frame — telemetry is the ack. */
void MotorMaster_HandleRobotCmd(const cmd_robot_t *cmd)
{
    if (cmd == NULL) return;
    host_cmd_seq = cmd->cmd_seq;
    uint8_t n = (cmd->n_chains > MAX_CHAINS) ? MAX_CHAINS : cmd->n_chains;
    for (uint8_t c = 0; c < n; c++) {
        uint8_t s = cmd->chains[c].chain_id;
        if (s >= NUM_SLAVES) { master_link_errors++; continue; }
        host_chain[s]       = cmd->chains[c];   /* struct copy (62 B) */
        host_chain_valid[s] = 1u;
    }
}

/* Drain the USB RX ring and dispatch every complete frame (main-loop context).
   Uses the resynchronizing proto_frame_scan: junk/misaligned bytes are dropped one
   at a time (counted), so a stray byte or lost packet can't desync us permanently. */
void MotorMaster_ProcessUsbRx(void)
{
    static uint8_t  scan[768];     /* ≥ 2 × max frame (16 + 480) for headroom */
    static uint16_t scan_len = 0;
    static uint8_t  in_resync = 0;

    /* Drain ring → scan buffer (consumer side; tail is ours). */
    while (scan_len < sizeof(scan) && usb_rx_tail != usb_rx_head) {
        scan[scan_len++] = usb_rx_ring[usb_rx_tail];
        usb_rx_tail = (uint16_t)((usb_rx_tail + 1u) & USB_RX_RING_MASK);
    }

    uint16_t off = 0;
    while ((uint16_t)(scan_len - off) >= MSG_HEADER_SIZE) {
        ProtoScanResult r;
        ProtoScanStatus st = proto_frame_scan(scan + off, (uint16_t)(scan_len - off), &r);

        if (st == PROTO_SCAN_NEED_MORE) break;         /* plausible header; await body */

        if (st == PROTO_SCAN_RESYNC) {                 /* junk → drop byte(s), count run */
            if (!in_resync) { master_rx_resyncs++; in_resync = 1u; }
            master_rx_discarded += r.consumed;
            master_link_errors++;
            off += r.consumed;
            continue;
        }

        in_resync = 0u;                                /* PROTO_SCAN_FRAME */
        master_rx_frames++;
        if (r.type == MSG_ROBOT_CMD) {
            if (r.pay_len >= sizeof(cmd_robot_t)) {
                cmd_robot_t cmd;
                memcpy(&cmd, r.payload, sizeof(cmd));
                MotorMaster_HandleRobotCmd(&cmd);
            } else {
                master_link_errors++;
            }
        } else if (r.type == MSG_PING) {
            MsgHeader h;                               /* echo the request's seq/src */
            memcpy(&h, r.payload - MSG_HEADER_SIZE, MSG_HEADER_SIZE);
            uint8_t frame[MSG_HEADER_SIZE];
            uint16_t n = proto_build(frame, sizeof(frame), MSG_PING, h.seq,
                                     NODE_MASTER, (uint8_t)h.src, HAL_GetTick(), NULL, 0u);
            if (n > 0u) usb_tx_write(frame, n);
        }
        off += r.consumed;
    }

    if (off > 0u) {                                    /* slide unconsumed tail down */
        scan_len = (uint16_t)(scan_len - off);
        if (scan_len > 0u) memmove(scan, scan + off, scan_len);
    } else if (scan_len == sizeof(scan)) {             /* defensive: never wedge full */
        if (!in_resync) { master_rx_resyncs++; in_resync = 1u; }
        master_rx_discarded++;
        scan_len--;
        memmove(scan, scan + 1, scan_len);
    }
}

/* ── Public API ──────────────────────────────────────────────────────────── */
void MotorMaster_Init(SPI_HandleTypeDef *hspi, UART_HandleTypeDef *huart)
{
    master_hspi  = hspi;
    master_huart = huart;
    CS_ALL_HIGH();
    memset(slave_alive, 0, sizeof(slave_alive));
    memset(slave_motors_alive, 0, sizeof(slave_motors_alive));
    memset(host_chain, 0, sizeof(host_chain));
    memset(host_chain_valid, 0, sizeof(host_chain_valid));
    memset(latest_tele, 0, sizeof(latest_tele));
    memset(slave_crc_errors, 0, sizeof(slave_crc_errors));
    memset(spi_seq, 0, sizeof(spi_seq));
    memset(prev_echo, 0, sizeof(prev_echo));
    memset(have_prev_echo, 0, sizeof(have_prev_echo));
    memset(echo_stall, 0, sizeof(echo_stall));
    memset(slave_seq_gaps, 0, sizeof(slave_seq_gaps));
    memset(slave_cmd_crc_errors, 0, sizeof(slave_cmd_crc_errors));
}

/* Kept for ABI compatibility – not actively used */
void MotorMaster_ParseRxBuffer(void) { (void)buf_rx_jet2master; }
void MotorMaster_FormatTxBuffer(void) {}

/* Forward one slave's current command chain, then ingest its telemetry. */
static void poll_one_slave(uint8_t s)
{
    /* Snapshot the host command for this slave under a brief mask (the USB ISR
       writes host_chain/host_cmd_seq). A ROBOT_CMD chain if one has arrived, else
       a NOP keepalive (still refreshes the slave watchdog and clocks telemetry). */
    uint8_t     opcode;
    cmd_chain_t chain_local;
    uint16_t    cmd_seq_local;
    __disable_irq();
    if (host_chain_valid[s]) {
        opcode      = SPI_OP_ROBOT_CMD;
        chain_local = host_chain[s];
    } else {
        opcode = SPI_OP_NOP;
        memset(&chain_local, 0, sizeof(chain_local));
        chain_local.chain_id = s;
        chain_local.n_motors = slave_motor_counts[s];
    }
    cmd_seq_local = host_cmd_seq;
    __enable_irq();

    uint8_t seq_to_send = spi_seq[s]++;
    tele_chain_t tele;
    HAL_StatusTypeDef st = spi_exchange((SpiDevId)s, opcode, seq_to_send,
                                        master_cycle_id, cmd_seq_local,
                                        &chain_local, &tele);

    if (st == HAL_OK) {   /* CRC verified inside spi_exchange = valid + present */
        /* Echo-stall diagnostics on the SPI link-health seq (sustained freeze =
           a command link that stopped confirming). Any advance resets the streak. */
        uint8_t echo = tele.spi_seq_echo;
        if (have_prev_echo[s]) {
            if (echo == prev_echo[s]) {
                if (++echo_stall[s] >= SEQ_STALL_POLLS) { slave_seq_gaps[s]++; echo_stall[s] = 0u; }
            } else {
                echo_stall[s] = 0u;
            }
        }
        prev_echo[s]      = echo;
        have_prev_echo[s] = 1u;

        /* A motor is "alive" once discovery advanced it past BOOT/DISCOVERING. */
        uint8_t alive_mask = 0u;
        uint8_t nm = (tele.n_motors > MAX_MOTORS_PER_SLAVE) ? MAX_MOTORS_PER_SLAVE
                                                            : tele.n_motors;
        for (uint8_t i = 0; i < nm; i++) {
            uint8_t st8 = tele.motors[i].state;
            if (st8 != LIFE_BOOT && st8 != LIFE_DISCOVERING)
                alive_mask |= (uint8_t)(1u << i);
        }
        slave_alive[s]        = 1u;
        slave_motors_alive[s] = alive_mask;
        latest_tele[s]        = tele;    /* struct copy */
    } else {
        /* Absent slave / garbage frame — mark offline; reset the seq-echo baseline
           so recovery doesn't false-count. The host command mailbox is left intact
           (level-triggered: it reapplies once the slave is back). */
        slave_alive[s]        = 0u;
        slave_motors_alive[s] = 0u;
        have_prev_echo[s]     = 0u;
        echo_stall[s]         = 0u;
    }
}

void MotorMaster_ProcessLoop(void)
{
    static uint16_t cycles_since_status = 0;

    /* Drain the USB RX ring (scan + dispatch host frames) and pump the TX ring
       every iteration — both single-producer here in the main loop. */
    MotorMaster_ProcessUsbRx();
    usb_tx_pump_responses();
    usb_tx_pump();

    /* One hardware-timed cycle (TIM2 ISR only flagged us; the work is here): poll
       every slave in order, then immediately emit this cycle's tele_robot_t. */
    if (master_cycle_take(NULL)) {
        uint32_t t0 = cycle_us_now();           /* cycle service start (µs) */
        master_cycle_id++;
        for (uint8_t s = 0; s < NUM_SLAVES; s++) {
            poll_one_slave(s);
        }
        emit_robot_tele(t0);                     /* stamp master_time_us = cycle start */

        /* 20 Hz status — cycle divider off the 200 Hz cycle (its own schedule). */
        if (++cycles_since_status >= MASTER_STATUS_DIV) {
            cycles_since_status = 0;
            uint32_t now_ms = HAL_GetTick();
            emit_master_status(now_ms);
            for (uint8_t s = 0; s < NUM_SLAVES; s++) emit_slave_status(s, now_ms);
        }
    }
}
