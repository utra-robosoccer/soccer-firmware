/* master_app — see master_app.h. Policy and orchestration only: every wire byte
 * goes through spi_link (slaves) or usb_link (host). */
#include "master_app.h"
#include "master_cycle.h"
#include "spi_link.h"
#include "usb_link.h"
#include "imu.h"   /* optional BMI088 IMU on I2C1 (merged from the IMU PR) */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

static UART_HandleTypeDef *master_huart = NULL;

/* The 200 Hz cycle is driven by the TIM2 hardware interrupt (master_cycle): one
 * cycle = poll all slaves then emit tele_robot_t. Rates come from the generated
 * MASTER_POLL_HZ. Status is a cycle divider off that (20 Hz). */
#define MASTER_STATUS_DIV (MASTER_POLL_HZ / 20u)   /* cycles per status emit (=10 @ 200 Hz) */

/* ── host-command mailbox (pure logic: common/include/cmd_mailbox.h) ───────
   Double-buffered, apply-at-n+1, latest-wins. USB dispatch posts; the swap at the
   start of every master cycle publishes to the ACTIVE buffer the slave polls read.
   See docs/architecture.md. */
static CmdMailbox  g_mbox;
static uint16_t    master_cycle_id = 0;            /* master cycle counter (per cycle) */

/* Host-death dead-man (task 6): the FSM that forces STOP to the slaves when the
   host goes quiet while armed; its inputs live in the mailbox. */
static HostWatchdog  g_host_wd;
static HostLinkAction g_host_action = HOST_LINK_OK;

/* ── master status counters (USB transport counters live in usb_link) ─────── */
uint32_t master_link_errors = 0;
#ifdef USB_TX_TEST
static volatile uint8_t g_txtest = 0; /* test-only: armed by a 0xB0 cmd.reserved sentinel */
#endif

static uint16_t tx_seq = 0;

/* The CDC stream carries ONLY framed protocol messages (docs/protocol.md) —
   there is deliberately NO printf-to-USB path. Diagnostics go to UART4
   (dedicated PA0/PA1) via debug_printf, compiled out unless the build defines
   MASTER_DEBUG_UART. NOTE: this uses BLOCKING HAL_UART_Transmit (~87 µs/char
   budget at 115200), so MASTER_DEBUG_UART builds are NOT timing-representative
   — debug only, never a release flag. */
#ifdef MASTER_DEBUG_UART
void debug_printf(const char *fmt, ...)
{
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0 && master_huart != NULL) {
        if (n > (int)sizeof(buf) - 1) n = (int)sizeof(buf) - 1;
        HAL_UART_Transmit(master_huart, (uint8_t *)buf, (uint16_t)n, 20u);
    }
}
#endif

/* ── emit protocol frames over USB TX ring ───────────────────────────────── */
/* Aggregate robot state + per-slave alive mask (shared by status + telemetry). */
static RobotState compute_robot_state(uint8_t *alive_mask_out)
{
    uint8_t alive_mask = 0u, any_alive = 0u, all_full = 1u;
    for (uint8_t s = 0; s < NUM_SLAVES; s++) {
        const SpiLinkSlave *sl = spi_link_slave(s);
        if (sl->alive) {
            alive_mask |= (uint8_t)(1u << s);
            any_alive = 1u;
            uint8_t full = (uint8_t)((1u << slave_motor_counts[s]) - 1u);
            if (sl->motors_alive != full) all_full = 0u;
        } else {
            all_full = 0u;
        }
    }
    if (alive_mask_out) *alive_mask_out = alive_mask;
    return !any_alive ? ROBOT_INIT : (all_full ? ROBOT_READY : ROBOT_DEGRADED);
}

/* Any motor currently armed (ENABLING/ENABLED/STOPPING), per the latest telemetry —
   gates the host-death watchdog so it never trips when everything is already off. */
static uint8_t any_motor_armed(void)
{
    for (uint8_t s = 0; s < NUM_SLAVES; s++) {
        const SpiLinkSlave *sl = spi_link_slave(s);
        if (!sl->alive) continue;
        uint8_t nm = (sl->tele.n_motors > MAX_MOTORS_PER_SLAVE)
                     ? MAX_MOTORS_PER_SLAVE : sl->tele.n_motors;
        for (uint8_t i = 0; i < nm; i++) {
            uint8_t st = sl->tele.motors[i].drive_state;
            if (st == DRV_ENABLING || st == DRV_ENABLED || st == DRV_STOPPING)
                return 1u;
        }
    }
    return 0u;
}

static void emit_master_status(uint32_t now_ms)
{
    /* robot_state and per-slave reachability are NOT here (they ride
       tele_robot_t / tele_chain_t.link every cycle); status carries the
       build fingerprint and the slow link counters. */
    MasterStatus pay = {0};
    pay.schema_hash = PROTO_SCHEMA_HASH;
    for (uint8_t s = 0; s < NUM_SLAVES && s < 8u; s++) {
        if (spi_link_slave(s)->mismatch) pay.slave_proto_mismatch |= (uint8_t)(1u << s);
    }
    pay.uptime_ms      = now_ms;
    pay.link_errors    = master_link_errors + usb_rx_scan_errors; /* app + scan rejects */
    pay.rx_frames      = usb_rx_frames;
    pay.master_poll_hz = MASTER_POLL_HZ;         /* configured rates (motor_config.h) */
    pay.telemetry_hz   = TELEMETRY_HZ;
    pay.slave_tick_hz  = SLAVE_TICK_HZ;
    pay.host_cmd_hz    = HOST_CMD_HZ;
    pay.rx_resyncs        = usb_rx_resyncs;
    pay.rx_discarded_bytes = usb_rx_discarded;

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
    const SpiLinkSlave *sl = spi_link_slave(s);
    SlaveStatus pay = {0};
    pay.slave_id       = s;
    pay.motors_alive   = sl->motors_alive;
    pay.uptime_ms      = now_ms;
    pay.crc_errors     = sl->crc_errors;
    pay.cmd_crc_errors = sl->cmd_crc_errors;   /* slave rejected these commands */
    pay.seq_gaps       = sl->seq_gaps;         /* master saw these echo gaps    */

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
       master_time_us is this cycle's real TIM2 µs timestamp (stamped by the caller).
       cmd_seq_active is the command actually applied this cycle; the four cmd_* counters
       classify host-loop timing against the master clock (see cmd_mailbox_swap). Hardware
       cycle overruns remain in MasterStatus.missed_deadlines. */
    tele_robot_t pay;
    memset(&pay, 0, sizeof(pay));
    pay.cycle_id         = master_cycle_id;
    pay.master_time_us   = master_time_us;
    pay.last_cmd_seq_rx  = g_mbox.cmd_seq_rx;
    pay.cmd_seq_active   = g_mbox.cmd_seq_active;
    pay.cmd_on_time      = g_mbox.cnt_on_time;
    pay.cmd_late         = g_mbox.cnt_late;
    pay.cmd_missing      = g_mbox.cnt_missing;
    pay.cmd_duplicate    = g_mbox.cnt_duplicate;
    /* Reporting priority (spec §robot_state): HOST_LOST > PROTO_MISMATCH >
       the computed INIT/READY/DEGRADED. */
    pay.robot_state      = (g_host_action != HOST_LINK_OK) ? (uint8_t)ROBOT_HOST_LOST
                         : spi_link_any_mismatch()         ? (uint8_t)ROBOT_PROTO_MISMATCH
                                                           : (uint8_t)compute_robot_state(NULL);

    /* Every CONFIGURED chain is emitted every cycle at its fixed slot
       (chains[s] = slave s), so chain position is stable whether or not the
       slave is reachable — the host never remaps by set-bit order. The master
       authors the per-chain link byte; an un-alive chain carries n_motors = 0
       and no (stale) motor data. */
    for (uint8_t s = 0; s < NUM_SLAVES && s < MAX_CHAINS; s++) {
        const SpiLinkSlave *sl = spi_link_slave(s);
        if (sl->alive) {
            memcpy(&pay.chains[s], &sl->tele, sizeof(tele_chain_t));
            pay.chains[s].link = CHAIN_LINK_ALIVE;
        } else {
            pay.chains[s].n_motors = 0u;
            pay.chains[s].link     = sl->mismatch ? CHAIN_LINK_MISMATCH
                                                  : CHAIN_LINK_LOST;
        }
        pay.chains[s].chain_id = s;    /* stamp the slave index — the slave hardcodes 0,
                                          so multi-slave telemetry would otherwise collide */
    }
    pay.n_chains = (NUM_SLAVES < MAX_CHAINS) ? NUM_SLAVES : MAX_CHAINS;

    /* Transmit only the POPULATED prefix (header fields + nc chains), not the full
       fixed-size struct. The struct stays MAX_CHAINS-sized in memory, but sending a
       single 714 B frame per tick at 200 Hz (~146 KB/s) swamps the pure-Python host
       and builds a ~110 ms receive backlog; the prefix is ~129 B for one chain. The
       host parses by n_chains, so a truncated payload decodes identically. */
    uint16_t pay_len = (uint16_t)(offsetof(tele_robot_t, chains) +
                                  (uint16_t)pay.n_chains * sizeof(tele_chain_t));
    uint8_t frame[MSG_HEADER_SIZE + sizeof(tele_robot_t)];
    uint16_t n = proto_build(frame, sizeof(frame),
                              MSG_ROBOT_TELE, tx_seq++,
                              NODE_MASTER, NODE_JETSON,
                              HAL_GetTick(),   /* header ts_ms (ms) — host live-detect uses this */
                              (const uint8_t *)&pay, pay_len);
    if (n > 0u) usb_tx_write(frame, n);
}

/* Frame dispatch: usb_link's RX scanner delivers complete frames here (main-loop
   context). WHAT a frame means is decided here, not in the transport. */
static void on_usb_frame(const ProtoScanResult *r)
{
    if (r->type == MSG_ROBOT_CMD) {
        if (r->pay_len >= sizeof(cmd_robot_t)) {
            cmd_robot_t cmd;
            memcpy(&cmd, r->payload, sizeof(cmd));
#ifdef SPI_INJECT_TEST
            if (cmd.reserved == 0xDEu) { g_spi_inject = 1u; return; } /* sentinel (reserved) */
#endif
#ifdef USB_TX_TEST
            if (cmd.reserved == 0xB0u) { g_txtest = 1u; return; }     /* sentinel (reserved) */
#endif
            master_link_errors += cmd_mailbox_post(&g_mbox, &cmd);    /* bad chain_ids */
        } else {
            master_link_errors++;
        }
    } else if (r->type == MSG_PING) {
        MsgHeader h;                               /* echo the request's seq/src */
        memcpy(&h, r->payload - MSG_HEADER_SIZE, MSG_HEADER_SIZE);
        uint8_t frame[MSG_HEADER_SIZE];
        uint16_t n = proto_build(frame, sizeof(frame), MSG_PING, h.seq,
                                 NODE_MASTER, (uint8_t)h.src, HAL_GetTick(), NULL, 0u);
        if (n > 0u) usb_tx_write(frame, n);
    }
}

/* Decide the chain to send slave s this cycle (the active buffer is set only by
   cmd_mailbox_swap at cycle start, same main-loop context — no masking needed):
     - host-death tripped → the master OVERRIDES with STOP for every motor;
     - else an applied host command → re-send it (ROBOT_CMD);
     - else (nothing ever applied) → a NOP keepalive, which clocks telemetry but does
       NOT refresh the slave's command watchdog (task 6).
   The version gate is spi_link_poll's job, not ours. */
static void poll_one_slave(uint8_t s)
{
    uint8_t     opcode;
    cmd_chain_t chain_local;
    if (g_host_action != HOST_LINK_OK) {
        opcode = SPI_OP_ROBOT_CMD;
        memset(&chain_local, 0, sizeof(chain_local));
        chain_local.chain_id = s;
        uint8_t nm = slave_motor_counts[s];
        chain_local.n_motors = nm;
        /* Host loss: force STOP into every slot and let the slave own the ramp
           (STOPPING → DISABLED, drive_fault NONE; robot_state = HOST_LOST is the
           record). No tuple is carried (VALID stays clear). */
        for (uint8_t i = 0; i < nm && i < MAX_MOTORS_PER_SLAVE; i++) {
            chain_local.motors[i].drive_req = DRV_REQ_STOP;
            chain_local.motors[i].flags     = 0u;
        }
    } else if (g_mbox.active_valid[s]) {
        opcode      = SPI_OP_ROBOT_CMD;
        chain_local = g_mbox.active[s];
    } else {
        opcode = SPI_OP_NOP;
        memset(&chain_local, 0, sizeof(chain_local));
        chain_local.chain_id = s;
        chain_local.n_motors = slave_motor_counts[s];
    }

    spi_link_poll(s, opcode, &chain_local, master_cycle_id, g_mbox.cmd_seq_active);
}

#ifdef USB_TX_TEST
/* Emit self-describing frames of the packet-boundary lengths {63,64,65,128,129,496}
   through the TX slot ring (the path under test): [A5 A5][len u16][idx u8][pattern…].
   64 and 128 exercise the ZLP; 496 is the max telemetry frame. The host verifies all
   six arrive intact and in order. */
static void usb_tx_emit_test_frames(void)
{
    static const uint16_t lens[6] = { 63u, 64u, 65u, 128u, 129u, 496u };
    static uint8_t f[512];
    for (uint8_t k = 0; k < 6u; k++) {
        uint16_t L = lens[k];
        f[0] = 0xA5u; f[1] = 0xA5u;
        f[2] = (uint8_t)(L & 0xFFu); f[3] = (uint8_t)(L >> 8);
        f[4] = k;
        for (uint16_t i = 5u; i < L; i++) f[i] = (uint8_t)((k * 31u + i) & 0xFFu);
        usb_tx_write(f, L);
    }
}
#endif

void master_app_init(SPI_HandleTypeDef *hspi, UART_HandleTypeDef *huart)
{
    master_huart = huart;
    spi_link_init(hspi);
    cmd_mailbox_reset(&g_mbox);
    host_watchdog_reset(&g_host_wd); g_host_action = HOST_LINK_OK;
}

void master_app_loop(void)
{
    static uint16_t cycles_since_status = 0;
    static uint32_t next_imu_poll_ms   = 0;

    /* Drain the USB RX ring (scan + dispatch host frames) and pump the TX ring
       every iteration — both single-producer here in the main loop. */
    usb_rx_process(on_usb_frame);
    usb_tx_pump_responses();
    usb_tx_pump();

    /* One hardware-timed cycle (TIM2 ISR only flagged us; the work is here): poll
       every slave in order, then immediately emit this cycle's tele_robot_t. */
    if (master_cycle_take(NULL)) {
        uint32_t t0 = cycle_us_now();           /* cycle service start (µs) */
        master_cycle_id++;
        cmd_mailbox_swap(&g_mbox, master_cycle_id);  /* apply-at-n+1: newest complete set */
        /* Host-death dead-man: if the host went quiet for too long while armed, override
           what we send the slaves (forced STOP), latched until a RESET-bearing command. */
        g_host_action = host_watchdog_step(&g_host_wd, g_mbox.cycles_since_fresh,
                                           any_motor_armed(), g_mbox.fresh_fault_reset,
                                           HOST_LOST_CYCLES);
        for (uint8_t s = 0; s < NUM_SLAVES; s++) {
            poll_one_slave(s);
        }
#ifdef USB_TX_TEST
        if (g_txtest) {                          /* emit only the test frames this cycle */
            g_txtest = 0u;
            usb_tx_emit_test_frames();
        } else
#endif
        {
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

    /* Sample the BMI088 at 100 Hz (IMU PR). Placed AFTER the cycle block and gated on its
       own HAL_GetTick deadline, so it runs in the idle spin between TIM2 cycles rather than
       inside a cycle's service — it never delays the SPI exchange or telemetry emit.
       NOTE: ImuService_Poll does two BLOCKING I2C1 reads (~1.6 ms typical at 100 kHz; up to
       ~6 ms worst case if a present IMU NAKs/times out). A missing IMU → no-op. If the IMU
       read ever lands right after a cycle it can push that main-loop pass toward the 5 ms
       budget; see the merge notes (candidate: 400 kHz I2C or I2C DMA/interrupt). */
    uint32_t now_ms = HAL_GetTick();
    if ((int32_t)(now_ms - next_imu_poll_ms) >= 0) {
        next_imu_poll_ms = now_ms + IMU_SERVICE_POLL_PERIOD_MS;
        (void)ImuService_Poll();
    }
}
