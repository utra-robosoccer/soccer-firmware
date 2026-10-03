#include "spi_master.h"
#include "imu_service.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

static SPI_HandleTypeDef  *master_hspi  = NULL;
static UART_HandleTypeDef *master_huart = NULL;

/* Loop cadences. Command delivery (SPI poll) runs at 200 Hz — the ceiling for
 * the Jetson MIT stream, which the master coalesces (latest wins) above this
 * rate; telemetry/status to the host are decoupled and emitted independently. */
#define MASTER_POLL_PERIOD_MS   5u    /* 200 Hz SPI / command loop (per slave) */
#define MASTER_TELE_PERIOD_MS   5u    /* 200 Hz MOTOR_STATE telemetry */
#define MASTER_STATUS_PERIOD_MS 50u   /* 20 Hz MASTER/SLAVE_STATUS */

/* ── legacy USB-command path (unused, kept per spec / .h ABI) ────────────── */
uint8_t new_usb_packet_rx_flag = 0;
uint8_t buf_rx_jet2master[NUM_SLV * MAX_MOTORS_PER_SLAVE * USB_BYTES_PER_MOTOR];

/* ── per-slave command state ─────────────────────────────────────────────── */
static uint8_t   master_armed[NUM_SLAVES];
static uint8_t   send_disarm[NUM_SLAVES];
/* Per-motor one-shot command bitmasks (bit i = local motor i pending). Each bit
   is cleared only once telemetry confirms the state change, so rapid-fire
   arm/zero from the host can't overwrite itself while the slave is busy. */
static uint8_t   pending_arm_bits[NUM_SLAVES];
static uint8_t   pending_goto_zero_bits[NUM_SLAVES];
static SpiMitCmd pending_mit[NUM_SLAVES][MAX_MOTORS_PER_SLAVE];
static uint8_t   mit_pending[NUM_SLAVES];

/* ── master status counters (written by frame decoder in usbd_cdc_if.c) ─── */
uint32_t master_link_errors = 0;
uint32_t master_rx_frames   = 0;

/* ── per-slave runtime state ─────────────────────────────────────────────── */
static uint8_t      slave_alive[NUM_SLAVES];
static uint8_t      slave_motors_alive[NUM_SLAVES];
static MotorState   latest_atom[NUM_SLAVES][MAX_MOTORS_PER_SLAVE];
static uint32_t     slave_crc_errors[NUM_SLAVES];   /* telemetry frames failing CRC */
static uint8_t      spi_seq[NUM_SLAVES];             /* per-slave command seq counter  */
/* seq/echo diagnostics (fix 5) and relayed slave-side command-CRC count (fix 3). */
#define SEQ_STALL_POLLS 5u                           /* echo frozen this many polls = a gap */
static uint8_t      prev_echo[NUM_SLAVES];           /* last echo seen                  */
static uint8_t      have_prev_echo[NUM_SLAVES];      /* prev_echo valid yet             */
static uint8_t      echo_stall[NUM_SLAVES];          /* consecutive polls w/ no advance */
static uint32_t     slave_seq_gaps[NUM_SLAVES];      /* command-link stalls (echo froze) */
static uint32_t     slave_cmd_crc_errors[NUM_SLAVES];/* from slave_debug_rsvd[0..3]     */
static uint32_t     slave_zero_rejects[NUM_SLAVES];  /* from slave_debug_rsvd[4..7]     */
static uint32_t     slave_zero_rejects_seen[NUM_SLAVES]; /* last value we logged        */
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

/* ── SPI exchange with one slave (length sized to that slave's motor count) ──
   One full-duplex transfer: the command frame ([cmd][seq][SpiMitCmd×N]) rides
   in the TX prefix; the CRC-framed telemetry frame comes back in RX. The frame
   CRC doubles as an integrity check AND a presence check — an absent slave
   clocks back garbage that fails CRC, so no separate handshake is needed.
   Returns HAL_OK only on a CRC-valid frame; a CRC failure increments the
   slave's crc-error counter and returns HAL_ERROR. */
static HAL_StatusTypeDef spi_exchange(SpiDevId dev, uint8_t n_motors,
                                       uint8_t cmd, uint8_t seq,
                                       const uint8_t *mit_payload,
                                       uint8_t *motors_alive_out,
                                       uint8_t *echo_seq_out,
                                       MotorState *tele_out)
{
    uint8_t tx[SPI_MAX_PKT_SIZE];
    uint8_t rx[SPI_MAX_PKT_SIZE];
    uint16_t len = SPI_PKT_SIZE(n_motors);
    memset(tx, 0, len);
    tx[0] = cmd;
    tx[1] = seq;
    if (cmd == SPI_CMD_MIT && mit_payload != NULL) {
        memcpy(&tx[SPI_CMD_HDR_BYTES], mit_payload,
               (size_t)n_motors * sizeof(SpiMitCmd));
    }
    /* Fix 3: CRC-protect the command frame (every command, incl. HOLD/NOP). The
       slave verifies this before applying anything. */
    uint16_t ccrc = proto_crc16(tx, SPI_CMD_CRC_OFF(n_motors));
    tx[SPI_CMD_CRC_OFF(n_motors)]     = (uint8_t)(ccrc & 0xFFu);
    tx[SPI_CMD_CRC_OFF(n_motors) + 1] = (uint8_t)(ccrc >> 8);

    CS_SELECT(dev);
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(master_hspi, tx, rx,
                                                    len, HAL_MAX_DELAY);
    CS_ALL_HIGH();
    if (st != HAL_OK) return st;

    uint16_t rx_crc = (uint16_t)(rx[len - 2] | ((uint16_t)rx[len - 1] << 8));
    uint16_t calc   = proto_crc16(rx, (size_t)(len - SPI_TELE_CRC_BYTES));
    if (rx_crc != calc) {
        slave_crc_errors[(uint8_t)dev]++;
        return HAL_ERROR;              /* bad/absent frame → caller marks offline */
    }

    *motors_alive_out = rx[0];
    *echo_seq_out     = rx[1];
    memcpy(tele_out, &rx[SPI_TELE_HDR_BYTES],
           (size_t)n_motors * sizeof(MotorState));
    /* Relayed slave-side debug counters: cmd-CRC rejects (bytes 0..3) and
       GOTO_ZERO rejects (bytes 4..7), both u32 LE over the reserved region. */
    const uint8_t *dbg = &rx[SPI_TELE_DEBUG_OFF(n_motors)];
    slave_cmd_crc_errors[(uint8_t)dev] = (uint32_t)dbg[0] | ((uint32_t)dbg[1] << 8) |
                                         ((uint32_t)dbg[2] << 16) | ((uint32_t)dbg[3] << 24);
    slave_zero_rejects[(uint8_t)dev]   = (uint32_t)dbg[4] | ((uint32_t)dbg[5] << 8) |
                                         ((uint32_t)dbg[6] << 16) | ((uint32_t)dbg[7] << 24);
    return HAL_OK;
}

/* ── emit protocol frames over USB TX ring ───────────────────────────────── */
static void emit_master_status(uint32_t now_ms)
{
    uint8_t alive_mask = 0u;
    uint8_t any_alive  = 0u;
    uint8_t all_full   = 1u;
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
    RobotState rs = !any_alive ? ROBOT_INIT
                  : (all_full  ? ROBOT_READY : ROBOT_DEGRADED);

    MasterStatus pay = {0};
    pay.robot_state  = (uint8_t)rs;
    pay.slave_alive  = alive_mask;               /* bit s = slave s reachable */
    pay.uptime_ms    = now_ms;
    pay.link_errors  = master_link_errors;
    pay.rx_frames    = master_rx_frames;

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

static void emit_motor_state(uint8_t s, uint8_t idx, uint32_t now_ms)
{
    /* Pass-through: forward the raw MotorState atom to the host, which decodes
       raw→units with the generated transport bounds. The master no longer
       touches motor data in either direction. */
    MotorStatePayload pay;
    pay.slave_id  = s;
    pay.motor_idx = idx;
    pay.atom      = latest_atom[s][idx];

    uint8_t frame[MSG_HEADER_SIZE + sizeof(MotorStatePayload)];
    uint16_t n = proto_build(frame, sizeof(frame),
                              MSG_MOTOR_STATE, tx_seq++,
                              NODE_MASTER, NODE_JETSON,
                              now_ms,
                              (const uint8_t *)&pay, sizeof(pay));
    if (n > 0u) usb_tx_write(frame, n);
}

/* ── CONTROL_REQ handler (called from frame decoder) ────────────────────── */
void MotorMaster_HandleControlReq(const ControlReq *req, uint16_t req_seq)
{
    ControlResp resp = {0};
    resp.slave_id  = req->slave_id;
    resp.motor_idx = req->motor_idx;
    resp.cmd       = req->cmd;
    resp.req_seq   = req_seq;

    uint8_t s = req->slave_id;
    if (s >= NUM_SLAVES) {
        master_link_errors++;
        return;
    }

    switch ((ControlCmd)req->cmd) {
        case CTRL_ARM_HOLD:
            if (!slave_alive[s]) {
                resp.result    = CTRL_ERR_STATE;
                resp.new_state = MOTOR_BOOT;
                break;
            }
            if (req->motor_idx < slave_motor_counts[s])
                pending_arm_bits[s] |= (uint8_t)(1u << req->motor_idx);
            master_armed[s] = 1u;
            send_disarm[s]  = 0u;
            resp.result    = CTRL_OK;
            resp.new_state = MOTOR_ARMED_HOLD;
            break;
        case CTRL_DISABLE:
            send_disarm[s]            = 1u;
            master_armed[s]           = 0u;
            pending_arm_bits[s]       = 0u;
            pending_goto_zero_bits[s] = 0u;
            resp.result    = CTRL_OK;
            resp.new_state = MOTOR_IDLE;
            break;
        case CTRL_GOTO_ZERO:
            if (!slave_alive[s]) {
                resp.result    = CTRL_ERR_STATE;
                resp.new_state = MOTOR_BOOT;
                break;
            }
            if (req->motor_idx < slave_motor_counts[s])
                pending_goto_zero_bits[s] |= (uint8_t)(1u << req->motor_idx);
            master_armed[s] = 1u;   /* keep HOLD flowing for watchdog */
            send_disarm[s]  = 0u;
            resp.result    = CTRL_OK;
            resp.new_state = MOTOR_ZEROING;
            break;
        case CTRL_SET_ZERO:
            resp.result    = CTRL_ERR_STUB;
            resp.new_state = 0u;
            break;
        default:
            master_link_errors++;
            return;
    }

    uint8_t frame[MSG_HEADER_SIZE + sizeof(ControlResp)];
    uint16_t n = proto_build(frame, sizeof(frame),
                              MSG_CONTROL_RESP, tx_seq++,
                              NODE_MASTER, NODE_JETSON,
                              HAL_GetTick(),
                              (const uint8_t *)&resp, sizeof(resp));
    /* Runs in USB-ISR context (via CDC_Receive_FS) → post to the response queue;
       main drains it into the ring, keeping usb_tx_write single-producer. */
    if (n > 0u) usb_tx_post_from_isr(frame, n);
}

/* ── Public API ──────────────────────────────────────────────────────────── */
void MotorMaster_Init(SPI_HandleTypeDef *hspi, UART_HandleTypeDef *huart)
{
    master_hspi  = hspi;
    master_huart = huart;
    CS_ALL_HIGH();
    memset(slave_alive, 0, sizeof(slave_alive));
    memset(slave_motors_alive, 0, sizeof(slave_motors_alive));
    memset(master_armed, 0, sizeof(master_armed));
    memset(send_disarm, 0, sizeof(send_disarm));
    memset(pending_arm_bits, 0, sizeof(pending_arm_bits));
    memset(pending_goto_zero_bits, 0, sizeof(pending_goto_zero_bits));
    memset(pending_mit, 0, sizeof(pending_mit));
    memset(mit_pending, 0, sizeof(mit_pending));
    memset(latest_atom, 0, sizeof(latest_atom));
    memset(slave_crc_errors, 0, sizeof(slave_crc_errors));
    memset(spi_seq, 0, sizeof(spi_seq));
    memset(prev_echo, 0, sizeof(prev_echo));
    memset(have_prev_echo, 0, sizeof(have_prev_echo));
    memset(echo_stall, 0, sizeof(echo_stall));
    memset(slave_seq_gaps, 0, sizeof(slave_seq_gaps));
    memset(slave_cmd_crc_errors, 0, sizeof(slave_cmd_crc_errors));
    memset(slave_zero_rejects, 0, sizeof(slave_zero_rejects));
    memset(slave_zero_rejects_seen, 0, sizeof(slave_zero_rejects_seen));
}

void MotorMaster_SetMitCmd(uint8_t slave_id, uint8_t idx, float pos, float vel,
                            float kp, float kd, float tau_ff)
{
    (void)kp; (void)kd; (void)tau_ff;  /* slave uses default_kp/kd from motor_config */
    if (slave_id >= NUM_SLAVES) return;
    if (idx >= slave_motor_counts[slave_id]) return;
    pending_mit[slave_id][idx].pos   = pos;
    pending_mit[slave_id][idx].vel   = vel;
    pending_mit[slave_id][idx].valid = 1u;
    mit_pending[slave_id] = 1u;
}

/* Global arm/e-stop across all slaves (e.g. a master-level disable). */
void MotorMaster_SetArmed(uint8_t armed)
{
    for (uint8_t s = 0; s < NUM_SLAVES; s++) {
        if (armed) {
            send_disarm[s] = 0u;
        } else {
            send_disarm[s]            = 1u;
            pending_arm_bits[s]       = 0u;
            pending_goto_zero_bits[s] = 0u;
        }
        master_armed[s] = armed;
    }
}

/* Kept for ABI compatibility – not actively used */
void MotorMaster_ParseRxBuffer(void) { (void)buf_rx_jet2master; }
void MotorMaster_FormatTxBuffer(void) {}

/* Select + deliver one command to slave `s`, then ingest its telemetry. */
static void poll_one_slave(uint8_t s)
{
    uint8_t n = slave_motor_counts[s];

    uint8_t cmd;
    uint8_t sent_arm_idx       = 0;
    uint8_t sent_goto_zero_idx = 0;
    uint8_t sent_arm           = 0;
    uint8_t sent_goto_zero     = 0;
    uint8_t is_mit             = 0;
    SpiMitCmd mit_local[MAX_MOTORS_PER_SLAVE];

    /* Fix 2: command staging (pending_*, mit_pending, send_disarm) is written by
       the USB ISR. Select the command, SNAPSHOT the MIT setpoints, and claim the
       one-shot state in ONE short critical section so a mid-read ISR write can't
       split a setpoint or lose a bit. The pending_arm/goto_zero bits are only READ
       here; they're cleared on telemetry confirmation below, also under a mask. */
    __disable_irq();
    if (send_disarm[s]) {
        cmd = SPI_CMD_DISARM; send_disarm[s] = 0u;
    } else if (pending_goto_zero_bits[s]) {
        sent_goto_zero_idx = (uint8_t)__builtin_ctz(pending_goto_zero_bits[s]);
        cmd = SPI_CMD_GOTO_ZERO_IDX(sent_goto_zero_idx);
        sent_goto_zero = 1;
    } else if (pending_arm_bits[s]) {
        sent_arm_idx = (uint8_t)__builtin_ctz(pending_arm_bits[s]);
        cmd = SPI_CMD_ARM_IDX(sent_arm_idx);
        sent_arm = 1;
    } else if (mit_pending[s]) {
        cmd = SPI_CMD_MIT;
        memcpy(mit_local, pending_mit[s], (size_t)n * sizeof(SpiMitCmd));  /* coherent copy */
        mit_pending[s] = 0u;
        for (uint8_t i = 0; i < n; i++) pending_mit[s][i].valid = 0u;
        is_mit = 1;
    } else if (master_armed[s]) {
        cmd = SPI_CMD_HOLD;
    } else {
        cmd = SPI_CMD_NOP;
    }
    __enable_irq();

    uint8_t seq_to_send = spi_seq[s]++;
    uint8_t alive = 0;
    uint8_t echo  = 0;
    MotorState tele[MAX_MOTORS_PER_SLAVE];
    HAL_StatusTypeDef st = spi_exchange((SpiDevId)s, n, cmd, seq_to_send,
                                        is_mit ? (const uint8_t *)mit_local : NULL,
                                        &alive, &echo, tele);

    if (st == HAL_OK) {   /* CRC verified inside spi_exchange = valid + present */
        /* Fix 5 (diagnostics only): the echoed seq advances as the slave confirms
           commands, but it rides the COALESCED telemetry stream and is pipelined
           by the TX double-buffer, so per-poll it jitters (+1 or +2) — that's
           normal, not loss. A GAP is a sustained STALL: the echo frozen while we
           keep polling — the signature of a command link that stopped confirming
           (a blocking slave op, or a dead link). Any advance resets the streak, so
           idle reads 0. Single CRC-dropped commands are caught definitively by
           cmd_crc_errors instead; no retransmit here (MIT is latest-value; one-shots
           retry until confirmed below). */
        if (have_prev_echo[s]) {
            if (echo == prev_echo[s]) {
                if (++echo_stall[s] >= SEQ_STALL_POLLS) { slave_seq_gaps[s]++; echo_stall[s] = 0u; }
            } else {
                echo_stall[s] = 0u;
            }
        }
        prev_echo[s]      = echo;
        have_prev_echo[s] = 1u;

        /* Confirm one-shot bits against telemetry (retry until the state shows up).
           Clear under a mask — the USB ISR may |= a new bit concurrently. state
           carries the fault cause in its high nibble; mask to lifecycle. */
        if (sent_arm && SPI_STATE_LIFE(tele[sent_arm_idx].state) == MOTOR_ARMED_HOLD) {
            __disable_irq();
            pending_arm_bits[s] &= ~(uint8_t)(1u << sent_arm_idx);
            __enable_irq();
        }
        if (sent_goto_zero) {
            uint8_t zs = SPI_STATE_LIFE(tele[sent_goto_zero_idx].state);
            if (zs == MOTOR_ZEROING || zs == MOTOR_ARMED_HOLD) {
                __disable_irq();
                pending_goto_zero_bits[s] &= ~(uint8_t)(1u << sent_goto_zero_idx);
                __enable_irq();
            }
        }
        slave_alive[s]        = 1u;
        slave_motors_alive[s] = alive;
        memcpy(latest_atom[s], tele, (size_t)n * sizeof(MotorState));
    } else {
        /* Absent slave / garbage frame — mark offline and drop pending one-shots
           so they don't pile up against a board that isn't there. Mask the drops
           vs the ISR; reset the seq-echo baseline so recovery doesn't false-count. */
        slave_alive[s]        = 0u;
        slave_motors_alive[s] = 0u;
        have_prev_echo[s]     = 0u;
        echo_stall[s]         = 0u;
        __disable_irq();
        pending_arm_bits[s]       = 0u;
        pending_goto_zero_bits[s] = 0u;
        __enable_irq();
    }
}

void MotorMaster_ProcessLoop(void)
{
    static uint32_t next_poll_ms   = 0;
    static uint32_t next_tele_ms   = 0;
    static uint32_t next_status_ms = 0;
    static uint32_t next_imu_poll_ms = 0;
    uint32_t now = HAL_GetTick();

    /* Move ISR-posted responses (ControlResp/PONG) into the ring — this is the
       only place they enter usb_tx_write, keeping it single-producer — then drain
       the ring to USB. */
    usb_tx_pump_responses();
    usb_tx_pump();

    /* SPI poll — command delivery at 200 Hz, every slave each tick */
    if ((int32_t)(now - next_poll_ms) >= 0) {
        next_poll_ms += MASTER_POLL_PERIOD_MS;
        for (uint8_t s = 0; s < NUM_SLAVES; s++) {
            poll_one_slave(s);
        }
    }

    /* Telemetry emit (MOTOR_STATE) — one frame per motor, tagged with slave.
       GATED on slave_alive[s]: only emit atoms for a slave whose most recent poll
       CRC-passed (poll + tele run in lockstep here). A failed/absent poll → no
       MOTOR_STATE for that slave this tick, so on the host silence is meaningful
       (end-to-end freshness). SlaveStatus below still reports the silent slave. */
    if ((int32_t)(now - next_tele_ms) >= 0) {
        next_tele_ms += MASTER_TELE_PERIOD_MS;
        for (uint8_t s = 0; s < NUM_SLAVES; s++) {
            if (!slave_alive[s]) continue;
            for (uint8_t i = 0; i < slave_motor_counts[s]; i++) {
                emit_motor_state(s, i, now);
            }
        }
    }

    /* Status emit at 20 Hz */
    if ((int32_t)(now - next_status_ms) >= 0) {
        next_status_ms += MASTER_STATUS_PERIOD_MS;
        emit_master_status(now);
        for (uint8_t s = 0; s < NUM_SLAVES; s++) {
            emit_slave_status(s, now);
            /* Surface a climbing GOTO_ZERO reject count (relayed over the SPI
               debug region) as a host-visible log line — SlaveStatus has no free
               field in the frozen v1 wire layout, so this is the visibility path. */
            if (slave_zero_rejects[s] != slave_zero_rejects_seen[s]) {
                usb_printf("[zero-reject] slave%u refused GOTO_ZERO (count=%lu)\r\n",
                           (unsigned)s, (unsigned long)slave_zero_rejects[s]);
                slave_zero_rejects_seen[s] = slave_zero_rejects[s];
            }
        }
    }

    /* Sample the BMI088 at 100 Hz after motor and telemetry work for this pass. */
    if ((int32_t)(now - next_imu_poll_ms) >= 0) {
        next_imu_poll_ms = now + IMU_SERVICE_POLL_PERIOD_MS;
        (void)ImuService_Poll();
    }
}
