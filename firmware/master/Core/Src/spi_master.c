#include "spi_master.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

static SPI_HandleTypeDef  *master_hspi  = NULL;
static UART_HandleTypeDef *master_huart = NULL;

/* ── legacy USB-command path (unused, kept per spec) ────────────────────── */
uint8_t new_usb_packet_rx_flag = 0;
uint8_t buf_rx_jet2master[NUM_SLV * MAX_MOTORS_PER_SLAVE * USB_BYTES_PER_MOTOR];
static  uint8_t buf_tx_master2jet[NUM_SLV * MAX_MOTORS_PER_SLAVE * USB_BYTES_PER_MOTOR];
static  motor_cmd_t motor_cmds[MAX_MOTORS_PER_SLAVE * NUM_SLV];
static  motor_cmd_t motor_feedbacks[MAX_MOTORS_PER_SLAVE * NUM_SLV];
static  uint8_t motorID_lut[NUM_SLV][MAX_MOTORS_PER_SLAVE] = { {1, 2} };
        slv_motor_chain_t slaves[NUM_SLV];

/* ── armed-state tracking ────────────────────────────────────────────────── */
static uint8_t master_armed           = 0;
static uint8_t first_arm_packet       = 0;
static uint8_t send_disarm            = 0;
static uint8_t arm_motor_idx          = 0;
static uint8_t first_goto_zero_packet = 0;
static uint8_t goto_zero_motor_idx    = 0;
static SpiMitCmd pending_mit[N_MOTORS];
static uint8_t   mit_pending          = 0;

/* ── master status counters (written by frame decoder in usbd_cdc_if.c) ─── */
uint32_t master_link_errors = 0;
uint32_t master_rx_frames   = 0;

/* ── per-slave runtime state ─────────────────────────────────────────────── */
static uint8_t     slave_alive        = 0;
static uint8_t     slave_motors_alive = 0;
static SpiMotorTele latest_tele[N_MOTORS];
static uint16_t    tx_seq = 0;

/* ── helpers ─────────────────────────────────────────────────────────────── */
static float u16_to_f(uint16_t raw, float lo, float hi)
{
    return lo + (float)raw * (hi - lo) / 65535.0f;
}

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

/* ── SPI exchange with new telemetry format ──────────────────────────────── */
static HAL_StatusTypeDef spi_exchange(SpiDevId dev, uint8_t cmd,
                                       const uint8_t *mit_payload,
                                       uint8_t *motors_alive_out,
                                       SpiMotorTele *tele_out)
{
    uint8_t tx[SPI_RX_PKT_SIZE];
    uint8_t rx[SPI_RX_PKT_SIZE];
    memset(tx, 0, sizeof(tx));
    tx[0] = cmd;
    if (cmd == SPI_CMD_MIT && mit_payload != NULL) {
        memcpy(&tx[1], mit_payload, N_MOTORS * sizeof(SpiMitCmd));
    }

    CS_SELECT(dev);
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(master_hspi, tx, rx,
                                                    SPI_RX_PKT_SIZE,
                                                    HAL_MAX_DELAY);
    CS_ALL_HIGH();

    if (st == HAL_OK) {
        *motors_alive_out = rx[0];
        memcpy(tele_out, &rx[1], N_MOTORS * sizeof(SpiMotorTele));
    }
    return st;
}

/* ── emit protocol frames over USB TX ring ───────────────────────────────── */
static void emit_master_status(uint32_t now_ms)
{
    RobotState rs;
    if (!slave_alive) {
        rs = ROBOT_INIT;
    } else if (slave_motors_alive == ((1u << N_MOTORS) - 1u)) {
        rs = ROBOT_READY;
    } else {
        rs = ROBOT_DEGRADED;
    }

    MasterStatus pay = {0};
    pay.robot_state  = (uint8_t)rs;
    pay.slave_alive  = slave_alive;
    pay.motors_alive = slave_motors_alive;
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

static void emit_slave_status(uint32_t now_ms)
{
    SlaveStatus pay = {0};
    pay.motors_alive = slave_motors_alive;
    pay.motor_state  = (slave_motors_alive & 0x01u)
                         ? (uint8_t)latest_tele[0].state
                         : (uint8_t)MOTOR_BOOT;
    pay.uptime_ms    = now_ms;

    uint8_t frame[MSG_HEADER_SIZE + sizeof(SlaveStatus)];
    uint16_t n = proto_build(frame, sizeof(frame),
                              MSG_SLAVE_STATUS, tx_seq++,
                              NODE_MASTER, NODE_JETSON,
                              now_ms,
                              (const uint8_t *)&pay, sizeof(pay));
    if (n > 0u) usb_tx_write(frame, n);
}

static void emit_motor_state(uint8_t idx, uint32_t now_ms)
{
    const SpiMotorTele *t = &latest_tele[idx];
    MotorStatePayload pay = {0};
    pay.motor_idx    = idx;
    pay.state        = t->state;
    pay.pos          = u16_to_f(t->pos_raw, MOTOR_P_MIN, MOTOR_P_MAX);
    pay.vel          = u16_to_f(t->vel_raw, MOTOR_V_MIN, MOTOR_V_MAX);
    pay.tau          = u16_to_f(t->tau_raw, MOTOR_T_MIN, MOTOR_T_MAX);
    pay.temp         = (float)t->temp_c;
    pay.fault_flags  = t->fault_flags;
    pay.last_cmd_seq = t->last_cmd_seq;

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
    resp.motor_idx = req->motor_idx;
    resp.cmd       = req->cmd;
    resp.req_seq   = req_seq;

    switch ((ControlCmd)req->cmd) {
        case CTRL_ARM_HOLD:
            if (!slave_alive) {
                resp.result    = CTRL_ERR_STATE;
                resp.new_state = MOTOR_BOOT;
                break;
            }
            arm_motor_idx = req->motor_idx;
            MotorMaster_SetArmed(1u);
            resp.result    = CTRL_OK;
            resp.new_state = MOTOR_ARMED_HOLD;
            break;
        case CTRL_DISABLE:
            MotorMaster_SetArmed(0u);
            resp.result    = CTRL_OK;
            resp.new_state = MOTOR_IDLE;
            break;
        case CTRL_GOTO_ZERO:
            if (!slave_alive) {
                resp.result    = CTRL_ERR_STATE;
                resp.new_state = MOTOR_BOOT;
                break;
            }
            goto_zero_motor_idx    = req->motor_idx;
            first_goto_zero_packet = 1;
            master_armed           = 1;   /* keep HOLD flowing for watchdog */
            first_arm_packet       = 0;
            send_disarm            = 0;
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
    if (n > 0u) usb_tx_write(frame, n);
}

/* ── Public API ──────────────────────────────────────────────────────────── */
void MotorMaster_Init(SPI_HandleTypeDef *hspi, UART_HandleTypeDef *huart)
{
    master_hspi  = hspi;
    master_huart = huart;
    CS_ALL_HIGH();

    for (uint8_t i = 0; i < NUM_SLV; i++) {
        slaves[i].slv_id             = (SpiDevId)i;
        slaves[i].slv_motor_cmds     = &motor_cmds[i * MAX_MOTORS_PER_SLAVE];
        slaves[i].slv_motor_feedbacks = &motor_feedbacks[i * MAX_MOTORS_PER_SLAVE];
        slaves[i].active_motor_count = 2;
        slaves[i].motorID_lut        = motorID_lut[i];
        for (uint8_t j = 0; j < 2u; j++) {
            slaves[i].slv_motor_cmds[j].motor_id = slaves[i].motorID_lut[j];
            slaves[i].slv_motor_cmds[j].position  = 0.0f;
            slaves[i].slv_motor_cmds[j].speed     = 0.0f;
        }
    }
}

void MotorMaster_SetMitCmd(uint8_t idx, float pos, float vel,
                            float kp, float kd, float tau_ff)
{
    (void)kp; (void)kd; (void)tau_ff;  /* slave uses default_kp/kd from motor_config */
    if (idx >= N_MOTORS) return;
    pending_mit[idx].pos   = pos;
    pending_mit[idx].vel   = vel;
    pending_mit[idx].valid = 1u;
    mit_pending = 1u;
}

void MotorMaster_SetArmed(uint8_t armed)
{
    if (armed) {
        first_arm_packet = 1;
        send_disarm      = 0;
    } else {
        send_disarm = 1;
    }
    master_armed = armed;
}

/* Kept for ABI compatibility – not actively used */
void MotorMaster_ParseRxBuffer(void) { (void)buf_tx_master2jet; }
void MotorMaster_FormatTxBuffer(void) {}

void MotorMaster_ProcessLoop(void)
{
    static uint32_t next_poll_ms   = 0;
    static uint32_t next_status_ms = 0;
    uint32_t now = HAL_GetTick();

    /* Drain USB TX ring buffer */
    usb_tx_pump();

    /* 50 ms SPI poll */
    if ((int32_t)(now - next_poll_ms) >= 0) {
        next_poll_ms += 50u;

        uint8_t cmd;
        const uint8_t *mit_payload = NULL;
        uint8_t sent_arm       = 0;
        uint8_t sent_goto_zero = 0;
        if (send_disarm) {
            cmd = SPI_CMD_DISARM; send_disarm = 0;
        } else if (first_goto_zero_packet) {
            cmd = SPI_CMD_GOTO_ZERO_IDX(goto_zero_motor_idx);
            sent_goto_zero = 1;             /* clear only on HAL_OK */
        } else if (mit_pending) {
            cmd = SPI_CMD_MIT;
            mit_payload = (const uint8_t *)pending_mit;
            mit_pending = 0u;
            /* valid flags cleared AFTER exchange so memcpy in spi_exchange sees valid=1 */
        } else if (master_armed) {
            if (first_arm_packet) {
                cmd = SPI_CMD_ARM_IDX(arm_motor_idx);
                sent_arm = 1;               /* clear only on HAL_OK */
            } else {
                cmd = SPI_CMD_HOLD;
            }
        } else {
            cmd = SPI_CMD_NOP;
        }

        uint8_t alive = 0;
        SpiMotorTele tele[N_MOTORS];
        HAL_StatusTypeDef st = spi_exchange(DEV1, cmd, mit_payload, &alive, tele);
        if (cmd == SPI_CMD_MIT) {
            for (uint8_t i = 0; i < N_MOTORS; i++) pending_mit[i].valid = 0u;
        }

        if (st == HAL_OK) {
            /* Confirm one-shot packets only after successful delivery */
            if (sent_arm)       first_arm_packet       = 0;
            if (sent_goto_zero) first_goto_zero_packet = 0;
            slave_alive        = 1u;
            slave_motors_alive = alive;
            memcpy(latest_tele, tele, sizeof(tele));

            for (uint8_t i = 0; i < N_MOTORS; i++) {
                emit_motor_state(i, now);
            }
        }
    }

    /* 20 Hz status emit */
    if ((int32_t)(now - next_status_ms) >= 0) {
        next_status_ms += 50u;
        emit_master_status(now);
        emit_slave_status(now);
    }
}
