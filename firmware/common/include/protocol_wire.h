/* protocol_wire.h — the wire contract: every struct, enum, flag, scale and
 * frame size for host <-> master (USB) and master <-> slave (SPI).
 * Hand-written; replaces the generated protocol_gen.h. Edit this file directly.
 *
 * Include "protocol.h" (the wrapper), not this file: it pulls this in and adds
 * the fixed-point, CRC16 and USB frame build/scan helpers.
 *
 * Shared as-is by the master, the slaves and the C++ host (stm32_transport).
 * The Python tools mirror it with ctypes and are checked by the fixture test.
 *
 * Rules that keep one header safe to share:
 *   - every struct is packed, fields are fixed-width integers only
 *   - all multi-byte fields are little-endian (STM32, Jetson and x86 all are)
 *   - enums are stored on the wire as uint8_t; the enum type only names values
 *   - no bitfields, no pointers, no floats
 *   - every struct size is pinned by a static assert at the bottom
 *
 * There is NO version or schema hash. A size change fails the length/CRC
 * checks; a same-size reorder does not — always flash every board together.
 *
 * Naming: cmd_*  = host -> robot      tele_*   = robot -> host, every cycle
 *         status_* = counters/health nested inside a tele_* struct
 *         <level> = master | slave | motor, named after who AUTHORS the data
 *         rs_*    = values reported by the RobStride motor itself
 */
#ifndef PROTOCOL_WIRE_H
#define PROTOCOL_WIRE_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#define PROTO_STATIC_ASSERT(cond, msg) static_assert(cond, msg)
#else
#define PROTO_STATIC_ASSERT(cond, msg) _Static_assert(cond, msg)
#endif

#define PROTO_PACKED __attribute__((packed))

/* ── limits ──────────────────────────────────────────────────────────────── */
#define MAX_SLAVES            4u   /* wire array length; n_slaves says how many are sent */
#define MAX_MOTORS_PER_SLAVE  5u   /* wire array length; n_motors says how many are valid */
#define CMD_SEQ_NONE          0u   /* "no host command applied yet"; host counts from 1 */

/* ── fixed-point scales (value = wire / scale) ───────────────────────────── */
#define PROTO_POS_SCALE   10000.0f  /* rad   -> i16, covers +-3.2767 rad (joint frame) */
#define PROTO_VEL_SCALE     100.0f  /* rad/s -> i16 */
#define PROTO_TAU_SCALE     100.0f  /* N*m   -> i16 */
#define PROTO_KP_SCALE       10.0f  /* Kp    -> u16, covers 0..5000 */
#define PROTO_KD_SCALE      100.0f  /* Kd    -> u16, covers 0..100 */
#define PROTO_TEMP_SCALE     10.0f  /* degC  -> i16 (the motor's native 0.1 degC) */
/* IMU raw counts -> SI (BMI088 at +-3 g, +-1000 deg/s; SI value = raw * LSB) */
#define IMU_ACCEL_LSB_MPS2  ((9.81f * 3.0f) / 32768.0f)
#define IMU_GYRO_LSB_RPS    ((0.01745329251f * 1000.0f) / 32768.0f)

/* ═════════════════════════════ enums (u8 on the wire) ═════════════════════ */

/* Host -> motor request. Level-triggered: repeat it every cycle. */
typedef enum {
    MOTOR_REQ_STOP   = 0u,  /* controlled stop (damp, then off). Zeroed command = stop. */
    MOTOR_REQ_ENABLE = 1u,  /* DISABLED -> ENABLING -> ENABLED; rejected unless READY */
} MotorReq;

/* Host -> slave request. Level-triggered. */
typedef enum {
    SLAVE_REQ_NONE  = 0u,
    SLAVE_REQ_RESET = 1u,  /* accepted only while no motor is armed: re-run the id
                              sweep + verify, clear every motor fault, repeat while the
                              request stays asserted. The slave stays RESETTING until
                              the request is DROPPED, then exits to READY if the sweep
                              and verify passed (motors still dirty stay MOTOR_FAULT)
                              or BLOCKED if they did not. No timeout. */
} SlaveReq;

/* The slave's view of one motor. */
typedef enum {
    MOTOR_ABSENT   = 0u,  /* did not answer the sweep, or went silent since */
    MOTOR_DISABLED = 1u,  /* off, polled every cycle */
    MOTOR_ENABLING = 2u,  /* enable sent, waiting for the motor to report RS_MODE_RUN */
    MOTOR_ENABLED  = 3u,  /* applies cmd_motor_t.mit every cycle (all-zero = limp) */
    MOTOR_STOPPING = 4u,  /* controlled stop in progress (damping) */
    MOTOR_FAULT    = 5u,  /* latched, see MotorFault; only SLAVE_REQ_RESET leaves */
} MotorState;

/* Why a motor is in MOTOR_FAULT. First cause wins. The comment names the stop
 * used: EMERGENCY = off now, re-sent until confirmed; CONTROLLED = damp, then off. */
typedef enum {
    MOTOR_FAULT_NONE             = 0u,
    MOTOR_FAULT_FEEDBACK_TIMEOUT = 1u,  /* EMERGENCY  — no feedback frame in time */
    MOTOR_FAULT_ENABLE_FAILED    = 2u,  /* EMERGENCY  — never reported RUN after enable */
    MOTOR_FAULT_LOST_ENABLE      = 3u,  /* EMERGENCY  — enabled motor dropped out of RUN */
    MOTOR_FAULT_OVERTORQUE       = 4u,  /* EMERGENCY  — |tau| above the configured max */
    MOTOR_FAULT_RS_REPORTED      = 5u,  /* EMERGENCY  — motor set fault bits / CALI */
    MOTOR_FAULT_MASTER_LOST      = 6u,  /* CONTROLLED — slave stopped hearing the master */
} MotorFault;
/* An enable refused for position outside the soft limits (+ ready margin) is a
 * plain rejection (MOTOR_FLAG_REQ_REJECTED), not a fault — the motor stays
 * DISABLED and loses MOTOR_FLAG_READY until the shaft is moved back in. */
/* Host loss is not a motor fault: the master sends MOTOR_REQ_STOP to every motor
 * (CONTROLLED, ends DISABLED) and reports it in status_master_t.host.lost. */

/* "Mode status" the motor reports in every feedback frame (enabled or not).
 * Not the control mode: that is the run_mode register (0x7005) — name it
 * RsRunMode if a second control mode is ever added. */
typedef enum {
    RS_MODE_RESET = 0u,  /* off */
    RS_MODE_CALI  = 1u,  /* encoder calibration (service mode; never expected) */
    RS_MODE_RUN   = 2u,  /* enabled */
} RsMode;

typedef enum {
    SLAVE_BOOT      = 0u,  /* sweep / verify still running */
    SLAVE_READY     = 1u,  /* roster matches and every motor passed verify */
    SLAVE_BLOCKED   = 2u,  /* sweep or verify failed, see boot_result_t; no enables */
    SLAVE_RESETTING = 3u,  /* SLAVE_REQ_RESET in progress */
} SlaveState;

/* Summary computed by the master from the slaves and motors; drives nothing. */
typedef enum {
    ROBOT_BOOT     = 0u,  /* a configured slave is not alive yet or still in SLAVE_BOOT */
    ROBOT_BLOCKED  = 1u,  /* any slave is SLAVE_BLOCKED */
    ROBOT_READY    = 2u,  /* all slaves READY and every motor has MOTOR_FLAG_READY */
    ROBOT_RUNNING  = 3u,  /* every motor is MOTOR_ENABLED */
    ROBOT_DEGRADED = 4u,  /* anything else: a fault, a lost slave, a partial enable */
} RobotState;

typedef enum {
    USB_MSG_PING = 1u,  /* either direction; the reply echoes seq */
    USB_MSG_CMD  = 2u,  /* host -> master: one cmd_master_t per host tick */
    USB_MSG_TELE = 3u,  /* master -> host: one tele_master_t per master cycle */
} UsbMsgType;

typedef enum {
    SPI_OP_NOP = 0u,  /* keepalive: no fresh host command this cycle */
    SPI_OP_CMD = 1u,  /* frame carries a fresh cmd_slave_t */
} SpiOpcode;

/* ═════════════════════════════════ flags ══════════════════════════════════ */

/* tele_motor_t.flags */
#define MOTOR_FLAG_READY         (1u << 0)  /* an ENABLE would be accepted right now */
#define MOTOR_FLAG_REQ_REJECTED  (1u << 1)  /* the last request was refused */
#define MOTOR_FLAG_CMD_STALE     (1u << 2)  /* holding the last command (none fresh) */
#define MOTOR_FLAG_CLAMPED_POS   (1u << 3)  /* commanded pos hit a soft limit */
#define MOTOR_FLAG_CLAMPED_TAU   (1u << 4)  /* feedforward cut at a limit */
#define MOTOR_FLAG_CLAMPED_GAIN  (1u << 5)  /* kp/kd clamped to the model's range */
#define MOTOR_FLAG_SATURATED     (1u << 6)  /* a fixed-point field saturated */

/* rs_feedback_t.fault_bits (feedback-frame fault bits, as the motor sends them) */
#define RS_FAULT_UNDERVOLTAGE  (1u << 0)
#define RS_FAULT_OVERCURRENT   (1u << 1)
#define RS_FAULT_OVERTEMP      (1u << 2)
#define RS_FAULT_ENCODER       (1u << 3)
#define RS_FAULT_STALL         (1u << 4)
#define RS_FAULT_UNCALIBRATED  (1u << 5)

/* imu_t.flags */
#define IMU_FLAG_INIT_OK  (1u << 0)  /* both sensor dies initialised */
#define IMU_FLAG_VALID    (1u << 1)  /* accel/gyro hold a real sample */

/* ══════════════════════ commands: host -> master -> slave ═════════════════ */

/* Setpoint for RobStride operation-control (MIT) mode:
 * torque = kp*(pos - q) + kd*(vel - dq) + tau_ff. All zero = no effort (limp).
 * A second control mode later = a sibling struct (e.g. rs_vel_t) plus a
 * control-mode field in cmd_motor_t. */
typedef struct PROTO_PACKED {
    int16_t  pos;     /* PROTO_POS_SCALE */
    int16_t  vel;     /* PROTO_VEL_SCALE */
    uint16_t kp;      /* PROTO_KP_SCALE */
    uint16_t kd;      /* PROTO_KD_SCALE */
    int16_t  tau_ff;  /* PROTO_TAU_SCALE */
} rs_mit_t;           /* 10 B */

typedef struct PROTO_PACKED {
    uint8_t  req;  /* MotorReq */
    rs_mit_t mit;  /* applied every cycle while MOTOR_ENABLED */
} cmd_motor_t;     /* 11 B */

/* Always all MAX_MOTORS_PER_SLAVE slots; the slave uses the ones it is
 * configured for. An unused slot is zero (= MOTOR_REQ_STOP). */
typedef struct PROTO_PACKED {
    uint8_t     req;  /* SlaveReq */
    cmd_motor_t motors[MAX_MOTORS_PER_SLAVE];
} cmd_slave_t;        /* 56 B */

/* Sent truncated to the CONFIGURED slave count: the payload must be exactly
 * CMD_MASTER_SIZE(NUM_SLAVES). The master rejects (and counts in
 * usb_stats_t.rx_errors) any other length — a cheap host/firmware config check. */
typedef struct PROTO_PACKED {
    uint16_t    cycle_id;  /* echo of the last tele_master_t.cycle_id seen */
    uint16_t    cmd_seq;   /* one per host tick, >= 1 */
    uint8_t     test;      /* test-build sentinels; 0 in normal operation */
    cmd_slave_t slaves[MAX_SLAVES];
} cmd_master_t;            /* 5 B + 56 B per slave */

/* ═════════════════════ telemetry: slave -> master -> host ═════════════════ */

/* ── motor ── */

/* Mirror of what the motor itself last reported. */
typedef struct PROTO_PACKED {
    int16_t  pos;            /* PROTO_POS_SCALE */
    int16_t  vel;            /* PROTO_VEL_SCALE */
    int16_t  tau;            /* PROTO_TAU_SCALE */
    int16_t  temp;           /* PROTO_TEMP_SCALE */
    uint8_t  mode;           /* RsMode */
    uint8_t  fault_bits;     /* RS_FAULT_* */
    uint32_t fault_word;     /* full fault register (0x3022), read after a fault;
                                0 = clear, 0xFFFFFFFF = read failed */
    uint32_t time_slave_us;  /* slave clock when this feedback frame arrived */
} rs_feedback_t;             /* 18 B */

typedef struct PROTO_PACKED {
    uint8_t       state;             /* MotorState */
    uint8_t       fault;             /* MotorFault */
    uint8_t       flags;             /* MOTOR_FLAG_* */
    uint16_t      last_applied_seq;  /* host cmd_seq last confirmed applied */
    rs_feedback_t rs;
} tele_motor_t;                      /* 23 B */

/* ── slave ── */

/* Result of the boot id sweep + per-motor verify (bit i = motor slot i). */
typedef struct PROTO_PACKED {
    uint8_t found_mask;          /* configured motors that answered the sweep */
    uint8_t unexpected_id;       /* first CAN id that answered but is not configured; 0 = none */
    uint8_t zero_sta_fail_mask;  /* zero_sta read failed or != 1 */
    uint8_t version_fail_mask;   /* firmware version != the expected bytes for the model */
} boot_result_t;                 /* 4 B */

/* The slave's side of the SPI link. All counters wrap; the host takes deltas. */
typedef struct PROTO_PACKED {
    uint16_t cmd_crc_errors;  /* command frames rejected on CRC */
    uint16_t resyncs;         /* DMA realign events */
    uint16_t tx_arm_fails;    /* telemetry TX-arm failures (then retried) */
    uint8_t  seq_echo;        /* spi_header_t.spi_seq of the last command frame */
} spi_slave_stats_t;          /* 7 B */

typedef struct PROTO_PACKED {
    uint16_t tx_errors;  /* frames the CAN controller refused or dropped */
    uint8_t  tec;        /* controller transmit error counter */
    uint8_t  rec;        /* controller receive error counter */
} can_stats_t;           /* 4 B */

typedef struct PROTO_PACKED {
    boot_result_t     boot;
    spi_slave_stats_t spi;
    can_stats_t       can;
} status_slave_t;        /* 15 B */

/* Everything here is authored by the slave; the master forwards it untouched.
 * The master's own view of this slave is status_master_t.spi[slave]. */
typedef struct PROTO_PACKED {
    uint32_t       time_slave_us;  /* slave clock at the SPI exchange that delivered
                                      the command this telemetry answers */
    uint8_t        state;          /* SlaveState */
    uint8_t        n_motors;
    status_slave_t status;
    tele_motor_t   motors[MAX_MOTORS_PER_SLAVE];
} tele_slave_t;                    /* 136 B */

/* ── master ── */

/* Host command loop, judged against the master cycle. Counters wrap. */
typedef struct PROTO_PACKED {
    uint8_t  lost;             /* 1 = host watchdog tripped; the master is forcing STOP */
    uint16_t last_cmd_seq_rx;  /* last cmd_seq received */
    uint16_t on_time;          /* cycles that applied a fresh, current command */
    uint16_t late;             /* cycles that applied a fresh command for an older cycle */
    uint16_t missing;          /* cycles with no fresh command (held the last one) */
    uint16_t duplicate;        /* commands overwritten before they were applied */
} host_stats_t;                /* 11 B */

typedef struct PROTO_PACKED {
    uint16_t rx_frames;           /* good frames received from the host */
    uint16_t rx_errors;           /* bad CRC or unknown type */
    uint16_t rx_resyncs;          /* times the receiver had to hunt for a frame start */
    uint16_t rx_discarded_bytes;  /* bytes thrown away while resyncing */
    uint16_t tx_drops;            /* telemetry frames dropped because the TX ring was full */
} usb_stats_t;                    /* 10 B */

/* The master's side of the SPI link to one slave. */
typedef struct PROTO_PACKED {
    uint8_t  alive;       /* 1 = CRC-valid telemetry from this slave this cycle */
    uint16_t crc_errors;  /* telemetry frames that failed CRC */
    uint16_t seq_gaps;    /* echoed spi_seq did not advance */
} spi_master_stats_t;     /* 5 B */

typedef struct PROTO_PACKED {
    host_stats_t       host;
    usb_stats_t        usb;
    spi_master_stats_t spi[MAX_SLAVES];  /* index = slave */
    uint16_t           cycle_overruns;   /* master cycles that ran past their period */
} status_master_t;                       /* 43 B */

/* Raw BMI088 sample (on the master). Fusion happens on the host. */
typedef struct PROTO_PACKED {
    int16_t  accel[3];  /* X, Y, Z raw counts; * IMU_ACCEL_LSB_MPS2 */
    int16_t  gyro[3];   /* X, Y, Z raw counts; * IMU_GYRO_LSB_RPS */
    uint8_t  seq;       /* +1 per new sample; unchanged = repeat of the last one */
    uint8_t  flags;     /* IMU_FLAG_* */
    uint16_t errors;    /* failed sensor reads (wraps) */
} imu_t;                /* 16 B */

/* Sent truncated: only slaves[0..n_slaves-1] go on the wire (TELE_MASTER_SIZE).
 * n_slaves / n_motors are the CONFIGURED counts, kept in telemetry so a log
 * decodes without the config. A dead slave's slot is zeroed. */
typedef struct PROTO_PACKED {
    uint16_t        cycle_id;        /* +1 per master cycle */
    uint32_t        time_master_us;  /* master clock at the start of this cycle */
    uint16_t        cmd_seq_active;  /* cmd_seq sent to the slaves this cycle; 0 = none */
    uint8_t         robot_state;     /* RobotState */
    uint8_t         n_slaves;
    status_master_t status;
    imu_t           imu;
    tele_slave_t    slaves[MAX_SLAVES];
} tele_master_t;                     /* 69 B + 136 B per slave */

/* ═══════════════════════════════ USB framing ══════════════════════════════ */

/* Every USB frame = usb_header_t + payload. */
typedef struct PROTO_PACKED {
    uint16_t type;   /* UsbMsgType */
    uint16_t seq;    /* per-sender counter; a PING reply echoes it */
    uint16_t len;    /* payload bytes */
    uint16_t crc16;  /* CRC16-CCITT over header (crc16 = 0) + payload */
} usb_header_t;      /* 8 B */

#define PROTO_CRC_INIT  0xFFFFu
#define PROTO_CRC_POLY  0x1021u

#define CMD_MASTER_SIZE(n_slaves) \
    (offsetof(cmd_master_t, slaves) + (size_t)(n_slaves) * sizeof(cmd_slave_t))
#define TELE_MASTER_SIZE(n_slaves) \
    (offsetof(tele_master_t, slaves) + (size_t)(n_slaves) * sizeof(tele_slave_t))

#define USB_MAX_PAYLOAD     sizeof(tele_master_t)                           /* 613 B; a header claiming more is junk */
#define USB_MAX_FRAME_SIZE  (sizeof(usb_header_t) + USB_MAX_PAYLOAD)        /* 621 B */
#define USB_TX_SLOT_MAX     1024u  /* master TX slot; every frame must fit one */

/* ═══════════════════════════════ SPI framing ══════════════════════════════ */
/* One full-duplex exchange per slave per cycle, SPI_XFER_SIZE bytes each way. */

typedef struct PROTO_PACKED {
    uint8_t  opcode;    /* SpiOpcode */
    uint8_t  spi_seq;   /* +1 per exchange; the slave echoes it (link health only) */
    uint16_t cycle_id;  /* master cycle of this exchange */
    uint16_t cmd_seq;   /* host cmd_seq this command came from; 0 on a NOP */
} spi_header_t;         /* 6 B */

typedef struct PROTO_PACKED {  /* master -> slave */
    spi_header_t hdr;
    cmd_slave_t  cmd;
    uint16_t     crc16;
} spi_cmd_frame_t;             /* 64 B */

typedef struct PROTO_PACKED {  /* slave -> master */
    tele_slave_t tele;
    uint16_t     crc16;
} spi_tele_frame_t;            /* 138 B */

#define SPI_XFER_SIZE  sizeof(spi_tele_frame_t)  /* the larger of the two frames */

/* ═══════════════════════════════ size guards ══════════════════════════════ */
PROTO_STATIC_ASSERT(sizeof(rs_mit_t)           == 10u,  "rs_mit_t");
PROTO_STATIC_ASSERT(sizeof(cmd_motor_t)        == 11u,  "cmd_motor_t");
PROTO_STATIC_ASSERT(sizeof(cmd_slave_t)        == 56u,  "cmd_slave_t");
PROTO_STATIC_ASSERT(sizeof(cmd_master_t)       == 229u, "cmd_master_t");
PROTO_STATIC_ASSERT(sizeof(rs_feedback_t)      == 18u,  "rs_feedback_t");
PROTO_STATIC_ASSERT(sizeof(tele_motor_t)       == 23u,  "tele_motor_t");
PROTO_STATIC_ASSERT(sizeof(boot_result_t)      == 4u,   "boot_result_t");
PROTO_STATIC_ASSERT(sizeof(spi_slave_stats_t)  == 7u,   "spi_slave_stats_t");
PROTO_STATIC_ASSERT(sizeof(can_stats_t)        == 4u,   "can_stats_t");
PROTO_STATIC_ASSERT(sizeof(status_slave_t)     == 15u,  "status_slave_t");
PROTO_STATIC_ASSERT(sizeof(tele_slave_t)       == 136u, "tele_slave_t");
PROTO_STATIC_ASSERT(sizeof(host_stats_t)       == 11u,  "host_stats_t");
PROTO_STATIC_ASSERT(sizeof(usb_stats_t)        == 10u,  "usb_stats_t");
PROTO_STATIC_ASSERT(sizeof(spi_master_stats_t) == 5u,   "spi_master_stats_t");
PROTO_STATIC_ASSERT(sizeof(status_master_t)    == 43u,  "status_master_t");
PROTO_STATIC_ASSERT(sizeof(imu_t)              == 16u,  "imu_t");
PROTO_STATIC_ASSERT(sizeof(tele_master_t)      == 613u, "tele_master_t");
PROTO_STATIC_ASSERT(sizeof(usb_header_t)       == 8u,   "usb_header_t");
PROTO_STATIC_ASSERT(sizeof(spi_header_t)       == 6u,   "spi_header_t");
PROTO_STATIC_ASSERT(sizeof(spi_cmd_frame_t)    == 64u,  "spi_cmd_frame_t");
PROTO_STATIC_ASSERT(sizeof(spi_tele_frame_t)   == 138u, "spi_tele_frame_t");
PROTO_STATIC_ASSERT(sizeof(spi_cmd_frame_t) <= sizeof(spi_tele_frame_t), "SPI_XFER_SIZE");
PROTO_STATIC_ASSERT(USB_MAX_FRAME_SIZE <= USB_TX_SLOT_MAX, "USB frame fits one TX slot");
PROTO_STATIC_ASSERT(MAX_MOTORS_PER_SLAVE <= 8u, "motor masks are u8");

#ifdef __cplusplus
}
#endif
#endif /* PROTOCOL_WIRE_H */