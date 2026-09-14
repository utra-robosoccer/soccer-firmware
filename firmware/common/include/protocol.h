#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── node identifiers ────────────────────────────────────────────────────── */
typedef enum {
    NODE_JETSON  = 1u,
    NODE_MASTER  = 2u,
    NODE_SLAVE_0 = 3u,
    NODE_SLAVE_1 = 4u,
} NodeId;

/* ── message types ───────────────────────────────────────────────────────── */
typedef enum {
    MSG_PING          = 0x01u,
    MSG_MASTER_STATUS = 0x02u,
    MSG_SLAVE_STATUS  = 0x03u,
    MSG_MOTOR_STATE   = 0x04u,
    MSG_CONTROL_REQ   = 0x05u,
    MSG_CONTROL_RESP  = 0x06u,
    MSG_MOTOR_CMD     = 0x07u,  /* stub – not wired */
} MsgType;

/* ── robot / motor lifecycle ─────────────────────────────────────────────── */
typedef enum {
    ROBOT_INIT     = 0u,
    ROBOT_READY    = 1u,
    ROBOT_DEGRADED = 2u,
} RobotState;

typedef enum {
    MOTOR_BOOT        = 0u,
    MOTOR_DISCOVERING = 1u,
    MOTOR_IDLE        = 2u,
    MOTOR_ARMED_HOLD  = 3u,
    MOTOR_FAULT       = 4u,
    MOTOR_DISABLED    = 5u,
    MOTOR_ZEROING     = 6u,  /* creeping toward pos=0, transitions to ARMED_HOLD on arrival */
    MOTOR_ARMED_MIT   = 7u,  /* receiving live MIT commands from host */
} MotorLifecycle;            /* fits the low nibble of MotorState.state (0..15) */

/* Why a motor last left an armed state. LATCHED on the trip, cleared only on
 * re-arm/disable — carries intermittent-fault evidence across the transient.
 * Must fit the high nibble of MotorState.state (0..15). */
typedef enum {
    CAUSE_NONE        = 0u,
    CAUSE_OVERTORQUE  = 1u,  /* |tau| exceeded per-motor max_tau               */
    CAUSE_CAN_TIMEOUT = 2u,  /* this motor's Type-2 feedback went stale         */
    CAUSE_WATCHDOG    = 3u,  /* master-link (SPI command) watchdog expired      */
    CAUSE_MOTOR_FAULT = 4u,  /* RS motor's own Type-2 fault bits fired          */
} MotorFaultCause;

/* MotorState.state packs lifecycle (low nibble) + fault cause (high nibble). */
#define SPI_STATE_PACK(life, cause) ((uint8_t)(((life) & 0x0Fu) | ((uint8_t)(cause) << 4u)))
#define SPI_STATE_LIFE(s)           ((uint8_t)((s) & 0x0Fu))
#define SPI_STATE_CAUSE(s)          ((uint8_t)((s) >> 4u))

/* MotorState.cmd_flags — recomputed every tick (NOT latched). */
#define SPI_CMDFLAG_CLAMPED_POS (1u << 0u)  /* soft-limit clamped commanded pos */
#define SPI_CMDFLAG_CLAMPED_TAU (1u << 1u)  /* torque command clamped           */
#define SPI_CMDFLAG_CMD_STALE   (1u << 2u)  /* executing a held/watchdog cmd     */

/* ── control commands ────────────────────────────────────────────────────── */
typedef enum {
    CTRL_ARM_HOLD  = 0x01u,
    CTRL_DISABLE   = 0x02u,
    CTRL_SET_ZERO  = 0x03u,  /* stub – not implemented */
    CTRL_GOTO_ZERO = 0x04u,  /* enable, creep to pos=0, then hold */
} ControlCmd;

typedef enum {
    CTRL_OK        = 0u,
    CTRL_ERR_STATE = 1u,
    CTRL_ERR_STUB  = 2u,
    CTRL_ERR_MOTOR = 3u,
} ControlResult;

/* ── wire header: 16 bytes ───────────────────────────────────────────────── */
#ifdef __GNUC__
#  define PROTO_PACKED __attribute__((packed))
#else
#  define PROTO_PACKED
#endif

typedef struct PROTO_PACKED {
    uint16_t type;    /* MsgType                              */
    uint16_t seq;
    uint8_t  src;     /* NodeId                               */
    uint8_t  dst;
    uint32_t ts_ms;   /* HAL_GetTick() at send time           */
    uint16_t len;     /* payload bytes                        */
    uint16_t flags;   /* reserved, must be 0                  */
    uint16_t crc16;   /* CRC16-CCITT over hdr(crc=0)+payload  */
} MsgHeader;

#define MSG_HEADER_SIZE 16u

/* ── MotorState telemetry atom (16 B) ────────────────────────────────────────
 * The unit of slave→master telemetry, forwarded verbatim to the host. pos/vel/
 * tau are encoded over the GLOBAL transport bounds (widest model) — decode with
 * the generated MOTOR_P/V/T_MIN/MAX, not per-model tables. */
typedef struct PROTO_PACKED {
    uint16_t pos_raw;       /* HOME-FRAME wrapped [-pi,pi] over the +-4pi bound —
                               NOT the motor's multi-turn angle. MOTOR_P_MIN/MAX. */
    uint16_t vel_raw;       /* MOTOR_V_MIN..MOTOR_V_MAX → 0..65535               */
    uint16_t tau_raw;       /* MOTOR_T_MIN..MOTOR_T_MAX → 0..65535 (measured)    */
    uint8_t  temp_c;        /* integer degrees Celsius                           */
    uint8_t  state;         /* [3:0] MotorLifecycle | [7:4] MotorFaultCause      */
    uint8_t  motor_fault;   /* 6 compact Type-2 fault bits                       */
    uint8_t  cmd_flags;     /* SPI_CMDFLAG_* (recomputed each tick)              */
    uint32_t fault_word;    /* 0x3022 latched on fault entry; 0 clear; 0xFFFFFFFF read-fail */
    uint8_t  fb_age;        /* ms since this motor's last Type-2, saturating 255 */
    uint8_t  _rsvd;         /* 0                                                 */
} MotorState;               /* 16 bytes */

/* ── payload structs ─────────────────────────────────────────────────────── */

typedef struct PROTO_PACKED {
    uint8_t  robot_state;   /* RobotState                   */
    uint8_t  slave_alive;   /* bit 0 = slave 0 reachable    */
    uint8_t  motors_alive;  /* bit i = motor i alive        */
    uint32_t uptime_ms;
    uint32_t link_errors;   /* bad CRC or unknown msg_type  */
    uint32_t rx_frames;
} MasterStatus;

typedef struct PROTO_PACKED {
    uint8_t  slave_id;      /* which slave this status is for         */
    uint8_t  motors_alive;
    uint8_t  motor_state;   /* MotorLifecycle (low nibble) for motor 0 */
    uint32_t uptime_ms;
    uint32_t crc_errors;    /* SPI telemetry frames failing CRC       */
} SlaveStatus;

/* Master → Jetson per-motor telemetry: the raw MotorState atom is forwarded
 * verbatim (pass-through). The host decodes raw→units with the generated global
 * transport bounds. See MotorState below for field semantics. */
typedef struct PROTO_PACKED {
    uint8_t    slave_id;    /* which slave the motor is on            */
    uint8_t    motor_idx;   /* local index within that slave          */
    MotorState motor;       /* 16 B atom, exactly as received over SPI */
} MotorStatePayload;

typedef struct PROTO_PACKED {
    uint8_t  slave_id;      /* target slave                           */
    uint8_t  motor_idx;     /* local index within that slave          */
    uint8_t  cmd;           /* ControlCmd                             */
    uint8_t  reserved;
} ControlReq;

typedef struct PROTO_PACKED {
    uint8_t  slave_id;
    uint8_t  motor_idx;
    uint8_t  cmd;
    uint8_t  result;        /* ControlResult                */
    uint8_t  new_state;     /* MotorLifecycle after command  */
    uint16_t req_seq;
} ControlResp;

/* MSG_MOTOR_CMD – live MIT setpoint from host */
typedef struct PROTO_PACKED {
    uint8_t  slave_id;      /* target slave                           */
    uint8_t  motor_idx;     /* local index within that slave          */
    float    pos;
    float    vel;
    float    kp;
    float    kd;
    float    tau_ff;
} MotorCmd;

/* ── SPI wire format (master → slave MIT command, 9 bytes per motor) ────── */
typedef struct PROTO_PACKED {
    float   pos;    /* target position (rad)                            */
    float   vel;    /* feedforward velocity (rad/s)                     */
    uint8_t valid;  /* non-zero = this slot carries a live command      */
} SpiMitCmd;        /* 9 bytes */

/* ── SPI frame layout (one full-duplex transfer, length = tele frame) ──────
 *
 * slave → master telemetry frame (fixed):
 *   [alive_mask u8][echo_seq u8][MotorState × N][health_rsvd[8]=0][crc16 u16]
 *   crc16 = proto_crc16() over every preceding byte. health_rsvd is reserved
 *   for future per-slave health and is covered by the CRC.
 *
 * master → slave command frame rides in the prefix of the same transfer:
 *   [cmd u8][seq u8][SpiMitCmd × N ...]
 * The transfer length is the (larger) telemetry frame size. */
#define SPI_TELE_HDR_BYTES     2u                 /* alive_mask + echo_seq        */
#define SPI_TELE_HEALTH_BYTES  8u                 /* reserved per-slave health    */
#define SPI_TELE_CRC_BYTES     2u                 /* crc16 trailer                */
#define SPI_TELE_FRAME_SIZE(n) ((uint16_t)(SPI_TELE_HDR_BYTES + \
                                (uint16_t)(n) * (uint16_t)sizeof(MotorState) + \
                                SPI_TELE_HEALTH_BYTES + SPI_TELE_CRC_BYTES))

#define SPI_CMD_HDR_BYTES      2u                 /* cmd + seq                    */
#define SPI_CMD_FRAME_SIZE(n)  ((uint16_t)(SPI_CMD_HDR_BYTES + \
                                (uint16_t)(n) * (uint16_t)sizeof(SpiMitCmd)))

/* ── layout guards (catch C↔wire drift at compile time) ──────────────────── */
#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(MsgHeader) == MSG_HEADER_SIZE, "MsgHeader must be 16 bytes");
_Static_assert(sizeof(MotorState) == 16u,            "MotorState must be 16 bytes");
_Static_assert(sizeof(SpiMitCmd) == 9u,              "SpiMitCmd must be 9 bytes");
_Static_assert(MOTOR_ARMED_MIT   <= 0x0Fu,           "MotorLifecycle must fit 4 bits");
_Static_assert(CAUSE_MOTOR_FAULT <= 0x0Fu,           "MotorFaultCause must fit 4 bits");
#endif

/* ── CRC16-CCITT ─────────────────────────────────────────────────────────── */
#define PROTO_CRC_INIT 0xFFFFu
#define PROTO_CRC_POLY 0x1021u

static inline uint16_t proto_crc16_update(uint16_t crc,
                                           const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8u);
        for (uint8_t b = 0; b < 8u; ++b) {
            crc = (crc & 0x8000u)
                ? (uint16_t)((crc << 1u) ^ PROTO_CRC_POLY)
                : (uint16_t)(crc << 1u);
        }
    }
    return crc;
}

static inline uint16_t proto_crc16(const uint8_t *data, size_t len)
{
    return proto_crc16_update(PROTO_CRC_INIT, data, len);
}

/* Compute CRC for a complete message (header.crc16 must be 0) */
static inline uint16_t proto_frame_crc(const void *hdr_raw,
                                        const uint8_t *payload,
                                        uint16_t pay_len)
{
    MsgHeader tmp;
    memcpy(&tmp, hdr_raw, MSG_HEADER_SIZE);
    tmp.crc16 = 0u;
    uint16_t crc = proto_crc16((const uint8_t *)&tmp, MSG_HEADER_SIZE);
    if (pay_len > 0u && payload != NULL) {
        crc = proto_crc16_update(crc, payload, pay_len);
    }
    return crc;
}

/* Build a framed message into out[].  Returns total bytes written, 0 = error. */
static inline uint16_t proto_build(uint8_t *out, uint16_t cap,
                                    uint16_t type, uint16_t seq,
                                    uint8_t src,  uint8_t dst,
                                    uint32_t ts_ms,
                                    const uint8_t *payload, uint16_t pay_len)
{
    uint16_t total = (uint16_t)(MSG_HEADER_SIZE + pay_len);
    if (total > cap) return 0u;

    MsgHeader hdr;
    memset(&hdr, 0, sizeof(hdr));
    hdr.type  = type;
    hdr.seq   = seq;
    hdr.src   = src;
    hdr.dst   = dst;
    hdr.ts_ms = ts_ms;
    hdr.len   = pay_len;

    memcpy(out, &hdr, MSG_HEADER_SIZE);
    if (pay_len > 0u && payload != NULL) {
        memcpy(out + MSG_HEADER_SIZE, payload, pay_len);
    }
    ((MsgHeader *)out)->crc16 = proto_crc16(out, total);
    return total;
}

#ifdef __cplusplus
}
#endif
#endif /* PROTOCOL_H */
