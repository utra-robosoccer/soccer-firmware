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
} MotorLifecycle;

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
    uint8_t  motors_alive;
    uint8_t  motor_state;   /* MotorLifecycle for motor 0   */
    uint32_t uptime_ms;
} SlaveStatus;

typedef struct PROTO_PACKED {
    uint8_t  motor_idx;
    uint8_t  state;         /* MotorLifecycle               */
    float    pos;
    float    vel;
    float    tau;
    float    temp;
    uint16_t fault_flags;
    uint16_t last_cmd_seq;
} MotorStatePayload;

typedef struct PROTO_PACKED {
    uint8_t  motor_idx;
    uint8_t  cmd;           /* ControlCmd                   */
    uint16_t reserved;
} ControlReq;

typedef struct PROTO_PACKED {
    uint8_t  motor_idx;
    uint8_t  cmd;
    uint8_t  result;        /* ControlResult                */
    uint8_t  new_state;     /* MotorLifecycle after command  */
    uint16_t req_seq;
} ControlResp;

/* MSG_MOTOR_CMD – stub */
typedef struct PROTO_PACKED {
    uint8_t  motor_idx;
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

/* ── SPI wire format (slave → master telemetry, 13 bytes per motor) ─────── */
typedef struct PROTO_PACKED {
    uint8_t  motor_idx;
    uint8_t  state;         /* MotorLifecycle               */
    uint16_t fault_flags;
    uint16_t pos_raw;       /* P_MIN..P_MAX → 0..65535      */
    uint16_t vel_raw;       /* V_MIN..V_MAX → 0..65535      */
    uint16_t tau_raw;       /* T_MIN..T_MAX → 0..65535      */
    uint16_t last_cmd_seq;
    uint8_t  temp_c;        /* integer degrees Celsius      */
} SpiMotorTele;             /* 13 bytes */

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
