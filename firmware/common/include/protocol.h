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
    MSG_ROBOT_CMD     = 0x08u,  /* host→master: one cmd_robot_t per host tick    */
    MSG_ROBOT_TELE    = 0x09u,  /* master→host: one tele_robot_t per tele tick   */
} MsgType;

/* ── robot lifecycle ─────────────────────────────────────────────────────── */
typedef enum {
    ROBOT_INIT     = 0u,
    ROBOT_READY    = 1u,
    ROBOT_DEGRADED = 2u,
} RobotState;

/* ── per-motor lifecycle state (tele_motor_t.state, full u8) ─────────────────
 * The 5 commandable modes (IDLE/HOLD/MIT/DAMPED/TO_ZERO) plus the boot/discover
 * and fault states. See the mode-request state machine (motor_runtime). */
typedef enum {
    LIFE_BOOT        = 0u,
    LIFE_DISCOVERING = 1u,
    LIFE_IDLE        = 2u,
    LIFE_HOLD        = 3u,  /* armed, holding captured position                  */
    LIFE_MIT         = 4u,  /* armed, streaming MIT setpoints                    */
    LIFE_DAMPED      = 5u,  /* armed, Kp=0 + damping Kd (backdrive-safe)         */
    LIFE_TO_ZERO     = 6u,  /* armed, creeping to home-frame 0 then holding      */
    LIFE_FAULT       = 7u,  /* latched fault — see MotorFaultCause               */
} MotorLifecycle;

/* ── host→slave per-motor mode request (cmd_motor_t.mode_req) ──────────────── */
typedef enum {
    REQ_IDLE    = 0u,
    REQ_HOLD    = 1u,  /* the only request that arms a motor from IDLE          */
    REQ_MIT     = 2u,
    REQ_DAMPED  = 3u,
    REQ_TO_ZERO = 4u,
} MotorModeReq;

/* Why a motor is/last-was faulted. LATCHED on the trip; cleared only by an IDLE
 * request (then re-arm) or the fault_reset flag. */
typedef enum {
    CAUSE_NONE        = 0u,
    CAUSE_OVERTORQUE  = 1u,  /* |tau| exceeded per-motor max_tau                 */
    CAUSE_CAN_TIMEOUT = 2u,  /* this motor's Type-2 feedback went stale          */
    CAUSE_WATCHDOG    = 3u,  /* master-link (SPI command) watchdog expired       */
    CAUSE_MOTOR_FAULT = 4u,  /* RS motor's own Type-2 fault bits fired           */
    CAUSE_ZERO_TIMEOUT= 5u,  /* TO_ZERO made no progress toward home (stall)     */
    CAUSE_NOT_ENABLED = 6u,  /* armed motor reported not-running for K frames     */
    CAUSE_WOUND       = 7u,  /* HOLD refused: shaft wound beyond the safe single-
                                turn range (|offset| too large) — re-zero offline */
} MotorFaultCause;

/* ── cmd_motor_t.flags ───────────────────────────────────────────────────── */
#define CMD_FLAG_VALID            (1u << 0u)  /* this slot carries a live command  */
#define CMD_FLAG_USE_CONFIG_GAINS (1u << 1u)  /* ignore wire kp/kd, use config defaults */
#define CMD_FLAG_FAULT_RESET      (1u << 2u)  /* clear a latched fault this cycle   */

/* ── tele_motor_t.flags ──────────────────────────────────────────────────── */
#define TELE_FLAG_REQUEST_REJECTED (1u << 0u)  /* last mode request was illegal   */
#define TELE_FLAG_TO_ZERO_ARRIVED  (1u << 1u)  /* TO_ZERO has settled at home     */
#define TELE_FLAG_SATURATED        (1u << 2u)  /* a fixed-point field saturated    */
#define TELE_FLAG_CLAMPED_POS      (1u << 3u)  /* commanded pos hit a soft limit   */
#define TELE_FLAG_CLAMPED_TAU      (1u << 4u)  /* vel feedforward cancelled at limit */
#define TELE_FLAG_CMD_STALE        (1u << 5u)  /* executing a held/watchdog setpoint */

/* ── wire header: 16 bytes ───────────────────────────────────────────────── */
#ifdef __GNUC__
#  define PROTO_PACKED __attribute__((packed))
#else
#  define PROTO_PACKED
#endif

/* Wire-contract version. Carried in MsgHeader.ver_flags low byte; both ends drop
 * and count any frame whose version != this. High byte reserved (0). */
#define PROTO_VERSION 3u

/* cmd_seq sentinel: 0 = "no host command applied yet" (tele_motor_t.last_applied_seq
 * for pre-arm / idle). The host starts cmd_seq at 1 and skips 0 on wrap. */
#define CMD_SEQ_NONE 0u

/* ── hierarchy caps (fixed WIRE maximums; actual counts carried in n_ fields) ──
 * Distinct from the generated runtime counts (N_MOTORS / NUM_SLAVES): these size
 * the fixed-length wire arrays, so a mismatched build still has a stable layout. */
#define MAX_MOTORS_PER_CHAIN 5u
#define MAX_CHAINS           4u

/* ── fixed-point scales (shared; host mirrors these exactly) ────────────────
 * Universal scales covering every RobStride model (incl. RS03/04/06: Kp≤5000,
 * Kd≤100). Position is HOME-FRAME ±π only (±31416 at ×10000 — multi-turn is not
 * on the wire). Encode saturates and sets TELE_FLAG_SATURATED / the cmd path's
 * own saturation handling. */
#define PROTO_POS_SCALE 10000.0f  /* rad   → i16  (home-frame ±π)               */
#define PROTO_VEL_SCALE 100.0f    /* rad/s → i16                                */
#define PROTO_TAU_SCALE 100.0f    /* N·m   → i16                                */
#define PROTO_KP_SCALE  10.0f     /* Kp    → u16  (0..6553.5, covers 0..5000)   */
#define PROTO_KD_SCALE  100.0f    /* Kd    → u16  (0..655.35, covers 0..100)    */

typedef struct PROTO_PACKED {
    uint16_t type;    /* MsgType                              */
    uint16_t seq;
    uint8_t  src;     /* NodeId                               */
    uint8_t  dst;
    uint32_t ts_ms;   /* HAL_GetTick() at send time           */
    uint16_t len;     /* payload bytes                        */
    uint16_t ver_flags; /* low byte = PROTO_VERSION; high byte reserved 0 */
    uint16_t crc16;   /* CRC16-CCITT over hdr(crc=0)+payload  */
} MsgHeader;

#define MSG_HEADER_SIZE 16u

/* ── command hierarchy (host → master → slave) ───────────────────────────── */
typedef struct PROTO_PACKED {
    uint8_t  mode_req;   /* MotorModeReq                                        */
    int16_t  pos;        /* rad   × PROTO_POS_SCALE (home-frame ±π)             */
    int16_t  vel;        /* rad/s × PROTO_VEL_SCALE                             */
    uint16_t kp;         /*        × PROTO_KP_SCALE                             */
    uint16_t kd;         /*        × PROTO_KD_SCALE                             */
    int16_t  tau_ff;     /* N·m   × PROTO_TAU_SCALE                             */
    uint8_t  flags;      /* CMD_FLAG_*                                          */
} cmd_motor_t;           /* 12 bytes */

typedef struct PROTO_PACKED {
    uint8_t     chain_id;
    uint8_t     n_motors;                      /* valid slots (≤ MAX_MOTORS_PER_CHAIN) */
    cmd_motor_t motors[MAX_MOTORS_PER_CHAIN];
} cmd_chain_t;                                 /* 2 + 12·5 = 62 bytes */

typedef struct PROTO_PACKED {
    uint16_t    cycle_id;                      /* echo of last master cycle seen       */
    uint16_t    cmd_seq;                       /* one per host tick (≥1, 0 reserved)   */
    uint8_t     n_chains;                      /* valid chains (≤ MAX_CHAINS)          */
    uint8_t     reserved;
    cmd_chain_t chains[MAX_CHAINS];
} cmd_robot_t;                                 /* 6 + 62·4 = 254 bytes */

/* ── telemetry hierarchy (slave → master → host) ─────────────────────────── */
typedef struct PROTO_PACKED {
    int16_t  pos;             /* rad   × PROTO_POS_SCALE (home-frame)            */
    int16_t  vel;             /* rad/s × PROTO_VEL_SCALE                         */
    int16_t  tau;             /* N·m   × PROTO_TAU_SCALE (measured)             */
    uint8_t  temp_c;          /* integer °C                                     */
    uint8_t  state;           /* MotorLifecycle                                 */
    uint8_t  cause;           /* MotorFaultCause (latched)                      */
    uint8_t  motor_mode;      /* RS run_mode reported in Type-2 (0/1/2)         */
    uint8_t  motor_fault;     /* packed Type-2 fault bits                       */
    uint8_t  flags;           /* TELE_FLAG_*                                    */
    uint8_t  fb_age_ms;       /* ms since last Type-2, saturating 255           */
    uint32_t fault_word;      /* 0x3022 latched; 0 clear; 0xFFFFFFFF read-fail  */
    uint16_t last_applied_seq;/* host cmd_seq last confirmed applied (0 = none) */
    uint16_t reserved;        /* growth slot (0)                                */
} tele_motor_t;               /* 21 bytes */

typedef struct PROTO_PACKED {
    uint8_t      chain_id;
    uint8_t      n_motors;
    uint8_t      spi_seq_echo;    /* SPI link-health seq echoed (NOT cmd_seq)   */
    uint8_t      reserved;
    uint32_t     slave_time_us;   /* slave uptime (µs)                          */
    uint16_t     cmd_crc_errors;  /* SPI command frames rejected on CRC (slave) */
    uint16_t     can_tx_errors;   /* CAN TX errors seen by the slave            */
    tele_motor_t motors[MAX_MOTORS_PER_CHAIN];
} tele_chain_t;                   /* 12 + 21·5 = 117 bytes */

typedef struct PROTO_PACKED {
    uint16_t     cycle_id;         /* master poll counter at build time         */
    uint32_t     master_time_us;   /* master uptime (µs)                        */
    uint16_t     last_cmd_seq_rx;  /* last cmd_seq the master received from host */
    uint16_t     missed_deadlines; /* master-side missed poll/tele deadlines    */
    uint8_t      n_chains;
    uint8_t      robot_state;      /* RobotState                                */
    tele_chain_t chains[MAX_CHAINS];
} tele_robot_t;                    /* 12 + 117·4 = 480 bytes */

/* ── status (unchanged cadence, 20 Hz) ───────────────────────────────────── */
typedef struct PROTO_PACKED {
    uint8_t  robot_state;   /* RobotState                   */
    uint8_t  slave_alive;   /* bit s = slave s reachable    */
    uint32_t uptime_ms;
    uint32_t link_errors;   /* bad CRC or unknown msg_type  */
    uint32_t rx_frames;
    uint16_t master_poll_hz;    /* configured SPI poll rate      */
    uint16_t telemetry_hz;      /* configured MSG_ROBOT_TELE rate */
    uint16_t slave_tick_hz;     /* configured slave control-tick rate */
    uint16_t host_cmd_hz;       /* configured expected host cmd rate   */
} MasterStatus;                 /* 22 bytes */

typedef struct PROTO_PACKED {
    uint8_t  slave_id;        /* which slave this status is for              */
    uint8_t  motors_alive;    /* bit i = motor i alive                       */
    uint32_t uptime_ms;
    uint32_t crc_errors;      /* SPI telemetry frames failing CRC (master)   */
    uint32_t cmd_crc_errors;  /* SPI command frames the slave rejected on CRC */
    uint32_t seq_gaps;        /* master: echoed spi-seq gaps (dropped frames) */
} SlaveStatus;                /* 18 bytes */

/* ── SPI frame layout (one full-duplex transfer) ─────────────────────────────
 * One chain per slave, so the SPI payload is a single cmd_chain_t / tele_chain_t
 * (fixed size, no per-N arithmetic).
 *   master → slave: [opcode u8][spi_seq u8][cycle_id u16][cmd_seq u16]
 *                   [cmd_chain_t][crc16]                              = 70 B
 *   slave → master: [tele_chain_t][crc16]                            = 119 B
 * Transfer length = the larger of the two. */
#define SPI_OP_NOP        0x00u
#define SPI_OP_ROBOT_CMD  0x01u

#define SPI_CMD_HDR_BYTES  6u    /* opcode + spi_seq + cycle_id(u16) + cmd_seq(u16) */
#define SPI_CRC_BYTES      2u
#define SPI_CMD_CRC_OFF    ((uint16_t)(SPI_CMD_HDR_BYTES + sizeof(cmd_chain_t)))
#define SPI_CMD_FRAME_SIZE ((uint16_t)(SPI_CMD_CRC_OFF + SPI_CRC_BYTES))
#define SPI_TELE_CRC_OFF   ((uint16_t)(sizeof(tele_chain_t)))
#define SPI_TELE_FRAME_SIZE ((uint16_t)(SPI_TELE_CRC_OFF + SPI_CRC_BYTES))
#define SPI_XFER_SIZE (SPI_CMD_FRAME_SIZE > SPI_TELE_FRAME_SIZE \
                       ? SPI_CMD_FRAME_SIZE : SPI_TELE_FRAME_SIZE)

/* ── fixed-point encode/decode (shared; host mirrors the arithmetic) ────────
 * Round-half-away-from-zero, no math.h. Encoders saturate at the type limits and
 * set *sat (if non-NULL) on saturation. */
static inline int16_t proto_f_to_i16(float x, float scale, uint8_t *sat)
{
    float v = x * scale;
    v = (v >= 0.0f) ? (v + 0.5f) : (v - 0.5f);
    if (v >  32767.0f) { if (sat) *sat = 1u; return (int16_t)32767; }
    if (v < -32768.0f) { if (sat) *sat = 1u; return (int16_t)(-32768); }
    return (int16_t)v;
}
static inline uint16_t proto_f_to_u16(float x, float scale, uint8_t *sat)
{
    float v = x * scale + 0.5f;
    if (v > 65535.0f) { if (sat) *sat = 1u; return (uint16_t)65535; }
    if (v < 0.0f)     { return 0u; }   /* gains are non-negative */
    return (uint16_t)v;
}
static inline float proto_i16_to_f(int16_t r, float scale) { return (float)r / scale; }
static inline float proto_u16_to_f(uint16_t r, float scale) { return (float)r / scale; }

/* ── layout guards (catch C↔wire drift at compile time) ──────────────────── */
#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(MsgHeader)   == MSG_HEADER_SIZE,          "MsgHeader must be 16 bytes");
_Static_assert(sizeof(cmd_motor_t) == 12u,                      "cmd_motor_t must be 12 bytes");
_Static_assert(sizeof(cmd_chain_t) == 74u,                      "cmd_chain_t must be 74 bytes");
_Static_assert(sizeof(cmd_robot_t) == 6u + 74u * MAX_CHAINS,    "cmd_robot_t size");
_Static_assert(sizeof(tele_motor_t) == 21u,                     "tele_motor_t must be 21 bytes");
_Static_assert(sizeof(tele_chain_t) == 138u,                    "tele_chain_t must be 138 bytes");
_Static_assert(sizeof(tele_robot_t) == 12u + 138u * MAX_CHAINS, "tele_robot_t size");
_Static_assert(sizeof(MasterStatus) == 22u,                     "MasterStatus must be 22 bytes");
_Static_assert(sizeof(SlaveStatus)  == 18u,                     "SlaveStatus must be 18 bytes");
_Static_assert(LIFE_FAULT  <= 255u,                             "MotorLifecycle fits u8");
_Static_assert(CAUSE_WOUND <= 255u,                             "MotorFaultCause fits u8");
#endif

/* ── CRC16-CCITT ─────────────────────────────────────────────────────────── */
#define PROTO_CRC_INIT 0xFFFFu
#define PROTO_CRC_POLY 0x1021u

/* Byte-wise lookup table for CRC-16-CCITT (poly 0x1021, non-reflected, MSB-first).
   Table[b] = the bitwise CRC of the single byte b with a zero register, so the
   byte-wise update below is mathematically identical to the old bit-by-bit loop
   (and to the host's binascii.crc_hqx(..., 0xFFFF)) — the wire format is unchanged.
   `const` ⇒ the 512 B table lives in flash (.rodata), not RAM. Header-only: one
   copy per translation unit that CRCs (~512 B each of 512 KB flash). */
static const uint16_t PROTO_CRC16_TABLE[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0,
};

static inline uint16_t proto_crc16_update(uint16_t crc,
                                           const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; ++i) {
        crc = (uint16_t)((crc << 8u)
            ^ PROTO_CRC16_TABLE[((crc >> 8u) ^ data[i]) & 0xFFu]);
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
    hdr.type      = type;
    hdr.seq       = seq;
    hdr.src       = src;
    hdr.dst       = dst;
    hdr.ts_ms     = ts_ms;
    hdr.len       = pay_len;
    hdr.ver_flags = PROTO_VERSION;   /* low byte = version, high byte reserved 0 */

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
