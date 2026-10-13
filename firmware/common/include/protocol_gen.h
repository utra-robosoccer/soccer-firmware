/* AUTO-GENERATED — DO NOT EDIT.
 * Source:      firmware/common/protocol.yaml
 * Regenerate:  python3 scripts/gen_protocol.py
 *
 * SCHEMA_HASH 0x2EE4783D: CRC-32 of the schema's canonical wire
 * projection (parsed YAML minus doc/unit/hex keys, compact sorted-key
 * JSON, UTF-8) — see gen_protocol.py. Doc-only edits do not change it.
 */
#ifndef PROTOCOL_GEN_H
#define PROTOCOL_GEN_H

#include <stdint.h>
#include <stddef.h>   /* offsetof for the layout guards */

/* Packed-struct attribute; a wrapper may define it first. */
#ifndef PROTO_PACKED
#  ifdef __GNUC__
#    define PROTO_PACKED __attribute__((packed))
#  else
#    define PROTO_PACKED
#  endif
#endif

/* Wire-contract version. Carried in MsgHeader.ver_flags low byte; both
 * ends drop and count any frame whose version != this. */
#define PROTO_VERSION 9u
#define PROTO_SCHEMA_HASH 0x2EE4783Du

/* ── wire constants ─────────────────────────────────────────────────── */
/* cmd_seq sentinel: "no host command applied yet"
 * (tele_motor_t.last_applied_seq for pre-arm / idle). The host starts
 * cmd_seq at 1 and skips 0 on wrap. */
#define CMD_SEQ_NONE 0u
/* Fixed WIRE maximum (actual counts carried in n_ fields). Distinct from the
 * generated runtime counts (N_MOTORS / NUM_SLAVES): these size the
 * fixed-length wire arrays, so a mismatched build still has a stable layout. */
#define MAX_MOTORS_PER_CHAIN 5u
/* Fixed WIRE maximum number of chains (see MAX_MOTORS_PER_CHAIN). */
#define MAX_CHAINS 4u
/* CRC16-CCITT init value. */
#define PROTO_CRC_INIT 0xFFFFu
/* CRC16-CCITT polynomial (non-reflected, MSB-first). */
#define PROTO_CRC_POLY 0x1021u
/* SPI command opcode — no fresh host command this cycle. */
#define SPI_OP_NOP 0x00u
/* SPI command opcode — frame carries a fresh cmd_chain_t. */
#define SPI_OP_ROBOT_CMD 0x01u
/* SPI cmd frame header: opcode u8 + spi_seq u8 + cycle_id u16 + cmd_seq u16. */
#define SPI_CMD_HDR_BYTES 6u
/* CRC16 trailer on both SPI frame directions. */
#define SPI_CRC_BYTES 2u
/* Master USB TX slot size (usb_tx.c slot ring). Every full USB frame (header
 * + payload) must fit one slot — checked at generation time and by a
 * generated static assert (USB_MAX_FRAME_SIZE <= USB_TX_SLOT_MAX). */
#define USB_TX_SLOT_MAX 512u

/* ── NodeId (u8 on the wire) ── */
/* node identifiers */
typedef enum {
    NODE_JETSON  = 1u,
    NODE_MASTER  = 2u,
    NODE_SLAVE_0 = 3u,
    NODE_SLAVE_1 = 4u,
} NodeId;

/* ── MsgType (u16 on the wire) ── */
/* message types (MsgHeader.type) */
typedef enum {
    MSG_PING          = 0x01u,
    MSG_MASTER_STATUS = 0x02u,
    MSG_SLAVE_STATUS  = 0x03u,
    MSG_ROBOT_CMD     = 0x08u,  /* host→master: one cmd_robot_t per host tick */
    MSG_ROBOT_TELE    = 0x09u,  /* master→host: one tele_robot_t per tele tick */
} MsgType;

/* ── RobotState (u8 on the wire) ── */
/* robot lifecycle (tele_robot_t.robot_state). Reporting priority when
 * several conditions hold: HOST_LOST > PROTO_MISMATCH > the computed
 * INIT/READY/DEGRADED — docs/protocol_v9.md §Host-loss. */
typedef enum {
    ROBOT_INIT           = 0u,
    ROBOT_READY          = 1u,
    ROBOT_DEGRADED       = 2u,
    ROBOT_HOST_LOST      = 3u,  /* master's host-death watchdog tripped (forcing STOP) */
    ROBOT_PROTO_MISMATCH = 4u,  /* a responding slave fails the SPI version gate; NEW enables
       are refused robot-wide until every slave matches (reflash +
       rejoin needs no master reset) */
} RobotState;

/* ── DriveReq (u8 on the wire) ── */
/* Host→slave per-motor drive request (cmd_motor_t.drive_req).
 * Level-triggered: safe to repeat every cycle; a repeat in the target or
 * transit state is a no-op. The MIT tuple travels independently under
 * CMD_FLAG_VALID — there is no MIT request; in ENABLED every valid tuple is
 * simply applied. */
typedef enum {
    DRV_REQ_DISABLE = 0u,  /* Type-4 stop now */
    DRV_REQ_ENABLE  = 1u,  /* DISABLED → ENABLING → ENABLED; rejected elsewhere */
    DRV_REQ_STOP    = 2u,  /* ENABLED → STOPPING (ramp) → DISABLED */
    DRV_REQ_RESET   = 3u,  /* FAULT → CLEARING → DISABLED; never enables */
} DriveReq;

/* ── DriveState (u8 on the wire) ── */
/* per-motor drive state (tele_motor_t.drive_state under v9) */
typedef enum {
    DRV_BOOT        = 0u,
    DRV_DISCOVERING = 1u,
    DRV_ABSENT      = 2u,  /* no feedback; pinged ~100 ms; reply → DISABLED */
    DRV_DISABLED    = 3u,  /* motor off, polled every cycle */
    DRV_ENABLING    = 4u,  /* async handshake; rs_mode==Run confirms → ENABLED */
    DRV_ENABLED     = 5u,  /* limp until the first VALID tuple; then applies it */
    DRV_STOPPING    = 6u,  /* ramped stop; requested → DISABLED, fault → FAULT */
    DRV_CLEARING    = 7u,  /* RESET sent the Type-4 clear; judging the reply */
    DRV_FAULT       = 8u,  /* latched — see drive_fault; only RESET leaves */
} DriveState;

/* ── ChainLink (u8 on the wire) ── */
/* Per-chain SPI link state (tele_chain_t.link), authored by the MASTER when
 * it assembles tele_robot_t — the slave leaves the byte 0. Every configured
 * chain is emitted every cycle at its fixed slot (chains[s] = slave s), so
 * the host sees an unreachable or mismatched slave in the same cycle; a
 * chain that is not ALIVE carries n_motors = 0 and no motor data. */
typedef enum {
    CHAIN_LINK_LOST     = 0u,  /* no CRC-valid SPI telemetry from this slave this cycle */
    CHAIN_LINK_ALIVE    = 1u,
    CHAIN_LINK_MISMATCH = 2u,  /* slave responds but its proto_version/schema_hash differ
       (v9.1 gate) */
} ChainLink;

/* ── DriveFault (u8 on the wire) ── */
/* Why a motor is faulted (tele_motor_t.drive_fault under v9). Latched at
 * fault DETECTION (first fault wins); cleared on CLEARING entry so a
 * re-latch reports the current fault. */
typedef enum {
    DRVF_NONE           = 0u,
    DRVF_CAN_TIMEOUT    = 1u,  /* Type-2 feedback went stale (comms-broken) */
    DRVF_MASTER_LOST    = 2u,  /* SPI command stream died → autonomous stop ramp */
    DRVF_ENABLE_FAILED  = 3u,  /* ENABLING saw no rs_mode==Run in time */
    DRVF_NOT_ENABLED    = 4u,  /* enable monitor: enabled motor reports not-Run */
    DRVF_OVERTORQUE     = 5u,  /* |tau| exceeded per-motor max_tau */
    DRVF_MOTOR_REPORTED = 6u,  /* RS vendor fault bits / Cali mode */
    DRVF_WOUND          = 7u,  /* ENABLE refused: shaft wound beyond safe range */
} DriveFault;

/* ── cmd_motor_t.flags ── */
#define CMD_FLAG_VALID (1u << 0u)  /* the MIT tuple in this slot is live; drive_req applies regardless */
#define CMD_FLAG_USE_CONFIG_GAINS (1u << 1u)  /* ignore wire kp/kd, use config defaults */
/* bit2 reserved — was CMD_FLAG_FAULT_RESET (v8); RESET is a drive request in v9 */

/* ── tele_motor_t.flags ── */
#define TELE_FLAG_REQUEST_REJECTED (1u << 0u)  /* last drive request was illegal */
/* bit1 reserved — was TELE_FLAG_TO_ZERO_ARRIVED (v8); TO_ZERO is host-side in v9 */
#define TELE_FLAG_SATURATED (1u << 2u)  /* a fixed-point field saturated */
#define TELE_FLAG_CLAMPED_POS (1u << 3u)  /* commanded pos hit a soft limit */
#define TELE_FLAG_CLAMPED_TAU (1u << 4u)  /* vel feedforward cancelled at limit */
#define TELE_FLAG_CMD_STALE (1u << 5u)  /* executing a held/watchdog setpoint */
#define TELE_FLAG_CLAMPED_GAIN (1u << 6u)  /* wire kp/kd clamped to the model's range */

/* ── fixed-point scales (host mirrors these exactly) ────────────────── */
#define PROTO_POS_SCALE 10000.0f  /* rad → i16; home-frame ±π */
#define PROTO_VEL_SCALE 100.0f  /* rad/s → i16 */
#define PROTO_TAU_SCALE 100.0f  /* N·m → i16 */
#define PROTO_KP_SCALE 10.0f  /* Kp → u16; 0..6553.5, covers 0..5000 */
#define PROTO_KD_SCALE 100.0f  /* Kd → u16; 0..655.35, covers 0..100 */

/* wire header, 16 bytes. ver_flags low byte = PROTO_VERSION (both ends drop
 * and count any frame whose version differs); high byte reserved (0). */
typedef struct PROTO_PACKED {
    uint16_t type;                      /* MsgType */
    uint16_t seq;
    uint8_t src;                        /* NodeId */
    uint8_t dst;                        /* NodeId */
    uint32_t ts_ms;                     /* HAL_GetTick() at send time */
    uint16_t len;                       /* payload bytes */
    uint16_t ver_flags;                 /* low byte = PROTO_VERSION; high byte reserved 0 */
    uint16_t crc16;                     /* CRC16-CCITT over hdr(crc=0)+payload */
} MsgHeader;   /* 16 bytes */

/* per-motor command (host → master → slave), 12 bytes */
typedef struct PROTO_PACKED {
    uint8_t drive_req;                  /* DriveReq */
    int16_t pos;                        /* PROTO_POS_SCALE; rad × PROTO_POS_SCALE (home-frame ±π) */
    int16_t vel;                        /* PROTO_VEL_SCALE */
    uint16_t kp;                        /* PROTO_KP_SCALE */
    uint16_t kd;                        /* PROTO_KD_SCALE */
    int16_t tau_ff;                     /* PROTO_TAU_SCALE */
    uint8_t flags;                      /* cmd_motor_flags */
} cmd_motor_t;   /* 12 bytes */

/* one CAN chain's commands, 2 + 12·5 = 62 bytes */
typedef struct PROTO_PACKED {
    uint8_t chain_id;
    uint8_t n_motors;                   /* valid slots (≤ MAX_MOTORS_PER_CHAIN) */
    cmd_motor_t motors[MAX_MOTORS_PER_CHAIN];
} cmd_chain_t;   /* 62 bytes */

/* one host tick's full command, 6 + 62·4 = 254 bytes. Always transmitted
 * full-size (NOT truncated by n_chains). reserved carries the master test
 * sentinels (0xDE arm SPI-desync, 0xB0 arm USB-TX test; 0 in normal
 * operation). */
typedef struct PROTO_PACKED {
    uint16_t cycle_id;                  /* echo of last master cycle seen */
    uint16_t cmd_seq;                   /* one per host tick (≥1, 0 reserved) */
    uint8_t n_chains;                   /* valid chains (≤ MAX_CHAINS) */
    uint8_t reserved;                   /* test sentinels; 0 in normal operation */
    cmd_chain_t chains[MAX_CHAINS];
} cmd_robot_t;   /* 254 bytes */

/* per-motor telemetry (slave → master → host), 21 bytes */
typedef struct PROTO_PACKED {
    int16_t pos;                        /* PROTO_POS_SCALE; rad × PROTO_POS_SCALE (home-frame) */
    int16_t vel;                        /* PROTO_VEL_SCALE */
    int16_t tau;                        /* PROTO_TAU_SCALE; measured */
    uint8_t temp_c;                     /* integer °C */
    uint8_t drive_state;                /* DriveState */
    uint8_t drive_fault;                /* DriveFault; latched */
    uint8_t rs_mode;                    /* RS run_mode reported in Type-2 (0 reset / 1 cali / 2 run) */
    uint8_t rs_fault_bits;              /* packed Type-2 fault bits */
    uint8_t flags;                      /* tele_motor_flags */
    uint8_t fb_age_ms;                  /* ms since last Type-2, saturating 255 */
    uint32_t rs_fault_word;             /* 0x3022 latched; 0 clear; 0xFFFFFFFF read-fail */
    uint16_t last_applied_seq;          /* host cmd_seq last confirmed applied (0 = none) */
    uint16_t reserved;                  /* growth slot (0) */
} tele_motor_t;   /* 21 bytes */

/* one CAN chain's telemetry, 13 + 21·5 = 119 bytes */
typedef struct PROTO_PACKED {
    uint8_t chain_id;
    uint8_t n_motors;
    uint8_t link;                       /* ChainLink; master-authored; slave sends 0 */
    uint8_t spi_seq_echo;               /* SPI link-health seq echoed (NOT cmd_seq) */
    uint8_t spi_resyncs;                /* slave SPI DMA resync count (wraps; host deltas) */
    uint32_t slave_time_us;             /* slave uptime (µs) */
    uint16_t cmd_crc_errors;            /* SPI command frames rejected on CRC (slave) */
    uint16_t can_tx_errors;             /* CAN TX errors seen by the slave */
    uint8_t spi_tx_arm_fails;           /* slave TX-arm (HAL DMA) failures, then retried (wraps) */
    tele_motor_t motors[MAX_MOTORS_PER_CHAIN];
} tele_chain_t;   /* 119 bytes */

/* one telemetry tick, 20 + 119·4 = 496 bytes full-size. The master transmits
 * header + n_chains chains (chains is truncate_by n_chains), where n_chains
 * is the CONFIGURED slave count — every configured chain is emitted every
 * cycle at its fixed slot (chains[s] = slave s; see ChainLink), so chain
 * position is stable whether or not a slave is reachable. The four cmd_*
 * counters classify host-loop timing vs the master cycle at the cycle-start
 * mailbox swap (docs/architecture.md); free-running, the host reports
 * deltas. */
typedef struct PROTO_PACKED {
    uint16_t cycle_id;                  /* this telemetry's master cycle (increments/cycle) */
    uint32_t master_time_us;            /* master uptime (µs) */
    uint16_t last_cmd_seq_rx;           /* last cmd_seq the master received from host */
    uint16_t cmd_seq_active;            /* cmd_seq actually sent to the slaves THIS cycle (0 = holding) */
    uint16_t cmd_on_time;               /* applied a fresh command echoing the latest tele cycle */
    uint16_t cmd_late;                  /* applied a fresh command echoing an OLDER cycle */
    uint16_t cmd_missing;               /* cycle with no fresh command → held current targets */
    uint16_t cmd_duplicate;             /* command overwritten unapplied (host sent >1 per cycle) */
    uint8_t n_chains;
    uint8_t robot_state;                /* RobotState */
    tele_chain_t chains[MAX_CHAINS];    /* transmitted prefix sized by n_chains */
} tele_robot_t;   /* 496 bytes */

/* master status (20 Hz), 33 bytes. robot_state and slave reachability are
 * NOT here: robot_state rides tele_robot_t (emitted unconditionally every
 * cycle) and per-slave link state is tele_chain_t.link, so the host sees
 * both in the same cycle it reads motors. */
typedef struct PROTO_PACKED {
    uint32_t schema_hash;               /* the firmware's PROTO_SCHEMA_HASH; the host link compares it against its own on connect and refuses to run on a mismatch */
    uint8_t slave_proto_mismatch;       /* bit s = slave s responds but fails the SPI version gate (v9.1) */
    uint32_t uptime_ms;
    uint32_t link_errors;               /* bad CRC or unknown msg_type */
    uint32_t rx_frames;
    uint16_t master_poll_hz;            /* configured SPI poll rate */
    uint16_t telemetry_hz;              /* configured MSG_ROBOT_TELE rate */
    uint16_t slave_tick_hz;             /* configured slave control-tick rate */
    uint16_t host_cmd_hz;               /* configured expected host cmd rate */
    uint32_t rx_resyncs;                /* USB RX resync events (v5) */
    uint32_t rx_discarded_bytes;        /* bytes dropped during resync (v5) */
} MasterStatus;   /* 33 bytes */

/* SPI version gate (v9.1), 5 bytes, appended to BOTH SPI frame directions
 * between the payload and the CRC. Each side stamps its own build; the
 * receiver refuses a mismatched frame's payload (master: slave excluded from
 * alive/forwarding/enables, reported via tele_chain_t.link = MISMATCH and
 * MasterStatus.slave_proto_mismatch; slave: command ignored, counted in
 * cmd_crc_errors). Polling continues, so a reflashed board is re-accepted
 * without a reset on either side. */
typedef struct PROTO_PACKED {
    uint8_t proto_version;              /* sender's PROTO_VERSION */
    uint32_t schema_hash;               /* sender's PROTO_SCHEMA_HASH */
} spi_trailer_t;   /* 5 bytes */

/* per-slave status (20 Hz), 18 bytes */
typedef struct PROTO_PACKED {
    uint8_t slave_id;                   /* which slave this status is for */
    uint8_t motors_alive;               /* bit i = motor i alive */
    uint32_t uptime_ms;
    uint32_t crc_errors;                /* SPI telemetry frames failing CRC (master) */
    uint32_t cmd_crc_errors;            /* SPI command frames the slave rejected on CRC */
    uint32_t seq_gaps;                  /* master: echoed spi-seq gaps (dropped frames) */
} SlaveStatus;   /* 18 bytes */

/* ── derived wire constants (computed from the schema; asserted below) ── */
/* wire header size */
#define MSG_HEADER_SIZE 16u   /* sizeof(MsgHeader) */
/* Largest framed payload; a header claiming more than this is junk. The host
 * side keeps its own hand-written resync bound (≥ this). */
#define PROTO_MAX_PAYLOAD ((uint16_t)496u)   /* sizeof(tele_robot_t) */
/* spi_trailer_t offset in a master→slave SPI command frame */
#define SPI_CMD_TRAILER_OFF ((uint16_t)68u)   /* SPI_CMD_HDR_BYTES + sizeof(cmd_chain_t) */
/* CRC16 offset in a master→slave SPI command frame */
#define SPI_CMD_CRC_OFF ((uint16_t)73u)   /* SPI_CMD_HDR_BYTES + sizeof(cmd_chain_t) + sizeof(spi_trailer_t) */
/* master → slave: [opcode u8][spi_seq u8][cycle_id u16][cmd_seq
 * u16][cmd_chain_t][spi_trailer_t][crc16] */
#define SPI_CMD_FRAME_SIZE ((uint16_t)75u)   /* SPI_CMD_HDR_BYTES + sizeof(cmd_chain_t) + sizeof(spi_trailer_t) + SPI_CRC_BYTES */
/* spi_trailer_t offset in a slave→master SPI telemetry frame */
#define SPI_TELE_TRAILER_OFF ((uint16_t)119u)   /* sizeof(tele_chain_t) */
/* CRC16 offset in a slave→master SPI telemetry frame */
#define SPI_TELE_CRC_OFF ((uint16_t)124u)   /* sizeof(tele_chain_t) + sizeof(spi_trailer_t) */
/* slave → master: [tele_chain_t][spi_trailer_t][crc16] */
#define SPI_TELE_FRAME_SIZE ((uint16_t)126u)   /* sizeof(tele_chain_t) + sizeof(spi_trailer_t) + SPI_CRC_BYTES */
/* full-duplex SPI transfer length = the larger of the two frames */
#define SPI_XFER_SIZE 126u   /* max(SPI_CMD_FRAME_SIZE, SPI_TELE_FRAME_SIZE) */
/* largest full USB frame (header + largest payload); must fit one TX slot */
#define USB_MAX_FRAME_SIZE 512u   /* sizeof(MsgHeader) + sizeof(tele_robot_t) */

/* ── layout guards (catch C↔schema drift at compile time) ───────────── */
#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(MsgHeader) == 16u, "MsgHeader size");
_Static_assert(offsetof(MsgHeader, type) == 0u, "MsgHeader.type offset");
_Static_assert(offsetof(MsgHeader, seq) == 2u, "MsgHeader.seq offset");
_Static_assert(offsetof(MsgHeader, src) == 4u, "MsgHeader.src offset");
_Static_assert(offsetof(MsgHeader, dst) == 5u, "MsgHeader.dst offset");
_Static_assert(offsetof(MsgHeader, ts_ms) == 6u, "MsgHeader.ts_ms offset");
_Static_assert(offsetof(MsgHeader, len) == 10u, "MsgHeader.len offset");
_Static_assert(offsetof(MsgHeader, ver_flags) == 12u, "MsgHeader.ver_flags offset");
_Static_assert(offsetof(MsgHeader, crc16) == 14u, "MsgHeader.crc16 offset");
_Static_assert(sizeof(cmd_motor_t) == 12u, "cmd_motor_t size");
_Static_assert(offsetof(cmd_motor_t, drive_req) == 0u, "cmd_motor_t.drive_req offset");
_Static_assert(offsetof(cmd_motor_t, pos) == 1u, "cmd_motor_t.pos offset");
_Static_assert(offsetof(cmd_motor_t, vel) == 3u, "cmd_motor_t.vel offset");
_Static_assert(offsetof(cmd_motor_t, kp) == 5u, "cmd_motor_t.kp offset");
_Static_assert(offsetof(cmd_motor_t, kd) == 7u, "cmd_motor_t.kd offset");
_Static_assert(offsetof(cmd_motor_t, tau_ff) == 9u, "cmd_motor_t.tau_ff offset");
_Static_assert(offsetof(cmd_motor_t, flags) == 11u, "cmd_motor_t.flags offset");
_Static_assert(sizeof(cmd_chain_t) == 62u, "cmd_chain_t size");
_Static_assert(offsetof(cmd_chain_t, chain_id) == 0u, "cmd_chain_t.chain_id offset");
_Static_assert(offsetof(cmd_chain_t, n_motors) == 1u, "cmd_chain_t.n_motors offset");
_Static_assert(offsetof(cmd_chain_t, motors) == 2u, "cmd_chain_t.motors offset");
_Static_assert(sizeof(cmd_robot_t) == 254u, "cmd_robot_t size");
_Static_assert(offsetof(cmd_robot_t, cycle_id) == 0u, "cmd_robot_t.cycle_id offset");
_Static_assert(offsetof(cmd_robot_t, cmd_seq) == 2u, "cmd_robot_t.cmd_seq offset");
_Static_assert(offsetof(cmd_robot_t, n_chains) == 4u, "cmd_robot_t.n_chains offset");
_Static_assert(offsetof(cmd_robot_t, reserved) == 5u, "cmd_robot_t.reserved offset");
_Static_assert(offsetof(cmd_robot_t, chains) == 6u, "cmd_robot_t.chains offset");
_Static_assert(sizeof(tele_motor_t) == 21u, "tele_motor_t size");
_Static_assert(offsetof(tele_motor_t, pos) == 0u, "tele_motor_t.pos offset");
_Static_assert(offsetof(tele_motor_t, vel) == 2u, "tele_motor_t.vel offset");
_Static_assert(offsetof(tele_motor_t, tau) == 4u, "tele_motor_t.tau offset");
_Static_assert(offsetof(tele_motor_t, temp_c) == 6u, "tele_motor_t.temp_c offset");
_Static_assert(offsetof(tele_motor_t, drive_state) == 7u, "tele_motor_t.drive_state offset");
_Static_assert(offsetof(tele_motor_t, drive_fault) == 8u, "tele_motor_t.drive_fault offset");
_Static_assert(offsetof(tele_motor_t, rs_mode) == 9u, "tele_motor_t.rs_mode offset");
_Static_assert(offsetof(tele_motor_t, rs_fault_bits) == 10u, "tele_motor_t.rs_fault_bits offset");
_Static_assert(offsetof(tele_motor_t, flags) == 11u, "tele_motor_t.flags offset");
_Static_assert(offsetof(tele_motor_t, fb_age_ms) == 12u, "tele_motor_t.fb_age_ms offset");
_Static_assert(offsetof(tele_motor_t, rs_fault_word) == 13u, "tele_motor_t.rs_fault_word offset");
_Static_assert(offsetof(tele_motor_t, last_applied_seq) == 17u, "tele_motor_t.last_applied_seq offset");
_Static_assert(offsetof(tele_motor_t, reserved) == 19u, "tele_motor_t.reserved offset");
_Static_assert(sizeof(tele_chain_t) == 119u, "tele_chain_t size");
_Static_assert(offsetof(tele_chain_t, chain_id) == 0u, "tele_chain_t.chain_id offset");
_Static_assert(offsetof(tele_chain_t, n_motors) == 1u, "tele_chain_t.n_motors offset");
_Static_assert(offsetof(tele_chain_t, link) == 2u, "tele_chain_t.link offset");
_Static_assert(offsetof(tele_chain_t, spi_seq_echo) == 3u, "tele_chain_t.spi_seq_echo offset");
_Static_assert(offsetof(tele_chain_t, spi_resyncs) == 4u, "tele_chain_t.spi_resyncs offset");
_Static_assert(offsetof(tele_chain_t, slave_time_us) == 5u, "tele_chain_t.slave_time_us offset");
_Static_assert(offsetof(tele_chain_t, cmd_crc_errors) == 9u, "tele_chain_t.cmd_crc_errors offset");
_Static_assert(offsetof(tele_chain_t, can_tx_errors) == 11u, "tele_chain_t.can_tx_errors offset");
_Static_assert(offsetof(tele_chain_t, spi_tx_arm_fails) == 13u, "tele_chain_t.spi_tx_arm_fails offset");
_Static_assert(offsetof(tele_chain_t, motors) == 14u, "tele_chain_t.motors offset");
_Static_assert(sizeof(tele_robot_t) == 496u, "tele_robot_t size");
_Static_assert(offsetof(tele_robot_t, cycle_id) == 0u, "tele_robot_t.cycle_id offset");
_Static_assert(offsetof(tele_robot_t, master_time_us) == 2u, "tele_robot_t.master_time_us offset");
_Static_assert(offsetof(tele_robot_t, last_cmd_seq_rx) == 6u, "tele_robot_t.last_cmd_seq_rx offset");
_Static_assert(offsetof(tele_robot_t, cmd_seq_active) == 8u, "tele_robot_t.cmd_seq_active offset");
_Static_assert(offsetof(tele_robot_t, cmd_on_time) == 10u, "tele_robot_t.cmd_on_time offset");
_Static_assert(offsetof(tele_robot_t, cmd_late) == 12u, "tele_robot_t.cmd_late offset");
_Static_assert(offsetof(tele_robot_t, cmd_missing) == 14u, "tele_robot_t.cmd_missing offset");
_Static_assert(offsetof(tele_robot_t, cmd_duplicate) == 16u, "tele_robot_t.cmd_duplicate offset");
_Static_assert(offsetof(tele_robot_t, n_chains) == 18u, "tele_robot_t.n_chains offset");
_Static_assert(offsetof(tele_robot_t, robot_state) == 19u, "tele_robot_t.robot_state offset");
_Static_assert(offsetof(tele_robot_t, chains) == 20u, "tele_robot_t.chains offset");
_Static_assert(sizeof(MasterStatus) == 33u, "MasterStatus size");
_Static_assert(offsetof(MasterStatus, schema_hash) == 0u, "MasterStatus.schema_hash offset");
_Static_assert(offsetof(MasterStatus, slave_proto_mismatch) == 4u, "MasterStatus.slave_proto_mismatch offset");
_Static_assert(offsetof(MasterStatus, uptime_ms) == 5u, "MasterStatus.uptime_ms offset");
_Static_assert(offsetof(MasterStatus, link_errors) == 9u, "MasterStatus.link_errors offset");
_Static_assert(offsetof(MasterStatus, rx_frames) == 13u, "MasterStatus.rx_frames offset");
_Static_assert(offsetof(MasterStatus, master_poll_hz) == 17u, "MasterStatus.master_poll_hz offset");
_Static_assert(offsetof(MasterStatus, telemetry_hz) == 19u, "MasterStatus.telemetry_hz offset");
_Static_assert(offsetof(MasterStatus, slave_tick_hz) == 21u, "MasterStatus.slave_tick_hz offset");
_Static_assert(offsetof(MasterStatus, host_cmd_hz) == 23u, "MasterStatus.host_cmd_hz offset");
_Static_assert(offsetof(MasterStatus, rx_resyncs) == 25u, "MasterStatus.rx_resyncs offset");
_Static_assert(offsetof(MasterStatus, rx_discarded_bytes) == 29u, "MasterStatus.rx_discarded_bytes offset");
_Static_assert(sizeof(spi_trailer_t) == 5u, "spi_trailer_t size");
_Static_assert(offsetof(spi_trailer_t, proto_version) == 0u, "spi_trailer_t.proto_version offset");
_Static_assert(offsetof(spi_trailer_t, schema_hash) == 1u, "spi_trailer_t.schema_hash offset");
_Static_assert(sizeof(SlaveStatus) == 18u, "SlaveStatus size");
_Static_assert(offsetof(SlaveStatus, slave_id) == 0u, "SlaveStatus.slave_id offset");
_Static_assert(offsetof(SlaveStatus, motors_alive) == 1u, "SlaveStatus.motors_alive offset");
_Static_assert(offsetof(SlaveStatus, uptime_ms) == 2u, "SlaveStatus.uptime_ms offset");
_Static_assert(offsetof(SlaveStatus, crc_errors) == 6u, "SlaveStatus.crc_errors offset");
_Static_assert(offsetof(SlaveStatus, cmd_crc_errors) == 10u, "SlaveStatus.cmd_crc_errors offset");
_Static_assert(offsetof(SlaveStatus, seq_gaps) == 14u, "SlaveStatus.seq_gaps offset");
_Static_assert(NODE_SLAVE_1 <= 255u, "NodeId fits u8");
_Static_assert(MSG_ROBOT_TELE <= 65535u, "MsgType fits u16");
_Static_assert(ROBOT_PROTO_MISMATCH <= 255u, "RobotState fits u8");
_Static_assert(DRV_REQ_RESET <= 255u, "DriveReq fits u8");
_Static_assert(DRV_FAULT <= 255u, "DriveState fits u8");
_Static_assert(CHAIN_LINK_MISMATCH <= 255u, "ChainLink fits u8");
_Static_assert(DRVF_WOUND <= 255u, "DriveFault fits u8");
/* Every full USB frame must fit one master TX slot (usb_tx.c ring). */
_Static_assert(USB_MAX_FRAME_SIZE <= USB_TX_SLOT_MAX, "USB frame fits one TX slot");
#endif

#endif /* PROTOCOL_GEN_H */
