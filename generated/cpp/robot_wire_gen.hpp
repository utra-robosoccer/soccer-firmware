// AUTO-GENERATED — DO NOT EDIT.
// Source:      firmware/common/protocol.yaml
// Regenerate:  python3 scripts/gen_protocol.py
//
// SCHEMA_HASH 0xBC39C97D: CRC-32 of the schema's canonical wire
// projection (parsed YAML minus doc/unit/hex keys, compact sorted-key
// JSON, UTF-8) — see gen_protocol.py. Doc-only edits do not change it.
//
// Header-only mirror of the soccer wire protocol for C++ consumers
// (soccer-software pins this file + generated/cpp/fixtures/). Packed
// little-endian structs; encode/decode are whole-struct memcpy, so a
// LITTLE-ENDIAN host is assumed (same assumption as the firmware side).
// Never take references or pointers to the packed members — use the
// to_bytes/from_bytes helpers and whole-field loads/stores.
#ifndef ROBOT_WIRE_GEN_HPP
#define ROBOT_WIRE_GEN_HPP

#include <array>
#include <cstddef>   // offsetof for the layout guards
#include <cstdint>
#include <cstring>

namespace robot_wire {

constexpr std::uint8_t  PROTO_VERSION = 9u;
constexpr std::uint32_t PROTO_SCHEMA_HASH = 0xBC39C97Du;

// ── wire constants ──
constexpr std::uint16_t CMD_SEQ_NONE = 0u;
constexpr std::uint16_t MAX_MOTORS_PER_CHAIN = 5u;
constexpr std::uint16_t MAX_CHAINS = 4u;
constexpr std::uint16_t PROTO_CRC_INIT = 0xFFFFu;
constexpr std::uint16_t PROTO_CRC_POLY = 0x1021u;
constexpr std::uint16_t SPI_OP_NOP = 0x00u;
constexpr std::uint16_t SPI_OP_ROBOT_CMD = 0x01u;
constexpr std::uint16_t SPI_CMD_HDR_BYTES = 6u;
constexpr std::uint16_t SPI_CRC_BYTES = 2u;
constexpr std::uint16_t USB_TX_SLOT_MAX = 512u;

// node identifiers
enum class NodeId : std::uint8_t {
    NODE_JETSON  = 1u,
    NODE_MASTER  = 2u,
    NODE_SLAVE_0 = 3u,
    NODE_SLAVE_1 = 4u,
};

// message types (MsgHeader.type)
enum class MsgType : std::uint16_t {
    MSG_PING          = 0x01u,
    MSG_MASTER_STATUS = 0x02u,
    MSG_SLAVE_STATUS  = 0x03u,
    MSG_ROBOT_CMD     = 0x08u,  // host→master: one cmd_robot_t per host tick
    MSG_ROBOT_TELE    = 0x09u,  // master→host: one tele_robot_t per tele tick
};

// robot lifecycle (tele_robot_t.robot_state / MasterStatus.robot_state)
enum class RobotState : std::uint8_t {
    ROBOT_INIT      = 0u,
    ROBOT_READY     = 1u,
    ROBOT_DEGRADED  = 2u,
    ROBOT_HOST_LOST = 3u,  // master's host-death watchdog tripped (forcing STOP)
};

// Host→slave per-motor drive request (cmd_motor_t.drive_req).
// Level-triggered: safe to repeat every cycle; a repeat in the target or
// transit state is a no-op. The MIT tuple travels independently under
// CMD_FLAG_VALID — there is no MIT request; in ENABLED every valid tuple is
// simply applied.
enum class DriveReq : std::uint8_t {
    DRV_REQ_DISABLE = 0u,  // Type-4 stop now
    DRV_REQ_ENABLE  = 1u,  // DISABLED → ENABLING → ENABLED; rejected elsewhere
    DRV_REQ_STOP    = 2u,  // ENABLED → STOPPING (ramp) → DISABLED
    DRV_REQ_RESET   = 3u,  // FAULT → CLEARING → DISABLED; never enables
};

// per-motor drive state (tele_motor_t.drive_state under v9)
enum class DriveState : std::uint8_t {
    DRV_BOOT        = 0u,
    DRV_DISCOVERING = 1u,
    DRV_ABSENT      = 2u,  // no feedback; pinged ~100 ms; reply → DISABLED
    DRV_DISABLED    = 3u,  // motor off, polled every cycle
    DRV_ENABLING    = 4u,  // async handshake; rs_mode==Run confirms → ENABLED
    DRV_ENABLED     = 5u,  // limp until the first VALID tuple; then applies it
    DRV_STOPPING    = 6u,  // ramped stop; requested → DISABLED, fault → FAULT
    DRV_CLEARING    = 7u,  // RESET sent the Type-4 clear; judging the reply
    DRV_FAULT       = 8u,  // latched — see drive_fault; only RESET leaves
};

// Why a motor is faulted (tele_motor_t.drive_fault under v9). Latched at
// fault DETECTION (first fault wins); cleared on CLEARING entry so a
// re-latch reports the current fault.
enum class DriveFault : std::uint8_t {
    DRVF_NONE           = 0u,
    DRVF_CAN_TIMEOUT    = 1u,  // Type-2 feedback went stale (comms-broken)
    DRVF_MASTER_LOST    = 2u,  // SPI command stream died → autonomous stop ramp
    DRVF_ENABLE_FAILED  = 3u,  // ENABLING saw no rs_mode==Run in time
    DRVF_NOT_ENABLED    = 4u,  // enable monitor: enabled motor reports not-Run
    DRVF_OVERTORQUE     = 5u,  // |tau| exceeded per-motor max_tau
    DRVF_MOTOR_REPORTED = 6u,  // RS vendor fault bits / Cali mode
    DRVF_WOUND          = 7u,  // ENABLE refused: shaft wound beyond safe range
};

// ── cmd_motor_t.flags ──
constexpr std::uint8_t CMD_FLAG_VALID = 1u << 0u;  // the MIT tuple in this slot is live; drive_req applies regardless
constexpr std::uint8_t CMD_FLAG_USE_CONFIG_GAINS = 1u << 1u;  // ignore wire kp/kd, use config defaults
// bit 2 reserved — was CMD_FLAG_FAULT_RESET (v8); RESET is a drive request in v9

// ── tele_motor_t.flags ──
constexpr std::uint8_t TELE_FLAG_REQUEST_REJECTED = 1u << 0u;  // last drive request was illegal
// bit 1 reserved — was TELE_FLAG_TO_ZERO_ARRIVED (v8); TO_ZERO is host-side in v9
constexpr std::uint8_t TELE_FLAG_SATURATED = 1u << 2u;  // a fixed-point field saturated
constexpr std::uint8_t TELE_FLAG_CLAMPED_POS = 1u << 3u;  // commanded pos hit a soft limit
constexpr std::uint8_t TELE_FLAG_CLAMPED_TAU = 1u << 4u;  // vel feedforward cancelled at limit
constexpr std::uint8_t TELE_FLAG_CMD_STALE = 1u << 5u;  // executing a held/watchdog setpoint
constexpr std::uint8_t TELE_FLAG_CLAMPED_GAIN = 1u << 6u;  // wire kp/kd clamped to the model's range

// ── fixed-point scales ──
constexpr float PROTO_POS_SCALE = 10000.0f;  // rad → i16; home-frame ±π
constexpr float PROTO_VEL_SCALE = 100.0f;  // rad/s → i16
constexpr float PROTO_TAU_SCALE = 100.0f;  // N·m → i16
constexpr float PROTO_KP_SCALE = 10.0f;  // Kp → u16; 0..6553.5, covers 0..5000
constexpr float PROTO_KD_SCALE = 100.0f;  // Kd → u16; 0..655.35, covers 0..100

#pragma pack(push, 1)

// wire header, 16 bytes. ver_flags low byte = PROTO_VERSION (both ends drop
// and count any frame whose version differs); high byte reserved (0).
struct MsgHeader {
    std::uint16_t type;                 // MsgType
    std::uint16_t seq;
    std::uint8_t src;                   // NodeId
    std::uint8_t dst;                   // NodeId
    std::uint32_t ts_ms;                // HAL_GetTick() at send time
    std::uint16_t len;                  // payload bytes
    std::uint16_t ver_flags;            // low byte = PROTO_VERSION; high byte reserved 0
    std::uint16_t crc16;                // CRC16-CCITT over hdr(crc=0)+payload
};

// per-motor command (host → master → slave), 12 bytes
struct cmd_motor_t {
    std::uint8_t drive_req;             // DriveReq
    std::int16_t pos;                   // PROTO_POS_SCALE; rad × PROTO_POS_SCALE (home-frame ±π)
    std::int16_t vel;                   // PROTO_VEL_SCALE
    std::uint16_t kp;                   // PROTO_KP_SCALE
    std::uint16_t kd;                   // PROTO_KD_SCALE
    std::int16_t tau_ff;                // PROTO_TAU_SCALE
    std::uint8_t flags;                 // cmd_motor_flags
};

// one CAN chain's commands, 2 + 12·5 = 62 bytes
struct cmd_chain_t {
    std::uint8_t chain_id;
    std::uint8_t n_motors;              // valid slots (≤ MAX_MOTORS_PER_CHAIN)
    cmd_motor_t motors[MAX_MOTORS_PER_CHAIN];
};

// one host tick's full command, 6 + 62·4 = 254 bytes. Always transmitted
// full-size (NOT truncated by n_chains). reserved carries the master test
// sentinels (0xDE arm SPI-desync, 0xB0 arm USB-TX test; 0 in normal
// operation).
struct cmd_robot_t {
    std::uint16_t cycle_id;             // echo of last master cycle seen
    std::uint16_t cmd_seq;              // one per host tick (≥1, 0 reserved)
    std::uint8_t n_chains;              // valid chains (≤ MAX_CHAINS)
    std::uint8_t reserved;              // test sentinels; 0 in normal operation
    cmd_chain_t chains[MAX_CHAINS];
};

// per-motor telemetry (slave → master → host), 21 bytes
struct tele_motor_t {
    std::int16_t pos;                   // PROTO_POS_SCALE; rad × PROTO_POS_SCALE (home-frame)
    std::int16_t vel;                   // PROTO_VEL_SCALE
    std::int16_t tau;                   // PROTO_TAU_SCALE; measured
    std::uint8_t temp_c;                // integer °C
    std::uint8_t drive_state;           // DriveState
    std::uint8_t drive_fault;           // DriveFault; latched
    std::uint8_t rs_mode;               // RS run_mode reported in Type-2 (0 reset / 1 cali / 2 run)
    std::uint8_t rs_fault_bits;         // packed Type-2 fault bits
    std::uint8_t flags;                 // tele_motor_flags
    std::uint8_t fb_age_ms;             // ms since last Type-2, saturating 255
    std::uint32_t rs_fault_word;        // 0x3022 latched; 0 clear; 0xFFFFFFFF read-fail
    std::uint16_t last_applied_seq;     // host cmd_seq last confirmed applied (0 = none)
    std::uint16_t reserved;             // growth slot (0)
};

// one CAN chain's telemetry, 12 + 21·5 = 118 bytes
struct tele_chain_t {
    std::uint8_t chain_id;
    std::uint8_t n_motors;
    std::uint8_t spi_seq_echo;          // SPI link-health seq echoed (NOT cmd_seq)
    std::uint8_t spi_resyncs;           // slave SPI DMA resync count (wraps; host deltas)
    std::uint32_t slave_time_us;        // slave uptime (µs)
    std::uint16_t cmd_crc_errors;       // SPI command frames rejected on CRC (slave)
    std::uint16_t can_tx_errors;        // CAN TX errors seen by the slave
    std::uint8_t spi_tx_arm_fails;      // slave TX-arm (HAL DMA) failures, then retried (wraps)
    tele_motor_t motors[MAX_MOTORS_PER_CHAIN];
};

// one telemetry tick, 20 + 118·4 = 492 bytes full-size. The master transmits
// header + n_chains chains only (chains is truncate_by n_chains), keeping
// the 200 Hz stream small. The four cmd_* counters classify host-loop timing
// vs the master cycle at the cycle-start mailbox swap
// (docs/architecture.md); free-running, the host reports deltas.
struct tele_robot_t {
    std::uint16_t cycle_id;             // this telemetry's master cycle (increments/cycle)
    std::uint32_t master_time_us;       // master uptime (µs)
    std::uint16_t last_cmd_seq_rx;      // last cmd_seq the master received from host
    std::uint16_t cmd_seq_active;       // cmd_seq actually sent to the slaves THIS cycle (0 = holding)
    std::uint16_t cmd_on_time;          // applied a fresh command echoing the latest tele cycle
    std::uint16_t cmd_late;             // applied a fresh command echoing an OLDER cycle
    std::uint16_t cmd_missing;          // cycle with no fresh command → held current targets
    std::uint16_t cmd_duplicate;        // command overwritten unapplied (host sent >1 per cycle)
    std::uint8_t n_chains;
    std::uint8_t robot_state;           // RobotState
    tele_chain_t chains[MAX_CHAINS];    // transmitted prefix sized by n_chains
};

// master status (20 Hz), 30 bytes
struct MasterStatus {
    std::uint8_t robot_state;           // RobotState
    std::uint8_t slave_alive;           // bit s = slave s reachable
    std::uint32_t uptime_ms;
    std::uint32_t link_errors;          // bad CRC or unknown msg_type
    std::uint32_t rx_frames;
    std::uint16_t master_poll_hz;       // configured SPI poll rate
    std::uint16_t telemetry_hz;         // configured MSG_ROBOT_TELE rate
    std::uint16_t slave_tick_hz;        // configured slave control-tick rate
    std::uint16_t host_cmd_hz;          // configured expected host cmd rate
    std::uint32_t rx_resyncs;           // USB RX resync events (v5)
    std::uint32_t rx_discarded_bytes;   // bytes dropped during resync (v5)
};

// per-slave status (20 Hz), 18 bytes
struct SlaveStatus {
    std::uint8_t slave_id;              // which slave this status is for
    std::uint8_t motors_alive;          // bit i = motor i alive
    std::uint32_t uptime_ms;
    std::uint32_t crc_errors;           // SPI telemetry frames failing CRC (master)
    std::uint32_t cmd_crc_errors;       // SPI command frames the slave rejected on CRC
    std::uint32_t seq_gaps;             // master: echoed spi-seq gaps (dropped frames)
};

#pragma pack(pop)

// ── derived wire constants ──
constexpr std::uint16_t MSG_HEADER_SIZE = 16u;  // sizeof(MsgHeader)
constexpr std::uint16_t PROTO_MAX_PAYLOAD = 492u;  // sizeof(tele_robot_t)
constexpr std::uint16_t SPI_CMD_CRC_OFF = 68u;  // SPI_CMD_HDR_BYTES + sizeof(cmd_chain_t)
constexpr std::uint16_t SPI_CMD_FRAME_SIZE = 70u;  // SPI_CMD_HDR_BYTES + sizeof(cmd_chain_t) + SPI_CRC_BYTES
constexpr std::uint16_t SPI_TELE_CRC_OFF = 118u;  // sizeof(tele_chain_t)
constexpr std::uint16_t SPI_TELE_FRAME_SIZE = 120u;  // sizeof(tele_chain_t) + SPI_CRC_BYTES
constexpr std::uint16_t SPI_XFER_SIZE = 120u;  // max(SPI_CMD_FRAME_SIZE, SPI_TELE_FRAME_SIZE)
constexpr std::uint16_t USB_MAX_FRAME_SIZE = 508u;  // sizeof(MsgHeader) + sizeof(tele_robot_t)

// ── layout guards ──
static_assert(sizeof(MsgHeader) == 16u, "MsgHeader size");
static_assert(offsetof(MsgHeader, type) == 0u, "MsgHeader.type offset");
static_assert(offsetof(MsgHeader, seq) == 2u, "MsgHeader.seq offset");
static_assert(offsetof(MsgHeader, src) == 4u, "MsgHeader.src offset");
static_assert(offsetof(MsgHeader, dst) == 5u, "MsgHeader.dst offset");
static_assert(offsetof(MsgHeader, ts_ms) == 6u, "MsgHeader.ts_ms offset");
static_assert(offsetof(MsgHeader, len) == 10u, "MsgHeader.len offset");
static_assert(offsetof(MsgHeader, ver_flags) == 12u, "MsgHeader.ver_flags offset");
static_assert(offsetof(MsgHeader, crc16) == 14u, "MsgHeader.crc16 offset");
static_assert(sizeof(cmd_motor_t) == 12u, "cmd_motor_t size");
static_assert(offsetof(cmd_motor_t, drive_req) == 0u, "cmd_motor_t.drive_req offset");
static_assert(offsetof(cmd_motor_t, pos) == 1u, "cmd_motor_t.pos offset");
static_assert(offsetof(cmd_motor_t, vel) == 3u, "cmd_motor_t.vel offset");
static_assert(offsetof(cmd_motor_t, kp) == 5u, "cmd_motor_t.kp offset");
static_assert(offsetof(cmd_motor_t, kd) == 7u, "cmd_motor_t.kd offset");
static_assert(offsetof(cmd_motor_t, tau_ff) == 9u, "cmd_motor_t.tau_ff offset");
static_assert(offsetof(cmd_motor_t, flags) == 11u, "cmd_motor_t.flags offset");
static_assert(sizeof(cmd_chain_t) == 62u, "cmd_chain_t size");
static_assert(offsetof(cmd_chain_t, chain_id) == 0u, "cmd_chain_t.chain_id offset");
static_assert(offsetof(cmd_chain_t, n_motors) == 1u, "cmd_chain_t.n_motors offset");
static_assert(offsetof(cmd_chain_t, motors) == 2u, "cmd_chain_t.motors offset");
static_assert(sizeof(cmd_robot_t) == 254u, "cmd_robot_t size");
static_assert(offsetof(cmd_robot_t, cycle_id) == 0u, "cmd_robot_t.cycle_id offset");
static_assert(offsetof(cmd_robot_t, cmd_seq) == 2u, "cmd_robot_t.cmd_seq offset");
static_assert(offsetof(cmd_robot_t, n_chains) == 4u, "cmd_robot_t.n_chains offset");
static_assert(offsetof(cmd_robot_t, reserved) == 5u, "cmd_robot_t.reserved offset");
static_assert(offsetof(cmd_robot_t, chains) == 6u, "cmd_robot_t.chains offset");
static_assert(sizeof(tele_motor_t) == 21u, "tele_motor_t size");
static_assert(offsetof(tele_motor_t, pos) == 0u, "tele_motor_t.pos offset");
static_assert(offsetof(tele_motor_t, vel) == 2u, "tele_motor_t.vel offset");
static_assert(offsetof(tele_motor_t, tau) == 4u, "tele_motor_t.tau offset");
static_assert(offsetof(tele_motor_t, temp_c) == 6u, "tele_motor_t.temp_c offset");
static_assert(offsetof(tele_motor_t, drive_state) == 7u, "tele_motor_t.drive_state offset");
static_assert(offsetof(tele_motor_t, drive_fault) == 8u, "tele_motor_t.drive_fault offset");
static_assert(offsetof(tele_motor_t, rs_mode) == 9u, "tele_motor_t.rs_mode offset");
static_assert(offsetof(tele_motor_t, rs_fault_bits) == 10u, "tele_motor_t.rs_fault_bits offset");
static_assert(offsetof(tele_motor_t, flags) == 11u, "tele_motor_t.flags offset");
static_assert(offsetof(tele_motor_t, fb_age_ms) == 12u, "tele_motor_t.fb_age_ms offset");
static_assert(offsetof(tele_motor_t, rs_fault_word) == 13u, "tele_motor_t.rs_fault_word offset");
static_assert(offsetof(tele_motor_t, last_applied_seq) == 17u, "tele_motor_t.last_applied_seq offset");
static_assert(offsetof(tele_motor_t, reserved) == 19u, "tele_motor_t.reserved offset");
static_assert(sizeof(tele_chain_t) == 118u, "tele_chain_t size");
static_assert(offsetof(tele_chain_t, chain_id) == 0u, "tele_chain_t.chain_id offset");
static_assert(offsetof(tele_chain_t, n_motors) == 1u, "tele_chain_t.n_motors offset");
static_assert(offsetof(tele_chain_t, spi_seq_echo) == 2u, "tele_chain_t.spi_seq_echo offset");
static_assert(offsetof(tele_chain_t, spi_resyncs) == 3u, "tele_chain_t.spi_resyncs offset");
static_assert(offsetof(tele_chain_t, slave_time_us) == 4u, "tele_chain_t.slave_time_us offset");
static_assert(offsetof(tele_chain_t, cmd_crc_errors) == 8u, "tele_chain_t.cmd_crc_errors offset");
static_assert(offsetof(tele_chain_t, can_tx_errors) == 10u, "tele_chain_t.can_tx_errors offset");
static_assert(offsetof(tele_chain_t, spi_tx_arm_fails) == 12u, "tele_chain_t.spi_tx_arm_fails offset");
static_assert(offsetof(tele_chain_t, motors) == 13u, "tele_chain_t.motors offset");
static_assert(sizeof(tele_robot_t) == 492u, "tele_robot_t size");
static_assert(offsetof(tele_robot_t, cycle_id) == 0u, "tele_robot_t.cycle_id offset");
static_assert(offsetof(tele_robot_t, master_time_us) == 2u, "tele_robot_t.master_time_us offset");
static_assert(offsetof(tele_robot_t, last_cmd_seq_rx) == 6u, "tele_robot_t.last_cmd_seq_rx offset");
static_assert(offsetof(tele_robot_t, cmd_seq_active) == 8u, "tele_robot_t.cmd_seq_active offset");
static_assert(offsetof(tele_robot_t, cmd_on_time) == 10u, "tele_robot_t.cmd_on_time offset");
static_assert(offsetof(tele_robot_t, cmd_late) == 12u, "tele_robot_t.cmd_late offset");
static_assert(offsetof(tele_robot_t, cmd_missing) == 14u, "tele_robot_t.cmd_missing offset");
static_assert(offsetof(tele_robot_t, cmd_duplicate) == 16u, "tele_robot_t.cmd_duplicate offset");
static_assert(offsetof(tele_robot_t, n_chains) == 18u, "tele_robot_t.n_chains offset");
static_assert(offsetof(tele_robot_t, robot_state) == 19u, "tele_robot_t.robot_state offset");
static_assert(offsetof(tele_robot_t, chains) == 20u, "tele_robot_t.chains offset");
static_assert(sizeof(MasterStatus) == 30u, "MasterStatus size");
static_assert(offsetof(MasterStatus, robot_state) == 0u, "MasterStatus.robot_state offset");
static_assert(offsetof(MasterStatus, slave_alive) == 1u, "MasterStatus.slave_alive offset");
static_assert(offsetof(MasterStatus, uptime_ms) == 2u, "MasterStatus.uptime_ms offset");
static_assert(offsetof(MasterStatus, link_errors) == 6u, "MasterStatus.link_errors offset");
static_assert(offsetof(MasterStatus, rx_frames) == 10u, "MasterStatus.rx_frames offset");
static_assert(offsetof(MasterStatus, master_poll_hz) == 14u, "MasterStatus.master_poll_hz offset");
static_assert(offsetof(MasterStatus, telemetry_hz) == 16u, "MasterStatus.telemetry_hz offset");
static_assert(offsetof(MasterStatus, slave_tick_hz) == 18u, "MasterStatus.slave_tick_hz offset");
static_assert(offsetof(MasterStatus, host_cmd_hz) == 20u, "MasterStatus.host_cmd_hz offset");
static_assert(offsetof(MasterStatus, rx_resyncs) == 22u, "MasterStatus.rx_resyncs offset");
static_assert(offsetof(MasterStatus, rx_discarded_bytes) == 26u, "MasterStatus.rx_discarded_bytes offset");
static_assert(sizeof(SlaveStatus) == 18u, "SlaveStatus size");
static_assert(offsetof(SlaveStatus, slave_id) == 0u, "SlaveStatus.slave_id offset");
static_assert(offsetof(SlaveStatus, motors_alive) == 1u, "SlaveStatus.motors_alive offset");
static_assert(offsetof(SlaveStatus, uptime_ms) == 2u, "SlaveStatus.uptime_ms offset");
static_assert(offsetof(SlaveStatus, crc_errors) == 6u, "SlaveStatus.crc_errors offset");
static_assert(offsetof(SlaveStatus, cmd_crc_errors) == 10u, "SlaveStatus.cmd_crc_errors offset");
static_assert(offsetof(SlaveStatus, seq_gaps) == 14u, "SlaveStatus.seq_gaps offset");

// ── encode/decode: whole-struct memcpy (no packed-member references) ──
template <typename T>
inline std::array<std::uint8_t, sizeof(T)> to_bytes(const T& v) {
    std::array<std::uint8_t, sizeof(T)> out{};
    std::memcpy(out.data(), &v, sizeof(T));
    return out;
}

template <typename T>
inline T from_bytes(const std::uint8_t* buf) {
    T v{};
    std::memcpy(&v, buf, sizeof(T));
    return v;
}

}  // namespace robot_wire

#endif  // ROBOT_WIRE_GEN_HPP
