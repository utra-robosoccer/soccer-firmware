"""AUTO-GENERATED — DO NOT EDIT.

Source:      firmware/common/protocol.yaml
Regenerate:  python3 scripts/gen_protocol.py

SCHEMA_HASH 0xE25161C4: CRC-32 of the schema's canonical wire projection
(parsed YAML minus doc/unit/hex keys, compact sorted-key JSON, UTF-8) — see
gen_protocol.py. Doc-only edits do not change it.

Layout data for the little-endian packed wire structs, plus every enum
value (PLAIN INTS, not IntEnum — str() of an IntEnum changed in Python
3.11, which would silently alter stringified CSV/log output), flag bit,
fixed-point scale and wire constant. master_link.protocol is the public
API and re-exports from here; consumers should not import this directly."""
import struct as _struct

PROTO_VERSION = 10
SCHEMA_HASH = 0xE25161C4

# ── wire constants ──
CMD_SEQ_NONE = 0
MAX_MOTORS_PER_CHAIN = 5
MAX_CHAINS = 4
PROTO_CRC_INIT = 0xFFFF
PROTO_CRC_POLY = 0x1021
SPI_OP_NOP = 0x00
SPI_OP_ROBOT_CMD = 0x01
SPI_CMD_HDR_BYTES = 6
SPI_CRC_BYTES = 2
USB_TX_SLOT_MAX = 512

# ── NodeId (u8 on the wire) ──
NODE_JETSON = 1
NODE_MASTER = 2
NODE_SLAVE_0 = 3
NODE_SLAVE_1 = 4
NODE_ID_VALUE_NAMES = {1: "NODE_JETSON", 2: "NODE_MASTER", 3: "NODE_SLAVE_0", 4: "NODE_SLAVE_1"}

# ── MsgType (u16 on the wire) ──
MSG_PING = 0x01
MSG_MASTER_STATUS = 0x02
MSG_SLAVE_STATUS = 0x03
MSG_ROBOT_CMD = 0x08
MSG_ROBOT_TELE = 0x09
MSG_TYPE_VALUE_NAMES = {1: "MSG_PING", 2: "MSG_MASTER_STATUS", 3: "MSG_SLAVE_STATUS", 8: "MSG_ROBOT_CMD", 9: "MSG_ROBOT_TELE"}

# ── RobotState (u8 on the wire) ──
ROBOT_INIT = 0
ROBOT_READY = 1
ROBOT_DEGRADED = 2
ROBOT_HOST_LOST = 3
ROBOT_PROTO_MISMATCH = 4
ROBOT_STATE_VALUE_NAMES = {0: "ROBOT_INIT", 1: "ROBOT_READY", 2: "ROBOT_DEGRADED", 3: "ROBOT_HOST_LOST", 4: "ROBOT_PROTO_MISMATCH"}

# ── DriveReq (u8 on the wire) ──
DRV_REQ_DISABLE = 0
DRV_REQ_ENABLE = 1
DRV_REQ_STOP = 2
DRV_REQ_RESET = 3
DRIVE_REQ_VALUE_NAMES = {0: "DRV_REQ_DISABLE", 1: "DRV_REQ_ENABLE", 2: "DRV_REQ_STOP", 3: "DRV_REQ_RESET"}

# ── DriveState (u8 on the wire) ──
DRV_BOOT = 0
DRV_DISCOVERING = 1
DRV_ABSENT = 2
DRV_DISABLED = 3
DRV_ENABLING = 4
DRV_ENABLED = 5
DRV_STOPPING = 6
DRV_CLEARING = 7
DRV_FAULT = 8
DRIVE_STATE_VALUE_NAMES = {0: "DRV_BOOT", 1: "DRV_DISCOVERING", 2: "DRV_ABSENT", 3: "DRV_DISABLED", 4: "DRV_ENABLING", 5: "DRV_ENABLED", 6: "DRV_STOPPING", 7: "DRV_CLEARING", 8: "DRV_FAULT"}

# ── ChainLink (u8 on the wire) ──
CHAIN_LINK_LOST = 0
CHAIN_LINK_ALIVE = 1
CHAIN_LINK_MISMATCH = 2
CHAIN_LINK_VALUE_NAMES = {0: "CHAIN_LINK_LOST", 1: "CHAIN_LINK_ALIVE", 2: "CHAIN_LINK_MISMATCH"}

# ── DriveFault (u8 on the wire) ──
DRVF_NONE = 0
DRVF_CAN_TIMEOUT = 1
DRVF_MASTER_LOST = 2
DRVF_ENABLE_FAILED = 3
DRVF_NOT_ENABLED = 4
DRVF_OVERTORQUE = 5
DRVF_MOTOR_REPORTED = 6
DRVF_WOUND = 7
DRIVE_FAULT_VALUE_NAMES = {0: "DRVF_NONE", 1: "DRVF_CAN_TIMEOUT", 2: "DRVF_MASTER_LOST", 3: "DRVF_ENABLE_FAILED", 4: "DRVF_NOT_ENABLED", 5: "DRVF_OVERTORQUE", 6: "DRVF_MOTOR_REPORTED", 7: "DRVF_WOUND"}

# ── cmd_motor_flags ──
CMD_FLAG_VALID = 1 << 0
CMD_FLAG_USE_CONFIG_GAINS = 1 << 1
# bit 2 reserved — was CMD_FLAG_FAULT_RESET (v8); RESET is a drive request in v9

# ── tele_motor_flags ──
TELE_FLAG_REQUEST_REJECTED = 1 << 0
# bit 1 reserved — was TELE_FLAG_TO_ZERO_ARRIVED (v8); TO_ZERO is host-side in v9
TELE_FLAG_SATURATED = 1 << 2
TELE_FLAG_CLAMPED_POS = 1 << 3
TELE_FLAG_CLAMPED_TAU = 1 << 4
TELE_FLAG_CMD_STALE = 1 << 5
TELE_FLAG_CLAMPED_GAIN = 1 << 6

# ── fixed-point scales ──
PROTO_POS_SCALE = 10000.0
PROTO_VEL_SCALE = 100.0
PROTO_TAU_SCALE = 100.0
PROTO_KP_SCALE = 10.0
PROTO_KD_SCALE = 100.0

# ── struct formats ('<' = little-endian, packed) ──
# Nested structs (a trailing struct array) get a _PREFIX format covering
# the scalar fields before the array; SZ_* is always the full wire size.
FMT_MSG_HEADER = "<HHBBIHHH"
SZ_MSG_HEADER = 16
OFF_MSG_HEADER = {"type": 0, "seq": 2, "src": 4, "dst": 5, "ts_ms": 6, "len": 10, "ver_flags": 12, "crc16": 14}
FMT_CMD_MOTOR = "<BhhHHhB"
SZ_CMD_MOTOR = 12
OFF_CMD_MOTOR = {"drive_req": 0, "pos": 1, "vel": 3, "kp": 5, "kd": 7, "tau_ff": 9, "flags": 11}
FMT_CMD_CHAIN_PREFIX = "<BB"
SZ_CMD_CHAIN = 62
OFF_CMD_CHAIN = {"chain_id": 0, "n_motors": 1, "motors": 2}
FMT_CMD_ROBOT_PREFIX = "<HHBB"
SZ_CMD_ROBOT = 254
OFF_CMD_ROBOT = {"cycle_id": 0, "cmd_seq": 2, "n_chains": 4, "reserved": 5, "chains": 6}
FMT_TELE_MOTOR = "<hhhBBBBBBBIHH"
SZ_TELE_MOTOR = 21
OFF_TELE_MOTOR = {"pos": 0, "vel": 2, "tau": 4, "temp_c": 6, "drive_state": 7, "drive_fault": 8, "rs_mode": 9, "rs_fault_bits": 10, "flags": 11, "fb_age_ms": 12, "rs_fault_word": 13, "last_applied_seq": 17, "reserved": 19}
FMT_TELE_CHAIN_PREFIX = "<BBBBBIHHB"
SZ_TELE_CHAIN = 119
OFF_TELE_CHAIN = {"chain_id": 0, "n_motors": 1, "link": 2, "spi_seq_echo": 3, "spi_resyncs": 4, "slave_time_us": 5, "cmd_crc_errors": 9, "can_tx_errors": 11, "spi_tx_arm_fails": 13, "motors": 14}
FMT_TELE_ROBOT_PREFIX = "<HIHHHHHHBB"
SZ_TELE_ROBOT = 496
OFF_TELE_ROBOT = {"cycle_id": 0, "master_time_us": 2, "last_cmd_seq_rx": 6, "cmd_seq_active": 8, "cmd_on_time": 10, "cmd_late": 12, "cmd_missing": 14, "cmd_duplicate": 16, "n_chains": 18, "robot_state": 19, "chains": 20}
FMT_MASTER_STATUS = "<IBIIIHHHHII"
SZ_MASTER_STATUS = 33
OFF_MASTER_STATUS = {"schema_hash": 0, "slave_proto_mismatch": 4, "uptime_ms": 5, "link_errors": 9, "rx_frames": 13, "master_poll_hz": 17, "telemetry_hz": 19, "slave_tick_hz": 21, "host_cmd_hz": 23, "rx_resyncs": 25, "rx_discarded_bytes": 29}
FMT_SPI_TRAILER = "<BI"
SZ_SPI_TRAILER = 5
OFF_SPI_TRAILER = {"proto_version": 0, "schema_hash": 1}
FMT_SLAVE_STATUS = "<BBIIII"
SZ_SLAVE_STATUS = 18
OFF_SLAVE_STATUS = {"slave_id": 0, "motors_alive": 1, "uptime_ms": 2, "crc_errors": 6, "cmd_crc_errors": 10, "seq_gaps": 14}

# ── derived wire constants ──
MSG_HEADER_SIZE = 16   # sizeof(MsgHeader)
PROTO_MAX_PAYLOAD = 496   # sizeof(tele_robot_t)
SPI_CMD_TRAILER_OFF = 68   # SPI_CMD_HDR_BYTES + sizeof(cmd_chain_t)
SPI_CMD_CRC_OFF = 73   # SPI_CMD_HDR_BYTES + sizeof(cmd_chain_t) + sizeof(spi_trailer_t)
SPI_CMD_FRAME_SIZE = 75   # SPI_CMD_HDR_BYTES + sizeof(cmd_chain_t) + sizeof(spi_trailer_t) + SPI_CRC_BYTES
SPI_TELE_TRAILER_OFF = 119   # sizeof(tele_chain_t)
SPI_TELE_CRC_OFF = 124   # sizeof(tele_chain_t) + sizeof(spi_trailer_t)
SPI_TELE_FRAME_SIZE = 126   # sizeof(tele_chain_t) + sizeof(spi_trailer_t) + SPI_CRC_BYTES
SPI_XFER_SIZE = 126   # max(SPI_CMD_FRAME_SIZE, SPI_TELE_FRAME_SIZE)
USB_MAX_FRAME_SIZE = 512   # sizeof(MsgHeader) + sizeof(tele_robot_t)

# ── generic pack/unpack (field tables + recursive helpers) ──
# STRUCTS[name] = [(field, 'scalar', fmt) | (field, 'array', struct, n)]
STRUCTS = {
    "MsgHeader": [('type', 'scalar', 'H'), ('seq', 'scalar', 'H'), ('src', 'scalar', 'B'), ('dst', 'scalar', 'B'), ('ts_ms', 'scalar', 'I'), ('len', 'scalar', 'H'), ('ver_flags', 'scalar', 'H'), ('crc16', 'scalar', 'H')],
    "cmd_motor_t": [('drive_req', 'scalar', 'B'), ('pos', 'scalar', 'h'), ('vel', 'scalar', 'h'), ('kp', 'scalar', 'H'), ('kd', 'scalar', 'H'), ('tau_ff', 'scalar', 'h'), ('flags', 'scalar', 'B')],
    "cmd_chain_t": [('chain_id', 'scalar', 'B'), ('n_motors', 'scalar', 'B'), ('motors', 'array', 'cmd_motor_t', 5)],
    "cmd_robot_t": [('cycle_id', 'scalar', 'H'), ('cmd_seq', 'scalar', 'H'), ('n_chains', 'scalar', 'B'), ('reserved', 'scalar', 'B'), ('chains', 'array', 'cmd_chain_t', 4)],
    "tele_motor_t": [('pos', 'scalar', 'h'), ('vel', 'scalar', 'h'), ('tau', 'scalar', 'h'), ('temp_c', 'scalar', 'B'), ('drive_state', 'scalar', 'B'), ('drive_fault', 'scalar', 'B'), ('rs_mode', 'scalar', 'B'), ('rs_fault_bits', 'scalar', 'B'), ('flags', 'scalar', 'B'), ('fb_age_ms', 'scalar', 'B'), ('rs_fault_word', 'scalar', 'I'), ('last_applied_seq', 'scalar', 'H'), ('reserved', 'scalar', 'H')],
    "tele_chain_t": [('chain_id', 'scalar', 'B'), ('n_motors', 'scalar', 'B'), ('link', 'scalar', 'B'), ('spi_seq_echo', 'scalar', 'B'), ('spi_resyncs', 'scalar', 'B'), ('slave_time_us', 'scalar', 'I'), ('cmd_crc_errors', 'scalar', 'H'), ('can_tx_errors', 'scalar', 'H'), ('spi_tx_arm_fails', 'scalar', 'B'), ('motors', 'array', 'tele_motor_t', 5)],
    "tele_robot_t": [('cycle_id', 'scalar', 'H'), ('master_time_us', 'scalar', 'I'), ('last_cmd_seq_rx', 'scalar', 'H'), ('cmd_seq_active', 'scalar', 'H'), ('cmd_on_time', 'scalar', 'H'), ('cmd_late', 'scalar', 'H'), ('cmd_missing', 'scalar', 'H'), ('cmd_duplicate', 'scalar', 'H'), ('n_chains', 'scalar', 'B'), ('robot_state', 'scalar', 'B'), ('chains', 'array', 'tele_chain_t', 4)],
    "MasterStatus": [('schema_hash', 'scalar', 'I'), ('slave_proto_mismatch', 'scalar', 'B'), ('uptime_ms', 'scalar', 'I'), ('link_errors', 'scalar', 'I'), ('rx_frames', 'scalar', 'I'), ('master_poll_hz', 'scalar', 'H'), ('telemetry_hz', 'scalar', 'H'), ('slave_tick_hz', 'scalar', 'H'), ('host_cmd_hz', 'scalar', 'H'), ('rx_resyncs', 'scalar', 'I'), ('rx_discarded_bytes', 'scalar', 'I')],
    "spi_trailer_t": [('proto_version', 'scalar', 'B'), ('schema_hash', 'scalar', 'I')],
    "SlaveStatus": [('slave_id', 'scalar', 'B'), ('motors_alive', 'scalar', 'B'), ('uptime_ms', 'scalar', 'I'), ('crc_errors', 'scalar', 'I'), ('cmd_crc_errors', 'scalar', 'I'), ('seq_gaps', 'scalar', 'I')],
}


def unpack_struct(name, data, off=0):
    """Decode one wire struct into a dict (struct arrays → lists of dicts).
    Raw wire values only — no fixed-point decode, no truncation handling."""
    out = {}
    for row in STRUCTS[name]:
        if row[1] == "scalar":
            fmt = "<" + row[2]
            vals = _struct.unpack_from(fmt, data, off)
            out[row[0]] = vals[0] if len(vals) == 1 else list(vals)
            off += _struct.calcsize(fmt)
        else:
            _, _, sub, n = row
            sz = SZ_BY_STRUCT[sub]
            out[row[0]] = [unpack_struct(sub, data, off + i * sz)
                           for i in range(n)]
            off += n * sz
    return out


def pack_struct(name, values):
    """Inverse of unpack_struct. Missing fields pack as zero; a short
    struct-array list is zero-filled to the fixed wire length."""
    out = bytearray()
    for row in STRUCTS[name]:
        if row[1] == "scalar":
            fmt = "<" + row[2]
            v = values.get(row[0], 0)
            out += _struct.pack(fmt, *v) if isinstance(v, (list, tuple)) \
                else _struct.pack(fmt, v)
        else:
            _, _, sub, n = row
            elems = values.get(row[0], [])
            for i in range(n):
                out += pack_struct(sub, elems[i] if i < len(elems) else {})
    return bytes(out)


SZ_BY_STRUCT = {"MsgHeader": SZ_MSG_HEADER, "cmd_motor_t": SZ_CMD_MOTOR, "cmd_chain_t": SZ_CMD_CHAIN, "cmd_robot_t": SZ_CMD_ROBOT, "tele_motor_t": SZ_TELE_MOTOR, "tele_chain_t": SZ_TELE_CHAIN, "tele_robot_t": SZ_TELE_ROBOT, "MasterStatus": SZ_MASTER_STATUS, "spi_trailer_t": SZ_SPI_TRAILER, "SlaveStatus": SZ_SLAVE_STATUS}
