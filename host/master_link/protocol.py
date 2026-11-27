"""Canonical host-side protocol library for the soccer master link.

Hand-written ctypes mirror of ``firmware/common/include/protocol_wire.h`` —
every struct, enum, flag and scale, byte-for-byte. The cross-language fixture
test (host/tests/test_wire_fixture.py) compiles the C header, dumps every
struct filled with known values, and compares this module's decode field by
field, so drift between the two files fails CI.

Framing: ``[usb_header_t (8 B)][payload]`` over USB CDC, CRC16-CCITT (poly
0x1021, init 0xFFFF) over the header (crc zeroed) + payload. There is NO
version or schema hash — a size change fails the length/CRC checks; flash
every board together with a matching host checkout.
"""
from __future__ import annotations

import binascii
import ctypes
import struct

# ── limits / scales (protocol_wire.h) ─────────────────────────────────────────
MAX_SLAVES           = 4
MAX_MOTORS_PER_SLAVE = 5
CMD_SEQ_NONE         = 0

POS_SCALE  = 10000.0   # rad   → i16
VEL_SCALE  = 100.0     # rad/s → i16
TAU_SCALE  = 100.0     # N·m   → i16
KP_SCALE   = 10.0      # Kp    → u16
KD_SCALE   = 100.0     # Kd    → u16
TEMP_SCALE = 10.0      # °C    → i16 (motor-native 0.1 °C)

IMU_ACCEL_LSB_MPS2 = (9.81 * 3.0) / 32768.0            # ±3 g
IMU_GYRO_LSB_RPS   = (0.01745329251 * 1000.0) / 32768.0  # ±1000 °/s

# ── enums (u8 on the wire) ────────────────────────────────────────────────────
MOTOR_REQ_STOP   = 0
MOTOR_REQ_ENABLE = 1
MOTOR_REQ_NAMES  = {0: "STOP", 1: "ENABLE"}

SLAVE_REQ_NONE  = 0
SLAVE_REQ_RESET = 1
SLAVE_REQ_NAMES = {0: "NONE", 1: "RESET"}

MOTOR_ABSENT   = 0
MOTOR_DISABLED = 1
MOTOR_ENABLING = 2
MOTOR_ENABLED  = 3
MOTOR_STOPPING = 4
MOTOR_FAULT    = 5
MOTOR_STATE_NAMES = {0: "ABSENT", 1: "DISABLED", 2: "ENABLING", 3: "ENABLED",
                     4: "STOPPING", 5: "FAULT"}

MOTOR_FAULT_NONE             = 0
MOTOR_FAULT_FEEDBACK_TIMEOUT = 1
MOTOR_FAULT_ENABLE_FAILED    = 2
MOTOR_FAULT_LOST_ENABLE      = 3
MOTOR_FAULT_OVERTORQUE       = 4
MOTOR_FAULT_RS_REPORTED      = 5
MOTOR_FAULT_MASTER_LOST      = 6
MOTOR_FAULT_NAMES = {0: "NONE", 1: "FEEDBACK_TIMEOUT", 2: "ENABLE_FAILED",
                     3: "LOST_ENABLE", 4: "OVERTORQUE", 5: "RS_REPORTED",
                     6: "MASTER_LOST"}

RS_MODE_RESET = 0
RS_MODE_CALI  = 1
RS_MODE_RUN   = 2
RS_MODE_NAMES = {0: "RESET", 1: "CALI", 2: "RUN"}

SLAVE_BOOT      = 0
SLAVE_READY     = 1
SLAVE_BLOCKED   = 2
SLAVE_RESETTING = 3
SLAVE_STATE_NAMES = {0: "BOOT", 1: "READY", 2: "BLOCKED", 3: "RESETTING"}

ROBOT_BOOT     = 0
ROBOT_BLOCKED  = 1
ROBOT_READY    = 2
ROBOT_RUNNING  = 3
ROBOT_DEGRADED = 4
ROBOT_STATE_NAMES = {0: "BOOT", 1: "BLOCKED", 2: "READY", 3: "RUNNING",
                     4: "DEGRADED"}

USB_MSG_PING = 1
USB_MSG_CMD  = 2
USB_MSG_TELE = 3
MSG_NAMES = {1: "PING", 2: "CMD", 3: "TELE"}

SPI_OP_NOP = 0
SPI_OP_CMD = 1

# ── flags ─────────────────────────────────────────────────────────────────────
MOTOR_FLAG_READY        = 1 << 0
MOTOR_FLAG_REQ_REJECTED = 1 << 1
MOTOR_FLAG_CMD_STALE    = 1 << 2
MOTOR_FLAG_CLAMPED_POS  = 1 << 3
MOTOR_FLAG_CLAMPED_TAU  = 1 << 4
MOTOR_FLAG_CLAMPED_GAIN = 1 << 5
MOTOR_FLAG_SATURATED    = 1 << 6

RS_FAULT_UNDERVOLTAGE = 1 << 0
RS_FAULT_OVERCURRENT  = 1 << 1
RS_FAULT_OVERTEMP     = 1 << 2
RS_FAULT_ENCODER      = 1 << 3
RS_FAULT_STALL        = 1 << 4
RS_FAULT_UNCALIBRATED = 1 << 5

IMU_FLAG_INIT_OK = 1 << 0
IMU_FLAG_VALID   = 1 << 1

# ── packed-struct mirrors ─────────────────────────────────────────────────────
_u8, _u16, _u32, _i16 = (ctypes.c_uint8, ctypes.c_uint16, ctypes.c_uint32,
                         ctypes.c_int16)


class _Wire(ctypes.LittleEndianStructure):
    _pack_ = 1

    def __repr__(self):  # debugging aid; not on any hot path
        fs = ", ".join(f"{n}={getattr(self, n)!r}" for n, _ in self._fields_)
        return f"{type(self).__name__}({fs})"


class RsMit(_Wire):
    _fields_ = [("pos", _i16), ("vel", _i16), ("kp", _u16), ("kd", _u16),
                ("tau_ff", _i16)]


class CmdMotor(_Wire):
    _fields_ = [("req", _u8), ("mit", RsMit)]


class CmdSlave(_Wire):
    _fields_ = [("req", _u8), ("motors", CmdMotor * MAX_MOTORS_PER_SLAVE)]


class CmdMaster(_Wire):
    _fields_ = [("cycle_id", _u16), ("cmd_seq", _u16), ("test", _u8),
                ("slaves", CmdSlave * MAX_SLAVES)]


class RsFeedback(_Wire):
    _fields_ = [("pos", _i16), ("vel", _i16), ("tau", _i16), ("temp", _i16),
                ("mode", _u8), ("fault_bits", _u8), ("fault_word", _u32),
                ("time_slave_us", _u32)]


class TeleMotor(_Wire):
    _fields_ = [("state", _u8), ("fault", _u8), ("flags", _u8),
                ("last_applied_seq", _u16), ("rs", RsFeedback)]


class BootResult(_Wire):
    _fields_ = [("found_mask", _u8), ("unexpected_id", _u8),
                ("zero_sta_fail_mask", _u8), ("version_fail_mask", _u8)]


class SpiSlaveStats(_Wire):
    _fields_ = [("cmd_crc_errors", _u16), ("resyncs", _u16),
                ("tx_arm_fails", _u16), ("seq_echo", _u8)]


class CanStats(_Wire):
    _fields_ = [("tx_errors", _u16), ("tec", _u8), ("rec", _u8)]


class StatusSlave(_Wire):
    _fields_ = [("boot", BootResult), ("spi", SpiSlaveStats), ("can", CanStats)]


class TeleSlave(_Wire):
    _fields_ = [("time_slave_us", _u32), ("state", _u8), ("n_motors", _u8),
                ("status", StatusSlave),
                ("motors", TeleMotor * MAX_MOTORS_PER_SLAVE)]


class HostStats(_Wire):
    _fields_ = [("lost", _u8), ("last_cmd_seq_rx", _u16), ("on_time", _u16),
                ("late", _u16), ("missing", _u16), ("duplicate", _u16)]


class UsbStats(_Wire):
    _fields_ = [("rx_frames", _u16), ("rx_errors", _u16), ("rx_resyncs", _u16),
                ("rx_discarded_bytes", _u16), ("tx_drops", _u16)]


class SpiMasterStats(_Wire):
    _fields_ = [("alive", _u8), ("crc_errors", _u16), ("seq_gaps", _u16)]


class StatusMaster(_Wire):
    _fields_ = [("host", HostStats), ("usb", UsbStats),
                ("spi", SpiMasterStats * MAX_SLAVES),
                ("cycle_overruns", _u16)]


class Imu(_Wire):
    _fields_ = [("accel", _i16 * 3), ("gyro", _i16 * 3), ("seq", _u8),
                ("flags", _u8), ("errors", _u16)]


class TeleMaster(_Wire):
    _fields_ = [("cycle_id", _u16), ("time_master_us", _u32),
                ("cmd_seq_active", _u16), ("robot_state", _u8),
                ("n_slaves", _u8), ("status", StatusMaster), ("imu", Imu),
                ("slaves", TeleSlave * MAX_SLAVES)]


class UsbHeader(_Wire):
    _fields_ = [("type", _u16), ("seq", _u16), ("len", _u16), ("crc16", _u16)]


# ── size guards (mirror the header's static asserts) ──────────────────────────
_EXPECT_SIZES = {
    RsMit: 10, CmdMotor: 11, CmdSlave: 56, CmdMaster: 229,
    RsFeedback: 18, TeleMotor: 23, BootResult: 4, SpiSlaveStats: 7,
    CanStats: 4, StatusSlave: 15, TeleSlave: 136, HostStats: 11,
    UsbStats: 10, SpiMasterStats: 5, StatusMaster: 43, Imu: 16,
    TeleMaster: 613, UsbHeader: 8,
}
for _cls, _sz in _EXPECT_SIZES.items():
    assert ctypes.sizeof(_cls) == _sz, (_cls.__name__, ctypes.sizeof(_cls), _sz)

HDR_SIZE = ctypes.sizeof(UsbHeader)

_CMD_SLAVES_OFF  = CmdMaster.slaves.offset    # 5
_TELE_SLAVES_OFF = TeleMaster.slaves.offset   # 69


def cmd_master_size(n_slaves: int) -> int:
    """CMD_MASTER_SIZE(n) — the exact command payload length the master accepts."""
    return _CMD_SLAVES_OFF + n_slaves * ctypes.sizeof(CmdSlave)


def tele_master_size(n_slaves: int) -> int:
    """TELE_MASTER_SIZE(n) — the truncated telemetry payload length."""
    return _TELE_SLAVES_OFF + n_slaves * ctypes.sizeof(TeleSlave)


USB_MAX_PAYLOAD = ctypes.sizeof(TeleMaster)   # 613


def seq_ge(a: int, b: int) -> bool:
    """Wrap-aware uint16 ``a >= b`` (RFC-1982). True when ``a`` is at or ahead of
    ``b`` within half the 16-bit space, so a counter wrapping 65535→1 still
    compares correctly. Used to match last_applied_seq against a cmd_seq."""
    return ((a - b) & 0xFFFF) < 0x8000


# ── fixed-point encode/decode (mirror proto_f_to_* in protocol.h) ─────────────
def enc_i16(x: float, scale: float) -> tuple[int, bool]:
    """(value, saturated) — round half away from zero, saturate at i16 limits."""
    v = x * scale
    v = v + 0.5 if v >= 0 else v - 0.5
    iv = int(v)
    if iv > 32767:
        return 32767, True
    if iv < -32768:
        return -32768, True
    return iv, False


def enc_u16(x: float, scale: float) -> tuple[int, bool]:
    v = int(x * scale + 0.5)
    if v > 65535:
        return 65535, True
    if v < 0:
        return 0, False
    return v, False


def dec_i16(r: int, scale: float) -> float: return r / scale
def dec_u16(r: int, scale: float) -> float: return r / scale


# ── command pack / parse ──────────────────────────────────────────────────────
def pack_cmd_master(cycle_id: int, cmd_seq: int, slaves: list, *,
                    test: int = 0) -> bytes:
    """Build the exact CMD_MASTER_SIZE(len(slaves)) payload.

    ``slaves`` is one dict per CONFIGURED slave, in slave order:
        {"req": SLAVE_REQ_*, "motors": [ {req, pos, vel, kp, kd, tau_ff}, ... ]}
    Unspecified motor slots are zero (= MOTOR_REQ_STOP, limp mit). ``test``
    carries the master test sentinels (0xDE SPI-desync, 0xB0 USB-TX test)."""
    cm = CmdMaster()
    cm.cycle_id = cycle_id & 0xFFFF
    cm.cmd_seq  = cmd_seq & 0xFFFF
    cm.test     = test & 0xFF
    for si, sl in enumerate(slaves):
        cs = cm.slaves[si]
        cs.req = sl.get("req", SLAVE_REQ_NONE) & 0xFF
        for mi, m in enumerate(sl.get("motors", [])):
            mo = cs.motors[mi]
            mo.req = m.get("req", MOTOR_REQ_STOP) & 0xFF
            mo.mit.pos,    _ = enc_i16(m.get("pos", 0.0), POS_SCALE)
            mo.mit.vel,    _ = enc_i16(m.get("vel", 0.0), VEL_SCALE)
            mo.mit.kp,     _ = enc_u16(m.get("kp", 0.0), KP_SCALE)
            mo.mit.kd,     _ = enc_u16(m.get("kd", 0.0), KD_SCALE)
            mo.mit.tau_ff, _ = enc_i16(m.get("tau_ff", 0.0), TAU_SCALE)
    return bytes(cm)[:cmd_master_size(len(slaves))]


def parse_cmd_master(p: bytes) -> CmdMaster | None:
    """Decode a cmd_master_t payload (any truncation ≥ the fixed prefix).
    Returns the ctypes struct; the caller knows the configured slave count."""
    if len(p) < _CMD_SLAVES_OFF:
        return None
    return CmdMaster.from_buffer_copy(p.ljust(ctypes.sizeof(CmdMaster), b"\x00"))


# ── telemetry pack / parse ────────────────────────────────────────────────────
def parse_tele_master(p: bytes) -> TeleMaster | None:
    """Decode a tele_master_t payload (sent truncated to n_slaves). Returns the
    ctypes struct — read ``slaves[:tm.n_slaves]``; absent slots are zeroed."""
    if len(p) < _TELE_SLAVES_OFF:
        return None
    tm = TeleMaster.from_buffer_copy(p.ljust(ctypes.sizeof(TeleMaster), b"\x00"))
    if len(p) < tele_master_size(min(tm.n_slaves, MAX_SLAVES)):
        return None
    return tm


def pack_tele_master(tm: TeleMaster) -> bytes:
    """Serialize a TeleMaster to its truncated wire form (tests/simulation)."""
    return bytes(tm)[:tele_master_size(min(tm.n_slaves, MAX_SLAVES))]


# ── CRC16-CCITT (must match firmware proto_crc16) ─────────────────────────────
def crc16(data: bytes) -> int:
    """CRC16-CCITT (poly 0x1021, init 0xFFFF), identical to firmware proto_crc16.
    binascii.crc_hqx is the same polynomial in C — verified byte-identical."""
    return binascii.crc_hqx(data, 0xFFFF)


# ── frame encode / decode ─────────────────────────────────────────────────────
_HDR = struct.Struct("<HHHH")   # type, seq, len, crc16
_seq = 0


def encode_frame(msg_type: int, payload: bytes = b"", seq: int | None = None) -> bytes:
    """Build one USB frame. ``seq`` defaults to a module-global per-sender
    counter (a PING reply passes the request's seq explicitly)."""
    global _seq
    if seq is None:
        seq = _seq
        _seq = (_seq + 1) & 0xFFFF
    frame = bytearray(_HDR.pack(msg_type, seq, len(payload), 0)) + payload
    checksum = crc16(bytes(frame))
    frame[6] = checksum & 0xFF
    frame[7] = (checksum >> 8) & 0xFF
    return bytes(frame)


def decode_frame(buf: bytearray, on_reject=None, on_frame=None):
    """Decode one frame from ``buf``. Returns ``(msg_type, seq, payload,
    consumed)`` or ``None``; the caller removes ``consumed`` accepted bytes.
    Drops one byte and retries on an unknown type / implausible length / bad
    CRC (the resynchronizing scan, mirroring firmware proto_frame_scan).
    ``on_reject("discard", bytes)`` sees coalesced dropped bytes;
    ``on_frame(raw)`` sees each whole accepted frame."""
    discarded = bytearray()

    def _flush_discard():
        if discarded and on_reject is not None:
            on_reject("discard", bytes(discarded))
        del discarded[:]

    while len(buf) >= HDR_SIZE:
        msg_type, seq, pay_len, crc_wire = _HDR.unpack_from(buf, 0)
        if msg_type not in MSG_NAMES or pay_len > USB_MAX_PAYLOAD:
            discarded.append(buf[0])
            del buf[0]
            continue
        total = HDR_SIZE + pay_len
        if len(buf) < total:
            _flush_discard()
            return None
        check = bytearray(buf[:total])
        check[6] = 0
        check[7] = 0
        if crc16(bytes(check)) != crc_wire:
            discarded.append(buf[0])
            del buf[0]
            continue
        _flush_discard()
        if on_frame is not None:
            on_frame(bytes(buf[:total]))
        payload = bytes(buf[HDR_SIZE:total])
        return (msg_type, seq, payload, total)
    _flush_discard()
    return None
