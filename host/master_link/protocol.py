"""Canonical host-side protocol library for the soccer master link.

Single source of truth for the master↔host USB wire format. Mirrors the firmware
in ``firmware/common/include/protocol.h`` byte-for-byte: a raw
``[MsgHeader(16 B)][payload]`` frame over USB CDC, CRC16-CCITT (poly 0x1021, init
0xFFFF) over the header (crc zeroed) + payload. Not COBS-framed.

PROTO_VERSION 3 introduced the robot/chain/motor hierarchy: the host sends one
``cmd_robot_t`` per tick (``MSG_ROBOT_CMD``) and the master returns one
``tele_robot_t`` per telemetry tick (``MSG_ROBOT_TELE``). Scalar fields are
fixed-point (shared scales below), replacing the old bounds-based u16 mapping.
"""
from __future__ import annotations

import binascii
from dataclasses import dataclass
import struct
import time

# ── node ids ──────────────────────────────────────────────────────────────────
NODE_JETSON  = 1
NODE_MASTER  = 2
NODE_SLAVE_0 = 3

# ── message types ─────────────────────────────────────────────────────────────
MSG_PING          = 0x01
MSG_MASTER_STATUS = 0x02
MSG_SLAVE_STATUS  = 0x03
MSG_ROBOT_CMD     = 0x08
MSG_ROBOT_TELE    = 0x09

MSG_NAMES = {
    MSG_PING: "PING", MSG_MASTER_STATUS: "MASTER_STATUS",
    MSG_SLAVE_STATUS: "SLAVE_STATUS", MSG_ROBOT_CMD: "ROBOT_CMD",
    MSG_ROBOT_TELE: "ROBOT_TELE",
}

# ── hierarchy caps (mirror protocol.h) ────────────────────────────────────────
MAX_MOTORS_PER_CHAIN = 5
MAX_CHAINS           = 4

# ── per-motor lifecycle (tele_motor_t.state) ──────────────────────────────────
(LIFE_BOOT, LIFE_DISCOVERING, LIFE_IDLE, LIFE_HOLD, LIFE_MIT, LIFE_DAMPED,
 LIFE_TO_ZERO, LIFE_FAULT) = range(8)
LIFECYCLE_NAMES = {
    0: "BOOT", 1: "DISCOVERING", 2: "IDLE", 3: "HOLD", 4: "MIT",
    5: "DAMPED", 6: "TO_ZERO", 7: "FAULT",
}

# ── host→slave mode requests (cmd_motor_t.mode_req) ───────────────────────────
(REQ_IDLE, REQ_HOLD, REQ_MIT, REQ_DAMPED, REQ_TO_ZERO) = range(5)
MODE_REQ_NAMES = {0: "IDLE", 1: "HOLD", 2: "MIT", 3: "DAMPED", 4: "TO_ZERO"}

# ── RS motor run_mode reported in Type-2 (tele_motor_t.motor_mode) ────────────
MOTOR_MODE_NAMES = {0: "RESET", 1: "CAL", 2: "NORMAL"}

# ── fault cause (tele_motor_t.cause, latched) ─────────────────────────────────
(CAUSE_NONE, CAUSE_OVERTORQUE, CAUSE_CAN_TIMEOUT, CAUSE_WATCHDOG,
 CAUSE_MOTOR_FAULT, CAUSE_ZERO_TIMEOUT, CAUSE_NOT_ENABLED, CAUSE_WOUND) = range(8)
CAUSE_NAMES = {
    0: "NONE", 1: "OVERTORQUE", 2: "CAN_TIMEOUT", 3: "WATCHDOG", 4: "MOTOR_FAULT",
    5: "ZERO_TIMEOUT", 6: "NOT_ENABLED", 7: "WOUND",
}

ROBOT_STATE_NAMES = {0: "INIT", 1: "READY", 2: "DEGRADED"}

# ── cmd_motor_t.flags ─────────────────────────────────────────────────────────
CMD_FLAG_VALID            = 1 << 0
CMD_FLAG_USE_CONFIG_GAINS = 1 << 1
CMD_FLAG_FAULT_RESET      = 1 << 2

# ── tele_motor_t.flags ────────────────────────────────────────────────────────
TELE_FLAG_REQUEST_REJECTED = 1 << 0
TELE_FLAG_TO_ZERO_ARRIVED  = 1 << 1
TELE_FLAG_SATURATED        = 1 << 2
TELE_FLAG_CLAMPED_POS      = 1 << 3
TELE_FLAG_CLAMPED_TAU      = 1 << 4
TELE_FLAG_CMD_STALE        = 1 << 5

# ── fixed-point scales (mirror protocol.h exactly) ────────────────────────────
POS_SCALE = 10000.0   # rad   → i16 (home-frame ±π)
VEL_SCALE = 100.0     # rad/s → i16
TAU_SCALE = 100.0     # N·m   → i16
KP_SCALE  = 10.0      # Kp    → u16
KD_SCALE  = 100.0     # Kd    → u16

# ── wire-contract version ─────────────────────────────────────────────────────
PROTO_VERSION = 5   # v5: +rx_resyncs/rx_discarded_bytes in MasterStatus (resync RX)
                    # v4: MAX_CHAINS 6→4 (4 slave chains) shrank the robot frames

# ── wire layouts (little-endian, packed) ──────────────────────────────────────
HDR_FMT  = "<HHBBIHHH"          # type,seq,src,dst,ts_ms,len,ver_flags,crc  → 16 B
HDR_SIZE = struct.calcsize(HDR_FMT)

FMT_MASTER_STATUS = "<BBIIIHHHHII" # +4 u16 rates +rx_resyncs,rx_discarded_bytes → 30 B
FMT_SLAVE_STATUS  = "<BBIIII"     # slave_id,motors_alive,uptime,crc,cmd_crc,seq_gaps (18)

FMT_CMD_MOTOR     = "<BhhHHhB"    # mode_req,pos,vel,kp,kd,tau_ff,flags → 12 B
SZ_CMD_MOTOR      = struct.calcsize(FMT_CMD_MOTOR)
FMT_CMD_CHAIN_HDR = "<BB"         # chain_id,n_motors
SZ_CMD_CHAIN      = 2 + SZ_CMD_MOTOR * MAX_MOTORS_PER_CHAIN          # 62
FMT_CMD_ROBOT_HDR = "<HHBB"       # cycle_id,cmd_seq,n_chains,reserved
SZ_CMD_ROBOT      = 6 + SZ_CMD_CHAIN * MAX_CHAINS                    # 254 (6 + 62*4)

FMT_TELE_MOTOR    = "<hhhBBBBBBBIHH"  # pos,vel,tau,temp,state,cause,mode,fault,flags,fb_age,fault_word,last_applied_seq,reserved → 21
SZ_TELE_MOTOR     = struct.calcsize(FMT_TELE_MOTOR)
FMT_TELE_CHAIN_HDR = "<BBBBIHH"   # chain_id,n_motors,spi_seq_echo,spi_resyncs,slave_time_us,cmd_crc_errors,can_tx_errors
SZ_TELE_CHAIN     = 12 + SZ_TELE_MOTOR * MAX_MOTORS_PER_CHAIN        # 117
FMT_TELE_ROBOT_HDR = "<HIHHBB"    # cycle_id,master_time_us,last_cmd_seq_rx,missed_deadlines,n_chains,robot_state
SZ_TELE_ROBOT     = 12 + SZ_TELE_CHAIN * MAX_CHAINS                  # 480 (12 + 117*4)

assert HDR_SIZE == 16, HDR_SIZE
assert SZ_CMD_MOTOR == 12, SZ_CMD_MOTOR
assert SZ_CMD_CHAIN == 62, SZ_CMD_CHAIN
assert SZ_CMD_ROBOT == 254, SZ_CMD_ROBOT
assert SZ_TELE_MOTOR == 21, SZ_TELE_MOTOR
assert SZ_TELE_CHAIN == 117, SZ_TELE_CHAIN
assert SZ_TELE_ROBOT == 480, SZ_TELE_ROBOT
assert struct.calcsize(FMT_MASTER_STATUS) == 30
assert struct.calcsize(FMT_SLAVE_STATUS) == 18


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


# ── decoded telemetry dataclass ───────────────────────────────────────────────
@dataclass(frozen=True)
class TeleMotor:
    pos_raw: int
    vel_raw: int
    tau_raw: int
    temp_c: int
    state: int
    cause: int
    motor_mode: int
    motor_fault: int
    flags: int
    fb_age_ms: int
    fault_word: int
    last_applied_seq: int
    reserved: int

    @classmethod
    def unpack(cls, data: bytes, off: int = 0) -> "TeleMotor":
        return cls(*struct.unpack_from(FMT_TELE_MOTOR, data, off))

    @property
    def pos(self) -> float: return dec_i16(self.pos_raw, POS_SCALE)
    @property
    def vel(self) -> float: return dec_i16(self.vel_raw, VEL_SCALE)
    @property
    def tau(self) -> float: return dec_i16(self.tau_raw, TAU_SCALE)
    @property
    def temp(self) -> float: return float(self.temp_c)
    @property
    def lifecycle(self) -> int: return self.state
    @property
    def lifecycle_name(self) -> str: return LIFECYCLE_NAMES.get(self.state, f"?{self.state}")
    @property
    def cause_name(self) -> str: return CAUSE_NAMES.get(self.cause, f"?{self.cause}")
    @property
    def motor_mode_name(self) -> str: return MOTOR_MODE_NAMES.get(self.motor_mode, f"?{self.motor_mode}")

    @property
    def request_rejected(self) -> bool: return bool(self.flags & TELE_FLAG_REQUEST_REJECTED)
    @property
    def to_zero_arrived(self) -> bool: return bool(self.flags & TELE_FLAG_TO_ZERO_ARRIVED)
    @property
    def saturated(self) -> bool: return bool(self.flags & TELE_FLAG_SATURATED)
    @property
    def clamped_pos(self) -> bool: return bool(self.flags & TELE_FLAG_CLAMPED_POS)
    @property
    def clamped_tau(self) -> bool: return bool(self.flags & TELE_FLAG_CLAMPED_TAU)
    @property
    def cmd_stale(self) -> bool: return bool(self.flags & TELE_FLAG_CMD_STALE)


# ── command pack ──────────────────────────────────────────────────────────────
def pack_cmd_motor(mode_req: int, pos: float = 0.0, vel: float = 0.0,
                   kp: float = 0.0, kd: float = 0.0, tau_ff: float = 0.0,
                   flags: int = 0) -> bytes:
    """Pack one cmd_motor_t (12 B); sets TELE-side saturation is the slave's job,
    but we still clamp here so the wire value is well-defined."""
    p, _ = enc_i16(pos, POS_SCALE)
    v, _ = enc_i16(vel, VEL_SCALE)
    kpi, _ = enc_u16(kp, KP_SCALE)
    kdi, _ = enc_u16(kd, KD_SCALE)
    t, _ = enc_i16(tau_ff, TAU_SCALE)
    return struct.pack(FMT_CMD_MOTOR, mode_req & 0xFF, p, v, kpi, kdi, t, flags & 0xFF)


def pack_robot_cmd(cycle_id: int, cmd_seq: int, chains: list) -> bytes:
    """Build a full cmd_robot_t (248 B).

    ``chains`` is a list (≤ MAX_CHAINS) of dicts:
        {"chain_id": int, "motors": [ {mode_req, pos, vel, kp, kd, tau_ff, flags}, ... ]}
    Unused chain/motor slots are zero-filled (invalid)."""
    out = bytearray()
    n_chains = len(chains)
    out += struct.pack(FMT_CMD_ROBOT_HDR, cycle_id & 0xFFFF, cmd_seq & 0xFFFF,
                       n_chains & 0xFF, 0)
    for ci in range(MAX_CHAINS):
        if ci < n_chains:
            ch = chains[ci]
            motors = ch["motors"]
            out += struct.pack(FMT_CMD_CHAIN_HDR, ch["chain_id"] & 0xFF, len(motors) & 0xFF)
            for mi in range(MAX_MOTORS_PER_CHAIN):
                if mi < len(motors):
                    m = motors[mi]
                    out += pack_cmd_motor(
                        m.get("mode_req", REQ_IDLE), m.get("pos", 0.0), m.get("vel", 0.0),
                        m.get("kp", 0.0), m.get("kd", 0.0), m.get("tau_ff", 0.0),
                        m.get("flags", 0))
                else:
                    out += b"\x00" * SZ_CMD_MOTOR
        else:
            out += b"\x00" * SZ_CMD_CHAIN
    assert len(out) == SZ_CMD_ROBOT, len(out)
    return bytes(out)


def parse_robot_cmd(p: bytes) -> dict:
    """Decode a cmd_robot_t payload (for round-trip tests / a host-side slave sim).
    Motor scalars come back in engineering units."""
    if len(p) < SZ_CMD_ROBOT:
        return {}
    cycle_id, cmd_seq, n_chains, _res = struct.unpack_from(FMT_CMD_ROBOT_HDR, p, 0)
    base0 = struct.calcsize(FMT_CMD_ROBOT_HDR)   # 6
    chains = []
    for ci in range(min(n_chains, MAX_CHAINS)):
        base = base0 + ci * SZ_CMD_CHAIN
        chain_id, n_motors = struct.unpack_from(FMT_CMD_CHAIN_HDR, p, base)
        mbase = base + 2
        motors = []
        for mi in range(min(n_motors, MAX_MOTORS_PER_CHAIN)):
            mode_req, pos, vel, kp, kd, tau, flags = \
                struct.unpack_from(FMT_CMD_MOTOR, p, mbase + mi * SZ_CMD_MOTOR)
            motors.append(dict(mode_req=mode_req, pos=dec_i16(pos, POS_SCALE),
                               vel=dec_i16(vel, VEL_SCALE), kp=dec_u16(kp, KP_SCALE),
                               kd=dec_u16(kd, KD_SCALE), tau_ff=dec_i16(tau, TAU_SCALE),
                               flags=flags))
        chains.append(dict(chain_id=chain_id, n_motors=n_motors, motors=motors))
    return dict(cycle_id=cycle_id, cmd_seq=cmd_seq, n_chains=n_chains, chains=chains)


# ── telemetry pack (mirror of the firmware builder; for tests/simulation) ─────
def pack_tele_motor(pos_raw: int, vel_raw: int, tau_raw: int, temp_c: int,
                    state: int, cause: int, motor_mode: int, motor_fault: int,
                    flags: int, fb_age_ms: int, fault_word: int,
                    last_applied_seq: int, reserved: int = 0) -> bytes:
    """Pack one tele_motor_t (21 B) from RAW wire values (ints, already scaled)."""
    return struct.pack(FMT_TELE_MOTOR, pos_raw, vel_raw, tau_raw, temp_c & 0xFF,
                       state & 0xFF, cause & 0xFF, motor_mode & 0xFF,
                       motor_fault & 0xFF, flags & 0xFF, fb_age_ms & 0xFF,
                       fault_word & 0xFFFFFFFF, last_applied_seq & 0xFFFF,
                       reserved & 0xFFFF)


def pack_robot_tele(cycle_id: int, master_time_us: int, last_cmd_seq_rx: int,
                    missed_deadlines: int, robot_state: int, chains: list) -> bytes:
    """Build a full tele_robot_t (480 B). ``chains`` (≤ MAX_CHAINS) is a list of
    dicts {chain_id, spi_seq_echo, slave_time_us, cmd_crc_errors, can_tx_errors,
    motors:[raw-field dict ...]}; unused slots zero-filled."""
    out = bytearray()
    n_chains = len(chains)
    out += struct.pack(FMT_TELE_ROBOT_HDR, cycle_id & 0xFFFF,
                       master_time_us & 0xFFFFFFFF, last_cmd_seq_rx & 0xFFFF,
                       missed_deadlines & 0xFFFF, n_chains & 0xFF, robot_state & 0xFF)
    for ci in range(MAX_CHAINS):
        if ci < n_chains:
            ch = chains[ci]
            motors = ch["motors"]
            out += struct.pack(FMT_TELE_CHAIN_HDR, ch["chain_id"] & 0xFF,
                               len(motors) & 0xFF, ch.get("spi_seq_echo", 0) & 0xFF,
                               ch.get("spi_resyncs", 0) & 0xFF,
                               ch.get("slave_time_us", 0) & 0xFFFFFFFF,
                               ch.get("cmd_crc_errors", 0) & 0xFFFF,
                               ch.get("can_tx_errors", 0) & 0xFFFF)
            for mi in range(MAX_MOTORS_PER_CHAIN):
                if mi < len(motors):
                    out += pack_tele_motor(**motors[mi])
                else:
                    out += b"\x00" * SZ_TELE_MOTOR
        else:
            out += b"\x00" * SZ_TELE_CHAIN
    assert len(out) == SZ_TELE_ROBOT, len(out)
    return bytes(out)


# ── telemetry parse ───────────────────────────────────────────────────────────
def parse_robot_tele(p: bytes) -> dict:
    """Decode a tele_robot_t payload into nested dicts (chains → motors=TeleMotor).

    Accepts either the full fixed-size struct or just the populated prefix (the
    master transmits header + n_chains chains to keep the 200 Hz stream small)."""
    base0 = struct.calcsize(FMT_TELE_ROBOT_HDR)   # 12
    if len(p) < base0:
        return {}
    cycle_id, master_us, last_cmd_seq_rx, missed, n_chains, robot_state = \
        struct.unpack_from(FMT_TELE_ROBOT_HDR, p, 0)
    if len(p) < base0 + n_chains * SZ_TELE_CHAIN:
        return {}
    chains = []
    for ci in range(MAX_CHAINS):
        if ci >= n_chains:
            break
        base = base0 + ci * SZ_TELE_CHAIN
        chain_id, n_motors, spi_seq_echo, spi_resyncs, slave_us, cmd_crc, can_tx = \
            struct.unpack_from(FMT_TELE_CHAIN_HDR, p, base)
        mbase = base + 12
        motors = [TeleMotor.unpack(p, mbase + mi * SZ_TELE_MOTOR)
                  for mi in range(min(n_motors, MAX_MOTORS_PER_CHAIN))]
        chains.append(dict(chain_id=chain_id, n_motors=n_motors,
                           spi_seq_echo=spi_seq_echo, spi_resyncs=spi_resyncs,
                           slave_time_us=slave_us,
                           cmd_crc_errors=cmd_crc, can_tx_errors=can_tx, motors=motors))
    return dict(cycle_id=cycle_id, master_time_us=master_us,
                last_cmd_seq_rx=last_cmd_seq_rx, missed_deadlines=missed,
                n_chains=n_chains, robot_state=robot_state, chains=chains)


# ── CRC16-CCITT (must match firmware proto_crc16) ─────────────────────────────
# Table-driven (byte-wise), byte-identical to the firmware's bit-by-bit loop
# (poly 0x1021, init 0xFFFF, non-reflected). The big tele_robot_t frame (248 B) is
# CRC'd on every RX; a bit-by-bit loop costs ~1.8 ms/frame and backs up the 200 Hz
# pipeline, so this precomputed table keeps host decode cheap.
def _make_crc_table():
    table = []
    for b in range(256):
        crc = b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
        table.append(crc)
    return table


_CRC_TABLE = _make_crc_table()


def _crc16_update(crc: int, data: bytes) -> int:
    """Table-driven incremental update (kept as a pure fallback)."""
    t = _CRC_TABLE
    for byte in data:
        crc = ((crc << 8) ^ t[((crc >> 8) ^ byte) & 0xFF]) & 0xFFFF
    return crc


def crc16(data: bytes) -> int:
    """CRC16-CCITT (poly 0x1021, init 0xFFFF), identical to firmware proto_crc16.
    Uses binascii.crc_hqx (C, ~90x faster than the Python loop) — verified
    byte-identical — so decoding the 200 Hz telemetry stream stays cheap."""
    return binascii.crc_hqx(data, 0xFFFF)


# ── frame encode / decode ─────────────────────────────────────────────────────
_seq = 0
_MAX_PAYLOAD = 1024   # tele_robot_t is 248 B; must exceed the largest payload

version_errors = 0    # CRC-valid frames dropped for a wrong protocol version


def encode_frame(msg_type: int, src: int, dst: int, payload: bytes = b"") -> bytes:
    global _seq
    ts_ms = (time.monotonic_ns() // 1_000_000) & 0xFFFFFFFF
    hdr = struct.pack(HDR_FMT, msg_type, _seq, src, dst, ts_ms, len(payload),
                      PROTO_VERSION, 0)
    frame = bytearray(hdr) + payload
    checksum = crc16(bytes(frame))
    frame[14] = checksum & 0xFF
    frame[15] = (checksum >> 8) & 0xFF
    _seq = (_seq + 1) & 0xFFFF
    return bytes(frame)


def decode_frame(buf: bytearray, on_reject=None, on_frame=None):
    """Decode one frame from ``buf`` (mutated in place on resync).

    Returns ``(msg_type, seq, ts_ms, payload, consumed)`` or ``None``. Drops one
    byte and retries on bad CRC / implausible length; consumes and counts a
    CRC-valid frame whose version != PROTO_VERSION. Callbacks as before."""
    global version_errors
    discarded = bytearray()

    def _flush_discard():
        if discarded and on_reject is not None:
            on_reject("discard", bytes(discarded))
        del discarded[:]

    while len(buf) >= HDR_SIZE:
        msg_type, seq, src, dst, ts_ms, pay_len, ver_flags, crc_wire = \
            struct.unpack_from(HDR_FMT, buf, 0)
        if pay_len > _MAX_PAYLOAD:
            discarded.append(buf[0])
            del buf[0]
            continue
        total = HDR_SIZE + pay_len
        if len(buf) < total:
            if (ver_flags & 0xFF) == PROTO_VERSION:
                _flush_discard()
                return None
            discarded.append(buf[0])
            del buf[0]
            continue
        check = bytearray(buf[:total])
        check[14] = 0
        check[15] = 0
        if crc16(bytes(check)) != crc_wire:
            discarded.append(buf[0])
            del buf[0]
            continue
        if (ver_flags & 0xFF) != PROTO_VERSION:
            _flush_discard()
            version_errors += 1
            if on_reject is not None:
                on_reject("version", bytes(buf[:total]))
            del buf[:total]
            continue
        _flush_discard()
        if on_frame is not None:
            on_frame(bytes(buf[:total]))
        payload = bytes(buf[HDR_SIZE:total])
        return (msg_type, seq, ts_ms, payload, total)
    _flush_discard()
    return None


# ── status parsers ────────────────────────────────────────────────────────────
def parse_master_status(p: bytes) -> dict:
    if len(p) < struct.calcsize(FMT_MASTER_STATUS):
        return {}
    rs, sa, up, le, rf, poll_hz, tele_hz, tick_hz, host_hz, resyncs, discarded = \
        struct.unpack_from(FMT_MASTER_STATUS, p)
    return dict(robot_state=rs, slave_alive=sa, uptime_ms=up, link_errors=le,
                rx_frames=rf, master_poll_hz=poll_hz, telemetry_hz=tele_hz,
                slave_tick_hz=tick_hz, host_cmd_hz=host_hz,
                rx_resyncs=resyncs, rx_discarded_bytes=discarded)


def parse_slave_status(p: bytes) -> dict:
    if len(p) < struct.calcsize(FMT_SLAVE_STATUS):
        return {}
    sid, ma, up, crc_err, cmd_crc_err, seq_gaps = struct.unpack_from(FMT_SLAVE_STATUS, p)
    return dict(slave_id=sid, motors_alive=ma, uptime_ms=up,
                crc_errors=crc_err, cmd_crc_errors=cmd_crc_err, seq_gaps=seq_gaps)


def age_ms(now: float, t_last_msg: float, fb_age_at_receipt: int) -> float:
    """End-to-end per-motor staleness (ms): host time since the last telemetry for
    this motor, plus the CAN-hop age that frame reported."""
    return (now - t_last_msg) * 1000.0 + fb_age_at_receipt
