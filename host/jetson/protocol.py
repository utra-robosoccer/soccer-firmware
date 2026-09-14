"""Canonical host-side protocol library for the soccer master link.

Single source of truth for the master↔host USB wire format. Matches the firmware
in ``firmware/common/include/protocol.h`` byte-for-byte: a raw
``[MsgHeader(16 B)][payload]`` frame streamed over USB CDC, with CRC16-CCITT
(poly 0x1021, init 0xFFFF) computed over the header (crc field zeroed) followed
by the payload. This link is **not** COBS-framed.

The slave→master telemetry ``MotorState`` atom is forwarded verbatim by the
master (pass-through); the host decodes raw→engineering units here using the
GLOBAL transport bounds generated from the active config
(``tools/motor_config_gen.py``) — not per-model tables, because the slave encodes
over those same global bounds.
"""
from __future__ import annotations

from dataclasses import dataclass
import struct
import time

# Global SPI transport bounds, generated from the active config. The slave
# encodes pos/vel/tau over THESE bounds, so the host must decode with them.
# Fallback = widest model (RS02) if the generated module isn't importable.
try:
    from motor_config_gen import (  # type: ignore
        MOTOR_P_MIN, MOTOR_P_MAX, MOTOR_V_MIN, MOTOR_V_MAX, MOTOR_T_MIN, MOTOR_T_MAX,
    )
except Exception:  # pragma: no cover - fallback for standalone use
    MOTOR_P_MIN, MOTOR_P_MAX = -12.57, 12.57
    MOTOR_V_MIN, MOTOR_V_MAX = -44.0, 44.0
    MOTOR_T_MIN, MOTOR_T_MAX = -17.0, 17.0


# ── node ids ──────────────────────────────────────────────────────────────────
NODE_JETSON  = 1
NODE_MASTER  = 2
NODE_SLAVE_0 = 3

# ── message types ─────────────────────────────────────────────────────────────
MSG_PING          = 0x01
MSG_MASTER_STATUS = 0x02
MSG_SLAVE_STATUS  = 0x03
MSG_MOTOR_STATE   = 0x04
MSG_CONTROL_REQ   = 0x05
MSG_CONTROL_RESP  = 0x06
MSG_MOTOR_CMD     = 0x07

MSG_NAMES = {
    MSG_PING: "PING", MSG_MASTER_STATUS: "MASTER_STATUS",
    MSG_SLAVE_STATUS: "SLAVE_STATUS", MSG_MOTOR_STATE: "MOTOR_STATE",
    MSG_CONTROL_REQ: "CONTROL_REQ", MSG_CONTROL_RESP: "CONTROL_RESP",
    MSG_MOTOR_CMD: "MOTOR_CMD",
}

# ── lifecycle (low nibble of MotorState.state) ────────────────────────────────
LIFECYCLE_NAMES = {
    0: "BOOT", 1: "DISCOVERING", 2: "IDLE", 3: "ARMED_HOLD", 4: "FAULT",
    5: "DISABLED", 6: "ZEROING", 7: "ARMED_MIT",
}
# ── fault cause (high nibble of MotorState.state) ─────────────────────────────
CAUSE_NONE, CAUSE_OVERTORQUE, CAUSE_CAN_TIMEOUT, CAUSE_WATCHDOG, CAUSE_MOTOR_FAULT = range(5)
CAUSE_NAMES = {
    0: "NONE", 1: "OVERTORQUE", 2: "CAN_TIMEOUT", 3: "WATCHDOG", 4: "MOTOR_FAULT",
}

# ── cmd_flags bits (recomputed each tick by the slave) ────────────────────────
CMDFLAG_CLAMPED_POS = 1 << 0
CMDFLAG_CLAMPED_TAU = 1 << 1
CMDFLAG_CMD_STALE   = 1 << 2

# ── control commands / results ────────────────────────────────────────────────
CTRL_ARM_HOLD  = 0x01
CTRL_DISABLE   = 0x02
CTRL_SET_ZERO  = 0x03
CTRL_GOTO_ZERO = 0x04
CTRL_RESULT_NAMES = {0: "OK", 1: "ERR_STATE", 2: "ERR_STUB", 3: "ERR_MOTOR"}

ROBOT_STATE_NAMES = {0: "INIT", 1: "READY", 2: "DEGRADED"}

# v1 telemetry contract version — carried in MsgHeader.ver_flags low byte.
PROTO_VERSION = 1

# ── wire layouts (little-endian, packed) ──────────────────────────────────────
HDR_FMT  = "<HHBBIHHH"          # type,seq,src,dst,ts_ms,len,ver_flags,crc  → 16 B
HDR_SIZE = struct.calcsize(HDR_FMT)

FMT_MASTER_STATUS = "<BBIII"    # robot_state,slave_alive,uptime,link_errs,rx_frames (14)
FMT_SLAVE_STATUS  = "<BBII"     # slave_id,motors_alive,uptime,crc_errors (10)
FMT_CONTROL_REQ   = "<BBBB"     # slave_id,motor_idx,cmd,reserved (4)
FMT_CONTROL_RESP  = "<BBBBBH"   # slave_id,motor_idx,cmd,result,new_state,req_seq (7)
FMT_MOTOR_CMD     = "<BBfffff"  # slave_id,motor_idx,pos,vel,kp,kd,tau_ff (22)

MOTORSTATE_FMT  = "<HHHBBBBIBB"  # 16 B telemetry atom
MOTORSTATE_SIZE = struct.calcsize(MOTORSTATE_FMT)
FMT_MOTOR_STATE_HDR = "<BB"      # slave_id, motor_idx before the atom

assert HDR_SIZE == 16, HDR_SIZE
assert MOTORSTATE_SIZE == 16, MOTORSTATE_SIZE
assert struct.calcsize(FMT_SLAVE_STATUS) == 10
assert struct.calcsize(FMT_MASTER_STATUS) == 14


def _decode(raw: int, lo: float, hi: float) -> float:
    """Inverse of the slave's f_to_u16: raw 0..65535 → [lo, hi]."""
    return lo + raw * (hi - lo) / 65535.0


@dataclass(frozen=True)
class MotorState:
    """One motor's 16-byte telemetry atom, forwarded verbatim by the master.

    ``pos_raw`` is the HOME-FRAME wrapped ``[-pi, pi]`` position scaled over the
    ``±4π`` transport bound — NOT the motor's multi-turn angle. ``vel_raw`` /
    ``tau_raw`` are scaled over the global transport bounds (widest model).
    """
    pos_raw: int
    vel_raw: int
    tau_raw: int
    temp_c: int
    state: int          # [3:0] lifecycle | [7:4] fault cause
    motor_fault: int    # 6 compact Type-2 fault bits
    cmd_flags: int
    fault_word: int     # 0x3022 latched on fault; 0 clear; 0xFFFFFFFF read-fail
    fb_age: int         # ms since this motor's last Type-2, saturating 255
    reserved_v2: int    # reserved growth byte (0); append-only evolution

    @classmethod
    def unpack(cls, data: bytes) -> "MotorState":
        return cls(*struct.unpack_from(MOTORSTATE_FMT, data, 0))

    @property
    def lifecycle(self) -> int: return self.state & 0x0F
    @property
    def cause(self) -> int: return self.state >> 4
    @property
    def lifecycle_name(self) -> str: return LIFECYCLE_NAMES.get(self.lifecycle, f"?{self.lifecycle}")
    @property
    def cause_name(self) -> str: return CAUSE_NAMES.get(self.cause, f"?{self.cause}")

    @property
    def pos(self) -> float: return _decode(self.pos_raw, MOTOR_P_MIN, MOTOR_P_MAX)
    @property
    def vel(self) -> float: return _decode(self.vel_raw, MOTOR_V_MIN, MOTOR_V_MAX)
    @property
    def tau(self) -> float: return _decode(self.tau_raw, MOTOR_T_MIN, MOTOR_T_MAX)
    @property
    def temp(self) -> float: return float(self.temp_c)

    @property
    def clamped_pos(self) -> bool: return bool(self.cmd_flags & CMDFLAG_CLAMPED_POS)
    @property
    def clamped_tau(self) -> bool: return bool(self.cmd_flags & CMDFLAG_CLAMPED_TAU)
    @property
    def cmd_stale(self) -> bool: return bool(self.cmd_flags & CMDFLAG_CMD_STALE)


# ── CRC16-CCITT (must match firmware proto_crc16) ─────────────────────────────
def _crc16_update(crc: int, data: bytes) -> int:
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def crc16(data: bytes) -> int:
    return _crc16_update(0xFFFF, data)


# ── frame encode / decode ─────────────────────────────────────────────────────
_seq = 0
_MAX_PAYLOAD = 256

# Count of CRC-valid frames dropped for a wrong protocol version (diagnostics).
version_errors = 0


def encode_frame(msg_type: int, src: int, dst: int, payload: bytes = b"") -> bytes:
    global _seq
    ts_ms = (time.monotonic_ns() // 1_000_000) & 0xFFFFFFFF
    # ver_flags = PROTO_VERSION in the low byte, high byte reserved 0.
    hdr = struct.pack(HDR_FMT, msg_type, _seq, src, dst, ts_ms, len(payload),
                      PROTO_VERSION, 0)
    frame = bytearray(hdr) + payload
    checksum = crc16(bytes(frame))
    frame[14] = checksum & 0xFF
    frame[15] = (checksum >> 8) & 0xFF
    _seq = (_seq + 1) & 0xFFFF
    return bytes(frame)


def decode_frame(buf: bytearray):
    """Decode one frame from ``buf`` (mutated in place on resync).

    Returns ``(msg_type, seq, ts_ms, payload, consumed)`` or ``None`` if no
    complete valid frame is present yet. Drops one byte and retries on bad CRC
    or implausible payload length; consumes and counts a CRC-valid frame whose
    protocol version != PROTO_VERSION.
    """
    global version_errors
    while len(buf) >= HDR_SIZE:
        msg_type, seq, src, dst, ts_ms, pay_len, ver_flags, crc_wire = \
            struct.unpack_from(HDR_FMT, buf, 0)
        if pay_len > _MAX_PAYLOAD:
            del buf[0]
            continue
        total = HDR_SIZE + pay_len
        if len(buf) < total:
            return None
        check = bytearray(buf[:total])
        check[14] = 0
        check[15] = 0
        if crc16(bytes(check)) != crc_wire:
            del buf[0]
            continue
        if (ver_flags & 0xFF) != PROTO_VERSION:
            # CRC-valid but wrong version → consume the whole frame, don't resync.
            version_errors += 1
            del buf[:total]
            continue
        payload = bytes(buf[HDR_SIZE:total])
        return (msg_type, seq, ts_ms, payload, total)
    return None


# ── payload parsers ───────────────────────────────────────────────────────────
def parse_master_status(p: bytes) -> dict:
    if len(p) < struct.calcsize(FMT_MASTER_STATUS):
        return {}
    rs, sa, up, le, rf = struct.unpack_from(FMT_MASTER_STATUS, p)
    return dict(robot_state=rs, slave_alive=sa,
                uptime_ms=up, link_errors=le, rx_frames=rf)


def parse_slave_status(p: bytes) -> dict:
    if len(p) < struct.calcsize(FMT_SLAVE_STATUS):
        return {}
    sid, ma, up, crc_err = struct.unpack_from(FMT_SLAVE_STATUS, p)
    return dict(slave_id=sid, motors_alive=ma,
                uptime_ms=up, crc_errors=crc_err)


def parse_motor_state(p: bytes) -> dict:
    """Decode a MotorStatePayload: [slave_id][motor_idx][MotorState atom].

    Returns the addressing fields plus the decoded atom (engineering units and
    the ``MotorState`` object under ``atom``)."""
    if len(p) < struct.calcsize(FMT_MOTOR_STATE_HDR) + MOTORSTATE_SIZE:
        return {}
    sid, idx = struct.unpack_from(FMT_MOTOR_STATE_HDR, p, 0)
    ms = MotorState.unpack(p[struct.calcsize(FMT_MOTOR_STATE_HDR):])
    return dict(slave_id=sid, motor_idx=idx, atom=ms,
                state=ms.lifecycle, cause=ms.cause,
                pos=ms.pos, vel=ms.vel, tau=ms.tau, temp=ms.temp,
                motor_fault=ms.motor_fault, cmd_flags=ms.cmd_flags,
                fault_word=ms.fault_word, fb_age=ms.fb_age)


def age_ms(now: float, t_last_msg: float, fb_age_at_receipt: int) -> float:
    """End-to-end per-motor staleness (ms): host time since the last MOTOR_STATE
    for this motor, plus the CAN-hop age that frame reported. With emission
    gating, a silent slave stops advancing t_last_msg so this climbs — the
    end-to-end freshness signal, unlike the atom's fb_age which freezes."""
    return (now - t_last_msg) * 1000.0 + fb_age_at_receipt


def parse_control_resp(p: bytes) -> dict:
    if len(p) < struct.calcsize(FMT_CONTROL_RESP):
        return {}
    sid, idx, cmd, result, new_st, req_seq = struct.unpack_from(FMT_CONTROL_RESP, p)
    return dict(slave_id=sid, motor_idx=idx, cmd=cmd, result=result,
                new_state=new_st, req_seq=req_seq)
