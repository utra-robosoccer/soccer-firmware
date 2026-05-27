"""RS02 CAN frame encoding and decoding — Private Protocol (29-bit extended).

Sources: RS02 user manual (chapter 4), soccer-firmware/slave/robostride.h/.c

29-bit extended ID layout:  mode[28:24] | data[23:8] | node_id[7:0]
"""
import struct
from dataclasses import dataclass
from typing import Optional

COMM_MIT        = 1   # operation control (type 1)
COMM_FEEDBACK   = 2   # motor feedback (type 2)
COMM_ENABLE     = 3
COMM_DISABLE    = 4
COMM_SET_ZERO   = 6
COMM_READ_PARAM = 17
COMM_WRITE_PARAM = 18

MASTER_ID = 0xFD      # host CAN ID (robostride.h: CAN_MASTER_ID)

# Fault bit positions within the 6-bit fault field (data[21:16] of CAN ID)
FAULT_UNCALIBRATED   = 5
FAULT_STALL_OVERLOAD = 4
FAULT_ENCODER        = 3
FAULT_OVERHEAT       = 2
FAULT_DRIVER         = 1
FAULT_UNDERVOLTAGE   = 0

# Parameter ranges — manual spec (chapter 4, page 56 code example)
_P_MIN  = -12.57;  _P_MAX  =  12.57   # rad
_V_MIN  = -44.0;   _V_MAX  =  44.0    # rad/s
_KP_MIN =   0.0;   _KP_MAX = 500.0
_KD_MIN =   0.0;   _KD_MAX =   5.0
_T_MIN  = -17.0;   _T_MAX  =  17.0    # Nm


def _enc(val: float, lo: float, hi: float, bits: int) -> int:
    """Linear float-to-uint using (2^bits - 1) denominator (manual formula, avoids overflow)."""
    clamped = max(lo, min(hi, val))
    return int((clamped - lo) * ((1 << bits) - 1) / (hi - lo))


def _dec(raw: int, lo: float, hi: float, bits: int) -> float:
    return lo + (hi - lo) * raw / ((1 << bits) - 1)


def make_id(mode: int, node_id: int, data: int = 0) -> int:
    """Pack 29-bit extended CAN ID."""
    return ((mode & 0x1F) << 24) | ((data & 0xFFFF) << 8) | (node_id & 0xFF)


def parse_id(arb_id: int) -> tuple[int, int, int]:
    """Returns (mode, data_field, node_id)."""
    return (arb_id >> 24) & 0x1F, (arb_id >> 8) & 0xFFFF, arb_id & 0xFF


def encode_mit(
    motor_id: int,
    pos: float,
    vel: float,
    kp: float,
    kd: float,
    torque: float,
) -> tuple[int, bytes]:
    """Type 1 operation control frame.

    Torque feed-forward is packed into the CAN ID data field (bits 23:8).
    Payload is 8 bytes big-endian: pos[2] vel[2] kp[2] kd[2].
    """
    torq_u = _enc(torque, _T_MIN, _T_MAX, 16)
    arb_id = make_id(COMM_MIT, motor_id, data=torq_u)
    payload = struct.pack(
        ">HHHH",
        _enc(pos, _P_MIN, _P_MAX, 16),
        _enc(vel, _V_MIN, _V_MAX, 16),
        _enc(kp,  _KP_MIN, _KP_MAX, 16),
        _enc(kd,  _KD_MIN, _KD_MAX, 16),
    )
    return arb_id, payload


def encode_enable(motor_id: int) -> tuple[int, bytes]:
    return make_id(COMM_ENABLE, motor_id, data=MASTER_ID), bytes(8)


def encode_disable(motor_id: int) -> tuple[int, bytes]:
    return make_id(COMM_DISABLE, motor_id, data=MASTER_ID), bytes(8)


def encode_set_zero(motor_id: int) -> tuple[int, bytes]:
    """Type 6 set-zero: payload byte 0 must be 0x01 (robostride.c confirmation)."""
    return make_id(COMM_SET_ZERO, motor_id, data=MASTER_ID), bytes([0x01]) + bytes(7)


@dataclass
class Feedback:
    motor_id: int
    mode: int
    faults: int       # 6-bit field; use FAULT_* constants to test individual bits
    pos: float        # rad
    vel: float        # rad/s
    torque: float     # Nm
    temp: float       # °C

    @property
    def ok(self) -> bool:
        return self.faults == 0

    def fault_str(self) -> str:
        names = {
            FAULT_UNCALIBRATED:   "uncalibrated",
            FAULT_STALL_OVERLOAD: "stall/overload",
            FAULT_ENCODER:        "encoder",
            FAULT_OVERHEAT:       "overheat",
            FAULT_DRIVER:         "driver",
            FAULT_UNDERVOLTAGE:   "undervoltage",
        }
        active = [names[b] for b in range(6) if self.faults & (1 << b)]
        return ",".join(active) if active else "none"

    def __str__(self) -> str:
        return (
            f"pos={self.pos:.3f} vel={self.vel:.3f} torq={self.torque:.3f} "
            f"temp={self.temp:.1f} mode={self.mode} faults={self.fault_str()}"
        )


def decode_feedback(arb_id: int, data: bytes) -> Optional["Feedback"]:
    """Type 2 feedback frame.

    CAN ID data field layout (robostride.h masks):
      bits 7:0   — motor_id  (RS_FB_DATA_ID_MASK   = 0x00FF)
      bits 13:8  — faults    (RS_FB_DATA_FAULT_MASK = 0x3F00, >> 8)
      bits 15:14 — mode      (RS_FB_DATA_MODE_MASK  = 0xC000, >> 14)

    Payload (8 bytes big-endian): pos[2] vel[2] torque[2] temp_raw[2]
    Temperature: temp_raw / 10.0 °C
    """
    mode, data_field, _ = parse_id(arb_id)
    if mode != COMM_FEEDBACK or len(data) < 8:
        return None

    motor_id = data_field & 0x00FF
    faults   = (data_field & 0x3F00) >> 8
    fb_mode  = (data_field & 0xC000) >> 14

    p_raw, v_raw, t_raw, temp_raw = struct.unpack(">HHHH", data[:8])
    return Feedback(
        motor_id=motor_id,
        mode=fb_mode,
        faults=faults,
        pos=_dec(p_raw, _P_MIN, _P_MAX, 16),
        vel=_dec(v_raw, _V_MIN, _V_MAX, 16),
        torque=_dec(t_raw, _T_MIN, _T_MAX, 16),
        temp=temp_raw / 10.0,
    )
