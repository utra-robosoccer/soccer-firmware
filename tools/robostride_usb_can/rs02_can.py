"""RS02 CAN frame encoding and decoding - private 29-bit extended protocol.

Sources: RS02 user manual (chapter 4), soccer-firmware/slave/robostride.h/.c

29-bit extended ID layout:  mode[28:24] | data[23:8] | node_id[7:0]
"""
import struct
from dataclasses import dataclass
from typing import Optional

COMM_GET_ID     = 0   # get device ID / MCU ID
COMM_MIT        = 1   # operation control (type 1)
COMM_FEEDBACK   = 2   # motor feedback (type 2)
COMM_ENABLE     = 3
COMM_DISABLE    = 4
COMM_SET_ZERO   = 6
COMM_SET_ID     = 7
COMM_READ_PARAM = 17
COMM_WRITE_PARAM = 18

MASTER_ID = 0xFD      # host CAN ID (robostride.h: CAN_MASTER_ID)
RUN_MODE_MIT = 0
RUN_MODE_POSITION = 1
RUN_MODE_VELOCITY = 2
RUN_MODE_CURRENT = 3
RUN_MODE_CSP = 4
RUN_MODE_REGISTER = 0x7005

# Fault bit positions within the 6-bit fault field (data[21:16] of CAN ID)
FAULT_UNCALIBRATED   = 5
FAULT_STALL_OVERLOAD = 4
FAULT_ENCODER        = 3
FAULT_OVERHEAT       = 2
FAULT_DRIVER         = 1
FAULT_UNDERVOLTAGE   = 0

# Parameter ranges from the RS02 user manual / K-Scale Robstride host protocol.
P_MIN  = -12.57
P_MAX  =  12.57   # rad, approximately +/- 4*pi
V_MIN  = -44.0
V_MAX  =  44.0    # rad/s
KP_MIN =   0.0
KP_MAX = 500.0
KD_MIN =   0.0
KD_MAX =   5.0
T_MIN  = -17.0
T_MAX  =  17.0    # Nm


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


def encode_raw_frame(arb_id: int, data: bytes) -> bytes:
    """Wrap a 29-bit extended CAN frame in the CH341 Robstride AT format."""
    if len(data) > 8:
        raise ValueError("CAN payload must be at most 8 bytes")
    raw_id = ((arb_id & 0x1FFFFFFF) << 3) | 0x04
    return b"AT" + struct.pack(">I", raw_id) + bytes([len(data)]) + data + b"\r\n"


@dataclass
class RawFrame:
    arb_id: int
    data: bytes
    end: int


def decode_raw_frame(buf: bytes) -> Optional[RawFrame]:
    """Scan a serial byte buffer for one complete CH341 Robstride AT frame."""
    for start in range(max(0, len(buf) - 7)):
        if buf[start:start + 2] != b"AT":
            continue
        if start + 8 > len(buf):
            return None

        data_len = buf[start + 6]
        total = 7 + data_len + 2
        end = start + total
        if end > len(buf):
            return None
        if buf[end - 2:end] != b"\r\n":
            continue

        raw_id = struct.unpack(">I", buf[start + 2:start + 6])[0]
        arb_id = (raw_id >> 3) & 0x1FFFFFFF
        return RawFrame(arb_id=arb_id, data=buf[start + 7:start + 7 + data_len], end=end)
    return None


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
    torq_u = _enc(torque, T_MIN, T_MAX, 16)
    arb_id = make_id(COMM_MIT, motor_id, data=torq_u)
    payload = struct.pack(
        ">HHHH",
        _enc(pos, P_MIN, P_MAX, 16),
        _enc(vel, V_MIN, V_MAX, 16),
        _enc(kp,  KP_MIN, KP_MAX, 16),
        _enc(kd,  KD_MIN, KD_MAX, 16),
    )
    return arb_id, payload


def encode_get_motor_id(motor_id: int) -> tuple[int, bytes]:
    """Type 0 get device ID / MCU ID."""
    return make_id(COMM_GET_ID, motor_id, data=MASTER_ID), bytes(8)


def encode_enable(motor_id: int) -> tuple[int, bytes]:
    return make_id(COMM_ENABLE, motor_id, data=MASTER_ID), bytes(8)


def encode_disable(motor_id: int) -> tuple[int, bytes]:
    return make_id(COMM_DISABLE, motor_id, data=MASTER_ID), bytes(8)


def encode_set_zero(motor_id: int) -> tuple[int, bytes]:
    """Type 6 set-zero: payload byte 0 must be 0x01 (robostride.c confirmation)."""
    return make_id(COMM_SET_ZERO, motor_id, data=MASTER_ID), bytes([0x01]) + bytes(7)


def encode_set_motor_id(motor_id: int, new_motor_id: int) -> tuple[int, bytes]:
    """Type 7 set CAN ID: data high byte is new ID, low byte is master ID."""
    data = ((new_motor_id & 0xFF) << 8) | (MASTER_ID & 0xFF)
    return make_id(COMM_SET_ID, motor_id, data=data), bytes(8)


def encode_change_motor_mode(motor_id: int, run_mode: int = RUN_MODE_MIT) -> tuple[int, bytes]:
    """Type 18 write param: set run mode register 0x7005."""
    payload = bytearray(8)
    payload[0] = RUN_MODE_REGISTER & 0xFF
    payload[1] = (RUN_MODE_REGISTER >> 8) & 0xFF
    payload[4] = run_mode & 0xFF
    return make_id(COMM_WRITE_PARAM, motor_id, data=MASTER_ID), bytes(payload)


@dataclass
class DeviceId:
    motor_id: int
    mcu_id: int

    def __str__(self) -> str:
        return f"motor_id={self.motor_id} mcu_id=0x{self.mcu_id:016X}"


@dataclass
class Feedback:
    motor_id: int
    mode: int          # motor status: 0 reset, 1 calibration, 2 normal
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

    def status_str(self) -> str:
        names = {
            0: "reset",
            1: "calibration",
            2: "normal",
        }
        return names.get(self.mode, f"unknown({self.mode})")

    def __str__(self) -> str:
        return (
            f"pos={self.pos:.3f} vel={self.vel:.3f} torq={self.torque:.3f} "
            f"temp={self.temp:.1f} status={self.status_str()} faults={self.fault_str()}"
        )


def decode_feedback(arb_id: int, data: bytes) -> Optional["Feedback"]:
    """Type 2 feedback frame.

    CAN ID data field layout (robostride.h masks):
      bits 7:0   — motor_id  (RS_FB_DATA_ID_MASK   = 0x00FF)
      bits 13:8  — faults    (RS_FB_DATA_FAULT_MASK = 0x3F00, >> 8)
      bits 15:14 — status    (RS_FB_DATA_MODE_MASK  = 0xC000, >> 14)

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
        pos=_dec(p_raw, P_MIN, P_MAX, 16),
        vel=_dec(v_raw, V_MIN, V_MAX, 16),
        torque=_dec(t_raw, T_MIN, T_MAX, 16),
        temp=temp_raw / 10.0,
    )


def decode_device_id(arb_id: int, data: bytes) -> Optional[DeviceId]:
    """Type 0 device-ID response. Payload is the 64-bit MCU ID, little-endian."""
    mode, data_field, _ = parse_id(arb_id)
    if mode != COMM_GET_ID or len(data) < 8:
        return None

    return DeviceId(
        motor_id=data_field & 0x00FF,
        mcu_id=int.from_bytes(data[:8], byteorder="little"),
    )
