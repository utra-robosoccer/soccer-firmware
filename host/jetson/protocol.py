from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import struct
from typing import Optional


FRAME_DELIMITER = 0x00
MAX_PAYLOAD_LEN = 256

HEADER_FORMAT = "<HHBBIHHH"
MOTOR_CMD_FORMAT = "<fffff"
MOTOR_STATE_FORMAT = "<ffffIH"

HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
MOTOR_CMD_SIZE = struct.calcsize(MOTOR_CMD_FORMAT)
MOTOR_STATE_SIZE = struct.calcsize(MOTOR_STATE_FORMAT)

CRC16_INIT = 0xFFFF
CRC16_POLY = 0x1021


class ProtocolError(ValueError):
    pass


class MsgType(IntEnum):
    PING = 1
    DISCOVER = 2
    MOTOR_STATE_REQ = 3
    ARM_HOLD = 4
    DISABLE = 5
    MOTOR_STATE = 6


class NodeId(IntEnum):
    JETSON = 1
    MASTER = 2
    SLAVE = 3
    BROADCAST = 255


class MotorLifecycleState(IntEnum):
    UNDISCOVERED = 0
    DISCOVERING = 1
    IDLE = 2
    ARMING_HOLD = 3
    ARMED_HOLD = 4
    MIT_CONTROL = 5
    ZEROING = 6
    DISABLING = 7
    DISABLED = 8
    FAULT = 9


@dataclass(frozen=True)
class MsgHeader:
    type: MsgType
    seq: int
    source: int
    target: int
    ts_us: int
    length: int
    flags: int = 0
    crc: int = 0

    def pack(self, *, crc: Optional[int] = None) -> bytes:
        return struct.pack(
            HEADER_FORMAT,
            int(self.type),
            self.seq,
            self.source,
            self.target,
            self.ts_us,
            self.length,
            self.flags,
            self.crc if crc is None else crc,
        )

    @classmethod
    def unpack(cls, data: bytes) -> "MsgHeader":
        if len(data) != HEADER_SIZE:
            raise ProtocolError(f"header must be {HEADER_SIZE} bytes")
        msg_type, seq, source, target, ts_us, length, flags, crc = struct.unpack(HEADER_FORMAT, data)
        return cls(MsgType(msg_type), seq, source, target, ts_us, length, flags, crc)


@dataclass(frozen=True)
class MotorCmd:
    pos: float
    vel: float
    kp: float
    kd: float
    tau: float

    def pack(self) -> bytes:
        return struct.pack(MOTOR_CMD_FORMAT, self.pos, self.vel, self.kp, self.kd, self.tau)

    @classmethod
    def unpack(cls, data: bytes) -> "MotorCmd":
        if len(data) != MOTOR_CMD_SIZE:
            raise ProtocolError(f"MotorCmd must be {MOTOR_CMD_SIZE} bytes")
        return cls(*struct.unpack(MOTOR_CMD_FORMAT, data))


@dataclass(frozen=True)
class MotorState:
    pos: float
    vel: float
    tau: float
    temp: float
    fault: int
    last_cmd_seq: int

    def pack(self) -> bytes:
        return struct.pack(
            MOTOR_STATE_FORMAT,
            self.pos,
            self.vel,
            self.tau,
            self.temp,
            self.fault,
            self.last_cmd_seq,
        )

    @classmethod
    def unpack(cls, data: bytes) -> "MotorState":
        if len(data) != MOTOR_STATE_SIZE:
            raise ProtocolError(f"MotorState must be {MOTOR_STATE_SIZE} bytes")
        return cls(*struct.unpack(MOTOR_STATE_FORMAT, data))


@dataclass(frozen=True)
class Message:
    header: MsgHeader
    payload: bytes = b""


def crc16_ccitt(data: bytes, crc: int = CRC16_INIT) -> int:
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ CRC16_POLY) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    out = bytearray([0])
    code_index = 0
    code = 1

    for byte in data:
        if byte == 0:
            out[code_index] = code
            code_index = len(out)
            out.append(0)
            code = 1
        else:
            out.append(byte)
            code += 1
            if code == 0xFF:
                out[code_index] = code
                code_index = len(out)
                out.append(0)
                code = 1

    out[code_index] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    out = bytearray()
    index = 0

    while index < len(data):
        code = data[index]
        if code == 0:
            raise ProtocolError("COBS payload contains delimiter")
        index += 1

        end = index + code - 1
        if end > len(data):
            raise ProtocolError("COBS code overruns payload")
        out.extend(data[index:end])
        index = end

        if code != 0xFF and index < len(data):
            out.append(0)

    return bytes(out)


def encode_message(
    msg_type: MsgType,
    *,
    seq: int,
    source: int,
    target: int,
    ts_us: int,
    payload: bytes = b"",
    flags: int = 0,
) -> bytes:
    if len(payload) > MAX_PAYLOAD_LEN:
        raise ProtocolError("payload too large")

    header = MsgHeader(msg_type, seq, source, target, ts_us, len(payload), flags, 0)
    packet_without_crc = header.pack(crc=0) + payload
    crc = crc16_ccitt(packet_without_crc)
    packet = header.pack(crc=crc) + payload
    return cobs_encode(packet) + bytes([FRAME_DELIMITER])


def decode_frame(frame: bytes) -> Message:
    if frame.endswith(bytes([FRAME_DELIMITER])):
        frame = frame[:-1]
    if not frame:
        raise ProtocolError("empty frame")

    packet = cobs_decode(frame)
    if len(packet) < HEADER_SIZE:
        raise ProtocolError("packet shorter than header")

    header = MsgHeader.unpack(packet[:HEADER_SIZE])
    payload = packet[HEADER_SIZE:]
    if len(payload) != header.length:
        raise ProtocolError("payload length mismatch")

    expected_crc = crc16_ccitt(header.pack(crc=0) + payload)
    if header.crc != expected_crc:
        raise ProtocolError("CRC mismatch")

    return Message(header, payload)
