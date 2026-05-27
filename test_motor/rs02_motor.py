"""RS02 motor high-level API."""
import time
from typing import Optional
import can

from rs02_can import (
    encode_mit, encode_enable, encode_disable, encode_set_zero,
    decode_feedback, Feedback,
)

_GOTO_KP  = 15.0
_GOTO_KD  = 1.0
_GOTO_TOL = 0.02   # rad
_GOTO_HZ  = 100.0  # Hz


class RS02Motor:
    def __init__(self, bus: can.BusABC, motor_id: int, rx_timeout: float = 0.1):
        self.bus = bus
        self.motor_id = motor_id
        self.rx_timeout = rx_timeout

    def _send(self, arb_id: int, data: bytes) -> None:
        self.bus.send(can.Message(
            arbitration_id=arb_id, data=data, is_extended_id=True,
        ))

    def _recv(self) -> Optional[Feedback]:
        deadline = time.monotonic() + self.rx_timeout
        while time.monotonic() < deadline:
            msg = self.bus.recv(timeout=max(0.0, deadline - time.monotonic()))
            if msg is None:
                return None
            fb = decode_feedback(msg.arbitration_id, bytes(msg.data))
            if fb is not None and fb.motor_id == self.motor_id:
                return fb
        return None

    def enable(self) -> Optional[Feedback]:
        self._send(*encode_enable(self.motor_id))
        return self._recv()

    def disable(self) -> Optional[Feedback]:
        self._send(*encode_disable(self.motor_id))
        return self._recv()

    def set_zero(self) -> Optional[Feedback]:
        self._send(*encode_set_zero(self.motor_id))
        return self._recv()

    def mit_control(
        self,
        pos: float = 0.0,
        vel: float = 0.0,
        kp: float = 0.0,
        kd: float = 0.0,
        torque: float = 0.0,
    ) -> Optional[Feedback]:
        self._send(*encode_mit(self.motor_id, pos, vel, kp, kd, torque))
        return self._recv()

    def read(self) -> Optional[Feedback]:
        """Zero-gains MIT ping — elicits feedback without applying torque."""
        return self.mit_control()

    def goto(
        self,
        pos: float,
        speed: float,
        kp: float = _GOTO_KP,
        kd: float = _GOTO_KD,
        timeout: float = 5.0,
    ) -> Optional[Feedback]:
        """Drive to target position at up to `speed` rad/s (velocity feed-forward)."""
        deadline = time.monotonic() + timeout
        dt = 1.0 / _GOTO_HZ
        fb = self.read()
        while time.monotonic() < deadline:
            current = fb.pos if fb else 0.0
            error = pos - current
            if abs(error) < _GOTO_TOL:
                break
            vel_cmd = speed * (1.0 if error > 0 else -1.0)
            t0 = time.monotonic()
            fb = self.mit_control(pos=pos, vel=vel_cmd, kp=kp, kd=kd)
            elapsed = time.monotonic() - t0
            if elapsed < dt:
                time.sleep(dt - elapsed)
        return fb
