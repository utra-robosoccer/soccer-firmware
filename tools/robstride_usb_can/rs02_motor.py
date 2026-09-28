"""RS02 motor API over raw CH341 Robstride serial framing."""
import math
import time
from collections.abc import Callable, Sequence
from typing import Optional

import serial

from rs02_can import (
    DeviceId,
    Feedback,
    decode_device_id,
    decode_feedback,
    decode_raw_frame,
    encode_change_motor_mode,
    encode_disable,
    encode_enable,
    encode_get_motor_id,
    encode_mit,
    encode_raw_frame,
    encode_set_motor_id,
    encode_set_zero,
)

DEFAULT_PORT = "/dev/ttyCH341USB0"
DEFAULT_BAUD = 921600

_GOTO_KP = 15.0
_GOTO_KD = 1.0
_GOTO_TOL = 0.02
_GOTO_HZ = 100.0
_GOTO_ACCEL = 0.05
_GOTO_SETTLE_FRAMES = 5
_ZERO_VEL_TOL = 0.03


def sine_setpoint(
    center: float,
    amplitude: float,
    frequency: float,
    elapsed: float,
    duration: float,
    ramp_time: float,
) -> tuple[float, float]:
    """Return position and feed-forward velocity for a sine command."""
    t = min(max(0.0, elapsed), duration)
    omega = 2.0 * math.pi * frequency
    phase = omega * t

    envelope = 1.0
    envelope_dot = 0.0
    ramp = min(ramp_time, duration * 0.5)
    if ramp > 0.0:
        if t < ramp:
            envelope = 0.5 - 0.5 * math.cos(math.pi * t / ramp)
            envelope_dot = 0.5 * math.pi / ramp * math.sin(math.pi * t / ramp)
        elif duration - t < ramp:
            tau = duration - t
            envelope = 0.5 - 0.5 * math.cos(math.pi * tau / ramp)
            envelope_dot = -0.5 * math.pi / ramp * math.sin(math.pi * tau / ramp)

    sin_phase = math.sin(phase)
    cos_phase = math.cos(phase)
    pos = center + amplitude * envelope * sin_phase
    vel = amplitude * (envelope_dot * sin_phase + envelope * omega * cos_phase)
    return pos, vel


class RS02Motor:
    def __init__(
        self,
        port: str = DEFAULT_PORT,
        motor_id: int = 1,
        baud: int = DEFAULT_BAUD,
        rx_timeout: float = 0.15,
    ):
        self.port = port
        self.motor_id = motor_id
        self.baud = baud
        self.rx_timeout = rx_timeout
        self._ser: Optional[serial.Serial] = None
        self._buf = b""

    def open(self) -> None:
        self._ser = serial.Serial(self.port, self.baud, timeout=0.02)
        self._ser.reset_input_buffer()
        self._buf = b""

    def close(self) -> None:
        if self._ser is not None and self._ser.is_open:
            self._ser.close()
        self._ser = None

    def __enter__(self) -> "RS02Motor":
        self.open()
        return self

    def __exit__(self, *_) -> None:
        self.close()

    def _require_open(self) -> serial.Serial:
        if self._ser is None or not self._ser.is_open:
            raise RuntimeError("serial port is not open")
        return self._ser

    def _send(self, arb_id: int, data: bytes) -> None:
        ser = self._require_open()
        ser.write(encode_raw_frame(arb_id, data))
        ser.flush()

    def _drain(self) -> None:
        ser = self._require_open()
        waiting = getattr(ser, "in_waiting", 0)
        if waiting:
            self._buf += ser.read(waiting)
            while True:
                frame = decode_raw_frame(self._buf)
                if frame is None:
                    break
                self._buf = self._buf[frame.end:]

    def _recv_feedback(self, timeout: Optional[float] = None, motor_id: Optional[int] = None) -> Optional[Feedback]:
        ser = self._require_open()
        wait_time = self.rx_timeout if timeout is None else timeout
        deadline = time.monotonic() + wait_time
        target_id = self.motor_id if motor_id is None else motor_id
        while True:
            if wait_time <= 0.0:
                waiting = getattr(ser, "in_waiting", 0)
                chunk = ser.read(waiting) if waiting else b""
            else:
                chunk = ser.read(256)
            if chunk:
                self._buf += chunk

            frame = decode_raw_frame(self._buf)
            if frame is None:
                if time.monotonic() >= deadline:
                    break
                continue

            self._buf = self._buf[frame.end:]
            fb = decode_feedback(frame.arb_id, frame.data)
            if fb is not None and fb.motor_id == target_id:
                return fb
            if time.monotonic() >= deadline:
                break
        return None

    def _recv_any_feedback(self, timeout: Optional[float] = None) -> Optional[Feedback]:
        ser = self._require_open()
        wait_time = self.rx_timeout if timeout is None else timeout
        deadline = time.monotonic() + wait_time
        while True:
            if wait_time <= 0.0:
                waiting = getattr(ser, "in_waiting", 0)
                chunk = ser.read(waiting) if waiting else b""
            else:
                chunk = ser.read(256)
            if chunk:
                self._buf += chunk

            frame = decode_raw_frame(self._buf)
            if frame is None:
                if time.monotonic() >= deadline:
                    break
                continue

            self._buf = self._buf[frame.end:]
            fb = decode_feedback(frame.arb_id, frame.data)
            if fb is not None:
                return fb
            if time.monotonic() >= deadline:
                break
        return None

    def _recv_device_id(self, timeout: Optional[float] = None) -> Optional[DeviceId]:
        ser = self._require_open()
        wait_time = self.rx_timeout if timeout is None else timeout
        deadline = time.monotonic() + wait_time
        while True:
            chunk = ser.read(256) if wait_time > 0.0 else b""
            if chunk:
                self._buf += chunk

            frame = decode_raw_frame(self._buf)
            if frame is None:
                if time.monotonic() >= deadline:
                    break
                continue

            self._buf = self._buf[frame.end:]
            device_id = decode_device_id(frame.arb_id, frame.data)
            if device_id is not None:
                return device_id
            if time.monotonic() >= deadline:
                break
        return None

    def _request_feedback(self, encoder, timeout: Optional[float] = None) -> Optional[Feedback]:
        return self._request_feedback_for(self.motor_id, encoder, timeout)

    def _request_feedback_for(self, motor_id: int, encoder, timeout: Optional[float] = None) -> Optional[Feedback]:
        ser = self._require_open()
        ser.reset_input_buffer()
        self._buf = b""
        self._send(*encoder(motor_id))
        return self._recv_feedback(timeout, motor_id=motor_id)

    def enable(self, timeout: Optional[float] = None) -> Optional[Feedback]:
        return self._request_feedback(encode_enable, timeout)

    def disable(self, timeout: Optional[float] = None) -> Optional[Feedback]:
        return self._request_feedback(encode_disable, timeout)

    def set_mit_mode(self, timeout: Optional[float] = None) -> Optional[Feedback]:
        return self._request_feedback(encode_change_motor_mode, timeout)

    def set_mit_mode_for(self, motor_id: int, timeout: Optional[float] = None) -> Optional[Feedback]:
        return self._request_feedback_for(motor_id, encode_change_motor_mode, timeout)

    def get_device_id(self, timeout: Optional[float] = None) -> Optional[DeviceId]:
        ser = self._require_open()
        ser.reset_input_buffer()
        self._buf = b""
        self._send(*encode_get_motor_id(self.motor_id))
        return self._recv_device_id(timeout)

    def set_zero(self, timeout: Optional[float] = None, stop_first: bool = True) -> Optional[Feedback]:
        """Set the current mechanical position as zero using comm type 6."""
        if stop_first:
            self.disable(timeout=timeout)
            time.sleep(0.1)
        fb = self._request_feedback(encode_set_zero, timeout)
        time.sleep(0.15)
        return fb if fb is not None else self.read_enable(timeout=timeout)

    def set_motor_id(self, new_motor_id: int, timeout: Optional[float] = None) -> Optional[Feedback]:
        """Set the motor CAN ID, then read feedback at the new ID."""
        ser = self._require_open()
        ser.reset_input_buffer()
        self._buf = b""
        self._send(*encode_set_motor_id(self.motor_id, new_motor_id))
        time.sleep(0.2)
        self.motor_id = new_motor_id
        return self.read(timeout=timeout)

    def mit_control(
        self,
        pos: float = 0.0,
        vel: float = 0.0,
        kp: float = 0.0,
        kd: float = 0.0,
        torque: float = 0.0,
        timeout: Optional[float] = None,
    ) -> Optional[Feedback]:
        ser = self._require_open()
        ser.reset_input_buffer()
        self._buf = b""
        self._send(*encode_mit(self.motor_id, pos, vel, kp, kd, torque))
        return self._recv_feedback(timeout)

    def mit_send(
        self,
        pos: float = 0.0,
        vel: float = 0.0,
        kp: float = 0.0,
        kd: float = 0.0,
        torque: float = 0.0,
    ) -> None:
        """Send one MIT frame without waiting for feedback."""
        self._send(*encode_mit(self.motor_id, pos, vel, kp, kd, torque))

    def mit_send_for(
        self,
        motor_id: int,
        pos: float = 0.0,
        vel: float = 0.0,
        kp: float = 0.0,
        kd: float = 0.0,
        torque: float = 0.0,
    ) -> None:
        """Send one MIT frame to a specific motor without waiting for feedback."""
        self._send(*encode_mit(motor_id, pos, vel, kp, kd, torque))

    def read(self, timeout: Optional[float] = None) -> Optional[Feedback]:
        """Read state with a zero-gain MIT ping."""
        return self.mit_control(timeout=timeout)

    def read_for(self, motor_id: int, timeout: Optional[float] = None) -> Optional[Feedback]:
        """Read state from a specific motor with a zero-gain MIT ping."""
        ser = self._require_open()
        ser.reset_input_buffer()
        self._buf = b""
        self._send(*encode_mit(motor_id, 0.0, 0.0, 0.0, 0.0, 0.0))
        return self._recv_feedback(timeout, motor_id=motor_id)

    def read_enable(self, timeout: Optional[float] = None) -> Optional[Feedback]:
        """Read state by sending Enable, which also energizes the motor."""
        return self.enable(timeout=timeout)

    def init_mit(self, timeout: Optional[float] = None) -> Optional[Feedback]:
        self.set_mit_mode(timeout=timeout)
        time.sleep(0.05)
        return self.enable(timeout=timeout)

    def goto(
        self,
        pos: float,
        rate: float,
        kp: float = _GOTO_KP,
        kd: float = _GOTO_KD,
        torque: float = 0.0,
        hz: float = _GOTO_HZ,
        accel: float = _GOTO_ACCEL,
        feedback_hz: float = 10.0,
        timeout: Optional[float] = None,
        progress: Optional[Callable[[int, Feedback], None]] = None,
    ) -> Optional[Feedback]:
        """Ramp host-side MIT setpoints toward `pos` with velocity and acceleration limits."""
        if rate <= 0.0:
            raise ValueError("rate must be > 0 rad/s")
        if hz <= 0.0:
            raise ValueError("hz must be > 0")
        if accel <= 0.0:
            raise ValueError("accel must be > 0 rad/s^2")
        if feedback_hz <= 0.0:
            raise ValueError("feedback_hz must be > 0")

        dt = 1.0 / hz
        fb = self.read()
        if fb is None:
            return None

        setpoint = fb.pos
        cmd_vel = 0.0
        distance = abs(pos - setpoint)
        if distance <= (rate * rate / accel):
            profile_time = 2.0 * (distance / accel) ** 0.5
        else:
            profile_time = (distance / rate) + (rate / accel)
        move_timeout = timeout if timeout is not None else max(1.0, profile_time + 1.0)
        deadline = time.monotonic() + move_timeout
        prev_time = time.monotonic()
        next_feedback = prev_time
        feedback_dt = 1.0 / feedback_hz
        settled = 0
        idx = 0

        while time.monotonic() < deadline:
            loop_start = time.monotonic()
            elapsed = max(0.0, loop_start - prev_time)
            prev_time = loop_start

            error = pos - setpoint
            if abs(error) <= _GOTO_TOL:
                setpoint = pos
                cmd_vel = 0.0
            else:
                direction = 1.0 if error > 0.0 else -1.0
                stop_limited_speed = (2.0 * accel * abs(error)) ** 0.5
                target_vel = direction * min(rate, stop_limited_speed)
                max_delta_v = accel * max(dt, elapsed)
                delta_v = max(-max_delta_v, min(max_delta_v, target_vel - cmd_vel))
                cmd_vel += delta_v

                step = cmd_vel * max(dt, elapsed)
                if abs(step) >= abs(error) or step * error < 0.0:
                    setpoint = pos
                    cmd_vel = 0.0
                else:
                    setpoint += step

            self.mit_send(
                pos=setpoint,
                vel=cmd_vel,
                kp=kp,
                kd=kd,
                torque=torque,
            )

            next_fb = None
            if loop_start >= next_feedback:
                next_feedback = loop_start + feedback_dt
                next_fb = self._recv_feedback(timeout=0.0)
                if next_fb is not None:
                    fb = next_fb
                    idx += 1
                    if progress is not None:
                        progress(idx, fb)
            else:
                self._drain()

            if abs(setpoint - pos) <= _GOTO_TOL and abs(fb.pos - pos) <= _GOTO_TOL:
                settled += 1
                if settled >= _GOTO_SETTLE_FRAMES:
                    break
            else:
                settled = 0

            remaining = dt - (time.monotonic() - loop_start)
            if remaining > 0.0:
                time.sleep(remaining)

        hold_fb = self.mit_control(pos=pos, vel=0.0, kp=kp, kd=kd, torque=torque)
        return hold_fb if hold_fb is not None else fb

    def zero_to_position(
        self,
        rate: float,
        pos: float = 0.0,
        kp: float = _GOTO_KP,
        kd: float = _GOTO_KD,
        hz: float = _GOTO_HZ,
        accel: float = _GOTO_ACCEL,
        feedback_hz: float = 10.0,
        timeout: Optional[float] = None,
        set_mech_zero: bool = False,
        disable_after: bool = False,
    ) -> Optional[Feedback]:
        fb = self.goto(
            pos=pos,
            rate=rate,
            kp=kp,
            kd=kd,
            hz=hz,
            accel=accel,
            feedback_hz=feedback_hz,
            timeout=timeout,
        )
        if fb is not None and set_mech_zero:
            fb = self.set_zero(timeout=timeout)
        if fb is not None and disable_after:
            disabled_fb = self.disable(timeout=timeout)
            if disabled_fb is not None:
                fb = disabled_fb
        return fb

    def zero_with_velocity(
        self,
        rate: float,
        pos: float = 0.0,
        kd: float = 1.5,
        torque: float = 0.0,
        hz: float = _GOTO_HZ,
        feedback_hz: float = 50.0,
        tolerance: float = _ZERO_VEL_TOL,
        slow_radius: float = 0.35,
        min_rate: float = 0.25,
        timeout: Optional[float] = None,
        disable_after: bool = True,
        progress: Optional[Callable[[int, Feedback], None]] = None,
    ) -> Optional[Feedback]:
        """Creep to `pos` using MIT velocity-only control, then stop and optionally disable."""
        if rate <= 0.0:
            raise ValueError("rate must be > 0 rad/s")
        if kd <= 0.0:
            raise ValueError("kd must be > 0")
        if hz <= 0.0:
            raise ValueError("hz must be > 0")
        if feedback_hz <= 0.0:
            raise ValueError("feedback_hz must be > 0")
        if tolerance <= 0.0:
            raise ValueError("tolerance must be > 0 rad")
        if slow_radius <= 0.0:
            raise ValueError("slow_radius must be > 0 rad")
        if min_rate < 0.0:
            raise ValueError("min_rate must be >= 0 rad/s")

        fb = self.read()
        if fb is None:
            return None

        distance = abs(fb.pos - pos)
        timeout_speed = max(min_rate, rate * 0.35)
        move_timeout = timeout if timeout is not None else max(4.0, (distance / timeout_speed) + 5.0)
        self.set_mit_mode(timeout=0.1)
        time.sleep(0.02)
        self.enable(timeout=0.1)
        time.sleep(0.02)

        dt = 1.0 / hz
        feedback_dt = 1.0 / feedback_hz
        deadline = time.monotonic() + move_timeout
        next_feedback = time.monotonic()
        idx = 0

        while time.monotonic() < deadline:
            loop_start = time.monotonic()
            error = pos - fb.pos
            if abs(error) <= tolerance:
                break

            direction = 1.0 if error > 0.0 else -1.0
            tapered_speed = rate * min(1.0, abs(error) / slow_radius)
            cmd_vel = direction * min(rate, max(min_rate, tapered_speed))

            self.mit_send(pos=pos, vel=cmd_vel, kp=0.0, kd=kd, torque=torque)

            if loop_start >= next_feedback:
                next_feedback = loop_start + feedback_dt
                next_fb = self._recv_feedback(timeout=0.002)
                if next_fb is not None:
                    fb = next_fb
                    idx += 1
                    if progress is not None:
                        progress(idx, fb)
            else:
                self._drain()

            remaining = dt - (time.monotonic() - loop_start)
            if remaining > 0.0:
                time.sleep(remaining)

        stop_fb = self.mit_control(pos=fb.pos, vel=0.0, kp=0.0, kd=kd, torque=0.0, timeout=0.1)
        if stop_fb is not None:
            fb = stop_fb
        if disable_after:
            disabled_fb = self.disable(timeout=0.15)
            if disabled_fb is not None:
                fb = disabled_fb
        return fb

    def sine(
        self,
        amplitude: float,
        frequency: float,
        duration: float,
        center: Optional[float] = None,
        kp: float = _GOTO_KP,
        kd: float = _GOTO_KD,
        torque: float = 0.0,
        hz: float = _GOTO_HZ,
        feedback_hz: float = 20.0,
        ramp_time: float = 1.0,
        disable_after: bool = True,
        progress: Optional[Callable[[int, Feedback, float, float], None]] = None,
    ) -> Optional[Feedback]:
        """Track a sine position command while sending the analytic velocity."""
        if amplitude < 0.0:
            raise ValueError("amplitude must be >= 0 rad")
        if frequency <= 0.0:
            raise ValueError("frequency must be > 0 Hz")
        if duration <= 0.0:
            raise ValueError("duration must be > 0 s")
        if hz <= 0.0:
            raise ValueError("hz must be > 0")
        if feedback_hz <= 0.0:
            raise ValueError("feedback_hz must be > 0")
        if ramp_time < 0.0:
            raise ValueError("ramp_time must be >= 0 s")

        fb = self.read()
        if fb is None:
            return None

        center_pos = fb.pos if center is None else center
        self.set_mit_mode(timeout=0.1)
        time.sleep(0.02)
        self.enable(timeout=0.1)
        time.sleep(0.02)

        dt = 1.0 / hz
        feedback_dt = 1.0 / feedback_hz
        start = time.monotonic()
        next_feedback = start
        idx = 0

        while True:
            loop_start = time.monotonic()
            elapsed = loop_start - start
            if elapsed >= duration:
                break

            cmd_pos, cmd_vel = sine_setpoint(
                center=center_pos,
                amplitude=amplitude,
                frequency=frequency,
                elapsed=elapsed,
                duration=duration,
                ramp_time=ramp_time,
            )
            self.mit_send(pos=cmd_pos, vel=cmd_vel, kp=kp, kd=kd, torque=torque)

            if loop_start >= next_feedback:
                next_feedback = loop_start + feedback_dt
                next_fb = self._recv_feedback(timeout=0.0)
                if next_fb is not None:
                    fb = next_fb
                    idx += 1
                    if progress is not None:
                        progress(idx, fb, cmd_pos, cmd_vel)
            else:
                self._drain()

            remaining = dt - (time.monotonic() - loop_start)
            if remaining > 0.0:
                time.sleep(remaining)

        hold_fb = self.mit_control(pos=center_pos, vel=0.0, kp=kp, kd=kd, torque=0.0, timeout=0.1)
        if hold_fb is not None:
            fb = hold_fb
        if disable_after:
            disabled_fb = self.disable(timeout=0.15)
            if disabled_fb is not None:
                fb = disabled_fb
        return fb

    def sine_many(
        self,
        motor_ids: Sequence[int],
        amplitude: float,
        frequency: float,
        duration: float,
        kp: float = _GOTO_KP,
        kd: float = _GOTO_KD,
        torque: float = 0.0,
        hz: float = _GOTO_HZ,
        feedback_hz: float = 20.0,
        ramp_time: float = 1.0,
        disable_after: bool = True,
        progress: Optional[Callable[[int, dict[int, Feedback], dict[int, tuple[float, float]]], None]] = None,
    ) -> Optional[dict[int, Feedback]]:
        """Track the same sine command on multiple motors over one serial connection."""
        ids = list(motor_ids)
        if not ids:
            raise ValueError("at least one motor ID is required")
        if len(set(ids)) != len(ids):
            raise ValueError("motor IDs must be unique")
        if amplitude < 0.0:
            raise ValueError("amplitude must be >= 0 rad")
        if frequency <= 0.0:
            raise ValueError("frequency must be > 0 Hz")
        if duration <= 0.0:
            raise ValueError("duration must be > 0 s")
        if hz <= 0.0:
            raise ValueError("hz must be > 0")
        if feedback_hz <= 0.0:
            raise ValueError("feedback_hz must be > 0")
        if ramp_time < 0.0:
            raise ValueError("ramp_time must be >= 0 s")

        feedback: dict[int, Feedback] = {}
        centers: dict[int, float] = {}
        for motor_id in ids:
            fb = self.read_for(motor_id)
            if fb is None:
                self.set_mit_mode_for(motor_id, timeout=0.1)
                time.sleep(0.02)
                self._request_feedback_for(motor_id, encode_enable, timeout=0.1)
                time.sleep(0.02)
                fb = self.read_for(motor_id)
            if fb is None:
                return None
            feedback[motor_id] = fb
            centers[motor_id] = fb.pos

        for motor_id in ids:
            self.set_mit_mode_for(motor_id, timeout=0.1)
            time.sleep(0.02)
            self._request_feedback_for(motor_id, encode_enable, timeout=0.1)
            time.sleep(0.02)

        dt = 1.0 / hz
        feedback_dt = 1.0 / feedback_hz
        start = time.monotonic()
        next_feedback = start
        idx = 0

        while True:
            loop_start = time.monotonic()
            elapsed = loop_start - start
            if elapsed >= duration:
                break

            commands = {
                motor_id: sine_setpoint(
                    center=centers[motor_id],
                    amplitude=amplitude,
                    frequency=frequency,
                    elapsed=elapsed,
                    duration=duration,
                    ramp_time=ramp_time,
                )
                for motor_id in ids
            }
            for motor_id, (cmd_pos, cmd_vel) in commands.items():
                self.mit_send_for(motor_id, pos=cmd_pos, vel=cmd_vel, kp=kp, kd=kd, torque=torque)

            if loop_start >= next_feedback:
                next_feedback = loop_start + feedback_dt
                for i in range(len(ids)):
                    next_fb = self._recv_any_feedback(timeout=0.002 if i == 0 else 0.0)
                    if next_fb is not None and next_fb.motor_id in centers:
                        feedback[next_fb.motor_id] = next_fb
                idx += 1
                if progress is not None:
                    progress(idx, feedback, commands)
            else:
                self._drain()

            remaining = dt - (time.monotonic() - loop_start)
            if remaining > 0.0:
                time.sleep(remaining)

        for motor_id in ids:
            self.mit_send_for(motor_id, pos=centers[motor_id], vel=0.0, kp=kp, kd=kd, torque=0.0)
        time.sleep(0.05)

        for motor_id in ids:
            hold_fb = self.read_for(motor_id, timeout=0.05)
            if hold_fb is not None:
                feedback[motor_id] = hold_fb

        if disable_after:
            for motor_id in ids:
                disabled_fb = self._request_feedback_for(motor_id, encode_disable, timeout=0.1)
                if disabled_fb is not None:
                    feedback[motor_id] = disabled_fb

        return feedback
