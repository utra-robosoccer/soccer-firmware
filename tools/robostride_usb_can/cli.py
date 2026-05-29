"""Raw CH341 CLI for Robstride USB-CAN debugging.

Examples:
  python3 tools/robostride_usb_can/cli.py read 1
  python3 tools/robostride_usb_can/cli.py init 1
  python3 tools/robostride_usb_can/cli.py mit 1 0 0 0 0 0
  python3 tools/robostride_usb_can/cli.py command 1 0 0 15 1 0 --duration 2
  python3 tools/robostride_usb_can/cli.py sine 1 --amp 0.20 --freq 0.25 --duration 8
  python3 tools/robostride_usb_can/cli.py sine-chain 1 2 --amp 0.20 --freq 0.25 --duration 8
  python3 tools/robostride_usb_can/cli.py goto 1 1.57 0.30
  python3 tools/robostride_usb_can/cli.py zero 1 --rate 0.30
  python3 tools/robostride_usb_can/cli.py zero-vel 1 --rate 0.50
  python3 tools/robostride_usb_can/cli.py off 1
  python3 tools/robostride_usb_can/cli.py scan-ids
  python3 tools/robostride_usb_can/cli.py set-zero 1
  python3 tools/robostride_usb_can/cli.py set-id 1 2
"""
import os
import time
from typing import Optional

import click

from rs02_can import Feedback
from rs02_motor import DEFAULT_BAUD, DEFAULT_PORT, RS02Motor

DEFAULT_CLI_PORT = os.environ.get("RS02_PORT", DEFAULT_PORT)
DEFAULT_CLI_BAUD = int(os.environ.get("RS02_BAUD", str(DEFAULT_BAUD)))


def _fmt(idx: int, fb: Optional[Feedback]) -> str:
    return f"[{idx}] {fb}" if fb else f"[{idx}] no feedback"


@click.group()
@click.option("--port", default=DEFAULT_CLI_PORT, show_default=True)
@click.option("--baud", default=DEFAULT_CLI_BAUD, show_default=True)
@click.option("--rx-timeout", default=0.15, show_default=True, help="feedback wait time per frame")
@click.pass_context
def cli(ctx: click.Context, port: str, baud: int, rx_timeout: float) -> None:
    """Robstride commands over raw CH341 USB-CAN serial framing."""
    ctx.ensure_object(dict)
    ctx.obj.update(port=port, baud=baud, rx_timeout=rx_timeout)


def _motor(ctx: click.Context, motor_id: int) -> RS02Motor:
    o = ctx.obj
    return RS02Motor(
        port=o["port"],
        baud=o["baud"],
        motor_id=motor_id,
        rx_timeout=o["rx_timeout"],
    )


def _validate_motor_id(value: int, name: str = "motor id") -> None:
    if value <= 0 or value > 0x7F:
        raise click.BadParameter(f"{name} must be in range 1..127")


@cli.command()
@click.argument("motor_id", type=int)
@click.pass_context
def init(ctx: click.Context, motor_id: int) -> None:
    """Set MIT mode, enable, then read feedback."""
    with _motor(ctx, motor_id) as m:
        print(_fmt(1, m.set_mit_mode()))
        time.sleep(0.05)
        print(_fmt(2, m.enable()))
        time.sleep(0.05)
        print(_fmt(3, m.read()))


@cli.command("mit-mode")
@click.argument("motor_id", type=int)
@click.pass_context
def mit_mode(ctx: click.Context, motor_id: int) -> None:
    """Set motor run mode to MIT."""
    with _motor(ctx, motor_id) as m:
        print(_fmt(1, m.set_mit_mode()))


@cli.command()
@click.argument("motor_id", type=int)
@click.option("--enable", "use_enable", is_flag=True, help="read by sending Enable instead of zero-gain MIT")
@click.pass_context
def read(ctx: click.Context, motor_id: int, use_enable: bool) -> None:
    """Read one motor feedback frame."""
    with _motor(ctx, motor_id) as m:
        print(_fmt(1, m.read_enable() if use_enable else m.read()))


@cli.command("get-id")
@click.argument("motor_id", type=int)
@click.pass_context
def get_id(ctx: click.Context, motor_id: int) -> None:
    """Read one motor's type-0 device ID / MCU UID."""
    _validate_motor_id(motor_id)
    with _motor(ctx, motor_id) as m:
        device_id = m.get_device_id()
        if device_id is None:
            raise click.ClickException(f"no type-0 ID response from target ID {motor_id}")
        print(device_id)


@cli.command("scan-ids")
@click.option("--start", default=1, show_default=True, help="first target ID to probe")
@click.option("--end", default=127, show_default=True, help="last target ID to probe")
@click.option("--timeout", default=0.04, show_default=True, help="feedback wait time per ID")
@click.pass_context
def scan_ids(ctx: click.Context, start: int, end: int, timeout: float) -> None:
    """Probe target IDs and print any Robstride type-0 ID responses."""
    _validate_motor_id(start, "start ID")
    _validate_motor_id(end, "end ID")
    if end < start:
        raise click.BadParameter("end must be >= start")
    if timeout <= 0.0:
        raise click.BadParameter("timeout must be > 0")

    found = []
    with _motor(ctx, start) as m:
        for target_id in range(start, end + 1):
            m.motor_id = target_id
            device_id = m.get_device_id(timeout=timeout)
            if device_id is not None:
                found.append(device_id)
                print(device_id)

    if not found:
        raise click.ClickException(f"no motors responded in ID range {start}..{end}")


@cli.command("read-loop")
@click.argument("motor_id", type=int)
@click.option("--hz", default=10.0, show_default=True, help="poll rate in Hz")
@click.option("--duration", default=5.0, show_default=True, help="run time in seconds")
@click.option("--enable", "use_enable", is_flag=True, help="read by sending Enable instead of zero-gain MIT")
@click.pass_context
def read_loop(ctx: click.Context, motor_id: int, hz: float, duration: float, use_enable: bool) -> None:
    """Poll feedback repeatedly."""
    if hz <= 0.0:
        raise click.BadParameter("hz must be > 0")

    dt = 1.0 / hz
    end = time.monotonic() + duration
    idx = 1
    with _motor(ctx, motor_id) as m:
        try:
            while time.monotonic() < end:
                loop_start = time.monotonic()
                print(_fmt(idx, m.read_enable() if use_enable else m.read()))
                idx += 1
                remaining = dt - (time.monotonic() - loop_start)
                if remaining > 0.0:
                    time.sleep(remaining)
        except KeyboardInterrupt:
            pass


@cli.command()
@click.argument("motor_id", type=int)
@click.argument("pos", type=float)
@click.argument("vel", type=float)
@click.argument("kp", type=float)
@click.argument("kd", type=float)
@click.argument("torq", type=float)
@click.pass_context
def mit(
    ctx: click.Context,
    motor_id: int,
    pos: float,
    vel: float,
    kp: float,
    kd: float,
    torq: float,
) -> None:
    """Send one MIT frame: mit ID POS VEL KP KD TORQ."""
    with _motor(ctx, motor_id) as m:
        print(_fmt(1, m.mit_control(pos=pos, vel=vel, kp=kp, kd=kd, torque=torq)))


@cli.command()
@click.argument("motor_id", type=int)
@click.argument("pos", type=float)
@click.argument("vel", type=float)
@click.argument("kp", type=float)
@click.argument("kd", type=float)
@click.argument("torq", type=float)
@click.option("--hz", default=100.0, show_default=True)
@click.option("--duration", default=2.0, show_default=True)
@click.pass_context
def command(
    ctx: click.Context,
    motor_id: int,
    pos: float,
    vel: float,
    kp: float,
    kd: float,
    torq: float,
    hz: float,
    duration: float,
) -> None:
    """Continuously send one MIT command for DURATION seconds."""
    if hz <= 0.0:
        raise click.BadParameter("hz must be > 0")

    dt = 1.0 / hz
    end = time.monotonic() + duration
    idx = 1
    with _motor(ctx, motor_id) as m:
        try:
            while time.monotonic() < end:
                loop_start = time.monotonic()
                fb = m.mit_control(pos=pos, vel=vel, kp=kp, kd=kd, torque=torq)
                print(_fmt(idx, fb))
                idx += 1
                remaining = dt - (time.monotonic() - loop_start)
                if remaining > 0.0:
                    time.sleep(remaining)
        except KeyboardInterrupt:
            pass


@cli.command("sine")
@click.argument("motor_id", type=int)
@click.option("--amp", default=0.20, show_default=True, help="sine amplitude in rad")
@click.option("--freq", default=0.25, show_default=True, help="sine frequency in Hz")
@click.option("--duration", default=8.0, show_default=True, help="run time in seconds")
@click.option("--center", type=float, default=None, help="center position in rad; defaults to current position")
@click.option("--kp", default=15.0, show_default=True)
@click.option("--kd", default=1.0, show_default=True)
@click.option("--torq", default=0.0, show_default=True)
@click.option("--hz", default=100.0, show_default=True, help="MIT command send rate in Hz")
@click.option("--feedback-hz", default=20.0, show_default=True, help="feedback sampling rate in Hz")
@click.option("--ramp-time", default=1.0, show_default=True, help="amplitude fade-in/out time in seconds")
@click.option("--hold", is_flag=True, help="keep motor enabled after the sine test")
@click.pass_context
def sine(
    ctx: click.Context,
    motor_id: int,
    amp: float,
    freq: float,
    duration: float,
    center: Optional[float],
    kp: float,
    kd: float,
    torq: float,
    hz: float,
    feedback_hz: float,
    ramp_time: float,
    hold: bool,
) -> None:
    """Track a sine wave with position and analytic velocity setpoints."""
    def progress(idx: int, fb: Feedback, cmd_pos: float, cmd_vel: float) -> None:
        if idx == 1 or idx % 10 == 0:
            print(f"[{idx}] cmd_pos={cmd_pos:.3f} cmd_vel={cmd_vel:.3f} {fb}")

    with _motor(ctx, motor_id) as m:
        fb = m.sine(
            amplitude=amp,
            frequency=freq,
            duration=duration,
            center=center,
            kp=kp,
            kd=kd,
            torque=torq,
            hz=hz,
            feedback_hz=feedback_hz,
            ramp_time=ramp_time,
            disable_after=not hold,
            progress=progress,
        )
        print(_fmt(0, fb))
        if fb is None:
            raise click.ClickException("no initial feedback; refusing to run sine blindly")


@cli.command("sine-chain")
@click.argument("motor_ids", nargs=-1, type=int)
@click.option("--amp", default=0.20, show_default=True, help="sine amplitude in rad")
@click.option("--freq", default=0.25, show_default=True, help="sine frequency in Hz")
@click.option("--duration", default=8.0, show_default=True, help="run time in seconds")
@click.option("--kp", default=15.0, show_default=True)
@click.option("--kd", default=1.0, show_default=True)
@click.option("--torq", default=0.0, show_default=True)
@click.option("--hz", default=100.0, show_default=True, help="MIT command send rate in Hz")
@click.option("--feedback-hz", default=20.0, show_default=True, help="feedback sampling rate in Hz")
@click.option("--ramp-time", default=1.0, show_default=True, help="amplitude fade-in/out time in seconds")
@click.option("--hold", is_flag=True, help="keep motors enabled after the sine test")
@click.pass_context
def sine_chain(
    ctx: click.Context,
    motor_ids: tuple[int, ...],
    amp: float,
    freq: float,
    duration: float,
    kp: float,
    kd: float,
    torq: float,
    hz: float,
    feedback_hz: float,
    ramp_time: float,
    hold: bool,
) -> None:
    """Track the same sine wave on multiple motor IDs."""
    if len(motor_ids) < 2:
        raise click.BadParameter("provide at least two motor IDs")
    if len(set(motor_ids)) != len(motor_ids):
        raise click.BadParameter("motor IDs must be unique")
    for motor_id in motor_ids:
        _validate_motor_id(motor_id)

    def progress(
        idx: int,
        feedback: dict[int, Feedback],
        commands: dict[int, tuple[float, float]],
    ) -> None:
        if idx == 1 or idx % 10 == 0:
            cmd_text = " ".join(
                f"id={motor_id} cmd_pos={commands[motor_id][0]:.3f} cmd_vel={commands[motor_id][1]:.3f}"
                for motor_id in motor_ids
            )
            fb_text = " | ".join(
                f"id={motor_id} {feedback[motor_id]}" for motor_id in motor_ids if motor_id in feedback
            )
            print(f"[{idx}] {cmd_text} | {fb_text}")

    with _motor(ctx, motor_ids[0]) as m:
        feedback = m.sine_many(
            motor_ids=motor_ids,
            amplitude=amp,
            frequency=freq,
            duration=duration,
            kp=kp,
            kd=kd,
            torque=torq,
            hz=hz,
            feedback_hz=feedback_hz,
            ramp_time=ramp_time,
            disable_after=not hold,
            progress=progress,
        )
        if feedback is None:
            raise click.ClickException("no initial feedback from every requested motor; refusing to run sine blindly")
        for motor_id in motor_ids:
            print(_fmt(motor_id, feedback.get(motor_id)))


@cli.command()
@click.argument("motor_id", type=int)
@click.argument("pos", type=float)
@click.argument("rate", type=float)
@click.option("--kp", default=15.0, show_default=True)
@click.option("--kd", default=1.0, show_default=True)
@click.option("--torq", default=0.0, show_default=True)
@click.option("--hz", default=100.0, show_default=True)
@click.option("--accel", default=0.05, show_default=True, help="acceleration limit in rad/s^2")
@click.option("--feedback-hz", default=10.0, show_default=True, help="feedback sampling rate in Hz")
@click.option("--timeout", type=float, default=None, help="overall move timeout in seconds")
@click.pass_context
def goto(
    ctx: click.Context,
    motor_id: int,
    pos: float,
    rate: float,
    kp: float,
    kd: float,
    torq: float,
    hz: float,
    accel: float,
    feedback_hz: float,
    timeout: Optional[float],
) -> None:
    """Ramp to position: goto ID POS RATE."""
    with _motor(ctx, motor_id) as m:
        fb = m.goto(
            pos=pos,
            rate=rate,
            kp=kp,
            kd=kd,
            torque=torq,
            hz=hz,
            accel=accel,
            feedback_hz=feedback_hz,
            timeout=timeout,
        )
        print(_fmt(1, fb))
        if fb is None:
            raise click.ClickException("no initial feedback; refusing to ramp blindly")


@cli.command()
@click.argument("motor_id", type=int)
@click.option("--rate", default=0.30, show_default=True, help="slow ramp rate in rad/s")
@click.option("--pos", default=0.0, show_default=True, help="zero target in rad")
@click.option("--kp", default=15.0, show_default=True)
@click.option("--kd", default=1.0, show_default=True)
@click.option("--hz", default=200.0, show_default=True, help="MIT command send rate in Hz")
@click.option("--accel", default=0.30, show_default=True, help="acceleration limit in rad/s^2")
@click.option("--feedback-hz", default=10.0, show_default=True, help="feedback sampling rate in Hz")
@click.option("--timeout", type=float, default=None, help="overall move timeout in seconds")
@click.option("--set-mech-zero", is_flag=True, help="send SetZero after the slow ramp")
@click.option("--hold", is_flag=True, help="keep motor enabled after reaching zero")
@click.pass_context
def zero(
    ctx: click.Context,
    motor_id: int,
    rate: float,
    pos: float,
    kp: float,
    kd: float,
    hz: float,
    accel: float,
    feedback_hz: float,
    timeout: Optional[float],
    set_mech_zero: bool,
    hold: bool,
) -> None:
    """Slowly ramp to the zero position, then disable by default."""
    with _motor(ctx, motor_id) as m:
        fb = m.zero_to_position(
            rate=rate,
            pos=pos,
            kp=kp,
            kd=kd,
            hz=hz,
            accel=accel,
            feedback_hz=feedback_hz,
            timeout=timeout,
            set_mech_zero=set_mech_zero,
            disable_after=not hold,
        )
        print(_fmt(1, fb))
        if fb is None:
            raise click.ClickException("no initial feedback; refusing to zero blindly")


@cli.command("zero-vel")
@click.argument("motor_id", type=int)
@click.option("--rate", default=0.50, show_default=True, help="velocity command away/toward zero in rad/s")
@click.option("--pos", default=0.0, show_default=True, help="zero target in rad")
@click.option("--kd", default=1.5, show_default=True, help="MIT damping/velocity gain")
@click.option("--torq", default=0.0, show_default=True)
@click.option("--hz", default=100.0, show_default=True, help="MIT command send rate in Hz")
@click.option("--feedback-hz", default=50.0, show_default=True, help="feedback sampling rate in Hz")
@click.option("--tolerance", default=0.03, show_default=True, help="stop when within this many rad")
@click.option("--slow-radius", default=0.35, show_default=True, help="begin reducing commanded speed inside this error")
@click.option("--min-rate", default=0.25, show_default=True, help="minimum commanded speed while outside tolerance")
@click.option("--timeout", type=float, default=None, help="overall move timeout in seconds")
@click.option("--hold", is_flag=True, help="keep motor enabled after reaching zero")
@click.pass_context
def zero_vel(
    ctx: click.Context,
    motor_id: int,
    rate: float,
    pos: float,
    kd: float,
    torq: float,
    hz: float,
    feedback_hz: float,
    tolerance: float,
    slow_radius: float,
    min_rate: float,
    timeout: Optional[float],
    hold: bool,
) -> None:
    """Creep to zero using MIT velocity-only control, then disable by default."""
    def progress(idx: int, fb: Feedback) -> None:
        if idx == 1 or idx % 10 == 0:
            print(_fmt(idx, fb))

    with _motor(ctx, motor_id) as m:
        fb = m.zero_with_velocity(
            rate=rate,
            pos=pos,
            kd=kd,
            torque=torq,
            hz=hz,
            feedback_hz=feedback_hz,
            tolerance=tolerance,
            slow_radius=slow_radius,
            min_rate=min_rate,
            timeout=timeout,
            disable_after=not hold,
            progress=progress,
        )
        print(_fmt(0, fb))
        if fb is None:
            raise click.ClickException("no initial feedback; refusing to zero blindly")
        if abs(fb.pos - pos) > tolerance:
            raise click.ClickException(
                f"zero-vel stopped at pos={fb.pos:.3f}, outside tolerance={tolerance:.3f}; "
                "increase --timeout, --rate, --kd, or --min-rate"
            )


@cli.command()
@click.argument("motor_id", type=int)
@click.pass_context
def enable(ctx: click.Context, motor_id: int) -> None:
    """Enable motor output."""
    with _motor(ctx, motor_id) as m:
        print(_fmt(1, m.enable()))


@cli.command()
@click.argument("motor_id", type=int)
@click.pass_context
def disable(ctx: click.Context, motor_id: int) -> None:
    """Disable motor output."""
    with _motor(ctx, motor_id) as m:
        print(_fmt(1, m.disable()))


@cli.command()
@click.argument("motor_id", type=int)
@click.pass_context
def off(ctx: click.Context, motor_id: int) -> None:
    """Disable motor output so it can be rotated manually."""
    with _motor(ctx, motor_id) as m:
        print(_fmt(1, m.disable()))


@cli.command("set-zero")
@click.argument("motor_id", type=int)
@click.option("--no-stop", is_flag=True, help="do not send Disable before SetZero")
@click.pass_context
def set_zero(ctx: click.Context, motor_id: int, no_stop: bool) -> None:
    """Set current mechanical position as zero using raw comm type 6."""
    with _motor(ctx, motor_id) as m:
        print(_fmt(1, m.set_zero(stop_first=not no_stop)))


@cli.command("set-id")
@click.argument("motor_id", type=int)
@click.argument("new_motor_id", type=int)
@click.pass_context
def set_id(ctx: click.Context, motor_id: int, new_motor_id: int) -> None:
    """Read current ID, disable output, set a new CAN ID, then verify."""
    _validate_motor_id(motor_id, "current motor id")
    _validate_motor_id(new_motor_id, "new motor id")
    if motor_id == new_motor_id:
        raise click.BadParameter("new motor id must be different from current motor id")

    with _motor(ctx, motor_id) as m:
        before = m.read()
        print(_fmt(1, before))
        if before is None:
            raise click.ClickException("no feedback from current ID; refusing to change motor ID")

        disabled = m.disable()
        print(_fmt(2, disabled))
        if disabled is None:
            raise click.ClickException("no feedback after Disable; refusing to change motor ID")
        time.sleep(0.1)

        after = m.set_motor_id(new_motor_id)
        print(_fmt(3, after))
        if after is None:
            raise click.ClickException(
                f"sent ID change {motor_id}->{new_motor_id}, but no feedback from new ID; "
                "check wiring and try reading both IDs"
            )
        if after.motor_id != new_motor_id:
            raise click.ClickException(f"expected feedback from ID {new_motor_id}, got ID {after.motor_id}")


if __name__ == "__main__":
    cli()
