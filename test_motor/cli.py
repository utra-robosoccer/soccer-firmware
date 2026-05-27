"""Interactive CLI for RS02 motor testing via CH341 USB-CAN.

Usage examples:
  python cli.py enable 1
  python cli.py read 1
  python cli.py read_loop 1 --hz 10 --duration 5
  python cli.py set_zero 1
  python cli.py goto_zero 1 --speed 0.5
  python cli.py goto 1 --pos 3.14 --speed 1.0
  python cli.py mit 1 --pos 0 --vel 0 --kp 15 --kd 1 --torq 0
  python cli.py disable 1

Output format: [N] pos=1.234 vel=0.567 torq=0.123 temp=45.0 mode=0 faults=none
"""
import time
from typing import Optional
import click
import can

from rs02_motor import RS02Motor
from rs02_can import Feedback


def _open_bus(interface: str, channel: str, bitrate: int) -> can.BusABC:
    return can.Bus(interface=interface, channel=channel, bitrate=bitrate)


def _fmt(idx: int, fb: Optional[Feedback]) -> str:
    return f"[{idx}] {fb}" if fb else f"[{idx}] no feedback"


@click.group()
@click.option("--interface", default="ch341",        show_default=True, help="python-can interface name")
@click.option("--channel",   default="/dev/ttyUSB0", show_default=True)
@click.option("--bitrate",   default=1_000_000,      show_default=True)
@click.pass_context
def cli(ctx: click.Context, interface: str, channel: str, bitrate: int) -> None:
    ctx.ensure_object(dict)
    ctx.obj.update(interface=interface, channel=channel, bitrate=bitrate)


def _motor(ctx: click.Context, motor_id: int) -> tuple[can.BusABC, RS02Motor]:
    o = ctx.obj
    bus = _open_bus(o["interface"], o["channel"], o["bitrate"])
    return bus, RS02Motor(bus, motor_id)


@cli.command()
@click.argument("motor_id", type=int)
@click.pass_context
def enable(ctx: click.Context, motor_id: int) -> None:
    """Enable motor (comm_type=3)."""
    bus, m = _motor(ctx, motor_id)
    print(_fmt(1, m.enable()))
    bus.shutdown()


@cli.command()
@click.argument("motor_id", type=int)
@click.pass_context
def disable(ctx: click.Context, motor_id: int) -> None:
    """Disable motor (comm_type=4)."""
    bus, m = _motor(ctx, motor_id)
    print(_fmt(1, m.disable()))
    bus.shutdown()


@cli.command()
@click.argument("motor_id", type=int)
@click.pass_context
def read(ctx: click.Context, motor_id: int) -> None:
    """Read one feedback frame."""
    bus, m = _motor(ctx, motor_id)
    print(_fmt(1, m.read()))
    bus.shutdown()


@cli.command("read_loop")
@click.argument("motor_id", type=int)
@click.option("--hz",       default=10.0, show_default=True, help="poll rate (Hz)")
@click.option("--duration", default=5.0,  show_default=True, help="run time (s)")
@click.pass_context
def read_loop(ctx: click.Context, motor_id: int, hz: float, duration: float) -> None:
    """Poll feedback at HZ for DURATION seconds."""
    bus, m = _motor(ctx, motor_id)
    dt = 1.0 / hz
    end = time.monotonic() + duration
    idx = 1
    try:
        while time.monotonic() < end:
            t0 = time.monotonic()
            print(_fmt(idx, m.read()))
            idx += 1
            remaining = dt - (time.monotonic() - t0)
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        pass
    finally:
        bus.shutdown()


@cli.command("set_zero")
@click.argument("motor_id", type=int)
@click.pass_context
def set_zero(ctx: click.Context, motor_id: int) -> None:
    """Set current position as the zero reference (comm_type=6)."""
    bus, m = _motor(ctx, motor_id)
    print(_fmt(1, m.set_zero()))
    bus.shutdown()


@cli.command("goto_zero")
@click.argument("motor_id", type=int)
@click.option("--speed", default=1.0, show_default=True, help="velocity feed-forward (rad/s)")
@click.pass_context
def goto_zero(ctx: click.Context, motor_id: int, speed: float) -> None:
    """Drive motor to position 0."""
    bus, m = _motor(ctx, motor_id)
    print(_fmt(1, m.goto(pos=0.0, speed=speed)))
    bus.shutdown()


@cli.command()
@click.argument("motor_id", type=int)
@click.option("--pos",   required=True, type=float, help="target position (rad)")
@click.option("--speed", default=1.0,  show_default=True, help="velocity feed-forward (rad/s)")
@click.pass_context
def goto(ctx: click.Context, motor_id: int, pos: float, speed: float) -> None:
    """Drive motor to target position."""
    bus, m = _motor(ctx, motor_id)
    print(_fmt(1, m.goto(pos=pos, speed=speed)))
    bus.shutdown()


@cli.command()
@click.argument("motor_id", type=int)
@click.option("--pos",  default=0.0, show_default=True)
@click.option("--vel",  default=0.0, show_default=True)
@click.option("--kp",   default=0.0, show_default=True)
@click.option("--kd",   default=0.0, show_default=True)
@click.option("--torq", default=0.0, show_default=True)
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
    """Send a raw MIT control frame and print feedback."""
    bus, m = _motor(ctx, motor_id)
    print(_fmt(1, m.mit_control(pos=pos, vel=vel, kp=kp, kd=kd, torque=torq)))
    bus.shutdown()


if __name__ == "__main__":
    cli()
