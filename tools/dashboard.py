#!/usr/bin/env python3
"""tools/dashboard.py — in-place rich TUI dashboard for soccer-firmware motor control.

Usage:  python3 tools/dashboard.py [PORT]
        PORT defaults to /dev/ttyACM0

Same keyboard controls as test_client.py:
  1-5   select active motor
  a/A   ARM_HOLD  active / all
  z/Z   GOTO_ZERO active / all
  s/S   SINE      active / all (toggle ±90° @ 0.5 Hz — arm first)
  d/D   DISABLE   active / all
  p     PING      q   QUIT
"""

import math
import os
import struct
import sys
import termios
import threading
import time
import tty
from collections import deque

import serial
from rich import box
from rich.console import Console, Group
from rich.live import Live
from rich.panel import Panel
from rich.rule import Rule
from rich.table import Table
from rich.text import Text

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_client as tc
from motor_config_gen import (MOTOR_DEFAULT_KD, MOTOR_DEFAULT_KP, MOTORS, N_MOTORS,
                              N_SLAVES, SLAVES, SLAVE_MOTOR_COUNTS)

# ── shared state ──────────────────────────────────────────────────────────────

class _MotorSnap:
    __slots__ = ("state", "pos", "vel", "tau", "temp", "fault", "updated")
    def __init__(self):
        self.state   = 2            # IDLE
        self.pos     = float("nan")
        self.vel     = float("nan")
        self.tau     = float("nan")
        self.temp    = float("nan")
        self.fault   = 0
        self.updated = 0.0

_lock   = threading.Lock()
_snap   = [_MotorSnap() for _ in range(N_MOTORS)]   # by global flattened index
_link   = {"rx": 0, "err": 0, "alive": 0, "uptime_ms": 0}
_master = {"robot_state": 0, "slave_alive": 0}
_slaves = [{"motors_alive": 0, "state": 0} for _ in range(N_SLAVES)]
_events: deque = deque(maxlen=6)

# Sine-streaming state, shared with the RX thread (populated in main()). Module
# scope so _ingest() can auto-stop a motor's sine when the slave drops it out of
# its armed state (e.g. the torque trip idles it).
sine_active: list = []
sine_stop:   list = []

# Slave lifecycle states that count as "armed" (actively driven). Mirrors
# protocol.h: ARMED_HOLD=3, ARMED_MIT=7.
_ARMED_STATES = (3, 7)


def _log(msg: str) -> None:
    ts = time.strftime("%H:%M:%S")
    with _lock:
        _events.appendleft(f"[dim]{ts}[/dim]  {msg}")


def _auto_stop_sine(idx: int) -> None:
    """Stop a motor's host-side sine when the motor leaves its armed state.

    The slave's torque trip idles a motor on overload; without this the sine
    thread keeps streaming, the ~ indicator stays lit, and a subsequent
    GOTO_ZERO gets overrun by the resumed sine the moment the motor re-arms."""
    if idx < len(sine_active) and sine_active[idx]:
        sine_stop[idx].set()
        sine_active[idx] = False
        _log(f"SINE motor {idx+1} [yellow]auto-stopped[/yellow] (motor left armed state)")


# ── state colour map ──────────────────────────────────────────────────────────

_STATE_STYLE = {
    "ARMED_MIT":   "bold magenta",
    "ARMED_HOLD":  "bold green",
    "ZEROING":     "bold cyan",
    "FAULT":       "bold red",
    "IDLE":        "yellow",
    "DISABLED":    "dim white",
    "BOOT":        "blue",
    "DISCOVERING": "blue",
}

# ── renderer (called from main thread only) ───────────────────────────────────

def _render(active: int, sine_active: list, port: str) -> Panel:
    with _lock:
        snaps  = [(s.state, s.pos, s.vel, s.tau, s.temp, s.fault, s.updated)
                  for s in _snap]
        rx     = _link["rx"]
        err    = _link["err"]
        uptime = _link["uptime_ms"]
        robot  = _master["robot_state"]
        salive = _master["slave_alive"]
        slaves = [dict(s) for s in _slaves]
        evts   = list(_events)

    now = time.monotonic()

    tbl = Table(box=box.SIMPLE_HEAVY, show_header=True,
                header_style="bold white", expand=True, padding=(0, 1))
    tbl.add_column("",        width=2,  justify="center")   # active marker
    tbl.add_column("id",      width=3,  justify="center")
    tbl.add_column("slv",     width=3,  justify="center")
    tbl.add_column("joint",   width=8)
    tbl.add_column("model",   width=5)
    tbl.add_column("state",   width=11)
    tbl.add_column("pos rad", width=9,  justify="right")
    tbl.add_column("vel r/s", width=9,  justify="right")
    tbl.add_column("tau Nm",  width=8,  justify="right")
    tbl.add_column("°C",      width=5,  justify="right")
    tbl.add_column("~",       width=2,  justify="center")   # sine indicator
    tbl.add_column("fault",   width=6,  justify="center")

    def fv(v):
        return f"{v:+.4f}" if v == v else "  ---  "

    gi = 0
    for s_idx, slave in enumerate(SLAVES):
        if s_idx > 0:
            tbl.add_section()
        online = bool(salive & (1 << s_idx))
        for cfg in slave["motors"]:
            st, pos, vel, tau, temp, fault, updated = snaps[gi]
            sel   = (gi == active)
            stale = (not online) or ((now - updated) > 0.5 if updated else True)
            vs    = "dim white" if stale else "white"

            sname   = tc.MOTOR_STATE_NAMES.get(st, f"?{st}")
            state_t = (Text("OFFLINE", style="dim red") if not online
                       else Text(sname, style=_STATE_STYLE.get(sname, "")))

            tbl.add_row(
                Text("▶", style="bold cyan") if sel else Text(""),
                Text(str(gi + 1), style="bold" if sel else ""),
                Text(f"s{s_idx}", style="cyan" if online else "dim red"),
                cfg["joint_name"],
                cfg["model"],
                state_t,
                Text(fv(pos),  style=vs),
                Text(fv(vel),  style=vs),
                Text(fv(tau),  style=vs),
                Text(f"{temp:.0f}" if temp == temp else " --", style=vs),
                Text("◉", style="bold magenta") if sine_active[gi] else Text("·", style="dim"),
                Text("OK",            style="green")    if not fault else
                Text(f"{fault:04x}", style="bold red"),
                style="on dark_blue" if sel else "",
            )
            gi += 1

    m = MOTORS[active]
    sine_on = [i + 1 for i in range(N_MOTORS) if sine_active[i]]
    sine_info = f"  [magenta]sine: {sine_on}[/magenta]" if sine_on else ""

    # Per-slave summary: ONLINE/OFFLINE + motors alive count.
    parts = []
    for s_idx in range(N_SLAVES):
        online = bool(salive & (1 << s_idx))
        cnt    = SLAVE_MOTOR_COUNTS[s_idx]
        nalive = bin(slaves[s_idx]["motors_alive"]).count("1")
        if online:
            parts.append(f"[green]slave{s_idx} ONLINE {nalive}/{cnt}[/green]")
        else:
            parts.append(f"[dim red]slave{s_idx} OFFLINE[/dim red]")
    rs_name = tc.ROBOT_STATE_NAMES.get(robot, "?")
    rs_style = {"READY": "bold green", "DEGRADED": "bold yellow",
                "INIT": "bold blue"}.get(rs_name, "white")

    status = Text.from_markup(
        f"[{rs_style}]robot: {rs_name}[/]   " + "   ".join(parts) +
        f"   [dim]rx={rx} err={err} up={uptime//1000}s[/dim]{sine_info}\n"
        f"[bold]active: id {active+1}[/bold]  s{m['slave']}.m{m['idx']} · "
        f"{m['joint_name']} · CAN {m['can_id']} · {m['model']}"
    )

    keys = Text.from_markup(
        "[dim]"
        f"[bold white]1–{N_MOTORS}[/] select  "
        "[bold white]a[/] ARM  [bold white]z[/] ZERO  [bold white]s[/] SINE  [bold white]d[/] DISABLE  "
        "│  [bold white]A/Z/S/D[/] all  "
        "[bold white]p[/] PING  [bold white]q[/] QUIT"
        "[/dim]"
    )

    log_lines = "\n".join(evts) if evts else "[dim](no events yet)[/dim]"

    body = Group(
        tbl,
        Rule(style="dim blue"),
        status,
        keys,
        Rule(style="dim blue"),
        Text.from_markup(log_lines),
    )
    return Panel(body,
                 title=f"[bold white]⚽ Motor Dashboard[/bold white]  [dim]{port}[/dim]",
                 border_style="blue")


# ── serial RX thread ──────────────────────────────────────────────────────────

def _serial_rx(ser: serial.Serial, stop: threading.Event) -> None:
    buf = bytearray()
    while not stop.is_set():
        n = ser.in_waiting
        if n:
            buf.extend(ser.read(n))
        while True:
            r = tc.decode_frame(buf)
            if r is None:
                break
            mt, seq, ts_ms, pl, consumed = r
            del buf[:consumed]
            _ingest(mt, pl)
        time.sleep(0.002)


def _ingest(mt: int, pl: bytes) -> None:
    if mt == tc.MSG_MOTOR_STATE:
        d = tc.parse_motor_state(pl)
        if d and d.get("gidx") is not None:
            i = d["gidx"]                        # global flattened index
            with _lock:
                s = _snap[i]
                prev_state = s.state
                s.state   = d["state"]
                s.pos     = d["pos"]
                s.vel     = d["vel"]
                s.tau     = d["tau"]
                s.temp    = d["temp"]
                s.fault   = d["fault_flags"]
                s.updated = time.monotonic()
            # Motor just dropped out of its armed state (e.g. slave torque trip
            # → IDLE) while a sine was running: stop the host-side sine.
            if prev_state in _ARMED_STATES and d["state"] not in _ARMED_STATES:
                _auto_stop_sine(i)

    elif mt == tc.MSG_MASTER_STATUS:
        d = tc.parse_master_status(pl)
        if d:
            with _lock:
                _link["rx"]            = d["rx_frames"]
                _link["err"]           = d["link_errors"]
                _link["uptime_ms"]     = d["uptime_ms"]
                _master["robot_state"] = d["robot_state"]
                _master["slave_alive"] = d["slave_alive"]

    elif mt == tc.MSG_SLAVE_STATUS:
        d = tc.parse_slave_status(pl)
        if d and 0 <= d["slave_id"] < N_SLAVES:
            with _lock:
                _slaves[d["slave_id"]]["motors_alive"] = d["motors_alive"]
                _slaves[d["slave_id"]]["state"]        = d["motor_state"]

    elif mt == tc.MSG_CONTROL_RESP:
        d = tc.parse_control_resp(pl)
        if d:
            res = tc.CTRL_RESULT_NAMES.get(d["result"], "?")
            ns  = tc.MOTOR_STATE_NAMES.get(d["new_state"], "?")
            _log(f"CTRL_RESP  s{d['slave_id']}.m{d['motor_idx']} → [bold]{ns}[/bold]  [{res}]")

    elif mt == tc.MSG_PING:
        _log("[green]PONG[/green]")


# ── main ─────────────────────────────────────────────────────────────────────

def main() -> None:
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    try:
        ser = serial.Serial(port, 115200, timeout=0)
    except serial.SerialException as e:
        sys.exit(f"Cannot open {port}: {e}")

    ser_lock     = threading.Lock()
    # Populate the module-level sine state in place so the RX thread shares it.
    sine_stop[:]   = [threading.Event() for _ in range(N_MOTORS)]
    sine_active[:] = [False] * N_MOTORS
    sine_threads   = [None] * N_MOTORS
    active       = [0]          # list so input thread can mutate via closure
    quit_event   = threading.Event()

    # ── control helpers ───────────────────────────────────────────────────────

    def _ser_write(data: bytes) -> None:
        with ser_lock:
            ser.write(data)

    def _send_ctrl(idx: int, cmd: int, name: str) -> None:
        slave_id, local_idx = tc._slave_local(idx)
        payload = struct.pack(tc.FMT_CONTROL_REQ, slave_id, local_idx, cmd, 0)
        _ser_write(tc.encode_frame(tc.MSG_CONTROL_REQ, tc.NODE_JETSON, tc.NODE_MASTER, payload))
        _log(f"→ [bold]{name}[/bold]  id {idx+1} (s{slave_id}.m{local_idx})")

    def _stop_sine(idx: int) -> None:
        if sine_active[idx]:
            sine_stop[idx].set()
            sine_active[idx] = False

    def _toggle_sine(idx: int) -> None:
        if sine_active[idx]:
            _stop_sine(idx)
            _log(f"SINE motor {idx+1} [dim]stopped[/dim]")
        else:
            with _lock:
                center = _snap[idx].pos
            if center != center:        # NaN guard
                center = 0.0
            sine_stop[idx].clear()
            sine_active[idx] = True
            th = threading.Thread(
                target=tc._sine_thread,
                args=(idx, ser, ser_lock, sine_stop[idx], center),
                daemon=True,
            )
            sine_threads[idx] = th
            th.start()
            _log(f"SINE motor {idx+1} [magenta]started[/magenta]  center={center:+.3f} rad")

    def _disable_all() -> None:
        for i in range(N_MOTORS):
            _stop_sine(i)
        for i in range(N_MOTORS):
            _send_ctrl(i, tc.CTRL_DISABLE, "DISABLE")

    # ── keyboard input thread ─────────────────────────────────────────────────

    def _input_loop() -> None:
        while not quit_event.is_set():
            try:
                ch = sys.stdin.read(1)
            except OSError:
                break
            if not ch:
                continue

            if ch.isdigit() and ch != "0" and int(ch) <= N_MOTORS:
                active[0] = int(ch) - 1
                m = MOTORS[active[0]]
                _log(f"selected motor {active[0]+1}  ({m['joint_name']} · {m['model']})")

            elif ch == "a":
                _send_ctrl(active[0], tc.CTRL_ARM_HOLD, "ARM_HOLD")
            elif ch == "z":
                _send_ctrl(active[0], tc.CTRL_GOTO_ZERO, "GOTO_ZERO")
            elif ch == "s":
                _toggle_sine(active[0])
            elif ch == "d":
                _stop_sine(active[0])
                _send_ctrl(active[0], tc.CTRL_DISABLE, "DISABLE")

            elif ch == "A":
                for i in range(N_MOTORS):
                    _send_ctrl(i, tc.CTRL_ARM_HOLD, "ARM_HOLD")
            elif ch == "Z":
                for i in range(N_MOTORS):
                    _send_ctrl(i, tc.CTRL_GOTO_ZERO, "GOTO_ZERO")
            elif ch == "S":
                for i in range(N_MOTORS):
                    if not sine_active[i]:
                        _toggle_sine(i)
            elif ch == "D":
                _disable_all()

            elif ch == "p":
                _ser_write(tc.encode_frame(tc.MSG_PING, tc.NODE_JETSON, tc.NODE_MASTER, b""))
                _log("→ [bold]PING[/bold]")

            elif ch in ("q", "\x03", "\x04"):   # q / Ctrl-C / Ctrl-D
                quit_event.set()

    # ── start threads ─────────────────────────────────────────────────────────

    rx_stop = threading.Event()
    threading.Thread(target=_serial_rx, args=(ser, rx_stop), daemon=True).start()

    fd       = sys.stdin.fileno()
    old_term = termios.tcgetattr(fd)

    try:
        tty.setcbreak(fd)
        threading.Thread(target=_input_loop, daemon=True).start()

        console = Console()
        with Live(console=console, screen=True, refresh_per_second=15) as live:
            while not quit_event.is_set():
                live.update(_render(active[0], sine_active, port))
                time.sleep(1 / 15)

    finally:
        rx_stop.set()
        quit_event.set()
        for i in range(N_MOTORS):
            _stop_sine(i)
        termios.tcsetattr(fd, termios.TCSADRAIN, old_term)
        ser.close()
        print("Dashboard closed.")


if __name__ == "__main__":
    main()
