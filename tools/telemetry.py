#!/usr/bin/env python3
"""tools/telemetry.py — read-only live motor telemetry viewer.

Shows one row per motor in the ACTIVE bench/robot config and decodes the
master's MOTOR_STATE frames. Read-only: no control keys, just look.

The motor table is built straight from the configs: `configs/active` names the
active setup, and its `configs/<setup>/slaveN.yaml` files define the motor chain
(slave index = sorted file order, matching the firmware's wire slave_id). So the
view always matches whatever setup is selected — no regeneration needed.

Usage:  python3 tools/telemetry.py [PORT]      (PORT default: /dev/ttyACM1)
Quit:   Ctrl-C
"""
import glob
import os
import sys
import threading
import time

import serial
import yaml
from rich import box
from rich.live import Live
from rich.table import Table
from rich.text import Text

_TOOLS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TOOLS)
sys.path.insert(0, _TOOLS)                                 # motor_config_gen (decode bounds)
sys.path.insert(0, os.path.join(_ROOT, "host", "jetson"))  # protocol
import protocol as P  # noqa: E402

DEFAULT_PORT = "/dev/ttyACM1"


def active_setup() -> str:
    try:
        with open(os.path.join(_ROOT, "configs", "active")) as fh:
            return fh.read().strip()
    except OSError:
        sys.exit("configs/active not found — select a setup (e.g. `echo bench-1-chain > configs/active`)")


def load_motor_table(setup: str):
    """Motors in the active setup, as (slave_id, local_idx) → info. slave_id is
    the sorted position of the slaveN.yaml file (matches firmware wire ids)."""
    files = sorted(glob.glob(os.path.join(_ROOT, "configs", setup, "slave*.yaml")))
    if not files:
        sys.exit(f"no slave*.yaml in configs/{setup}/")
    motors = []  # global order; each: dict(slave, idx, joint, model, can_id)
    for slave_id, path in enumerate(files):
        cfg = yaml.safe_load(open(path))
        for m in cfg["motors"]:
            motors.append(dict(slave=slave_id, idx=m["idx"], joint=m["joint_name"],
                               model=m["model"], can_id=m["can_id"]))
    return motors


# ── shared state ──────────────────────────────────────────────────────────────
_lock = threading.Lock()
_latest = {}   # (slave_id, local_idx) -> (parsed MOTOR_STATE dict, monotonic ts)
_frames = [0]
_connected = [False]


def reader(port):
    """Own the serial link and auto-reconnect. A master reset or USB hiccup just
    flips the status to reconnecting instead of killing the view."""
    buf = bytearray()
    ser = None
    while True:
        if ser is None:
            try:
                ser = serial.Serial(port, 115200, timeout=0.2)
                buf.clear()
                _connected[0] = True
            except serial.SerialException:
                _connected[0] = False
                time.sleep(0.5)
                continue
        try:
            data = ser.read(4096)
        except serial.SerialException:
            _connected[0] = False
            try:
                ser.close()
            except Exception:
                pass
            ser = None
            time.sleep(0.5)
            continue
        if not data:
            continue
        buf.extend(data)
        while True:
            r = P.decode_frame(buf)
            if r is None:
                break
            mt, _, _, pl, used = r
            del buf[:used]
            if mt == P.MSG_MOTOR_STATE:          # only motor messages, per spec
                d = P.parse_motor_state(pl)
                if d:
                    with _lock:
                        _latest[(d["slave_id"], d["motor_idx"])] = (d, time.monotonic())
                        _frames[0] += 1


_LIFE_STYLE = {
    "ARMED_HOLD": "bold green", "ARMED_MIT": "bold magenta", "ZEROING": "cyan",
    "IDLE": "yellow", "FAULT": "bold red", "DISABLED": "dim",
    "BOOT": "dim", "DISCOVERING": "dim",
}


def _flags_str(cf: int) -> str:
    parts = []
    if cf & P.CMDFLAG_CLAMPED_POS: parts.append("Cpos")
    if cf & P.CMDFLAG_CLAMPED_TAU: parts.append("Ctau")
    if cf & P.CMDFLAG_CMD_STALE:   parts.append("stale")
    return " ".join(parts) or "-"


def render(setup: str, port: str, motors: list) -> Table:
    with _lock:
        nframes = _frames[0]
    link = "[green]up[/]" if _connected[0] else "[red]reconnecting…[/]"
    t = Table(box=box.SIMPLE_HEAVY, expand=True, pad_edge=False,
              title=f"[bold]⚽ Motor Telemetry[/]   setup=[cyan]{setup}[/]   "
                    f"{port} {link}   [dim]{nframes} frames[/]")
    for col, just in (("id", "right"), ("joint", "left"), ("model", "left"),
                      ("state", "left"), ("cause", "left"),
                      ("pos", "right"), ("vel", "right"), ("tau", "right"),
                      ("°C", "right"), ("fault", "right"), ("word", "right"),
                      ("flags", "left"), ("fb", "right")):
        t.add_column(col, justify=just, no_wrap=True)

    now = time.monotonic()
    for g, m in enumerate(motors):
        with _lock:
            rec = _latest.get((m["slave"], m["idx"]))
        head = f"{g + 1}"
        if rec is None:
            t.add_row(head, m["joint"], m["model"], Text("— no data —", style="dim"),
                      "", "", "", "", "", "", "", "", "")
            continue
        d, ts = rec
        stale_link = (now - ts) > 0.5                       # host hasn't seen a frame recently
        life = P.LIFECYCLE_NAMES.get(d["state"], f"?{d['state']}")
        cause = P.CAUSE_NAMES.get(d["cause"], f"?{d['cause']}")
        fw = d["fault_word"]
        word = "----" if fw == 0 else ("READFAIL" if fw == 0xFFFFFFFF else f"{fw:08x}")
        fb = d["fb_age"]
        fb_txt = Text(f"{fb}", style="red" if fb >= 100 else ("yellow" if fb >= 20 else "green"))
        row_dim = "dim " if stale_link else ""
        t.add_row(
            head, m["joint"], m["model"],
            Text(life, style=row_dim + _LIFE_STYLE.get(life, "")),
            Text(cause, style="red" if cause != "NONE" else "dim"),
            f"{d['pos']:+.3f}", f"{d['vel']:+.3f}", f"{d['tau']:+.3f}",
            f"{d['temp']:.0f}",
            Text(f"{d['motor_fault']:02x}", style="red" if d["motor_fault"] else "dim"),
            Text(word, style="red" if fw not in (0,) else "dim"),
            _flags_str(d["cmd_flags"]),
            fb_txt,
        )
    return t


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    setup = active_setup()
    motors = load_motor_table(setup)

    threading.Thread(target=reader, args=(port,), daemon=True).start()
    try:
        with Live(render(setup, port, motors), refresh_per_second=10, screen=False) as live:
            while True:
                time.sleep(0.1)
                live.update(render(setup, port, motors))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
