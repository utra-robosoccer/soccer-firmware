#!/usr/bin/env python3
"""Plot a session's motor state: measured pos / vel / tau per motor, with commanded
values overlaid (where present) and fault events marked.

Give it a **session folder** or a **.bin** log — never a CSV:
    python3 host/analysis/plot_motor_state.py logs/2026-09-27/20-55-09_listen
    python3 host/analysis/plot_motor_state.py logs/2026-09-27/20-55-09_listen.bin

Given a .bin, it converts first if the session folder doesn't exist yet. It then
loads motor_state.csv (measured) and motor_cmd.csv (commanded, for overlay) itself.

One figure per motor (3 stacked axes: pos, vel, tau). Measured = solid; commanded
MIT = dashed; fault onsets = red dashed verticals; control commands = gray markers.
Times use master_ts_ms (firmware tick) when available, else host_ts; commands are
mapped onto that timeline via the measured rows' median offset.
"""
import argparse
import csv
import os
import sys
from collections import defaultdict

import convert_log  # sibling module (host/analysis on sys.path when run as a script)
from master_link.protocol import CAUSE_NAMES, LIFECYCLE_NAMES  # single source of truth


def _f(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


def _median(xs):
    xs = sorted(xs)
    n = len(xs)
    if n == 0:
        return 0.0
    return xs[n // 2] if n % 2 else 0.5 * (xs[n // 2 - 1] + xs[n // 2])


def resolve_session(target: str) -> str:
    """Return the session folder for `target` (a folder or a .bin), converting a
    .bin if its folder doesn't exist yet. Raises ValueError with a clear message
    for anything else."""
    if os.path.isdir(target):
        folder = target.rstrip("/")
        if not os.path.isfile(os.path.join(folder, "motor_state.csv")):
            raise ValueError(f"{folder!r} is not a session folder "
                             "(no motor_state.csv). Expected a convert_log output folder.")
        return folder
    if os.path.isfile(target) and target.endswith(".bin"):
        folder = os.path.splitext(target)[0]
        if not os.path.isdir(folder):
            print(f"converting {target} …")
            convert_log.convert(target)
        return folder
    raise ValueError(
        "expected a session folder or a .bin log file, got: " + repr(target) + "\n"
        "  e.g. logs/2026-09-27/20-55-09_listen  or  logs/2026-09-27/20-55-09_listen.bin")


# At port open the master's USB TX ring drains ~0.5 s of stale, buffered telemetry
# whose master_ts_ms sits far behind the live stream, then jumps forward to live. We
# trim that prefix: find the first large forward master_ts_ms jump in the opening
# window and keep only the rows after it.
STALE_JUMP_MS       = 1000.0   # a master_ts_ms step this big marks the stale→live edge
STALE_HOST_WINDOW_S = 2.0      # ...but only when it happens this soon after the first row


def _trim_stale_prefix(rows):
    """Drop buffered pre-open frames (see STALE_* above). Returns the live rows.

    Only trims a jump within STALE_HOST_WINDOW_S of the first row, so a legitimate
    mid-session gap (e.g. a master reboot) is left intact."""
    first_host = None
    prev = None
    for i, r in enumerate(rows):
        m = _f(r.get("master_ts_ms"))
        if m is None:
            continue
        h = _f(r.get("host_ts"))
        if first_host is None:
            first_host = h
        if prev is not None and (m - prev) > STALE_JUMP_MS:
            within = (first_host is None or h is None or (h - first_host) <= STALE_HOST_WINDOW_S)
            return rows[i:] if within else rows
        prev = m
    return rows


def _load_state(folder):
    """motor -> [(t, pos, vel, tau, cause)], plus the host->master time offset."""
    rows = list(csv.DictReader(open(os.path.join(folder, "motor_state.csv"), newline="")))
    use_master = any(_f(r.get("master_ts_ms")) is not None for r in rows)
    if use_master:
        n_before = len(rows)
        rows = _trim_stale_prefix(rows)
        n_trimmed = n_before - len(rows)
        if n_trimmed:
            print(f"trimmed {n_trimmed} stale pre-open frame(s) from the start")
    offsets = [(_f(r["master_ts_ms"]) / 1000.0 - _f(r["host_ts"]))
               for r in rows
               if use_master and _f(r.get("master_ts_ms")) is not None
               and _f(r.get("host_ts")) is not None]
    offset = _median(offsets) if offsets else 0.0

    state = defaultdict(list)
    for r in rows:
        t = (_f(r["master_ts_ms"]) / 1000.0) if use_master else _f(r.get("host_ts"))
        if t is None:
            continue
        state[r["motor"]].append((t, _f(r["pos"]), _f(r["vel"]), _f(r["tau"]),
                                  int(_f(r["cause"]) or 0), int(_f(r.get("state")) or 0)))
    return state, offset, ("master_ts_ms" if use_master else "host_ts")


def _load_cmd(folder, offset):
    """motor -> {'mit': [(t,pos,vel,tau)], 'ctrl': [(t,opcode)]}, times on the state
    timeline (host_ts + offset)."""
    path = os.path.join(folder, "motor_cmd.csv")
    out = defaultdict(lambda: {"mit": [], "ctrl": []})
    if not os.path.isfile(path):
        return out
    for r in csv.DictReader(open(path, newline="")):
        h = _f(r.get("host_ts"))
        if h is None:
            continue
        t = h + offset
        mode = r.get("mode_name", "")
        if mode == "MIT":
            out[r["motor"]]["mit"].append((t, _f(r["pos"]), _f(r["vel"]), _f(r["tau_ff"])))
        else:
            # Non-MIT mode requests (HOLD/DAMPED/TO_ZERO/IDLE) overlay as events.
            out[r["motor"]]["ctrl"].append((t, mode))
    return out


def _fault_onsets(samples):
    onsets, prev = [], 0
    for s in samples:
        cause = s[4]
        if cause != 0 and prev == 0:
            onsets.append((s[0], cause))
        prev = cause
    return onsets


def _mode_transitions(samples):
    """Lifecycle-state transitions [(t, state_int)] — one per change (incl. the first),
    so the timeline shows HOLD/MIT/DAMPED/TO_ZERO/IDLE/FAULT as labeled markers."""
    out, prev = [], None
    for s in samples:
        st = s[5]
        if st != prev:
            out.append((s[0], st))
            prev = st
    return out


def _session_start(folder):
    """(date, HH:MM:SS) parsed from logs/<date>/<HH-MM-SS>_<name> — the session wall start."""
    base = os.path.basename(folder)
    date = os.path.basename(os.path.dirname(folder))
    hms = base.split("_", 1)[0].replace("-", ":")
    return date, hms


def main(argv=None):
    ap = argparse.ArgumentParser(description="Plot a session's motor state")
    ap.add_argument("session", help="session folder or .bin log")
    ap.add_argument("--save", action="store_true", help="write motor_<N>.png into the folder")
    ap.add_argument("--no-show", action="store_true", help="don't open interactive windows")
    args = ap.parse_args(argv)

    try:
        folder = resolve_session(args.session)
    except ValueError as e:
        sys.exit(str(e))

    try:
        import matplotlib
        if args.no_show:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        sys.exit("matplotlib is required: pip install matplotlib")
    _NONINTERACTIVE = {"agg", "pdf", "ps", "svg", "template", "cairo"}
    interactive = matplotlib.get_backend().lower() not in _NONINTERACTIVE

    state, offset, base = _load_state(folder)
    if not state:
        sys.exit(f"no motor_state rows in {folder}/motor_state.csv")
    cmds = _load_cmd(folder, offset)
    print(f"session: {folder}   time base: {base}")

    clock = "master clock" if base == "master_ts_ms" else "host clock"
    xlabel = f"Time since streaming start [s] ({clock})"
    date, hms = _session_start(folder)
    logname = os.path.basename(folder)

    t0 = min(s[0] for m in state.values() for s in m)

    for motor in sorted(state, key=lambda m: (m == "", m)):
        samples = state[motor]
        ts = [s[0] - t0 for s in samples]
        series = {"pos [rad]": [s[1] for s in samples],
                  "vel [rad/s]": [s[2] for s in samples],
                  "tau [Nm]": [s[3] for s in samples]}

        fig, axes = plt.subplots(3, 1, sharex=True, figsize=(11, 7))
        fig.suptitle(f"motor {motor}   —   {logname}\n"
                     f"session start {date} {hms}  ({clock})", fontsize=10)

        mc = cmds.get(motor, {"mit": [], "ctrl": []})
        cmd_t = [m[0] - t0 for m in mc["mit"]]
        cmd_series = [[m[1] for m in mc["mit"]], [m[2] for m in mc["mit"]],
                      [m[3] for m in mc["mit"]]]  # pos, vel, tau_ff

        for ax, (label, meas), cmd_vals in zip(axes, series.items(), cmd_series):
            ax.plot(ts, meas, ls="none", marker="x", ms=3, mew=0.8, label="measured")
            if cmd_t and any(v is not None for v in cmd_vals):
                ax.plot(cmd_t, cmd_vals, ls="none", marker="+", ms=3, mew=0.8,
                        alpha=0.8, label="commanded")
                ax.legend(loc="upper right", fontsize=7)
            ax.set_ylabel(label)
            ax.grid(True, alpha=0.3)

        # Lifecycle mode changes (HOLD/MIT/DAMPED/TO_ZERO/IDLE/FAULT): labeled verticals.
        for t, st in _mode_transitions(samples):
            for ax in axes:
                ax.axvline(t - t0, color="steelblue", ls="-", lw=0.8, alpha=0.5)
            axes[0].annotate(LIFECYCLE_NAMES.get(st, f"?{st}"),
                             xy=(t - t0, 0.02), xycoords=("data", "axes fraction"),
                             color="steelblue", fontsize=7, rotation=90, va="bottom", ha="right")
        # Fault onsets: red verticals labeled with the cause (top).
        for t, cause in _fault_onsets(samples):
            for ax in axes:
                ax.axvline(t - t0, color="red", ls="--", lw=1.0, alpha=0.8)
            axes[0].annotate(CAUSE_NAMES.get(cause, f"?{cause}"),
                             xy=(t - t0, 1.0), xycoords=("data", "axes fraction"),
                             color="red", fontsize=8, rotation=90, va="top", ha="right")

        axes[-1].set_xlabel(xlabel)
        fig.tight_layout()
        if args.save or (not args.no_show and not interactive):
            out = os.path.join(folder, f"motor_{motor or 'NA'}.png")
            fig.savefig(out, dpi=120)
            print(f"wrote {out}")

    if not args.no_show:
        if interactive:
            plt.show()
        else:
            print("\nNo interactive matplotlib backend (Agg) — wrote PNGs above.\n"
                  "For interactive windows: install python3-tk or pip install PyQt5 (or pass --save).")


if __name__ == "__main__":
    main()
