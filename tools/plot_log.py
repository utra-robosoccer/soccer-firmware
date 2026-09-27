#!/usr/bin/env python3
"""Plot a dashboard session log: measured p / v / tau vs time per motor, with fault
events marked (and commands optionally overlaid).

Usage:
    python3 tools/plot_log.py logs/2026-09-26_20-40-00.csv [--save] [--no-show]

One figure per motor (3 stacked axes: pos, vel, tau). Fault onsets — a telemetry
row whose fault cause becomes non-zero — are drawn as red dashed vertical lines
labelled with the cause. Command timestamps are drawn as faint markers on the pos
axis so you can see when ARM/ZERO/MIT/etc. were sent relative to the motion.
"""
import argparse
import csv
import os
import sys
from collections import defaultdict

# Fault cause id -> name (mirror of protocol.h MotorFaultCause; kept local so the
# plotter has no import dependencies).
CAUSE_NAMES = {0: "NONE", 1: "OVERTORQUE", 2: "CAN_TIMEOUT", 3: "WATCHDOG",
               4: "MOTOR_FAULT", 5: "ZERO_TIMEOUT"}


def _f(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


def _median(xs):
    xs = sorted(xs)
    n = len(xs)
    return None if n == 0 else (xs[n // 2] if n % 2 else 0.5 * (xs[n // 2 - 1] + xs[n // 2]))


def _load(path):
    """Return (tele, cmds, base) where base names the time column used.

    Prefers master_ts_ms (firmware tick, ms -> s) over host_ts for telemetry when
    present, since it is free of host receive jitter. Commands are host-timed
    (no master tick), so they are mapped onto the master timeline via the median
    (master_ts_ms/1000 - host_ts) offset measured from telemetry rows. All times
    are seconds on one common base."""
    rows = list(csv.DictReader(open(path, newline="")))
    use_master = any(_f(r.get("master_ts_ms")) is not None
                     for r in rows if r["kind"] == "T")
    base = "master_ts_ms" if use_master else "host_ts"

    offset = 0.0
    if use_master:
        offs = [_f(r["master_ts_ms"]) / 1000.0 - _f(r["host_ts"])
                for r in rows if r["kind"] == "T"
                and _f(r.get("master_ts_ms")) is not None and _f(r.get("host_ts")) is not None]
        offset = _median(offs) or 0.0

    tele = defaultdict(list)   # motor -> list of (t, pos, vel, tau, cause)
    cmds = defaultdict(list)   # motor -> list of (t, opcode)
    for row in rows:
        if row["kind"] == "T":
            t = (_f(row["master_ts_ms"]) / 1000.0) if use_master else _f(row.get("host_ts"))
            if t is None:
                continue
            tele[row["motor"]].append((t, _f(row["pos"]), _f(row["vel"]),
                                       _f(row["tau"]), int(_f(row["cause"]) or 0)))
        elif row["kind"] == "C":
            h = _f(row.get("host_ts"))
            if h is None:
                continue
            cmds[row["motor"]].append((h + offset, row["opcode"]))
    return tele, cmds, base


def _fault_onsets(samples):
    """Times where the fault cause transitions from 0 -> non-zero."""
    onsets, prev = [], 0
    for t, _p, _v, _tau, cause in samples:
        if cause != 0 and prev == 0:
            onsets.append((t, cause))
        prev = cause
    return onsets


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("logfile")
    ap.add_argument("--save", action="store_true", help="write <log>_motorN.png files")
    ap.add_argument("--no-show", action="store_true", help="don't open interactive windows")
    args = ap.parse_args()

    try:
        import matplotlib
        if args.no_show:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        sys.exit("matplotlib is required: pip install matplotlib")

    tele, cmds, base = _load(args.logfile)
    if not tele:
        sys.exit(f"No telemetry rows in {args.logfile}")
    print(f"time base: {base}")

    t0 = min(s[0] for m in tele.values() for s in m)
    base = os.path.splitext(args.logfile)[0]

    for motor in sorted(tele, key=lambda m: (m == "", m)):
        samples = tele[motor]
        ts   = [s[0] - t0 for s in samples]
        pos  = [s[1] for s in samples]
        vel  = [s[2] for s in samples]
        tau  = [s[3] for s in samples]

        fig, axes = plt.subplots(3, 1, sharex=True, figsize=(11, 7))
        fig.suptitle(f"motor {motor}   ({os.path.basename(args.logfile)})")
        for ax, series, label in zip(axes, (pos, vel, tau),
                                     ("pos [rad]", "vel [rad/s]", "tau [Nm]")):
            ax.plot(ts, series, lw=0.9)
            ax.set_ylabel(label)
            ax.grid(True, alpha=0.3)

        # Fault onsets: red dashed verticals across all three axes.
        for t, cause in _fault_onsets(samples):
            for ax in axes:
                ax.axvline(t - t0, color="red", ls="--", lw=1.0, alpha=0.8)
            axes[0].annotate(CAUSE_NAMES.get(cause, f"?{cause}"),
                             xy=(t - t0, 1.0), xycoords=("data", "axes fraction"),
                             color="red", fontsize=8, rotation=90,
                             va="top", ha="right")

        # Commands: faint markers on the pos axis.
        for t, op in cmds.get(motor, []):
            axes[0].axvline(t - t0, color="gray", ls=":", lw=0.6, alpha=0.4)

        axes[-1].set_xlabel("t [s]")
        fig.tight_layout()
        if args.save:
            out = f"{base}_motor{motor or 'NA'}.png"
            fig.savefig(out, dpi=120)
            print(f"wrote {out}")

    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
