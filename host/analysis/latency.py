#!/usr/bin/env python3
"""End-to-end communication-latency analysis of a session .bin.

Two measurements, both on the host monotonic clock (the log's record timestamps):

  step latency  — for each commanded position step (TX MOTOR_CMD), the delay until the
                  first telemetry sample whose measured torque departs the pre-step
                  baseline by more than the noise threshold. This is the full
                  host→master→SPI→slave→CAN→motor→CAN→SPI→master→USB→host round trip
                  as seen in the torque response.
  ping RTT      — MSG_PING host↔master round trip (PONG echoes the request seq), for a
                  pure link-latency comparison with no motor/CAN in the path.

Usage:  python3 host/analysis/latency.py logs/<date>/<time>_latency.bin
"""
import argparse
import struct
import sys

from master_link.datalog.reader import BinaryLogReader
from master_link.datalog import format as LOG
from master_link import protocol as P

BASELINE_WINDOW_S = 0.5      # torque samples before the step used for baseline + noise
NOISE_FLOOR_NM    = 0.10     # minimum torque-change threshold (absolute)
NOISE_SIGMA       = 5.0      # ...or this many std devs of the pre-step noise, whichever larger


def _median(xs):
    xs = sorted(xs); n = len(xs)
    if not n: return 0.0
    return xs[n // 2] if n % 2 else 0.5 * (xs[n // 2 - 1] + xs[n // 2])


def _std(xs):
    n = len(xs)
    if n < 2: return 0.0
    m = sum(xs) / n
    return (sum((x - m) ** 2 for x in xs) / (n - 1)) ** 0.5


def _pct(sorted_xs, p):
    if not sorted_xs: return 0.0
    k = min(len(sorted_xs) - 1, int(round(p / 100.0 * (len(sorted_xs) - 1))))
    return sorted_xs[k]


def _dist(name, xs, unit="ms"):
    if not xs:
        print(f"{name}: no samples"); return
    s = sorted(xs)
    print(f"{name}  (n={len(s)}): min {s[0]:.2f}  median {_median(s):.2f}  "
          f"p95 {_pct(s,95):.2f}  max {s[-1]:.2f}  {unit}")


def analyze(path):
    r = BinaryLogReader(path)
    ms = []          # (ts_ns, tau) measured telemetry, host monotonic
    tx_step = []     # (ts_ns, pos) commanded position steps
    tx_ping = {}     # seq -> ts_ns
    rx_ping = {}     # seq -> ts_ns

    for rec in r:
        if rec.kind == LOG.RX_FRAME:
            res = P.decode_frame(bytearray(rec.payload))
            if not res:
                continue
            mt, seq, ts_ms, pl, _ = res
            if mt == P.MSG_MOTOR_STATE:
                d = P.parse_motor_state(pl)
                if d:
                    ms.append((rec.ts_ns, d["tau"]))
            elif mt == P.MSG_PING:
                rx_ping.setdefault(seq, rec.ts_ns)
        elif rec.kind == LOG.TX_FRAME:
            if len(rec.payload) < P.HDR_SIZE:
                continue
            mt, seq, src, dst, ts_ms, plen, ver, crc = struct.unpack_from(P.HDR_FMT, rec.payload)
            if mt == P.MSG_MOTOR_CMD:
                s, l, pos, vel, kp, kd, tau = struct.unpack_from(P.FMT_MOTOR_CMD, rec.payload, P.HDR_SIZE)
                tx_step.append((rec.ts_ns, pos))
            elif mt == P.MSG_PING:
                tx_ping.setdefault(seq, rec.ts_ns)

    ms.sort()
    ms_ns = [t for t, _ in ms]

    # ── step latency ──────────────────────────────────────────────────────────
    import bisect
    step_lat = []
    skipped = []
    for i, (tx_ns, pos) in enumerate(tx_step):
        base = [tau for (t, tau) in ms
                if tx_ns - BASELINE_WINDOW_S * 1e9 <= t <= tx_ns]
        if len(base) < 3:
            skipped.append((i, "no baseline")); continue
        b_med = _median(base)
        thr = max(NOISE_FLOOR_NM, NOISE_SIGMA * _std(base))
        j = bisect.bisect_right(ms_ns, tx_ns)
        hit = None
        # search only up to the next step (or 1 s) so a miss doesn't grab the next move
        next_tx = tx_step[i + 1][0] if i + 1 < len(tx_step) else tx_ns + int(1e9)
        while j < len(ms) and ms[j][0] < next_tx:
            if abs(ms[j][1] - b_med) > thr:
                hit = ms[j][0]; break
            j += 1
        if hit is None:
            skipped.append((i, "no torque response")); continue
        step_lat.append((hit - tx_ns) / 1e6)

    # ── ping RTT ──────────────────────────────────────────────────────────────
    ping_rtt = [(rx_ping[s] - tx_ping[s]) / 1e6 for s in tx_ping if s in rx_ping]

    print(f"log: {path}")
    print(f"steps commanded: {len(tx_step)}   torque-responses found: {len(step_lat)}"
          + (f"   skipped: {len(skipped)} {skipped}" if skipped else ""))
    print(f"threshold: max({NOISE_FLOOR_NM} Nm, {NOISE_SIGMA}·noise) over a "
          f"{BASELINE_WINDOW_S}s pre-step baseline\n")
    _dist("step latency (cmd → torque response)", step_lat)
    _dist("ping RTT     (host ↔ master)        ", ping_rtt)


def main(argv=None):
    ap = argparse.ArgumentParser(description="End-to-end latency analysis of a .bin")
    ap.add_argument("logfile")
    args = ap.parse_args(argv)
    analyze(args.logfile)


if __name__ == "__main__":
    main()
