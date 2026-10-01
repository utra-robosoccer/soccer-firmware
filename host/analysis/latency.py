#!/usr/bin/env python3
"""End-to-end communication-latency analysis of a session .bin.

All measurements use the host monotonic clock (the log's record timestamps):

  cmd_seq latency — the primary measure. Each TX MOTOR_CMD carries a cmd_seq (one per
                  policy tick, shared by all motors commanded that tick). A motor's
                  telemetry echoes last_applied_seq once the motor applies that command.
                  The latency of cmd_seq k is TX(k) → the first telemetry whose
                  last_applied_seq ≥ k (wrap-aware). This includes the FULL return path:
                  host→master→SPI→slave→CAN→motor, then motor→CAN→SPI→master→USB→host
                  for the echo to arrive. Reported two ways:
                    • per-motor — one latency per (motor, cmd_seq), plus a never-applied
                      count (commands whose echo never reached that motor);
                    • per-tick  — time until EVERY motor commanded that tick has
                      last_applied_seq ≥ k (max over the tick's motors), plus a
                      never-applied-tick count.
  step latency  — for each commanded position step, the delay until the first telemetry
                  sample whose measured torque departs the pre-step baseline by more than
                  the noise threshold (the physical torque-onset view, for comparison).
  ping RTT      — MSG_PING host↔master round trip (PONG echoes the request seq), for a
                  pure link-latency comparison with no motor/CAN in the path.

Usage:  python3 host/analysis/latency.py logs/<date>/<time>_latency.bin
"""
import argparse
import bisect
import struct
import sys

from master_link.datalog.reader import BinaryLogReader
from master_link.datalog import format as LOG
from master_link import protocol as P

BASELINE_WINDOW_S = 0.5      # torque samples before the step used for baseline + noise
NOISE_FLOOR_NM    = 0.10     # minimum torque-change threshold (absolute)
NOISE_SIGMA       = 5.0      # ...or this many std devs of the pre-step noise, whichever larger
APPLY_WINDOW_S    = 1.0      # how long to wait for a cmd_seq echo before calling it never-applied


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
    # cmd_seq pairing: per-motor telemetry (ts, last_applied_seq) and per-motor commands.
    tele_by_motor = {}   # (slave, local) -> [(ts_ns, last_applied_seq), ...] (time order)
    cmd_by_motor = {}    # (slave, local) -> [(ts_ns, cmd_seq), ...]
    tick_cmds = {}       # cmd_seq -> {"ts": first_ts_ns, "motors": set((slave, local))}

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
                    key = (d["slave_id"], d["motor_idx"])
                    tele_by_motor.setdefault(key, []).append(
                        (rec.ts_ns, d["last_applied_seq"]))
            elif mt == P.MSG_PING:
                rx_ping.setdefault(seq, rec.ts_ns)
        elif rec.kind == LOG.TX_FRAME:
            if len(rec.payload) < P.HDR_SIZE:
                continue
            mt, seq, src, dst, ts_ms, plen, ver, crc = struct.unpack_from(P.HDR_FMT, rec.payload)
            if mt == P.MSG_MOTOR_CMD:
                s, l, pos, vel, kp, kd, tau, cmd_seq = struct.unpack_from(
                    P.FMT_MOTOR_CMD, rec.payload, P.HDR_SIZE)
                tx_step.append((rec.ts_ns, pos))
                cmd_by_motor.setdefault((s, l), []).append((rec.ts_ns, cmd_seq))
                t = tick_cmds.setdefault(cmd_seq, {"ts": rec.ts_ns, "motors": set()})
                t["ts"] = min(t["ts"], rec.ts_ns)
                t["motors"].add((s, l))
            elif mt == P.MSG_PING:
                tx_ping.setdefault(seq, rec.ts_ns)

    ms.sort()
    ms_ns = [t for t, _ in ms]
    for v in tele_by_motor.values():
        v.sort()

    # ── step latency ──────────────────────────────────────────────────────────
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

    # ── cmd_seq latency (primary; wrap-aware, includes the return path) ─────────
    def _first_apply_ns(tele, tx_ns, target_seq):
        """ns of the first telemetry at/after tx_ns whose last_applied_seq ≥ target_seq
        (wrap-aware), within APPLY_WINDOW_S; None if never applied in that window."""
        lo = bisect.bisect_left(tele, (tx_ns,))
        deadline = tx_ns + int(APPLY_WINDOW_S * 1e9)
        for k in range(lo, len(tele)):
            ts, applied = tele[k]
            if ts > deadline:
                return None
            if P.seq_ge(applied, target_seq):
                return ts
        return None

    per_motor_lat = []
    per_motor_never = 0
    for key, cmds in cmd_by_motor.items():
        tele = tele_by_motor.get(key, [])
        for tx_ns, cmd_seq in cmds:
            hit = _first_apply_ns(tele, tx_ns, cmd_seq)
            if hit is None:
                per_motor_never += 1
            else:
                per_motor_lat.append((hit - tx_ns) / 1e6)

    per_tick_lat = []
    per_tick_never = 0
    for cmd_seq, info in tick_cmds.items():
        tx_ns = info["ts"]
        worst = None
        applied_all = True
        for key in info["motors"]:
            hit = _first_apply_ns(tele_by_motor.get(key, []), tx_ns, cmd_seq)
            if hit is None:
                applied_all = False
                break
            worst = hit if worst is None else max(worst, hit)
        if applied_all and worst is not None:
            per_tick_lat.append((worst - tx_ns) / 1e6)
        else:
            per_tick_never += 1

    # ── ping RTT ──────────────────────────────────────────────────────────────
    ping_rtt = [(rx_ping[s] - tx_ping[s]) / 1e6 for s in tx_ping if s in rx_ping]

    print(f"log: {path}")
    print(f"ticks commanded: {len(tick_cmds)}   motor-commands: "
          f"{sum(len(v) for v in cmd_by_motor.values())}")
    # Summarize skips by reason rather than dumping every index (a continuous sine,
    # as opposed to discrete steps, legitimately skips almost all of them).
    skip_by_reason = {}
    for _i, reason in skipped:
        skip_by_reason[reason] = skip_by_reason.get(reason, 0) + 1
    skip_summary = ", ".join(f"{n}× {r}" for r, n in sorted(skip_by_reason.items()))
    print(f"steps commanded: {len(tx_step)}   torque-responses found: {len(step_lat)}"
          + (f"   skipped: {len(skipped)} ({skip_summary})" if skipped else ""))
    print(f"threshold: max({NOISE_FLOOR_NM} Nm, {NOISE_SIGMA}·noise) over a "
          f"{BASELINE_WINDOW_S}s pre-step baseline; apply window {APPLY_WINDOW_S}s\n")
    _dist("cmd_seq latency per-tick  (TX → all motors applied)", per_tick_lat)
    print(f"    never-applied ticks:  {per_tick_never} / {len(tick_cmds)}")
    _dist("cmd_seq latency per-motor (TX → that motor applied) ", per_motor_lat)
    print(f"    never-applied motor-commands: {per_motor_never}")
    _dist("step latency  (cmd → torque response)              ", step_lat)
    _dist("ping RTT      (host ↔ master)                      ", ping_rtt)


def main(argv=None):
    ap = argparse.ArgumentParser(description="End-to-end latency analysis of a .bin")
    ap.add_argument("logfile")
    args = ap.parse_args(argv)
    analyze(args.logfile)


if __name__ == "__main__":
    main()
