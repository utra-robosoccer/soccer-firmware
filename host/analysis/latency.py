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
    tele_cmd_rx = []     # (ts_ns, last_cmd_seq_rx) — for master command-bunching analysis

    for rec in r:
        if rec.kind == LOG.RX_FRAME:
            res = P.decode_frame(bytearray(rec.payload))
            if not res:
                continue
            mt, seq, ts_ms, pl, _ = res
            if mt == P.MSG_ROBOT_TELE:
                d = P.parse_robot_tele(pl)
                if d:
                    tele_cmd_rx.append((rec.ts_ns, d["last_cmd_seq_rx"]))
                    for ch in d["chains"]:
                        sid = ch["chain_id"]
                        for local, mo in enumerate(ch["motors"]):
                            key = (sid, local)
                            ms.append((rec.ts_ns, mo.tau))
                            tele_by_motor.setdefault(key, []).append(
                                (rec.ts_ns, mo.last_applied_seq))
            elif mt == P.MSG_PING:
                rx_ping.setdefault(seq, rec.ts_ns)
        elif rec.kind == LOG.TX_FRAME:
            if len(rec.payload) < P.HDR_SIZE:
                continue
            mt, seq, src, dst, ts_ms, plen, ver, crc = struct.unpack_from(P.HDR_FMT, rec.payload)
            if mt == P.MSG_ROBOT_CMD:
                d = P.parse_robot_cmd(rec.payload[P.HDR_SIZE:])
                if not d:
                    continue
                cmd_seq = d["cmd_seq"]
                for ch in d["chains"]:
                    sid = ch["chain_id"]
                    for local, m in enumerate(ch["motors"]):
                        if m["mode_req"] != P.REQ_MIT:
                            continue   # only MIT targets get last_applied echoes
                        key = (sid, local)
                        tx_step.append((rec.ts_ns, m["pos"]))
                        cmd_by_motor.setdefault(key, []).append((rec.ts_ns, cmd_seq))
                        t = tick_cmds.setdefault(cmd_seq, {"ts": rec.ts_ns, "motors": set()})
                        t["ts"] = min(t["ts"], rec.ts_ns)
                        t["motors"].add(key)
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
    # last_applied_seq is monotonic, so the first telemetry with applied ≥ target is
    # either EXACTLY target (truly applied → real latency) or already PAST it
    # (the master's latest-wins mailbox replaced target with a newer seq before the
    # slave applied it → "superseded", a drop, NOT a +1-cycle latency). Reporting the
    # ≥-match as latency for a superseded command inflates the tail, so split them.
    def _classify(tele, tx_ns, target_seq):
        lo = bisect.bisect_left(tele, (tx_ns,))
        deadline = tx_ns + int(APPLY_WINDOW_S * 1e9)
        for k in range(lo, len(tele)):
            ts, applied = tele[k]
            if ts > deadline:
                return ("never", None)
            if P.seq_ge(applied, target_seq):
                return ("applied", ts) if applied == target_seq else ("superseded", ts)
        return ("never", None)

    per_motor_lat = []
    per_motor_never = 0
    per_motor_superseded = 0
    for key, cmds in cmd_by_motor.items():
        tele = tele_by_motor.get(key, [])
        for tx_ns, cmd_seq in cmds:
            kind, ts = _classify(tele, tx_ns, cmd_seq)
            if kind == "applied":
                per_motor_lat.append((ts - tx_ns) / 1e6)
            elif kind == "superseded":
                per_motor_superseded += 1
            else:
                per_motor_never += 1

    per_tick_lat = []
    per_tick_never = 0
    per_tick_superseded = 0
    for cmd_seq, info in tick_cmds.items():
        tx_ns = info["ts"]
        worst = None
        verdict = "applied"
        for key in info["motors"]:
            kind, ts = _classify(tele_by_motor.get(key, []), tx_ns, cmd_seq)
            if kind == "never":
                verdict = "never"; break
            if kind == "superseded":
                verdict = "superseded"; break
            worst = ts if worst is None else max(worst, ts)
        if verdict == "applied" and worst is not None:
            per_tick_lat.append((worst - tx_ns) / 1e6)
        elif verdict == "superseded":
            per_tick_superseded += 1
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
    print(f"    never-applied ticks:  {per_tick_never} / {len(tick_cmds)}"
          f"    superseded ticks: {per_tick_superseded}")
    _dist("cmd_seq latency per-motor (TX → that motor applied) ", per_motor_lat)
    print(f"    never-applied motor-commands: {per_motor_never}"
          f"    superseded (mailbox latest-wins): {per_motor_superseded}")

    # ── master command bunching (last_cmd_seq_rx advance per telemetry cycle) ───
    # If ≥2 host cmd_seqs land in the master's mailbox within one ~5 ms cycle, the
    # earlier one is overwritten (superseded) before it reaches the slave.
    tele_cmd_rx.sort()
    adv_hist = {}
    for i in range(1, len(tele_cmd_rx)):
        d = (tele_cmd_rx[i][1] - tele_cmd_rx[i - 1][1]) & 0xFFFF
        if d > 8:
            continue   # wrap/gap outlier — ignore
        adv_hist[d] = adv_hist.get(d, 0) + 1
    bunched = sum(n for d, n in adv_hist.items() if d >= 2)
    total_cyc = sum(adv_hist.values()) or 1
    hist_str = " ".join(f"+{d}:{n}" for d, n in sorted(adv_hist.items()))
    print(f"    cmd bunching (last_cmd_seq_rx advance/tele-cycle): {hist_str}")
    print(f"      cycles advancing ≥2 (bunched): {bunched}/{total_cyc} "
          f"= {100.0*bunched/total_cyc:.1f}%")

    _dist("step latency  (cmd → torque response)              ", step_lat)
    _dist("ping RTT      (host ↔ master)                      ", ping_rtt)


def main(argv=None):
    ap = argparse.ArgumentParser(description="End-to-end latency analysis of a .bin")
    ap.add_argument("logfile")
    args = ap.parse_args(argv)
    analyze(args.logfile)


if __name__ == "__main__":
    main()
