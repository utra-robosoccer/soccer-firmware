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


def analyze(path, host_rate=None, cycle_rate=None):
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
    tele_frames = []     # (ts_ns, cycle_id, cmd_seq_active, {key: last_applied}) — master-clock
    tx_cmd_echo = {}     # cmd_seq -> echoed cycle_id (the master cycle the host answered)
    tele_ctr = []        # (ts_ns, (on_time,late,missing,duplicate)) — bracketed to the cmd window

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
                    applied_map = {}
                    for ch in d["chains"]:
                        sid = ch["chain_id"]
                        for local, mo in enumerate(ch["motors"]):
                            key = (sid, local)
                            ms.append((rec.ts_ns, mo.tau))
                            tele_by_motor.setdefault(key, []).append(
                                (rec.ts_ns, mo.last_applied_seq))
                            applied_map[key] = mo.last_applied_seq
                    tele_frames.append((rec.ts_ns, d["cycle_id"],
                                        d["cmd_seq_active"], applied_map))
                    tele_ctr.append((rec.ts_ns, (d["cmd_on_time"], d["cmd_late"],
                                                 d["cmd_missing"], d["cmd_duplicate"])))
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
                tx_cmd_echo.setdefault(cmd_seq, d["cycle_id"])
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

    # ── master-clock command counters (tele_robot_t, run delta) ─────────────────
    # Free-running u16 counters classified by the master at its cycle-start mailbox swap:
    # on_time/late (host answered the latest/an older telemetry cycle), missing (no fresh
    # command that cycle → held), duplicate (host sent >1 command for one cycle).
    def _u16d(a, b):
        return (a - b) & 0xFFFF
    # Bracket the counters to the command-streaming window [first MIT TX, last MIT TX] so
    # idle cycles before/after (settle, HOLD arm, between runs) don't swamp the delta.
    if tele_ctr and tick_cmds:
        tele_ctr.sort()
        win_lo = min(info["ts"] for info in tick_cmds.values())
        win_hi = max(info["ts"] for info in tick_cmds.values())
        ct_ts = [t for t, _ in tele_ctr]
        i0 = min(bisect.bisect_left(ct_ts, win_lo), len(tele_ctr) - 1)
        i1 = min(bisect.bisect_right(ct_ts, win_hi) - 1, len(tele_ctr) - 1)
        c0, c1 = tele_ctr[i0][1], tele_ctr[max(i1, i0)][1]
        on, la, mi, du = (_u16d(c1[i], c0[i]) for i in range(4))
        tot = on + la + mi
        fresh = on + la                      # cycles that applied a new command
        pct = (lambda x: f"{100.0*x/tot:.1f}%" if tot else "–")
        # Split "missing" into holds EXPECTED because the host runs slower than the master
        # cycle (decimation N = cycles-per-command) vs EXCESS misses (holds beyond that — the
        # host dropped/skipped a command it was due to send). N is cycle_rate/host_rate; use
        # --host-rate if given, else infer from the applied cadence.
        if host_rate and cycle_rate:
            N = max(1, round(cycle_rate / host_rate))
        else:
            N = max(1, round(tot / fresh)) if fresh else 1
        expected_holds = fresh * (N - 1)
        excess = mi - expected_holds         # may be <0 if the host briefly outran its rate
        print("\nmaster command counters (over the command-streaming window):")
        print(f"    on_time {on} ({pct(on)})   late {la} ({pct(la)})   "
              f"missing {mi} ({pct(mi)})   duplicate {du}")
        print(f"    (on_time+late+missing = {tot} master cycles classified)")
        print(f"    holds: N≈{N} cycles/command → expected {expected_holds}, "
              f"excess misses {excess:+d}   (fresh commands {fresh})")

    # ── master-clock latency in CYCLES (answered E → applied A → confirmed C) ───
    # E = the master cycle the host echoed when it sent cmd_seq K (from the TX frame);
    # A = first telemetry whose cmd_seq_active ≥ K (the master applied K this cycle);
    # C = first telemetry whose last_applied_seq ≥ K (the slave confirmed it). cycle_id is
    # u16; a measurement run stays well under one wrap, so a signed-16 diff is exact.
    def _cyc(a, b):
        return ((a - b + 0x8000) & 0xFFFF) - 0x8000
    cyc_total, cyc_ans_apply, cyc_apply_conf = [], [], []
    cyc_never = 0
    frame_ts = [f[0] for f in tele_frames]
    for cmd_seq, info in tick_cmds.items():
        if cmd_seq not in tx_cmd_echo:
            continue
        E = tx_cmd_echo[cmd_seq]
        motors = info["motors"]
        A = C = None
        j = bisect.bisect_left(frame_ts, info["ts"])
        for k in range(j, len(tele_frames)):
            _ts, cyc, active, amap = tele_frames[k]
            if A is None and P.seq_ge(active, cmd_seq):
                A = cyc
            if C is None and all(P.seq_ge(amap.get(key, 0), cmd_seq) for key in motors):
                C = cyc
            if A is not None and C is not None:
                break
        if A is None or C is None:
            cyc_never += 1
            continue
        cyc_total.append(_cyc(C, E))
        cyc_ans_apply.append(_cyc(A, E))
        cyc_apply_conf.append(_cyc(C, A))
    if cyc_total:
        print("\nmaster-clock latency (cycles):")
        _dist("    total     answered → confirmed ", cyc_total, unit="cyc")
        _dist("    phase 1   answered → applied   ", cyc_ans_apply, unit="cyc")
        _dist("    phase 2   applied  → confirmed ", cyc_apply_conf, unit="cyc")
        print(f"    unresolved (no apply/confirm seen): {cyc_never}")

    print()
    _dist("step latency  (cmd → torque response)              ", step_lat)
    _dist("ping RTT      (host ↔ master)                      ", ping_rtt)


def main(argv=None):
    ap = argparse.ArgumentParser(description="End-to-end latency analysis of a .bin")
    ap.add_argument("logfile")
    ap.add_argument("--host-rate", type=float, default=None,
                    help="configured host command rate (Hz); splits 'missing' into expected "
                         "holds vs excess misses. Default: infer from the applied cadence.")
    ap.add_argument("--cycle-rate", type=float, default=200.0,
                    help="master cycle rate (Hz), for N = cycle/host (default 200).")
    args = ap.parse_args(argv)
    analyze(args.logfile, host_rate=args.host_rate, cycle_rate=args.cycle_rate)


if __name__ == "__main__":
    main()
