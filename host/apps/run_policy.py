#!/usr/bin/env python3
"""Headless policy runner.

Opens a MasterLink (binary logging starts immediately), loads a policy, and runs a
deadline-scheduled loop: latest_state() -> policy.step() -> send -> log LOOP_TIMING.
On Ctrl-C or a policy exception it disables any armed motors, writes a stop event,
flushes/closes the log, and prints the log path + a summary.

Usage:
    python3 host/apps/run_policy.py --policy listen --rate 50
    python3 host/apps/run_policy.py --policy listen --port /dev/ttyACM2
"""
import argparse
import sys
import time

from master_link import config_meta
from master_link import motor_config_gen as mc
from master_link.link import MasterLink, MotorCommand, MODE_IDLE
from policies.listen_policy import ListenPolicy
from policies.man_1s_1m_policy import Man1s1mPolicy
from policies.bench_sine import BenchSinePolicy
from policies.legs_sine import LegsSinePolicy
from policies.legs_walk import LegsWalkPolicy

POLICIES = {
    "listen": ListenPolicy,
    "man_1s_1m": Man1s1mPolicy,
    "bench_sine": BenchSinePolicy,
    "legs_sine": LegsSinePolicy,
    "legs_walk": LegsWalkPolicy,
}


def _pct(sorted_vals, p):
    if not sorted_vals:
        return 0.0
    k = min(len(sorted_vals) - 1, int(round((p / 100.0) * (len(sorted_vals) - 1))))
    return sorted_vals[k]


def main(argv=None):
    # Pre-parse --policy so we register only the SELECTED policy's args — different sine
    # policies reuse flag names (bench_sine and legs_sine both use --amp/--freq/…), which
    # would collide if every policy registered its args on one parser.
    pre = argparse.ArgumentParser(add_help=False)
    pre.add_argument("--policy", default="listen",
                     choices=sorted(POLICIES) + ["__invalid__"])
    pre_args, _ = pre.parse_known_args(argv)
    pcls_pre = POLICIES.get(pre_args.policy)

    ap = argparse.ArgumentParser(description="Headless policy runner")
    ap.add_argument("--policy", default="listen", choices=sorted(POLICIES),
                    help="policy to run (default: listen)")
    ap.add_argument("--port", default=None, help="serial port (default: auto-detect master)")
    ap.add_argument("--rate", type=float, default=50.0, help="control loop rate Hz (default 50)")
    ap.add_argument("--log-dir", default="logs", help="log directory (default: logs/)")
    ap.add_argument("--mirror-signals", action="store_true",
                    help="mirror telemetry signals to stdout (for debugging)")
    # Only the selected policy registers its own CLI args (e.g. --amp/--freq/--motors/--dur).
    if pcls_pre is not None and hasattr(pcls_pre, "add_args"):
        pcls_pre.add_args(ap)
    args = ap.parse_args(argv)

    # Config staleness: warn loudly, but don't refuse to run.
    ok, msg = config_meta.check_config_fresh()
    if not ok:
        sys.stderr.write("\n*** CONFIG WARNING: " + msg + " ***\n\n")
    else:
        print(msg)

    pcls = POLICIES[args.policy]
    policy = pcls.from_args(args) if hasattr(pcls, "from_args") else pcls()

    try:
        link = MasterLink(args.port, policy_name=policy.name, log_dir=args.log_dir)
    except RuntimeError as e:
        sys.exit(str(e))
    print(f"port {link.port}  |  logging to {link.log_path}")

    # Don't act on the stale burst that drains at port open: wait until every configured
    # motor's telemetry is confirmed live before setup()/the first step().
    ok, missing = link.wait_until_live(timeout=2.0)
    if not ok:
        link.log_event("error:no_live_telemetry")
        link.close()
        names = ", ".join(f"s{s}.m{l}" for (s, l) in sorted(missing)) or "(none seen)"
        sys.exit(f"no live telemetry within 2.0 s from motor(s): {names} "
                 "— is the master up, the slave powered, and the motors on the CAN bus?")
    print("telemetry live")

    # Verify the master's actual cycle rate matches the generated config BEFORE stepping:
    # N is derived from MASTER_POLL_HZ, and a wrong N would drive the policy at a rate it
    # wasn't tuned for. MasterStatus arrives at ~20 Hz, so it's here within ~50 ms.
    mstat = link.wait_master_status(timeout=0.5)
    if mstat is None:
        link.log_event("error:no_master_status"); link.close()
        sys.exit("no MasterStatus within 0.5 s — cannot verify the master cycle rate "
                 "(is the master firmware up to date?).")
    live_hz = mstat.get("master_poll_hz", 0)
    if live_hz != mc.MASTER_POLL_HZ:
        link.log_event("error:rate_mismatch"); link.close()
        sys.exit(f"master cycle-rate mismatch: generated config MASTER_POLL_HZ="
                 f"{mc.MASTER_POLL_HZ} Hz, but the master reports {live_hz} Hz. Regenerate "
                 f"the host config or reflash the master; refusing to run so the policy is "
                 f"not driven at the wrong rate.")

    N = max(1, round(mc.MASTER_POLL_HZ / args.rate))
    cycle_us = 1e6 / mc.MASTER_POLL_HZ
    stall_timeout = 5 * cycle_us / 1e6          # a few missed cycles → stall
    print(f"stepping on cycle_id %% {N} == 0  →  {mc.MASTER_POLL_HZ / N:g} Hz policy "
          f"(master {mc.MASTER_POLL_HZ} Hz); stall after {stall_timeout*1e3:.0f} ms")

    link.log_event("start")

    # Master time (ns) drives policy.step(): perfectly regular and identical on replay.
    # Host arrival time only feeds the loop-timing budget.
    def master_ns(state):
        return int(state.robot["master_time_us_mono"]) * 1000

    first = link.latest_state()
    policy.setup(first, master_ns(first) if first.robot else 0)

    budgets = []; decodes = []; steps = []; writes = []
    skips_total = late_steps = dropped_steps = seq = 0
    last_fno = link.robot_frame_no()
    last_period = None
    stop_reason = "stop"

    try:
        while True:
            res = link.wait_robot(after_frame=last_fno, timeout=stall_timeout)
            if res is None:
                stop_reason = "error:telemetry_stall"
                sys.stderr.write(f"\ntelemetry stall: no frame for >{stall_timeout*1e3:.0f} ms "
                                 "— stopping.\n")
                break
            state, fno, skipped, decode_ns = res
            skips_total += skipped
            last_fno = fno
            cid = state.robot["cycle_id"]
            period = cid // N

            # Step on cycle_id % N == 0 (deterministic). If an aligned frame was coalesced,
            # step on the next frame (late) but keep the schedule aligned; only count a
            # DROPPED step when a whole N-cycle period elapsed with no step.
            if last_period is None:
                if cid % N != 0:
                    continue                         # align the first step
            elif period <= last_period:
                continue                             # extra frame within a stepped period
            elif period > last_period + 1:
                dropped_steps += (period - last_period - 1)

            on_time = (cid % N == 0)
            if not on_time:
                late_steps += 1

            arrival_ns = state.robot["recv_ns"]
            t1 = time.monotonic_ns()
            action = policy.step(state, master_ns(state))
            t2 = time.monotonic_ns()
            if action.motors:
                link.send_robot_cmd(action.motors, echo_cycle=cid)
            t3 = time.monotonic_ns()

            last_period = period
            seq += 1
            decodes.append(decode_ns); steps.append(t2 - t1); writes.append(t3 - t2)
            budgets.append(t3 - arrival_ns)
            # LOOP_TIMING: period=decode, step, send=write, lateness=budget(arrival→written)
            link.log_loop_timing(seq, decode_ns, t2 - t1, t3 - t2, t3 - arrival_ns)
    except KeyboardInterrupt:
        stop_reason = "stop"
        print("\nCtrl-C — shutting down")
    except Exception as e:  # policy or link failure
        stop_reason = f"error:{type(e).__name__}:{e}"
        sys.stderr.write(f"\npolicy/loop error: {e}\n")
    finally:
        _shutdown(link, policy, stop_reason, args.rate, N, cycle_us, seq,
                  skips_total, late_steps, dropped_steps, budgets, decodes, steps, writes)
    if stop_reason.startswith("error:"):
        sys.exit(1)


def _stat_ms(label, ns_vals):
    if not ns_vals:
        print(f"{label} no samples"); return
    xs = sorted(v / 1e6 for v in ns_vals)
    mean = sum(xs) / len(xs)
    print(f"{label} mean {mean:.3f}  p99 {_pct(xs,99):.3f}  max {max(xs):.3f}  ms")


def _shutdown(link, policy, stop_reason, rate, N, cycle_us, seq,
              skips, late_steps, dropped_steps, budgets, decodes, steps, writes):
    # Disable any motor still armed (best-effort; safe even on listen). One
    # cmd_robot_t with an IDLE request for each armed motor.
    try:
        armed = link.latest_state().armed_motors()
        if armed:
            link.send_robot_cmd([MotorCommand(s, l, mode=MODE_IDLE) for (s, l) in armed])
            print(f"disabled {len(armed)} armed motor(s): {armed}")
    except Exception:
        pass
    try:
        policy.teardown()
    except Exception:
        pass
    link.log_event(stop_reason)
    log_path = link.log_path
    st = link.stats
    link.close()

    cycle_ms = cycle_us / 1e3
    budget_pct = (100.0 * (sum(budgets) / len(budgets) / 1e6) / cycle_ms) if budgets else 0.0

    print("\n── summary ──────────────────────────────────────────")
    print(f"log:            {log_path}")
    print(f"stepped:        {seq}   N={N} → {rate:g} Hz policy on a {1000.0/cycle_ms:g} Hz "
          f"master cycle ({cycle_ms:.2f} ms)")
    print(f"skips/late/drop:{skips} coalesced frames   {late_steps} late steps   "
          f"{dropped_steps} dropped steps")
    print(f"frames RX/TX:   {st['rx_frames']} / {st['tx_frames']}"
          f"   (tx_errors {st['tx_errors']})")
    print(f"discards:       {st['discard_records']} records / {st['discard_bytes']} bytes"
          f"   | wrong-version frames: {st['version_frames']}")
    if st["log_dropped"]:
        print(f"LOG DROPPED:    {st['log_dropped']} records (disk too slow)")
    _stat_ms("decode:        ", decodes)
    _stat_ms("policy step:   ", steps)
    _stat_ms("write:         ", writes)
    _stat_ms("budget (arrival→written):", budgets)
    print(f"budget uses     {budget_pct:.1f}% of one {cycle_ms:.2f} ms master cycle (mean)")
    print("─────────────────────────────────────────────────────")


if __name__ == "__main__":
    main()
