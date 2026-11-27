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
from master_link import protocol as P
from master_link.link import MasterLink, MotorCommand, REQ_STOP
from policies.listen_policy import ListenPolicy
from policies.man_1s_1m_policy import Man1s1mPolicy
from policies.bench_sine import BenchSinePolicy
from policies.legs_sine import LegsSinePolicy
from policies.legs_walk import LegsWalkPolicy
from policies.arm_timing import ArmTimingPolicy

POLICIES = {
    "listen": ListenPolicy,
    "man_1s_1m": Man1s1mPolicy,
    "bench_sine": BenchSinePolicy,
    "legs_sine": LegsSinePolicy,
    "legs_walk": LegsWalkPolicy,
    "arm_timing": ArmTimingPolicy,
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
    ap.add_argument("--reset-slaves", action="store_true",
                    help="before running, clear latched motor faults via a "
                         "slave-level reset (blocking re-sweep) on every slave")
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

    # Don't act on the stale burst that drains at port open: wait until the telemetry of
    # every motor this run will COMMAND is confirmed live before setup()/the first step().
    # A policy's "--motors s.l,..." narrows the wait — a bench policy driving one chain
    # must not require an unrelated (absent or version-mismatched) slave to be up. With
    # no subset given, every configured motor must be live.
    live_keys = None
    mspec = getattr(args, "motors", None)
    if mspec and mspec != "all":
        live_keys = set()
        for tok in mspec.split(","):
            tok = tok.strip()
            if tok:
                s_str, l_str = tok.split(".")
                live_keys.add((int(s_str), int(l_str)))
    ok, missing = link.wait_until_live(keys=live_keys, timeout=2.0)
    if not ok:
        link.log_event("error:no_live_telemetry")
        link.close()
        names = ", ".join(f"s{s}.m{l}" for (s, l) in sorted(missing)) or "(none seen)"
        sys.exit(f"no live telemetry within 2.0 s from motor(s): {names} "
                 "— is the master up, the slave powered, and the motors on the CAN bus?")
    print("telemetry live")

    # There is no wire version/schema check anymore: the 8-byte header + CRC +
    # exact-length command accept are the contract. Telemetry decoding at all
    # (live-detect above) plus the measured rate below are the sanity gates.

    # Verify the master's ACTUAL cycle rate matches the generated config BEFORE
    # stepping: N is derived from MASTER_POLL_HZ, and a wrong N would drive the
    # policy at a rate it wasn't tuned for. The rate is MEASURED from telemetry
    # time deltas (master_time_us over cycle_id), not read from a wire constant.
    live_hz = link.measure_cycle_hz()
    if live_hz is None:
        link.log_event("error:rate_unmeasurable"); link.close()
        sys.exit("could not measure the master cycle rate from telemetry "
                 "(stalled before enough cycles) — refusing to run.")
    if abs(live_hz - mc.MASTER_POLL_HZ) > 0.02 * mc.MASTER_POLL_HZ:
        link.log_event("error:rate_mismatch"); link.close()
        sys.exit(f"master cycle-rate mismatch: generated config MASTER_POLL_HZ="
                 f"{mc.MASTER_POLL_HZ} Hz, but telemetry measures {live_hz:g} Hz. "
                 f"Regenerate the host config or reflash the master; refusing to run so "
                 f"the policy is not driven at the wrong rate.")

    # Explicit recovery (--reset-slaves): assert SLAVE_REQ_RESET on every slave
    # (motors all STOP) until each one has entered RESETTING and settled clean,
    # then drop the request and confirm it left RESETTING. Each slave round
    # BLOCKS the slave for seconds (clear + re-sweep + verify) — this is the
    # only way out of a latched MOTOR_FAULT / MOTOR_ABSENT, and it is never
    # done automatically: stop and investigate, then reset deliberately.
    if args.reset_slaves:
        all_slaves = set(range(mc.N_SLAVES))
        print(f"resetting slave(s) {sorted(all_slaves)} (blocking re-sweep)…")
        link.log_event("reset_slaves:start")
        deadline = time.monotonic() + 30.0
        seen_resetting = set()
        ok = False
        while time.monotonic() < deadline:
            link.send_robot_cmd([], reset_slaves=all_slaves)
            time.sleep(0.05)
            st = link.latest_state()
            for sid, sl in st.slaves.items():
                if sl["state"] == P.SLAVE_RESETTING:
                    seen_resetting.add(sid)
            if seen_resetting >= all_slaves:
                dirty = [k for k, m in st.motors.items()
                         if m.state not in (P.MOTOR_DISABLED,) or m.rs_fault_bits]
                if not dirty:
                    ok = True
                    break
        # Drop the request (plain all-STOP) and let the slaves settle.
        for _ in range(10):
            link.send_robot_cmd([])
            time.sleep(0.05)
        st = link.latest_state()
        states = {sid: P.SLAVE_STATE_NAMES.get(sl["state"], sl["state"])
                  for sid, sl in st.slaves.items()}
        link.log_event(f"reset_slaves:done:{states}")
        print(f"reset done; slave states: {states}")
        if not ok or any(sl["state"] != P.SLAVE_READY for sl in st.slaves.values()):
            link.close()
            sys.exit("slave reset did not reach READY with clean motors — "
                     "check the bus/motors (boot_result in telemetry) and retry.")

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
    # Stop any motor still armed (best-effort; safe even on listen). One
    # command frame with a STOP request for each armed motor.
    try:
        armed = link.latest_state().armed_motors()
        if armed:
            link.send_robot_cmd([MotorCommand(s, l, req=REQ_STOP) for (s, l) in armed])
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
    print(f"discards:       {st['discard_records']} records / {st['discard_bytes']} bytes")
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
