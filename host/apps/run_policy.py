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
from master_link.link import MasterLink, MotorCommand, MODE_IDLE
from policies.listen_policy import ListenPolicy
from policies.man_1s_1m_policy import Man1s1mPolicy

POLICIES = {
    "listen": ListenPolicy,
    "man_1s_1m": Man1s1mPolicy,
}


def _pct(sorted_vals, p):
    if not sorted_vals:
        return 0.0
    k = min(len(sorted_vals) - 1, int(round((p / 100.0) * (len(sorted_vals) - 1))))
    return sorted_vals[k]


def main(argv=None):
    ap = argparse.ArgumentParser(description="Headless policy runner")
    ap.add_argument("--policy", default="listen", choices=sorted(POLICIES),
                    help="policy to run (default: listen)")
    ap.add_argument("--port", default=None, help="serial port (default: auto-detect master)")
    ap.add_argument("--rate", type=float, default=50.0, help="control loop rate Hz (default 50)")
    ap.add_argument("--log-dir", default="logs", help="log directory (default: logs/)")
    args = ap.parse_args(argv)

    # Config staleness: warn loudly, but don't refuse to run.
    ok, msg = config_meta.check_config_fresh()
    if not ok:
        sys.stderr.write("\n*** CONFIG WARNING: " + msg + " ***\n\n")
    else:
        print(msg)

    policy = POLICIES[args.policy]()

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

    period_ns = int(1e9 / args.rate)
    link.log_event("start")
    policy.setup(link.latest_state(), time.monotonic_ns())

    periods = []
    latenesses = []
    overruns = 0
    seq = 0
    prev_start = None
    next_deadline = time.monotonic_ns()
    stop_reason = "stop"

    try:
        while True:
            t0 = time.monotonic_ns()
            lateness = t0 - next_deadline

            state = link.latest_state()
            t1 = time.monotonic_ns()
            action = policy.step(state, t0)
            t2 = time.monotonic_ns()
            if action.motors:
                link.send_robot_cmd(action.motors)
            t3 = time.monotonic_ns()

            period = (t0 - prev_start) if prev_start is not None else period_ns
            link.log_loop_timing(seq, period, t2 - t1, t3 - t2, lateness)
            if prev_start is not None:
                periods.append(period)
            latenesses.append(lateness)
            if lateness > period_ns:
                overruns += 1
            prev_start = t0
            seq += 1

            next_deadline += period_ns
            now = time.monotonic_ns()
            if next_deadline <= now:
                next_deadline = now      # fell behind → resync the grid (don't burst)
            else:
                time.sleep((next_deadline - now) / 1e9)
    except KeyboardInterrupt:
        stop_reason = "stop"
        print("\nCtrl-C — shutting down")
    except Exception as e:  # policy or link failure
        stop_reason = f"error:{type(e).__name__}:{e}"
        sys.stderr.write(f"\npolicy/loop error: {e}\n")
    finally:
        _shutdown(link, policy, stop_reason, args.rate, seq, periods, latenesses, overruns)


def _shutdown(link, policy, stop_reason, rate, seq, periods, latenesses, overruns):
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

    per_ms = sorted(p / 1e6 for p in periods)
    late_ms = sorted(l / 1e6 for l in latenesses)
    mean = (sum(per_ms) / len(per_ms)) if per_ms else 0.0

    print("\n── summary ──────────────────────────────────────────")
    print(f"log:            {log_path}")
    print(f"ticks:          {seq}   target {rate:g} Hz ({1000.0/rate:.2f} ms)")
    print(f"frames RX/TX:   {st['rx_frames']} / {st['tx_frames']}"
          f"   (tx_errors {st['tx_errors']})")
    print(f"discards:       {st['discard_records']} records / {st['discard_bytes']} bytes"
          f"   | wrong-version frames: {st['version_frames']}")
    if st["log_dropped"]:
        print(f"LOG DROPPED:    {st['log_dropped']} records (disk too slow)")
    if per_ms:
        print(f"loop period ms: mean {mean:.3f}  p99 {_pct(per_ms,99):.3f}  max {max(per_ms):.3f}")
        print(f"lateness ms:    p99 {_pct(late_ms,99):.3f}  max {max(late_ms):.3f}"
              f"   overruns {overruns}")
    print("─────────────────────────────────────────────────────")


if __name__ == "__main__":
    main()
