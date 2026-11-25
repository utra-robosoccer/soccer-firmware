#!/usr/bin/env python3
"""Scenario tests: the real firmware against emulated RobStride motors, judged only through the host
link (what the Jetson would see) and the motors' own ground truth.

    python3 test_sim.py [--setup S ...]      default: every setup with ELFs under build/

Each setup runs one sequence: boot -> arm every motor -> track a small step -> motor-reported fault
and recovery -> unplugged motor -> host goes silent -> garbage on the host link. Behaviour only:
virtual time, never timing evidence (ADR-007). Takes ~1-2 minutes of wall time per setup.
"""
import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import robosim  # noqa: E402
from jetson import Host  # noqa: E402


class Checker:
    def __init__(self, setup):
        self.setup, self.failures = setup, []

    def check(self, name, ok, detail=""):
        print(f"  {'PASS' if ok else 'FAIL'}  {name}" + (f"  ({detail})" if detail else ""), flush=True)
        if not ok:
            self.failures.append(f"{self.setup}: {name} {detail}")
        return ok


def wait_for(h, cond, ms, step_ms=5):
    """Advance in small steps until cond() holds or `ms` of virtual time passed; return cond()."""
    t_end = h.robot.vt_ms + ms
    while h.robot.vt_ms < t_end:
        if cond():
            return True
        h.advance(step_ms)
    return cond()


def run_setup(setup):
    c = Checker(setup)
    print(f"\n== {setup}")
    with robosim.Robot(setup) as robot:
        h = Host(robot)
        P = h.P
        everyone = h.targets()
        state = lambda: {t: m.drive_state for t, m in h.motors().items()}   # noqa: E731

        # 1. boot
        h.boot()
        d = h.tele()
        c.check("boot: every chain ALIVE", d and all(ch["link"] == P.CHAIN_LINK_ALIVE for ch in d["chains"]))
        c.check("boot: robot READY", d and d["robot_state"] == P.ROBOT_READY,
                P.ROBOT_STATE_NAMES.get(d["robot_state"]) if d else "no telemetry")
        c.check("boot: every motor DISABLED", all(s == P.DRV_DISABLED for s in state().values())
                and len(state()) == len(everyone), str({k: P.DRIVE_STATE_NAMES.get(v) for k, v in state().items()}))
        wire = robot.wire()
        c.check("boot: no partial SPI exchanges", all(w["partial"] == 0 for w in wire), str(wire))

        # 2. arm every motor, holding where it is
        h.hold(P.DRV_REQ_ENABLE, everyone)
        armed = wait_for(h, lambda: all(m.drive_state == P.DRV_ENABLED and m.rs_mode == 2
                                        for m in h.motors().values()), 300)
        c.check("arm: every motor ENABLED with rs_mode Run within 300 ms", armed,
                str({k: m.drive_state_name for k, m in h.motors().items()}))
        truth = {t: robot.motor(t[0], h.can_id(t)) for t in everyone}
        c.check("arm: every emulated motor in Run", all(v["mode"] == "Run" for v in truth.values()))

        # 3. track a small step (well under every setup's torque trip: kp 15 x 0.02 rad = 0.3 N*m)
        target = {t: m.pos + 0.02 for t, m in h.motors().items()}
        for t, p in target.items():
            h.hold(P.DRV_REQ_ENABLE, [t], p)
        h.advance(1000)
        errs = {t: abs(robot.motor(t[0], h.can_id(t))["pos"] - p) for t, p in target.items()}
        c.check("track: every shaft within 3 mrad of its target after 1 s", max(errs.values()) < 0.003,
                f"worst {max(errs.values()) * 1000:.2f} mrad; friction deadband Fc/kp = 1.3 mrad")
        gap = max(abs(m.pos - robot.motor(t[0], h.can_id(t))["pos"]) for t, m in h.motors().items())
        c.check("track: telemetry position matches the shaft", gap < 0.001, f"{gap * 1000:.2f} mrad")

        # 4. a motor reports a fault, the host clears it and re-arms
        first = everyone[0]
        robot.motor_do(first[0], h.can_id(first), "InjectFault 2")     # overtemperature
        m = lambda t: h.motors()[t]                                      # noqa: E731
        c.check("fault: motor FAULT / MOTOR_REPORTED within 50 ms",
                wait_for(h, lambda: m(first).drive_fault == P.DRVF_MOTOR_REPORTED, 50),
                f"{m(first).drive_state_name}/{m(first).drive_fault_name}")
        c.check("fault: the other motors stay ENABLED",
                all(m(t).drive_state == P.DRV_ENABLED for t in everyone[1:]))
        h.hold(P.DRV_REQ_RESET, [first])
        h.advance(100)
        h.hold(P.DRV_REQ_ENABLE, [first])
        c.check("fault: RESET then ENABLE re-arms it within 300 ms",
                wait_for(h, lambda: m(first).drive_state == P.DRV_ENABLED, 300), m(first).drive_state_name)

        # 5. a motor goes silent on CAN
        last = everyone[-1]
        robot.motor_do(last[0], h.can_id(last), "Present false")
        c.check("unplug: FAULT / CAN_TIMEOUT within 150 ms",
                wait_for(h, lambda: m(last).drive_fault == P.DRVF_CAN_TIMEOUT, 150),
                f"{m(last).drive_state_name}/{m(last).drive_fault_name}")
        robot.motor_do(last[0], h.can_id(last), "Present true")
        h.hold(P.DRV_REQ_RESET, [last])
        h.advance(100)
        h.hold(P.DRV_REQ_ENABLE, [last])
        c.check("unplug: plugged back, RESET then ENABLE re-arms it",
                wait_for(h, lambda: m(last).drive_state == P.DRV_ENABLED, 300), m(last).drive_state_name)

        # 6. the host goes silent while armed: the master's host-death watchdog must stop everything
        h.held.clear()
        lost = wait_for(h, lambda: h.tele()["robot_state"] == P.ROBOT_HOST_LOST, 100)
        c.check("host death: robot_state HOST_LOST within 100 ms of silence", lost)
        h.advance(500)
        c.check("host death: no motor still ENABLED 0.5 s later",
                all(s != P.DRV_ENABLED for s in state().values()),
                str({k: P.DRIVE_STATE_NAMES.get(v) for k, v in state().items()}))
        c.check("host death: every emulated motor out of Run",
                all(robot.motor(t[0], h.can_id(t))["mode"] != "Run" for t in everyone))

        # 7. garbage on the host link, then a valid frame: the scanner must skip exactly the garbage.
        #    (Garbage alone sits in the scanner until 16 header bytes are there to judge.)
        before, cycle0 = h.status("master"), h.tele()["cycle_id"]
        robot.send(bytes([0xA5] * 7) + P.encode_frame(P.MSG_PING, P.NODE_JETSON, P.NODE_MASTER))
        h.raws.pop(P.MSG_PING, None)
        h.advance(100)
        after = h.status("master")
        c.check("junk: master discards exactly the 7 garbage bytes",
                after["rx_discarded_bytes"] - before["rx_discarded_bytes"] == 7,
                f"rx_discarded_bytes {before['rx_discarded_bytes']} -> {after['rx_discarded_bytes']}")
        c.check("junk: the PING behind the garbage is still answered", P.MSG_PING in h.raws)
        c.check("junk: telemetry keeps flowing", h.tele()["cycle_id"] >= cycle0 + 19)
    return c.failures


def main():
    ap = argparse.ArgumentParser(description="emulator scenario tests")
    ap.add_argument("--setup", action="append", help="setup(s) to test (default: every built setup)")
    a = ap.parse_args()
    setups = a.setup or sorted(d for d in os.listdir(os.path.join(HERE, "build"))
                               if os.path.isfile(os.path.join(HERE, "build", d, "master.elf")))
    if not setups:
        sys.exit("nothing built: python3 robosim.py build --setup <setup>")
    failures = []
    for s in setups:
        failures += run_setup(s)
    print(f"\n{len(failures)} failure(s)" + "".join(f"\n  {f}" for f in failures))
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
