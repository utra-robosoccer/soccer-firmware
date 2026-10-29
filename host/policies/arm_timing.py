"""arm_timing — measure per-motor DISABLED→ENABLED (enable) latency host-side.

Phases (all timing on the MASTER clock, t_ns):
  [0, idle_s/2)      send DISABLE every tick (known-off baseline);
  [idle_s/2, idle_s) send RESET every tick (clears any latched fault);
  [idle_s, ...)      send ENABLE + a VALID hold tuple (current position, config
                     gains) every tick — the v9 reference flow — and record:
                       t_req  = t_ns of the FIRST tick an ENABLE was sent,
                       t_hold = t_ns of the first telemetry showing ENABLED;
  all enabled (or --arm-timeout) → short settle, print the table, stop cleanly
  (run_policy's shutdown then disables the motors).

The number reported is the FULL pipeline as a policy experiences it: host tick →
mailbox apply (n+1) → slave handshake → Run confirmation → ENABLED visible in
telemetry. Resolution is one master cycle (5 ms @ 200 Hz). Streaming the tuple
through ENABLING means the motor is never limp once ENABLED shows.

Run:
    python3 host/apps/run_policy.py --policy arm_timing --rate 200
    python3 host/apps/run_policy.py --policy arm_timing --rate 200 --motors 0.0,0.1
"""
from .base import (Policy, Action, MotorCommand, LinkState,
                   REQ_DISABLE, REQ_ENABLE, REQ_RESET)
from master_link.protocol import DRV_ENABLED, DRV_DISABLED, DRIVE_STATE_NAMES
from master_link.motor_config_gen import MOTORS

IDLE_SECONDS      = 0.5    # disable/reset known-state window
ARM_TIMEOUT_S     = 3.0    # give up on a motor after this long in the HOLD phase
SETTLE_SECONDS    = 0.5    # hold after the last motor armed, then stop


def _parse_motors(spec):
    if not spec or spec == "all":
        return [(m["slave"], m["idx"]) for m in MOTORS]
    out = []
    for tok in spec.split(","):
        tok = tok.strip()
        if tok:
            s, l = tok.split(".")
            out.append((int(s), int(l)))
    return out


class ArmTimingPolicy(Policy):
    name = "arm_timing"

    def __init__(self, motors=None, idle_s=IDLE_SECONDS, arm_timeout=ARM_TIMEOUT_S):
        self._motors = _parse_motors(motors)
        self._idle_s = float(idle_s)
        self._arm_timeout = float(arm_timeout)
        self._t0 = None          # ns, first step
        self._t_req = None       # ns, first tick ENABLE was sent (same for all motors)
        self._t_hold = {}        # key -> ns of first ENABLED sighting
        self._hold_pos = {}      # key -> hold target captured once at ENABLE start
        self._baseline = {}      # key -> state observed at the end of the known-off phase
        self._done_at = None     # ns when every motor was armed (or timed out)
        self._reported = False

    @classmethod
    def add_args(cls, ap):
        ap.add_argument("--motors", default="all",
                        help='motors to arm: "all" or "s.l,s.l,..." (default: all)')
        ap.add_argument("--idle-s", type=float, default=IDLE_SECONDS,
                        help="seconds of IDLE before the HOLD phase (default 0.5)")
        ap.add_argument("--arm-timeout", type=float, default=ARM_TIMEOUT_S,
                        help="seconds to wait for HOLD before giving up (default 3)")

    @classmethod
    def from_args(cls, args):
        return cls(motors=getattr(args, "motors", "all"),
                   idle_s=getattr(args, "idle_s", IDLE_SECONDS),
                   arm_timeout=getattr(args, "arm_timeout", ARM_TIMEOUT_S))

    def _snap(self, state: LinkState, k):
        return state.motors.get(k) if state and state.motors else None

    def _report(self, state: LinkState):
        self._reported = True
        print("\narm_timing results (master-clock, resolution = 1 cycle):")
        print(f"  {'motor':>6}  {'arm time':>9}  {'final state':>11}  fault")
        for k in self._motors:
            snap = self._snap(state, k)
            sname = snap.drive_state_name if snap else "no-tele"
            cause = snap.drive_fault_name if snap else "-"
            t_h = self._t_hold.get(k)
            if t_h is not None and self._t_req is not None:
                ms = (t_h - self._t_req) / 1e6
                print(f"  s{k[0]}.m{k[1]:<3}  {ms:7.1f} ms  {sname:>11}  {cause}")
            else:
                print(f"  s{k[0]}.m{k[1]:<3}  {'TIMEOUT':>9}  {sname:>11}  {cause}")
        armed = [k for k in self._motors if k in self._t_hold]
        if armed and self._t_req is not None:
            spans = [(self._t_hold[k] - self._t_req) / 1e6 for k in armed]
            print(f"  armed {len(armed)}/{len(self._motors)}: "
                  f"first {min(spans):.1f} ms, last {max(spans):.1f} ms")

    def step(self, state: LinkState, t_ns: int) -> Action:
        if self._t0 is None:
            self._t0 = t_ns
        el = (t_ns - self._t0) / 1e9
        cmds = []

        # Phase A: known disabled state (DISABLE, then RESET for latched faults).
        if el < self._idle_s:
            req = REQ_DISABLE if el < self._idle_s / 2.0 else REQ_RESET
            for k in self._motors:
                cmds.append(MotorCommand(k[0], k[1], req=req))
            return Action(motors=cmds)

        # Phase B: ENABLE + hold tuple every tick; time the first ENABLED sighting.
        if self._t_req is None:
            self._t_req = t_ns
            for k in self._motors:
                snap = self._snap(state, k)
                self._baseline[k] = snap.drive_state if snap else None
                # Capture the hold target ONCE — re-sampling each tick would let a
                # loaded joint sag (the target would follow the measured position).
                self._hold_pos[k] = snap.pos if snap else 0.0
                if snap is not None and snap.drive_state != DRV_DISABLED:
                    print(f"arm_timing: s{k[0]}.m{k[1]} not DISABLED at ENABLE start "
                          f"({DRIVE_STATE_NAMES.get(snap.drive_state, snap.drive_state)}) — "
                          f"its number will not be a clean DISABLED→ENABLED measurement")

        for k in self._motors:
            snap = self._snap(state, k)
            if k not in self._t_hold and snap is not None and snap.drive_state == DRV_ENABLED:
                self._t_hold[k] = t_ns
            cmds.append(MotorCommand(k[0], k[1], req=REQ_ENABLE, valid=True,
                                     pos=self._hold_pos.get(k, 0.0),
                                     use_config_gains=True))

        arm_el = (t_ns - self._t_req) / 1e9
        all_armed = all(k in self._t_hold for k in self._motors)
        if self._done_at is None and (all_armed or arm_el > self._arm_timeout):
            self._done_at = t_ns
        if self._done_at is not None and (t_ns - self._done_at) / 1e9 > SETTLE_SECONDS:
            if not self._reported:
                self._report(state)
            raise KeyboardInterrupt      # clean stop via run_policy's shutdown
        return Action(motors=cmds)

    def teardown(self) -> None:
        if not self._reported and self._t_req is not None:
            # Interrupted early: still print what we saw.
            self._reported = True
            print("\narm_timing (partial):", {f"s{k[0]}.m{k[1]}":
                  f"{(v - self._t_req) / 1e6:.1f} ms" for k, v in self._t_hold.items()})
