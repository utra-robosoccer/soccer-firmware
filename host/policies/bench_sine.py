"""bench_sine — headless sine bench policy (the canonical measurement driver).

Arms every selected motor to HOLD (with a one-shot fault-reset) for a short settle, then
streams a MIT position sine CENTERED on each motor's position at start — so there is no
HOLD→sine step that would trip the overtorque guard. The commanded position is clamped to
each motor's soft limits as a host-side safety (the slave also clamps).

All timing comes from the injected master-clock t_ns (telemetry-driven loop), so a logged
run replays identically. No keyboard/TTY — suitable for CI/automation.

Args (via run_policy): --amp RAD (default 0.2), --freq HZ (default 0.4),
    --motors "s.l,s.l" or "all" (default all configured), --dur S (default: run until Ctrl-C).
"""
import math

from master_link.motor_config_gen import MOTORS, MOTOR_SOFT_MIN, MOTOR_SOFT_MAX

from .base import Policy, Action, MotorCommand, MODE_HOLD, MODE_MIT, LinkState

DEFAULT_AMP_RAD = 0.2      # ± swing; small + clamped to soft limits by default
DEFAULT_FREQ_HZ = 0.4
ARM_SECONDS     = 1.5      # HOLD settle before MIT
FAULT_RESET_S   = 0.3      # send fault-reset only at the very start of the arm


def _parse_motors(spec):
    """'all' or None → every configured motor; else 's.l,s.l' → [(slave, local), …]."""
    if not spec or spec == "all":
        return [(m["slave"], m["idx"]) for m in MOTORS]
    out = []
    for tok in spec.split(","):
        tok = tok.strip()
        if not tok:
            continue
        s, l = tok.split(".")
        out.append((int(s), int(l)))
    return out


class BenchSinePolicy(Policy):
    name = "bench_sine"

    def __init__(self, amp=DEFAULT_AMP_RAD, freq=DEFAULT_FREQ_HZ, motors=None, dur=None,
                 arm_s=ARM_SECONDS):
        self._amp = float(amp)
        self._w = 2.0 * math.pi * float(freq)
        self._dur = None if dur is None else float(dur)
        self._arm_s = float(arm_s)
        self._motors = _parse_motors(motors)
        # soft limits per (slave, local), via the global index
        gidx = {(m["slave"], m["idx"]): g for g, m in enumerate(MOTORS)}
        self._lim = {}
        for k in self._motors:
            g = gidx.get(k)
            self._lim[k] = (MOTOR_SOFT_MIN[g], MOTOR_SOFT_MAX[g]) if g is not None else None
        self._center = {}
        self._t0 = None

    @classmethod
    def add_args(cls, ap):
        ap.add_argument("--amp", type=float, default=DEFAULT_AMP_RAD,
                        help="bench_sine amplitude (rad, default %.2f)" % DEFAULT_AMP_RAD)
        ap.add_argument("--freq", type=float, default=DEFAULT_FREQ_HZ,
                        help="bench_sine frequency (Hz, default %.2f)" % DEFAULT_FREQ_HZ)
        ap.add_argument("--motors", default="all",
                        help="bench_sine motors: 'all' or 's.l,s.l' (default all)")
        ap.add_argument("--dur", type=float, default=None,
                        help="bench_sine run seconds after arm (default: until Ctrl-C)")

    @classmethod
    def from_args(cls, args):
        return cls(amp=getattr(args, "amp", DEFAULT_AMP_RAD),
                   freq=getattr(args, "freq", DEFAULT_FREQ_HZ),
                   motors=getattr(args, "motors", "all"),
                   dur=getattr(args, "dur", None))

    def setup(self, state: LinkState, t_ns: int) -> None:
        for k in self._motors:
            snap = state.motors.get(k) if state and state.motors else None
            self._center[k] = snap.pos if snap else 0.0
        self._t0 = t_ns

    def step(self, state: LinkState, t_ns: int) -> Action:
        el = (t_ns - self._t0) / 1e9
        if self._dur is not None and el > self._arm_s + self._dur:
            raise KeyboardInterrupt            # clean stop via run_policy's shutdown
        cmds = []
        for k in self._motors:
            s, l = k
            c = self._center.get(k, 0.0)
            if el < self._arm_s:               # arm + hold current position
                cmds.append(MotorCommand(s, l, mode=MODE_HOLD, pos=c,
                                         use_config_gains=True, fault_reset=(el < FAULT_RESET_S)))
            else:
                t = el - self._arm_s
                pos = c + self._amp * math.sin(self._w * t)
                vel = self._amp * self._w * math.cos(self._w * t)
                lim = self._lim.get(k)
                if lim is not None:            # host-side soft-limit clamp (slave also clamps)
                    pos = min(max(pos, lim[0]), lim[1])
                cmds.append(MotorCommand(s, l, mode=MODE_MIT, pos=pos, vel=vel,
                                         use_config_gains=True))
        return Action(motors=cmds)
