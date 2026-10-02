"""bench_sine — headless sine bench policy (the canonical measurement driver).

Arms every selected motor to HOLD (one-shot fault-reset) for a short settle, then starts a
MIT sine CENTERED ON 0: a minimum-jerk move from the motor's current position to the nearest
peak (±A), then the sine from that peak's phase — so position and velocity are continuous
from HOLD through the move into the sine (no step, no overtorque). If ±A would exceed a
motor's soft limits the run is refused with a clear message (never clamped silently).

All timing comes from the injected master-clock t_ns (telemetry-driven loop), so a logged
run replays identically. No keyboard/TTY — suitable for CI/automation.

Args (via run_policy): --amp RAD (default 0.2), --freq HZ (default 0.4),
    --move-speed RAD/S (default 0.5), --motors "s.l,s.l" or "all", --dur S.
"""
import math

from master_link.motor_config_gen import MOTORS, MOTOR_SOFT_MIN, MOTOR_SOFT_MAX

from .base import Policy, Action, MotorCommand, MODE_HOLD, MODE_MIT, MODE_IDLE, LinkState
from .sine_start import SineStart, fits_soft_limits

DEFAULT_AMP_RAD   = 0.2
DEFAULT_FREQ_HZ   = 0.4
DEFAULT_MOVE_SPEED = 0.5   # rad/s, min-jerk peak speed cap for the HOLD→peak move
ARM_SECONDS       = 1.5
FAULT_RESET_S     = 0.3


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


class BenchSinePolicy(Policy):
    name = "bench_sine"

    def __init__(self, amp=DEFAULT_AMP_RAD, freq=DEFAULT_FREQ_HZ, motors=None, dur=None,
                 move_speed=DEFAULT_MOVE_SPEED, arm_s=ARM_SECONDS,
                 drop_motor=None, drop_at=None):
        self._amp = float(amp)
        self._drop = None if not drop_motor else _parse_motors(drop_motor)[0]
        self._drop_at = None if drop_at is None else float(drop_at)
        self._omega = 2.0 * math.pi * float(freq)
        self._move_speed = float(move_speed)
        self._dur = None if dur is None else float(dur)
        self._arm_s = float(arm_s)
        self._motors = _parse_motors(motors)
        gidx = {(m["slave"], m["idx"]): g for g, m in enumerate(MOTORS)}
        self._lim = {}
        for k in self._motors:
            g = gidx.get(k)
            self._lim[k] = (MOTOR_SOFT_MIN[g], MOTOR_SOFT_MAX[g]) if g is not None else (None, None)
        self._t0 = None
        # Every motor uses the same --amp.
        self._amp_of = {k: self._amp for k in self._motors}
        self._traj = {k: SineStart(self._amp_of[k], self._omega, self._move_speed)
                      for k in self._motors}
        self._mit_begun = False

    @classmethod
    def add_args(cls, ap):
        ap.add_argument("--amp", type=float, default=DEFAULT_AMP_RAD,
                        help="bench_sine amplitude (rad, default %.2f)" % DEFAULT_AMP_RAD)
        ap.add_argument("--freq", type=float, default=DEFAULT_FREQ_HZ,
                        help="bench_sine frequency (Hz, default %.2f)" % DEFAULT_FREQ_HZ)
        ap.add_argument("--move-speed", type=float, default=DEFAULT_MOVE_SPEED,
                        help="HOLD→peak move peak speed (rad/s, default %.2f)" % DEFAULT_MOVE_SPEED)
        ap.add_argument("--motors", default="all",
                        help="bench_sine motors: 'all' or 's.l,s.l' (default all)")
        ap.add_argument("--dur", type=float, default=None,
                        help="bench_sine run seconds after arm (default: until Ctrl-C)")
        ap.add_argument("--drop-motor", default=None,
                        help="fault-isolation: CAN-disable this motor 's.l' mid-run")
        ap.add_argument("--drop-at", type=float, default=None,
                        help="seconds into MIT to drop --drop-motor (default: none)")

    @classmethod
    def from_args(cls, args):
        return cls(amp=getattr(args, "amp", DEFAULT_AMP_RAD),
                   freq=getattr(args, "freq", DEFAULT_FREQ_HZ),
                   motors=getattr(args, "motors", "all"),
                   dur=getattr(args, "dur", None),
                   move_speed=getattr(args, "move_speed", DEFAULT_MOVE_SPEED),
                   drop_motor=getattr(args, "drop_motor", None),
                   drop_at=getattr(args, "drop_at", None))

    def setup(self, state: LinkState, t_ns: int) -> None:
        # Fail fast, before arming, if the 0-centered sine would leave any soft range.
        for k in self._motors:
            lo, hi = self._lim[k]
            a = self._amp_of[k]
            if lo is not None and not fits_soft_limits(a, lo, hi):
                raise ValueError(
                    f"bench_sine refused: amplitude A={a:.3f} rad centered on 0 "
                    f"exceeds soft limits [{lo:.3f}, {hi:.3f}] on motor s{k[0]}.m{k[1]}. "
                    f"Reduce --amp.")
        self._t0 = t_ns

    def step(self, state: LinkState, t_ns: int) -> Action:
        el = (t_ns - self._t0) / 1e9
        if self._dur is not None and el > self._arm_s + self._dur:
            raise KeyboardInterrupt            # clean stop via run_policy's shutdown
        cmds = []
        if el < self._arm_s:                   # arm: HOLD current position
            for k in self._motors:
                snap = state.motors.get(k) if state and state.motors else None
                p = snap.pos if snap else 0.0
                cmds.append(MotorCommand(k[0], k[1], mode=MODE_HOLD, pos=p,
                                         use_config_gains=True, fault_reset=(el < FAULT_RESET_S)))
            return Action(motors=cmds)

        if not self._mit_begun:                # MIT start: read p0, plan each move
            for k in self._motors:
                snap = state.motors.get(k) if state and state.motors else None
                p0 = snap.pos if snap else 0.0
                lo, hi = self._lim[k]
                self._traj[k].begin(p0, t_ns, lo, hi, who=f"s{k[0]}.m{k[1]}")
            self._mit_begun = True

        drop_now = (self._drop is not None and self._drop_at is not None
                    and el >= self._arm_s + self._drop_at)
        for k in self._motors:
            if drop_now and k == self._drop:        # fault-isolation: CAN-disable one motor
                cmds.append(MotorCommand(k[0], k[1], mode=MODE_IDLE))
                continue
            pos, vel = self._traj[k].sample(t_ns)
            cmds.append(MotorCommand(k[0], k[1], mode=MODE_MIT, pos=pos, vel=vel,
                                     use_config_gains=True))
        return Action(motors=cmds)
