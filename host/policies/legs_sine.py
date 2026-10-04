"""legs_sine — a walk-imitation driver for the robot_legs setup.

Sine-waves the **hip_pitch** and **knee** joints of every leg, with the left and right
legs in **anti-phase** (half-cycle offset) so the motion alternates like a walk. Every other
joint (hip_roll, hip_yaw, ankle) is held at the pose captured when MIT begins, so the leg stays
controlled while only the two walk joints swing.

Each leg is one slave MCU / CAN bus; the peak sign (which direction the sine starts) is picked
by slave-index parity — even slaves (slave0 = left) start at +A, odd (slave1 = right) at -A —
so the two legs are a half cycle apart. hip_pitch and ankle on the same leg are in phase.

Like bench_sine: arm everything to HOLD, then MIT. The moving joints use the minimum-jerk
SineStart (continuous position + velocity from HOLD into the centered-on-0 sine); the amplitude
is soft-limit-checked per joint and refused (not clamped) if ±A leaves the range. All timing
comes from the master clock t_ns.
"""
import math

from master_link.motor_config_gen import MOTORS, MOTOR_SOFT_MIN, MOTOR_SOFT_MAX

from .base import Policy, Action, MotorCommand, MODE_HOLD, MODE_MIT, LinkState
from .sine_start import SineStart, fits_soft_limits

DEFAULT_AMP_RAD    = 0.30
DEFAULT_FREQ_HZ    = 0.5
DEFAULT_MOVE_SPEED = 3.0     # rad/s, HOLD→peak move cap
ARM_SECONDS        = 2.0
FAULT_RESET_S      = 0.5

# Joints that swing (matched by joint_name suffix, so it tracks the config).
MOVING_SUFFIXES = ("hip_pitch", "knee")


class LegsSinePolicy(Policy):
    name = "legs_sine"

    def __init__(self, amp=DEFAULT_AMP_RAD, freq=DEFAULT_FREQ_HZ, dur=None,
                 move_speed=DEFAULT_MOVE_SPEED, arm_s=ARM_SECONDS):
        self._amp = float(amp)
        self._omega = 2.0 * math.pi * float(freq)
        self._move_speed = float(move_speed)
        self._dur = None if dur is None else float(dur)
        self._arm_s = float(arm_s)

        gidx = {(m["slave"], m["idx"]): g for g, m in enumerate(MOTORS)}
        self._all = [(m["slave"], m["idx"]) for m in MOTORS]
        self._moving = [(m["slave"], m["idx"]) for m in MOTORS
                        if str(m["joint_name"]).endswith(MOVING_SUFFIXES)]
        # Per-leg peak sign → even slave (left) starts +A, odd (right) -A → legs anti-phase.
        self._sign = {k: (1 if k[0] % 2 == 0 else -1) for k in self._moving}
        self._lim = {}
        for k in self._moving:
            g = gidx.get(k)
            self._lim[k] = (MOTOR_SOFT_MIN[g], MOTOR_SOFT_MAX[g]) if g is not None else (None, None)
        self._traj = {k: SineStart(self._amp, self._omega, self._move_speed) for k in self._moving}
        self._hold = {}        # stationary joints: pose captured at MIT start
        self._t0 = None
        self._mit_begun = False

    @classmethod
    def add_args(cls, ap):
        ap.add_argument("--amp", type=float, default=DEFAULT_AMP_RAD,
                        help="legs_sine amplitude (rad, default %.2f)" % DEFAULT_AMP_RAD)
        ap.add_argument("--freq", type=float, default=DEFAULT_FREQ_HZ,
                        help="legs_sine frequency (Hz, default %.2f)" % DEFAULT_FREQ_HZ)
        ap.add_argument("--move-speed", type=float, default=DEFAULT_MOVE_SPEED,
                        help="HOLD→peak move peak speed (rad/s, default %.2f)" % DEFAULT_MOVE_SPEED)
        ap.add_argument("--dur", type=float, default=None,
                        help="legs_sine run seconds after arm (default: until Ctrl-C)")

    @classmethod
    def from_args(cls, args):
        return cls(amp=getattr(args, "amp", DEFAULT_AMP_RAD),
                   freq=getattr(args, "freq", DEFAULT_FREQ_HZ),
                   dur=getattr(args, "dur", None),
                   move_speed=getattr(args, "move_speed", DEFAULT_MOVE_SPEED))

    def setup(self, state: LinkState, t_ns: int) -> None:
        if not self._moving:
            raise ValueError("legs_sine: no hip_pitch/ankle joints in the active config "
                             "(is robot_legs active?).")
        for k in self._moving:
            lo, hi = self._lim[k]
            if lo is not None and not fits_soft_limits(self._amp, lo, hi):
                raise ValueError(
                    f"legs_sine refused: amplitude A={self._amp:.3f} rad centered on 0 "
                    f"exceeds soft limits [{lo:.3f}, {hi:.3f}] on motor s{k[0]}.m{k[1]}. "
                    f"Reduce --amp.")
        self._t0 = t_ns

    def step(self, state: LinkState, t_ns: int) -> Action:
        el = (t_ns - self._t0) / 1e9
        if self._dur is not None and el > self._arm_s + self._dur:
            raise KeyboardInterrupt            # clean stop via run_policy's shutdown
        cmds = []

        if el < self._arm_s:                   # arm: HOLD current position (all joints)
            for k in self._all:
                snap = state.motors.get(k) if state and state.motors else None
                p = snap.pos if snap else 0.0
                cmds.append(MotorCommand(k[0], k[1], mode=MODE_HOLD, pos=p,
                                         use_config_gains=True, fault_reset=(el < FAULT_RESET_S)))
            return Action(motors=cmds)

        if not self._mit_begun:                # MIT start: capture p0, plan each move
            for k in self._all:
                snap = state.motors.get(k) if state and state.motors else None
                p0 = snap.pos if snap else 0.0
                if k in self._traj:
                    lo, hi = self._lim[k]
                    self._traj[k].begin(p0, t_ns, lo, hi, who=f"s{k[0]}.m{k[1]}",
                                        peak_sign=self._sign[k])
                else:
                    self._hold[k] = p0         # stationary joint: hold the start pose
            self._mit_begun = True

        for k in self._all:
            if k in self._traj:                # swinging joint
                pos, vel = self._traj[k].sample(t_ns)
                cmds.append(MotorCommand(k[0], k[1], mode=MODE_MIT, pos=pos, vel=vel,
                                         use_config_gains=True))
            else:                              # held joint: stiff at the captured pose
                cmds.append(MotorCommand(k[0], k[1], mode=MODE_MIT, pos=self._hold[k], vel=0.0,
                                         use_config_gains=True))
        return Action(motors=cmds)
