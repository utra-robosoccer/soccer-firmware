"""legs_walk — a bipedal-walking driver for the robot_legs setup.

Commands the **hip_pitch** and **knee** of every leg from a planar 2-link
inverse-kinematic gait; hip_roll, hip_yaw and ankle are held at the pose
captured when MIT begins, exactly like legs_sine.

Gait (gait phase φ in rad; the RIGHT leg is the reference leg):

    x_f(φ) = −S·cos(φ)                     foot stride about the hip (−: +S = fwd)
    y_f(φ) = −H + h_c·(max(0, −sin φ))²    planted depth H, lift h_c in swing

φ ∈ [0, π] is STANCE — the foot is planted at depth H and sweeps −S → +S under
the hip, driving the body forward. φ ∈ [π, 2π] is SWING — the foot lifts to h_c
and returns +S → −S. The LEFT leg runs the identical trajectory half a cycle later
(φ + π), so the legs alternate. The 2-link IK (L1 thigh, L2 shank, hip at the
origin, +x forward, −y down) turns the foot point into joint angles:

    R    = √(x_f² + y_f²)
    knee = acos((R² − L1² − L2²) / (2·L1·L2))     0 = leg straight
    β    = acos((L1² + R² − L2²) / (2·L1·R))
    γ    = atan2(x_f, −y_f)                       hip→foot line off vertical
    hip  = γ + β                                  0 = thigh straight down

Motor mapping: the right leg's angles pass straight through (modulo the
--hip-sign / --knee-sign direction conventions). The left leg's actuators are
mirrored across the centerline, so its commands are negated by default
(q_left = −θ(φ+π)); pass --no-left-mirror if the mirror is already undone in
firmware / motor config.

Centering & launch. Each walking joint is commanded

    q(t) = p0 + env(t)·(center + sign·(θ(φ(t)) − θ_ref) − p0)

with p0 = the pose captured at MIT start, θ_ref = the IK angle at MID-STANCE
(foot directly under the hip), and env() a min-jerk 0→1 ramp over --blend s
(continuous position and velocity out of HOLD, like SineStart; with --dur it
ramps back down to the standing pose before the clean stop). By default
center = p0: the gait oscillates around wherever each leg stands when the
policy arms — stand the robot in its nominal walk stance before starting.
--hip-offset / --knee-offset pin the gait to absolute motor angles
(q = sign·θ + offset) instead.

Like legs_sine: arm everything to HOLD, then MIT. The gait is soft-limit-checked
per joint and refused (not clamped) — at setup when the gait span cannot fit the
joint's window (or absolute offsets don't fit), and again at MIT begin once the
calibrated range around p0 is known. The foot trajectory is reach-checked over a
full cycle at setup. All timing comes from the master clock t_ns.

Bring-up: robot on a stand, halved max_tau (see the YAML note), small gait first
(--step 4 --lift 3 --freq 0.3). Legs swing together instead of alternating →
--no-left-mirror. Knee bows the wrong way / hyperextends → --knee-sign -1. Hip
swings backward at heel-strike → --hip-sign -1 (or a negative --step to simply
walk in reverse).
"""
import math
import sys

from master_link.motor_config_gen import MOTORS, MOTOR_SOFT_MIN, MOTOR_SOFT_MAX

from .base import Policy, Action, MotorCommand, MODE_HOLD, MODE_MIT, LinkState

# Defaults are the Desmos reference gait. Lengths may be in ANY consistent unit
# — the IK output is angles, so only the ratios step:height:lift:l1:l2 matter.
DEFAULT_L1      = 20.0    # thigh,  hip → knee
DEFAULT_L2      = 20.0    # shank,  knee → ankle
DEFAULT_HEIGHT  = 36.0    # H — hip→foot distance while planted (stance knee bend)
DEFAULT_STEP    = 10.0    # S — half-stride: foot sweeps ±S about the hip
DEFAULT_LIFT    = 7.0     # h_c — extra foot clearance at mid-swing
DEFAULT_FREQ_HZ = 0.5     # gait cycles/s (one cycle steps each leg once)
DEFAULT_PHASE   = 0.0     # gait phase at start (rad); 0 = the Desmos t_0 frame
DEFAULT_BLEND_S = 2.0     # min-jerk ramp from HOLD into the gait
ARM_SECONDS     = 2.0
FAULT_RESET_S   = 0.5

REACH_MARGIN = 1e-3       # relative margin kept from full extension / full fold
N_SWEEP      = 1024       # per-cycle sweep for reach + joint-range checks
VEL_DT       = 1e-3       # s — central-difference step for the velocity ff

HIP_SUFFIX = "hip_pitch"
KNEE_SUFFIX = "knee"
LEFT_PHASE = math.pi      # left leg lags the right by half a cycle


def _minjerk(u):
    """Quintic 0→1 ramp with zero velocity/accel at both ends."""
    if u <= 0.0:
        return 0.0
    if u >= 1.0:
        return 1.0
    return u * u * u * (10.0 - 15.0 * u + 6.0 * u * u)


def _foot(phase, step, height, lift):
    """Foot position (x, y down) in the hip frame at gait phase (rad).

    The stride sweep carries a leading minus so a positive --step walks this robot
    FORWARD: the plain S·cos φ sweep (planted foot front→back) came out backward on
    the hardware, so the frame is flipped here once, for both runtime and dry-run.
    A negative --step still walks backward (the docstring/--step help hold)."""
    x = -step * math.cos(phase)
    up = max(0.0, -math.sin(phase))          # 0 during stance, −sin φ in swing
    return x, -height + lift * up * up


def _ik(x, y, l1, l2):
    """Planar 2-link IK → (hip, knee); 0 = straight, vertical leg.

    Arccos arguments are clamped, so a slightly unreachable target degrades
    smoothly to a straight leg instead of blowing up (setup refuses such gaits).
    """
    r = math.hypot(x, y)
    c_knee = (r * r - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
    c_beta = (l1 * l1 + r * r - l2 * l2) / (2.0 * l1 * r)
    knee = math.acos(max(-1.0, min(1.0, c_knee)))
    beta = math.acos(max(-1.0, min(1.0, c_beta)))
    gamma = math.atan2(x, -y)
    return gamma + beta, knee


def _sweep_gait(l1, l2, step, height, lift):
    """Sweep one gait cycle → (ref, range).

    ref   = {"hip","knee"} IK angles at mid-stance (foot under the hip, depth H)
    range = {"hip","knee"} (min, max) IK angle over the whole cycle
    Raises ValueError if the foot leaves the leg's reach anywhere.
    """
    r_lo = abs(l1 - l2) + REACH_MARGIN * (l1 + l2)
    r_hi = (l1 + l2) * (1.0 - REACH_MARGIN)
    rng = {"hip": [math.inf, -math.inf], "knee": [math.inf, -math.inf]}
    for i in range(N_SWEEP):
        phi = (2.0 * math.pi * i) / N_SWEEP
        x, y = _foot(phi, step, height, lift)
        r = math.hypot(x, y)
        if r > r_hi or r < r_lo:
            raise ValueError(f"foot trajectory out of reach at phase {phi:.3f} "
                             f"(needs R={r:.2f}, leg reaches [{r_lo:.2f}, {r_hi:.2f}])")
        hip, knee = _ik(x, y, l1, l2)
        rng["hip"][0] = min(rng["hip"][0], hip)
        rng["hip"][1] = max(rng["hip"][1], hip)
        rng["knee"][0] = min(rng["knee"][0], knee)
        rng["knee"][1] = max(rng["knee"][1], knee)
    hip_ref, knee_ref = _ik(0.0, -height, l1, l2)
    return {"hip": hip_ref, "knee": knee_ref}, {k: tuple(v) for k, v in rng.items()}


class LegsWalkPolicy(Policy):
    name = "legs_walk"

    def __init__(self, l1=DEFAULT_L1, l2=DEFAULT_L2, step=DEFAULT_STEP,
                 height=DEFAULT_HEIGHT, lift=DEFAULT_LIFT, freq=DEFAULT_FREQ_HZ,
                 phase=DEFAULT_PHASE, blend=DEFAULT_BLEND_S, dur=None,
                 hip_sign=1, knee_sign=1, mirror_left=True,
                 hip_offset=None, knee_offset=None, arm_s=ARM_SECONDS):
        if hip_sign not in (1, -1) or knee_sign not in (1, -1):
            raise ValueError("legs_walk: hip_sign/knee_sign must be +1 or -1.")
        self._l1, self._l2 = float(l1), float(l2)
        self._step, self._height, self._lift = float(step), float(height), float(lift)
        self._omega = 2.0 * math.pi * float(freq)
        self._freq = float(freq)
        self._phase0 = float(phase)
        self._blend = float(blend)
        self._dur = None if dur is None else float(dur)
        self._arm_s = float(arm_s)
        self._offset = {"hip": None if hip_offset is None else float(hip_offset),
                        "knee": None if knee_offset is None else float(knee_offset)}

        gidx = {(m["slave"], m["idx"]): g for g, m in enumerate(MOTORS)}
        self._all = [(m["slave"], m["idx"]) for m in MOTORS]
        # Walking joints (matched by joint_name suffix, so it tracks the config),
        # classified into leg side and hip/knee. Side from the name (L_/R_,
        # left/right); falls back to slave parity (slave0 = left) like legs_sine.
        self._moving = {}
        for m in MOTORS:
            name = str(m["joint_name"]).lower()
            if name.endswith(HIP_SUFFIX):
                kind = "hip"
            elif name.endswith(KNEE_SUFFIX):
                kind = "knee"
            else:
                continue
            if name.startswith("l") or "left" in name:
                side = "L"
            elif name.startswith("r") or "right" in name:
                side = "R"
            else:
                side = "L" if m["slave"] % 2 == 0 else "R"
            key = (m["slave"], m["idx"])
            g = gidx.get(key)
            base = hip_sign if kind == "hip" else knee_sign
            sign = base * (-1 if (side == "L" and mirror_left) else 1)
            self._moving[key] = {
                "kind": kind, "side": side, "sign": sign,
                "lohi": (MOTOR_SOFT_MIN[g], MOTOR_SOFT_MAX[g]) if g is not None else (None, None),
            }

        self._ref = {}      # kind → IK angle at mid-stance        (set in setup)
        self._range = {}    # kind → (min, max) IK angle per cycle (set in setup)
        self._p0 = {}       # every joint's pose captured at MIT start
        self._center = {}   # walking joint → gait centre in motor units
        self._t0 = None
        self._t_mit = None
        self._mit_begun = False

    # ------------------------------------------------------------------ cli

    @classmethod
    def add_args(cls, ap):
        ap.add_argument("--l1", type=float, default=DEFAULT_L1,
                        help="thigh length, hip→knee (any unit — only ratios matter; default %.1f)" % DEFAULT_L1)
        ap.add_argument("--l2", type=float, default=DEFAULT_L2,
                        help="shank length, knee→ankle (default %.1f)" % DEFAULT_L2)
        ap.add_argument("--step", type=float, default=DEFAULT_STEP,
                        help="half-stride S: the foot sweeps ±S about the hip each cycle "
                             "(negative = walk backward; default %.1f)" % DEFAULT_STEP)
        ap.add_argument("--height", type=float, default=DEFAULT_HEIGHT,
                        help="stance hip→foot distance H — sets the stance knee bend; must stay "
                             "inside the leg's reach (default %.1f)" % DEFAULT_HEIGHT)
        ap.add_argument("--lift", type=float, default=DEFAULT_LIFT,
                        help="swing-foot clearance h_c at mid-swing (0 ≤ h_c < --height; default %.1f)" % DEFAULT_LIFT)
        ap.add_argument("--freq", type=float, default=DEFAULT_FREQ_HZ,
                        help="gait cycles/s — one cycle steps each leg once (default %.2f)" % DEFAULT_FREQ_HZ)
        ap.add_argument("--phase", type=float, default=DEFAULT_PHASE,
                        help="gait phase at start, rad (0 = right foot planted at the rear of its "
                             "stance, left at front entering swing; default %.2f)" % DEFAULT_PHASE)
        ap.add_argument("--blend", type=float, default=DEFAULT_BLEND_S,
                        help="min-jerk ramp from the held pose into the gait, s (default %.1f)" % DEFAULT_BLEND_S)
        ap.add_argument("--dur", type=float, default=None,
                        help="gait seconds after arm, then ramp out over --blend and stop cleanly "
                             "(default: until Ctrl-C)")
        ap.add_argument("--hip-sign", type=int, choices=(1, -1), default=1,
                        help="RIGHT-leg hip_pitch convention: +1 if positive motor angle swings "
                             "the thigh forward (default +1)")
        ap.add_argument("--knee-sign", type=int, choices=(1, -1), default=1,
                        help="RIGHT-leg knee convention: +1 if positive motor angle bends the "
                             "knee (default +1)")
        ap.add_argument("--no-left-mirror", action="store_true",
                        help="don't negate the LEFT leg's commands (use if the mirror is already "
                             "undone in firmware/motor config)")
        ap.add_argument("--hip-offset", type=float, default=None,
                        help="absolute hip_pitch offset (rad); default calibrates the gait around "
                             "the pose captured at start")
        ap.add_argument("--knee-offset", type=float, default=None,
                        help="absolute knee offset (rad); default calibrates from the start pose")

    @classmethod
    def from_args(cls, args):
        return cls(l1=getattr(args, "l1", DEFAULT_L1),
                   l2=getattr(args, "l2", DEFAULT_L2),
                   step=getattr(args, "step", DEFAULT_STEP),
                   height=getattr(args, "height", DEFAULT_HEIGHT),
                   lift=getattr(args, "lift", DEFAULT_LIFT),
                   freq=getattr(args, "freq", DEFAULT_FREQ_HZ),
                   phase=getattr(args, "phase", DEFAULT_PHASE),
                   blend=getattr(args, "blend", DEFAULT_BLEND_S),
                   dur=getattr(args, "dur", None),
                   hip_sign=getattr(args, "hip_sign", 1),
                   knee_sign=getattr(args, "knee_sign", 1),
                   mirror_left=not getattr(args, "no_left_mirror", False),
                   hip_offset=getattr(args, "hip_offset", None),
                   knee_offset=getattr(args, "knee_offset", None))

    # ------------------------------------------------------------ planning

    def setup(self, state: LinkState, t_ns: int) -> None:
        if not self._moving:
            raise ValueError("legs_walk: no hip_pitch/knee joints in the active config "
                             "(is robot_legs active?).")
        legs = {}
        for info in self._moving.values():
            legs.setdefault(info["side"], set()).add(info["kind"])
        for side, kinds in sorted(legs.items()):
            if kinds != {"hip", "knee"}:
                raise ValueError(f"legs_walk refused: {side} leg has {sorted(kinds)} — a walk "
                                 "needs both hip_pitch and knee per leg.")
        if len(legs) == 1:
            print(f"legs_walk: only the {sorted(legs)[0]} leg has walk joints — running it "
                  "single-leg.", file=sys.stderr)
        if self._freq <= 0.0 or self._blend <= 0.0:
            raise ValueError("legs_walk refused: --freq and --blend must be > 0.")
        if self._lift < 0.0 or self._lift >= self._height:
            raise ValueError("legs_walk refused: --lift must be >= 0 and < --height "
                             "(the foot must stay below the hip).")
        if self._l1 <= 0.0 or self._l2 <= 0.0:
            raise ValueError("legs_walk refused: --l1 and --l2 must be > 0.")

        try:
            self._ref, self._range = _sweep_gait(self._l1, self._l2, self._step,
                                                 self._height, self._lift)
        except ValueError as e:
            raise ValueError(f"legs_walk refused: {e}. Lower --step/--height/--lift or fix --l1/--l2.")

        for key, info in self._moving.items():
            t_min, t_max = self._range[info["kind"]]
            lo, hi = info["lohi"]
            if lo is not None and (t_max - t_min) > (hi - lo) + 1e-9:
                raise ValueError(f"legs_walk refused: {info['kind']} gait spans "
                                 f"{t_max - t_min:.3f} rad but s{key[0]}.m{key[1]}'s soft-limit "
                                 f"window is only {hi - lo:.3f} rad. Reduce --step/--lift or "
                                 "widen the YAML soft limits.")
            if self._offset[info["kind"]] is not None:    # absolute mode: checkable now
                msg = self._limit_violation(key, self._offset[info["kind"]]
                                            + info["sign"] * self._ref[info["kind"]])
                if msg is not None:
                    raise ValueError(msg)
        self._t0 = t_ns

    def _gait_bounds(self, key, center):
        """Motor-space [lo, hi] the un-blended gait visits on `key` about `center`."""
        info = self._moving[key]
        t_min, t_max = self._range[info["kind"]]
        ref = self._ref[info["kind"]]
        a = center + info["sign"] * (t_min - ref)
        b = center + info["sign"] * (t_max - ref)
        return (a, b) if a <= b else (b, a)

    def _limit_violation(self, key, center):
        """None if the gait about `center` fits the soft limits, else a message."""
        lo, hi = self._moving[key]["lohi"]
        if lo is None:
            return None
        q_lo, q_hi = self._gait_bounds(key, center)
        if lo <= q_lo and q_hi <= hi:
            return None
        kind = self._moving[key]["kind"]
        return (f"legs_walk refused: {kind} gait needs [{q_lo:.3f}, {q_hi:.3f}] rad on "
                f"s{key[0]}.m{key[1]}, outside soft limits [{lo:.3f}, {hi:.3f}]. Reduce "
                "--step/--lift/--height, stand the robot nearer its walk stance, or widen "
                "the YAML soft limits.")

    # ------------------------------------------------------------- runtime

    def _envelope(self, emit):
        """Min-jerk 0→1 ramp-in over --blend s; with --dur, a 1→0 ramp-out that
        lands back on the standing pose before the clean stop."""
        env = _minjerk(emit / self._blend)
        if self._dur is not None:
            env *= 1.0 - _minjerk((emit - self._dur) / self._blend)
        return env

    def _cmd(self, key, emit):
        """Motor-space position of a walking joint `emit` s after MIT start:
        the IK gait, centred per joint, scaled in/out by the min-jerk envelope."""
        info = self._moving[key]
        phase = (self._phase0 + self._omega * emit
                 + (LEFT_PHASE if info["side"] == "L" else 0.0))
        x, y = _foot(phase, self._step, self._height, self._lift)
        hip, knee = _ik(x, y, self._l1, self._l2)
        theta = hip if info["kind"] == "hip" else knee
        gait = self._center[key] + info["sign"] * (theta - self._ref[info["kind"]])
        p0 = self._p0[key]
        return p0 + self._envelope(emit) * (gait - p0)

    def _print_plan(self):
        for side in ("R", "L"):
            keys = sorted(k for k, i in self._moving.items() if i["side"] == side)
            if not keys:
                continue
            parts = []
            for key in keys:
                kind = self._moving[key]["kind"]
                q_lo, q_hi = self._gait_bounds(key, self._center[key])
                parts.append(f"{kind} p0 {self._p0[key]:+.3f} → [{q_lo:+.3f}, {q_hi:+.3f}] rad")
            print(f"legs_walk: {side} leg  " + ", ".join(parts))

    def step(self, state: LinkState, t_ns: int) -> Action:
        el = (t_ns - self._t0) / 1e9
        if self._dur is not None and el > self._arm_s + self._dur + self._blend:
            raise KeyboardInterrupt            # clean stop via run_policy's shutdown
        cmds = []

        if el < self._arm_s:                   # arm: HOLD current position (all joints)
            for k in self._all:
                snap = state.motors.get(k) if state and state.motors else None
                p = snap.pos if snap else 0.0
                cmds.append(MotorCommand(k[0], k[1], mode=MODE_HOLD, pos=p,
                                         use_config_gains=True, fault_reset=(el < FAULT_RESET_S)))
            return Action(motors=cmds)

        if not self._mit_begun:                # MIT start: capture p0, centre the gait
            for k in self._all:
                snap = state.motors.get(k) if state and state.motors else None
                self._p0[k] = snap.pos if snap else 0.0
            for key, info in self._moving.items():
                off = self._offset[info["kind"]]
                if off is None:                # calibrated: gait passes through p0 at mid-stance
                    center = self._p0[key]
                else:                          # absolute: q = sign·θ + offset, about the ref
                    center = off + info["sign"] * self._ref[info["kind"]]
                msg = self._limit_violation(key, center)
                if msg is not None:
                    print(msg, file=sys.stderr)
                    raise KeyboardInterrupt    # never moved: clean stop, nothing commanded
                self._center[key] = center
            self._print_plan()
            self._t_mit = t_ns
            self._mit_begun = True

        emit = (t_ns - self._t_mit) / 1e9
        for k in self._all:
            if k in self._moving:              # walking joint: blended IK gait + vel feedforward
                q0 = self._cmd(k, emit - VEL_DT)
                q1 = self._cmd(k, emit)
                q2 = self._cmd(k, emit + VEL_DT)
                cmds.append(MotorCommand(k[0], k[1], mode=MODE_MIT, pos=q1,
                                         vel=(q2 - q0) / (2.0 * VEL_DT),
                                         use_config_gains=True))
            else:                              # held joint: stiff at the captured pose
                cmds.append(MotorCommand(k[0], k[1], mode=MODE_MIT, pos=self._p0[k], vel=0.0,
                                         use_config_gains=True))
        return Action(motors=cmds)


if __name__ == "__main__":
    # Dry-run: print the gait geometry for the given parameters — no hardware.
    import argparse
    ap = argparse.ArgumentParser(prog="legs_walk (dry-run)",
                                 description="Print the legs_walk gait geometry and exit.")
    LegsWalkPolicy.add_args(ap)
    args = ap.parse_args()
    try:
        ref, rng = _sweep_gait(args.l1, args.l2, args.step, args.height, args.lift)
    except ValueError as e:
        raise SystemExit(f"refused: {e}")
    print(f"gait: l1={args.l1:g} l2={args.l2:g} step={args.step:g} "
          f"height={args.height:g} lift={args.lift:g} freq={args.freq:g} Hz")
    print(f"mid-stance ref : hip {ref['hip']:+.3f}  knee {ref['knee']:+.3f} rad")
    for kind in ("hip", "knee"):
        lo, hi = rng[kind]
        print(f"{kind:>5} over cycle: [{lo:+.3f}, {hi:+.3f}] rad  "
              f"(span {hi - lo:.3f}, dev around ref [{lo - ref[kind]:+.3f}, {hi - ref[kind]:+.3f}])")
    print("phase   foot(x, y)          hip     knee")
    for i in range(17):
        phi = math.pi * i / 8.0
        x, y = _foot(phi, args.step, args.height, args.lift)
        hip, knee = _ik(x, y, args.l1, args.l2)
        tag = "stance" if math.sin(phi) >= 0.0 else "swing"
        print(f"{phi:5.2f}  ({x:+7.2f},{y:+7.2f})  {hip:+6.3f}  {knee:+6.3f}  {tag}")