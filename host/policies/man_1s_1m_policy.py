"""man_1s_1m — MANual, keyboard-driven bench policy (all motors together).

Level-triggered: every tick it sends the CURRENT mode request for all configured
motors (one cmd_robot_t). Same key feel as before, now over the mode state machine.

Keys (all motors together):
    a   HOLD       → arm + hold current position (the only arm-from-IDLE path)
    z   TO_ZERO    → creep to home-frame 0 and hold (arm with `a` first)
    s   MIT        → stream an in-range sine (arm with `a` first)
    x   DAMPED     → Kp=0 + damping Kd (backdrive by hand; arm first)
    d   IDLE       → disarm
    f   fault-reset→ clear a latched fault (sent once, with the current mode)
    q   QUIT       → clean shutdown
    ?   help

The sine is centred on 0 with amplitude A = min(|lo|, hi) - SINE_MARGIN, so it stays
inside the soft limits (never engages the slave's clamp). On 's' it starts from the
motor's current position with a minimum-jerk move to the nearest peak (±A), then the
sine from that peak's phase — position and velocity continuous from HOLD. Send rate is
run_policy's --rate.
"""
import math
import os
import select
import signal
import sys
import termios
import threading
import tty

from master_link.motor_config_gen import (
    MOTORS, MOTOR_SOFT_MIN, MOTOR_SOFT_MAX,
)

from .base import (
    Policy, Action, MotorCommand, LinkState,
    MODE_IDLE, MODE_HOLD, MODE_MIT, MODE_DAMPED, MODE_TO_ZERO,
)
from .sine_start import SineStart

# ── sine parameters: centered on 0, inside the soft limits (never hits the clamp). The
# sine STARTS from the motor's current position via a minimum-jerk move to the nearest
# peak, so position + velocity are continuous from HOLD into the sine. ───────────────────
SINE_FREQ_HZ    = 0.4
SINE_OMEGA      = 2.0 * math.pi * SINE_FREQ_HZ
SINE_MARGIN_DEG = 5.0
SINE_MARGIN_RAD = math.radians(SINE_MARGIN_DEG)
MOVE_MAX_SPEED  = 0.5   # rad/s, min-jerk peak speed cap for the HOLD→peak move

_MODE_OF = {
    "a": MODE_HOLD, "z": MODE_TO_ZERO, "s": MODE_MIT, "x": MODE_DAMPED, "d": MODE_IDLE,
}
_MODE_NAME = {MODE_IDLE: "IDLE", MODE_HOLD: "HOLD", MODE_MIT: "MIT",
              MODE_DAMPED: "DAMPED", MODE_TO_ZERO: "TO_ZERO"}

_HELP = (
    "  a HOLD   z TO_ZERO   s MIT/sine   x DAMPED   d IDLE\n"
    "  f fault-reset        q QUIT       ? help\n"
)


class Man1s1mPolicy(Policy):
    name = "man_1s_1m"

    def __init__(self):
        self._motors = []
        for g, m in enumerate(MOTORS):
            lo, hi = MOTOR_SOFT_MIN[g], MOTOR_SOFT_MAX[g]
            amp = max(0.0, min(hi, -lo) - SINE_MARGIN_RAD)   # largest 0-centered amplitude
            self._motors.append((m["slave"], m["idx"], lo, hi, amp))
        self._traj = {}     # (slave, local) -> SineStart, planned on MIT entry
        self._lock = threading.Lock()
        self._mode = MODE_IDLE
        self._fault_reset_pending = False
        self._mit_restart = False

        self._stop = threading.Event()
        self._kb = None
        self._fd = None
        self._old_term = None

    # ── lifecycle ───────────────────────────────────────────────────────────
    def setup(self, state: LinkState, t_ns: int) -> None:
        if not sys.stdin.isatty():
            sys.stderr.write("man_1s_1m: stdin is not a TTY — no keyboard control "
                             "(the loop will just idle-log). Run it in a terminal.\n")
            return
        self._fd = sys.stdin.fileno()
        self._old_term = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        print("\n── man_1s_1m (all motors together) ──")
        print(_HELP, flush=True)
        self._kb = threading.Thread(target=self._kb_loop, name="man-kbd", daemon=True)
        self._kb.start()

    def teardown(self) -> None:
        self._stop.set()
        if self._kb is not None:
            self._kb.join(timeout=1.0)
        if self._old_term is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_term)
            self._old_term = None

    # ── keyboard thread ───────────────────────────────────────────────────────
    def _set_mode(self, mode: int) -> None:
        with self._lock:
            self._mode = mode
            if mode == MODE_MIT:
                self._mit_restart = True
        print(f"→ {_MODE_NAME[mode]} (all)", flush=True)

    def _kb_loop(self) -> None:
        while not self._stop.is_set():
            try:
                r, _, _ = select.select([self._fd], [], [], 0.2)
            except (OSError, ValueError):
                break
            if not r:
                continue
            try:
                ch = os.read(self._fd, 1).decode(errors="ignore")
            except OSError:
                break
            if not ch:
                continue
            if ch in _MODE_OF:
                self._set_mode(_MODE_OF[ch])
            elif ch == "f":
                with self._lock:
                    self._fault_reset_pending = True
                print("→ fault-reset (once)", flush=True)
            elif ch in ("q", "\x03", "\x04"):
                print("→ QUIT", flush=True)
                os.kill(os.getpid(), signal.SIGINT)
                break
            elif ch == "?":
                print(_HELP, flush=True)

    # ── per-tick action ───────────────────────────────────────────────────────
    def step(self, state: LinkState, t_ns: int) -> Action:
        with self._lock:
            mode = self._mode
            restart = self._mit_restart
            self._mit_restart = False
            fault_reset = self._fault_reset_pending
            self._fault_reset_pending = False

        if mode == MODE_MIT and restart:
            # Plan each motor's smooth start from its CURRENT position; refuse (revert to
            # HOLD) if a 0-centered ±A would leave the soft limits — never clamp silently.
            trajs = {}
            for (s, l, lo, hi, amp) in self._motors:
                snap = state.motors.get((s, l)) if state and state.motors else None
                p0 = snap.pos if snap else 0.0
                tj = SineStart(amp, SINE_OMEGA, MOVE_MAX_SPEED)
                try:
                    tj.begin(p0, t_ns, lo, hi, who=f"s{s}.m{l}")
                except ValueError as e:
                    print(f"→ MIT refused: {e}", flush=True)
                    with self._lock:
                        self._mode = MODE_HOLD
                    mode = MODE_HOLD
                    trajs = {}
                    break
                trajs[(s, l)] = tj
            self._traj = trajs

        cmds = []
        for (s, l, lo, hi, amp) in self._motors:
            pos = vel = 0.0
            if mode == MODE_MIT:
                tj = self._traj.get((s, l))
                if tj is not None:
                    pos, vel = tj.sample(t_ns)
            cmds.append(MotorCommand(s, l, mode=mode, pos=pos, vel=vel,
                                     use_config_gains=True, fault_reset=fault_reset))
        return Action(motors=cmds)
