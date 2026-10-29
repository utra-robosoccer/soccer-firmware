"""man_1s_1m — MANual, keyboard-driven bench policy (all motors together).

Level-triggered (protocol v9): every tick it sends the CURRENT drive request +
tuple for all configured motors (one cmd_robot_t). Under v9 the host decides the
behavior: "hold", "sine" and "damp" are all ENABLE + a streamed MIT tuple.

Keys (all motors together):
    a   HOLD   → ENABLE + hold the position captured at the keypress
    s   SINE   → ENABLE + stream an in-range sine
    x   DAMP   → ENABLE + Kp=0 / damping Kd tuple (backdrive by hand)
    d   OFF    → DISABLE
    f   RESET  → clear a latched fault (sent once)
    q   QUIT   → clean shutdown
    ?   help
    (z TO_ZERO was firmware-side; it returns as a host behavior in step 5)

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
    REQ_DISABLE, REQ_ENABLE, REQ_RESET,
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

_MODE_OF = {"a": "hold", "s": "sine", "x": "damp", "d": "off"}
_MODE_NAME = {"off": "OFF (DISABLE)", "hold": "HOLD (enable + hold tuple)",
              "sine": "SINE (enable + MIT sine)", "damp": "DAMP (enable + Kd tuple)"}

_HELP = (
    "  a HOLD   s SINE   x DAMP   d OFF\n"
    "  f RESET (fault)   q QUIT   ? help\n"
)


class Man1s1mPolicy(Policy):
    name = "man_1s_1m"

    def __init__(self):
        self._motors = []
        for g, m in enumerate(MOTORS):
            lo, hi = MOTOR_SOFT_MIN[g], MOTOR_SOFT_MAX[g]
            amp = max(0.0, min(hi, -lo) - SINE_MARGIN_RAD)   # largest 0-centered amplitude
            self._motors.append((m["slave"], m["idx"], lo, hi, amp))
        self._traj = {}     # (slave, local) -> SineStart, planned on sine entry
        self._hold = {}     # (slave, local) -> position captured once at hold entry
        self._lock = threading.Lock()
        self._mode = "off"
        self._reset_pending = False
        self._mode_entered = False

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
    def _set_mode(self, mode: str) -> None:
        with self._lock:
            self._mode = mode
            self._mode_entered = True
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
                    self._reset_pending = True
                print("→ RESET (once)", flush=True)
            elif ch == "z":
                print("→ TO_ZERO is a host behavior now (step 5); not available here",
                      flush=True)
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
            entered = self._mode_entered
            self._mode_entered = False
            reset = self._reset_pending
            self._reset_pending = False

        if entered and mode == "hold":
            # Capture the hold targets ONCE at entry — re-sampling each tick would
            # let a loaded joint sag (the target would follow measured position).
            self._hold = {}
            for (s, l, lo, hi, amp) in self._motors:
                snap = state.motors.get((s, l)) if state and state.motors else None
                self._hold[(s, l)] = snap.pos if snap else 0.0

        if entered and mode == "sine":
            # Plan each motor's smooth start from its CURRENT position; refuse (revert
            # to hold) if a 0-centered ±A would leave the soft limits — never clamp.
            trajs = {}
            for (s, l, lo, hi, amp) in self._motors:
                snap = state.motors.get((s, l)) if state and state.motors else None
                p0 = snap.pos if snap else 0.0
                tj = SineStart(amp, SINE_OMEGA, MOVE_MAX_SPEED)
                try:
                    tj.begin(p0, t_ns, lo, hi, who=f"s{s}.m{l}")
                except ValueError as e:
                    print(f"→ SINE refused: {e}", flush=True)
                    with self._lock:
                        self._mode = "hold"
                        self._mode_entered = True   # capture holds next tick
                    mode = "off"                    # this tick: keep motors off
                    trajs = {}
                    break
                trajs[(s, l)] = tj
            self._traj = trajs

        cmds = []
        for (s, l, lo, hi, amp) in self._motors:
            if reset:
                # One RESET tick: clears a latched fault; no-op on healthy motors.
                cmds.append(MotorCommand(s, l, req=REQ_RESET))
                continue
            if mode == "off":
                cmds.append(MotorCommand(s, l, req=REQ_DISABLE))
            elif mode == "hold":
                cmds.append(MotorCommand(s, l, req=REQ_ENABLE, valid=True,
                                         pos=self._hold.get((s, l), 0.0),
                                         use_config_gains=True))
            elif mode == "damp":
                kd = next((m.get("default_kd", 1.0) for m in MOTORS
                           if (m["slave"], m["idx"]) == (s, l)), 1.0)
                cmds.append(MotorCommand(s, l, req=REQ_ENABLE, valid=True,
                                         pos=0.0, vel=0.0, kp=0.0, kd=kd,
                                         use_config_gains=False))
            else:  # sine
                pos = vel = 0.0
                tj = self._traj.get((s, l))
                if tj is not None:
                    pos, vel = tj.sample(t_ns)
                cmds.append(MotorCommand(s, l, req=REQ_ENABLE, valid=True,
                                         pos=pos, vel=vel, use_config_gains=True))
        return Action(motors=cmds)
