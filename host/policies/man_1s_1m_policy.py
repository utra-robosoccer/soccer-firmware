"""man_1s_1m — MANual, 1 Slave, 1 Motor.

A keyboard-driven policy for bench bring-up. Same key bindings as test_client /
dashboard, but with **no per-motor selection**: every command applies to ALL
configured motors at once. There is no live view — run_policy is already logging
the session, so plot the .bin afterwards.

Keys (all motors together):
    a   ARM_HOLD    → ARMED_HOLD (firmware holds position)
    z   GOTO_ZERO   → crawl to zero, then hold
    s   ARM_MIT     → stream a sine inside the soft limits (arm with `a` first)
    d   IDLE        → DISABLE (motor goes to IDLE)
    q   QUIT        → clean shutdown (run_policy disables + flushes the log)
    ?   help

The sine is centred on each motor's soft-limit midpoint and peaks SINE_MARGIN_DEG
INSIDE the limits, so it never engages the slave's clamp — the same in-range sweep
the dashboard used. (Commanding past the limits instead drives the joint into the
clamp; on turn-around the preserved return-velocity feedforward spikes the kd
torque term and trips CAUSE_OVERTORQUE — which is why a full ±90° command tripped
while this in-range sweep does not.) Control frames (a/z/d) are edge-triggered —
sent once per keypress; the MIT sine streams every tick while in ARM_MIT. Send rate
is run_policy's --rate (bump it, e.g. --rate 200, for a smoother sine).
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
    MOTORS, MOTOR_DEFAULT_KP, MOTOR_DEFAULT_KD, MOTOR_SOFT_MIN, MOTOR_SOFT_MAX,
)

from .base import Policy, Action, MitCommand, ControlRequest, ControlKind, LinkState

# ── sine parameters (stays inside the soft limits — never hits the clamp) ────────
SINE_FREQ_HZ    = 0.4                       # matches the dashboard/test_client sweep
SINE_OMEGA      = 2.0 * math.pi * SINE_FREQ_HZ
SINE_MARGIN_DEG = 5.0                       # keep the peaks this far inside each limit
SINE_MARGIN_RAD = math.radians(SINE_MARGIN_DEG)

# ── modes ───────────────────────────────────────────────────────────────────
_NONE, _IDLE, _ZERO, _HOLD, _MIT = range(5)
_CTRL_OF = {_IDLE: ControlKind.DISABLE, _ZERO: ControlKind.GOTO_ZERO, _HOLD: ControlKind.ARM}
_MODE_NAME = {_NONE: "—", _IDLE: "IDLE", _ZERO: "GOTO_ZERO", _HOLD: "ARM_HOLD", _MIT: "ARM_MIT"}

_HELP = (
    "  a ARM_HOLD (all)   z GOTO_ZERO (all)   s ARM_MIT/sine in-range (all)\n"
    "  d IDLE (all)       q QUIT             ? help\n"
)


class Man1s1mPolicy(Policy):
    name = "man_1s_1m"

    def __init__(self):
        # Every configured motor, addressed together (no selection). Each carries
        # its own sine center/amplitude derived from its soft limits, so the sweep
        # stays SINE_MARGIN_DEG inside the clamp on every motor.
        self._motors = []
        for g, m in enumerate(MOTORS):
            lo, hi = MOTOR_SOFT_MIN[g], MOTOR_SOFT_MAX[g]
            center = 0.5 * (lo + hi)
            amp = max(0.0, 0.5 * (hi - lo) - SINE_MARGIN_RAD)
            self._motors.append((m["slave"], m["idx"], MOTOR_DEFAULT_KP[g],
                                 MOTOR_DEFAULT_KD[g], center, amp))
        self._lock = threading.Lock()
        self._mode = _NONE
        self._ctrl_pending = False   # edge-triggered: emit one control frame this tick
        self._mit_t0 = 0             # ns; captured from the runner's t_ns on MIT entry
        self._mit_restart = False    # set on MIT entry; step() re-seeds _mit_t0 from t_ns

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
        tty.setcbreak(self._fd)   # leaves ISIG on, so Ctrl-C still reaches run_policy
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
            if mode == _MIT:
                self._mit_restart = True    # step() seeds _mit_t0 from the injected t_ns
            else:
                self._ctrl_pending = True   # control modes send one frame on entry
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
            if ch == "a":
                self._set_mode(_HOLD)
            elif ch == "z":
                self._set_mode(_ZERO)
            elif ch == "s":
                self._set_mode(_MIT)
            elif ch == "d":
                self._set_mode(_IDLE)
            elif ch in ("q", "\x03", "\x04"):
                print("→ QUIT", flush=True)
                # Trigger run_policy's clean KeyboardInterrupt shutdown path.
                os.kill(os.getpid(), signal.SIGINT)
                break
            elif ch == "?":
                print(_HELP, flush=True)

    # ── per-tick action ───────────────────────────────────────────────────────
    def step(self, state: LinkState, t_ns: int) -> Action:
        with self._lock:
            mode = self._mode
            if mode in _CTRL_OF and self._ctrl_pending:
                self._ctrl_pending = False
                kind = _CTRL_OF[mode]
                return Action(control=[ControlRequest(s, l, kind)
                                       for (s, l, _kp, _kd, _c, _a) in self._motors])
            if mode == _MIT and self._mit_restart:   # seed phase origin from injected time
                self._mit_t0 = t_ns
                self._mit_restart = False
            t0 = self._mit_t0

        if mode == _MIT:
            t = (t_ns - t0) / 1e9
            s_wt = math.sin(SINE_OMEGA * t)
            c_wt = math.cos(SINE_OMEGA * t)
            return Action(mit=[MitCommand(s, l, pos=center + amp * s_wt,
                                          vel=amp * SINE_OMEGA * c_wt, kp=kp, kd=kd)
                               for (s, l, kp, kd, center, amp) in self._motors])
        return Action()
