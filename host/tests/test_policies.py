"""Policy time-interface tests:
  1. run_policy passes the runner's monotonic-ns time into step().
  2. man_1s_1m derives its MIT sine phase from the injected t_ns, not a wall clock.
"""
import math
import os
import sys
import time
import unittest
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "host", "apps"))   # run_policy is a script, not a pkg

from policies.base import Policy, Action, MODE_MIT             # noqa: E402
from policies.man_1s_1m_policy import Man1s1mPolicy, SINE_OMEGA, SINE_FREQ_HZ  # noqa: E402


class _FakeState:
    def armed_motors(self):
        return []


class _FakeLink:
    """Minimal stand-in for MasterLink so run_policy.main() runs with no hardware."""
    def __init__(self, *a, **k):
        self.port = "fake"
        self.log_path = os.devnull
        self.stats = dict(rx_frames=0, tx_frames=0, tx_errors=0, discard_records=0,
                          discard_bytes=0, version_frames=0, log_dropped=0)

    def latest_state(self):
        return _FakeState()

    def wait_until_live(self, keys=None, timeout=2.0):
        return True, set()

    def send_robot_cmd(self, cmds): pass
    def log_event(self, text): pass
    def log_annotation(self, text): pass
    def log_loop_timing(self, *a): pass
    def close(self): pass


class _CapturePolicy(Policy):
    name = "capture"
    setup_t = None
    steps = []

    def __init__(self):
        _CapturePolicy.setup_t = None
        _CapturePolicy.steps = []

    def setup(self, state, t_ns):
        _CapturePolicy.setup_t = t_ns

    def step(self, state, t_ns):
        _CapturePolicy.steps.append(t_ns)
        if len(_CapturePolicy.steps) >= 4:
            raise KeyboardInterrupt   # end the runner loop cleanly
        return Action()


class RunnerPassesMonotonicTime(unittest.TestCase):
    def test_runner_injects_monotonic_ns(self):
        import run_policy
        with mock.patch.object(run_policy, "MasterLink", _FakeLink), \
             mock.patch.dict(run_policy.POLICIES, {"capture": _CapturePolicy}):
            before = time.monotonic_ns()
            run_policy.main(["--policy", "capture", "--rate", "1000"])
            after = time.monotonic_ns()

        steps = _CapturePolicy.steps
        self.assertGreaterEqual(len(steps), 4)
        self.assertIsNotNone(_CapturePolicy.setup_t)
        for t in steps:
            self.assertIsInstance(t, int)
            self.assertTrue(before <= t <= after)          # a real monotonic-ns reading
        self.assertEqual(steps, sorted(steps))             # non-decreasing (monotonic)
        self.assertTrue(before <= _CapturePolicy.setup_t <= steps[0])


class _NeverLiveLink(_FakeLink):
    def wait_until_live(self, keys=None, timeout=2.0):
        return False, {(0, 0)}


class RunPolicyLiveGate(unittest.TestCase):
    def test_timeout_exits_without_running_policy(self):
        import run_policy
        with mock.patch.object(run_policy, "MasterLink", _NeverLiveLink), \
             mock.patch.dict(run_policy.POLICIES, {"capture": _CapturePolicy}):
            with self.assertRaises(SystemExit) as cm:
                run_policy.main(["--policy", "capture", "--rate", "1000"])
        self.assertIn("s0.m0", str(cm.exception))          # names the dead motor
        self.assertEqual(_CapturePolicy.steps, [])         # policy never ran
        self.assertIsNone(_CapturePolicy.setup_t)          # setup never called


class ManMitUsesInjectedTime(unittest.TestCase):
    def test_mit_phase_follows_t_ns(self):
        p = Man1s1mPolicy()
        s, l, center, amp = p._motors[0]
        # simulate an 's' (MIT) keypress without a keyboard/clock
        with p._lock:
            p._mode = MODE_MIT
            p._mit_restart = True

        # Use times far from the real clock to prove no clock is read.
        t0 = 1_000_000_000_000            # arbitrary ns origin
        quarter_ns = int((0.25 / SINE_FREQ_HZ) * 1e9)   # T/4

        a0 = p.step(None, t0)             # first MIT tick → seeds phase origin, sin(0)=0
        self.assertTrue(a0.motors)
        self.assertEqual(a0.motors[0].mode, MODE_MIT)
        self.assertAlmostEqual(a0.motors[0].pos, center, places=6)
        self.assertAlmostEqual(a0.motors[0].vel, amp * SINE_OMEGA, places=6)  # cos(0)=1

        a1 = p.step(None, t0 + quarter_ns)               # quarter period → +peak
        self.assertAlmostEqual(a1.motors[0].pos, center + amp, places=4)
        self.assertAlmostEqual(a1.motors[0].vel, 0.0, places=4)

        a2 = p.step(None, t0)            # back to origin time → back to center (pure fn of t_ns)
        self.assertAlmostEqual(a2.motors[0].pos, center, places=6)

    def test_step_accepts_t_ns_signature(self):
        # listen now requests IDLE for every motor each tick (not empty).
        from policies.listen_policy import ListenPolicy
        from master_link.link import MODE_IDLE
        act = ListenPolicy().step(_FakeState(), 123)
        self.assertTrue(act.motors)
        self.assertTrue(all(c.mode == MODE_IDLE for c in act.motors))


if __name__ == "__main__":
    unittest.main()
