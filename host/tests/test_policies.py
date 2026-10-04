"""Policy + telemetry-driven runner tests.

Covers the task-5 loop: run_policy steps on cycle_id % N == 0 off scripted telemetry,
echoes the stepped cycle_id, counts coalesced skips / late-step catch-up / dropped steps,
passes MASTER time (not host arrival) into step(), refuses to start on a cycle-rate
mismatch, and exits cleanly on a telemetry stall. Plus: man_1s_1m derives its MIT phase
from the injected t_ns (no wall clock), and listen requests IDLE each tick.
"""
import io
import math
import os
import sys
import time
import unittest
from contextlib import redirect_stdout
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "host", "apps"))   # run_policy is a script, not a pkg

from policies.base import (  # noqa: E402
    Policy, Action, MODE_MIT, MODE_IDLE, MODE_HOLD, MotorCommand)
from policies.man_1s_1m_policy import Man1s1mPolicy, SINE_OMEGA, SINE_FREQ_HZ  # noqa: E402
from master_link import motor_config_gen as mc            # noqa: E402

POLL_HZ = mc.MASTER_POLL_HZ   # 200


class _Snap:
    def __init__(self, pos):
        self.pos = pos


class _FakeState:
    def __init__(self, robot=None, motors=None):
        self.robot = robot or {}
        self.motors = motors or {}

    def armed_motors(self):
        return []


class _FakeLink:
    """MasterLink stand-in. `frames` is a list of (cycle_id, adv) — adv is how far the
    frame counter jumps for that frame (1 = contiguous, >1 = coalesced ⇒ skips). Master
    time is cycle_id-derived so step() time is predictable. When frames run out, wait_robot
    returns None (a telemetry stall)."""
    def __init__(self, frames=(), poll_hz=POLL_HZ, have_master=True):
        self.port = "fake"
        self.log_path = os.devnull
        self.stats = dict(rx_frames=0, tx_frames=0, tx_errors=0, discard_records=0,
                          discard_bytes=0, version_frames=0, log_dropped=0)
        self._frames = list(frames)
        self._poll_hz = poll_hz
        self._have_master = have_master
        self._i = -1
        self._fno = 0
        self.sent = []          # list of (echo_cycle, [modes])

    # startup / verification
    def wait_until_live(self, keys=None, timeout=2.0):
        return True, set()

    def wait_master_status(self, timeout=1.0):
        return {"master_poll_hz": self._poll_hz} if self._have_master else None

    def robot_frame_no(self):
        return self._fno

    # telemetry
    def _robot(self, cid):
        return dict(cycle_id=cid, master_time_us_mono=cid * 1000,
                    recv_ns=time.monotonic_ns())

    def latest_state(self):
        if self._i < 0:
            return _FakeState({})
        return _FakeState(self._robot(self._frames[self._i][0]))

    def wait_robot(self, after_frame, timeout):
        self._i += 1
        if self._i >= len(self._frames):
            return None                      # stall
        cid, adv = self._frames[self._i]
        self._fno += adv
        skipped = max(0, self._fno - after_frame - 1)
        return (_FakeState(self._robot(cid)), self._fno, skipped, 1000)

    def send_robot_cmd(self, cmds, echo_cycle=None):
        self.sent.append((echo_cycle, [c.mode for c in cmds]))

    def log_event(self, text): pass
    def log_loop_timing(self, *a): pass
    def close(self): pass


class _StepPolicy(Policy):
    """Records (t_ns, cycle_id) per step and sends one IDLE command (so echo is exercised)."""
    name = "stepcap"
    steps = []

    def __init__(self):
        _StepPolicy.steps = []

    def step(self, state, t_ns):
        _StepPolicy.steps.append((t_ns, state.robot["cycle_id"]))
        return Action(motors=[MotorCommand(0, 0, mode=MODE_IDLE)])


def _run(frames, rate, poll_hz=POLL_HZ, have_master=True):
    """Run the runner against a scripted link; return (link, stdout, exit_code|None)."""
    import run_policy
    link = _FakeLink(frames, poll_hz=poll_hz, have_master=have_master)
    out = io.StringIO()
    code = None
    with mock.patch.object(run_policy, "MasterLink", lambda *a, **k: link), \
         mock.patch.dict(run_policy.POLICIES, {"stepcap": _StepPolicy}):
        try:
            with redirect_stdout(out):
                run_policy.main(["--policy", "stepcap", "--rate", str(rate)])
        except SystemExit as e:
            code = e.code
    return link, out.getvalue(), code


class TelemetryDrivenStepping(unittest.TestCase):
    def test_n1_steps_every_cycle_and_echoes_cid(self):
        frames = [(c, 1) for c in (10, 11, 12, 13)]
        link, out, code = _run(frames, rate=200)             # N=1
        echoes = [e for (e, _m) in link.sent]
        self.assertEqual(echoes, [10, 11, 12, 13])           # every cycle, echo = cid
        self.assertEqual(code, 1)                            # ended on the stall

    def test_n4_steps_only_on_aligned_cycles(self):
        frames = [(c, 1) for c in range(16, 25)]             # 16..24
        link, out, code = _run(frames, rate=50)              # N=4
        echoes = [e for (e, _m) in link.sent]
        self.assertEqual(echoes, [16, 20, 24])               # only cycle_id % 4 == 0

    def test_master_time_passed_to_step(self):
        frames = [(c, 1) for c in (20, 24, 28)]
        _run(frames, rate=50)
        # step t_ns must be master_time (cid*1000 us → *1000 ns), not host monotonic.
        for (t_ns, cid) in _StepPolicy.steps:
            self.assertEqual(t_ns, cid * 1000 * 1000)

    def test_late_step_catch_up_preserves_alignment(self):
        # Step 16, then coalesce past the aligned 20 (jump to 22), then 24 arrives.
        frames = [(16, 1), (22, 2), (24, 1)]
        link, out, code = _run(frames, rate=50)              # N=4
        echoes = [e for (e, _m) in link.sent]
        self.assertEqual(echoes, [16, 22, 24])               # late step on 22, realigns on 24
        self.assertIn("1 late steps", out)
        self.assertIn("0 dropped steps", out)

    def test_dropped_step_when_whole_period_missed(self):
        # After 16, jump clear past period 20 into period 24 (cycle 25) → period 20 dropped.
        frames = [(16, 1), (25, 9)]
        link, out, code = _run(frames, rate=50)              # N=4
        echoes = [e for (e, _m) in link.sent]
        self.assertEqual(echoes, [16, 25])                   # 25 is period 24 (late), 20 dropped
        self.assertIn("1 dropped steps", out)

    def test_skips_counted(self):
        frames = [(10, 1), (13, 3)]                          # 2 frames coalesced before 13
        link, out, code = _run(frames, rate=200)             # N=1
        self.assertIn("2 coalesced frames", out)


class RunPolicyRefusals(unittest.TestCase):
    def test_rate_mismatch_refuses(self):
        link, out, code = _run([(0, 1)], rate=200, poll_hz=100)   # master 100 ≠ config 200
        self.assertIsInstance(code, str)
        self.assertIn("mismatch", code)
        self.assertIn(str(POLL_HZ), code)                    # names the config value
        self.assertIn("100", code)                           # and the live value
        self.assertEqual(_StepPolicy.steps, [])              # never stepped

    def test_no_master_status_refuses(self):
        link, out, code = _run([(0, 1)], rate=200, have_master=False)
        self.assertIsInstance(code, str)
        self.assertIn("MasterStatus", code)

    def test_stall_exits_clean(self):
        link, out, code = _run([(0, 1), (1, 1)], rate=200)   # then frames run out → stall
        self.assertEqual(code, 1)
        self.assertIn("stall", out.lower() + " ")            # summary notes the stop


class _NeverLiveLink(_FakeLink):
    def wait_until_live(self, keys=None, timeout=2.0):
        return False, {(0, 0)}


class RunPolicyLiveGate(unittest.TestCase):
    def test_timeout_exits_without_running_policy(self):
        import run_policy
        with mock.patch.object(run_policy, "MasterLink", lambda *a, **k: _NeverLiveLink()), \
             mock.patch.dict(run_policy.POLICIES, {"stepcap": _StepPolicy}):
            with self.assertRaises(SystemExit) as cm:
                run_policy.main(["--policy", "stepcap", "--rate", "50"])
        self.assertIn("s0.m0", str(cm.exception))
        self.assertEqual(_StepPolicy.steps, [])


class ManMitUsesInjectedTime(unittest.TestCase):
    def test_mit_starts_from_current_pos_on_master_time(self):
        p = Man1s1mPolicy()
        s, l, lo, hi, amp = p._motors[0]
        k = (s, l)
        st = _FakeState(motors={k: _Snap(0.2)})     # current position 0.2
        with p._lock:
            p._mode = MODE_MIT
            p._mit_restart = True
        t0 = 1_000_000_000_000

        a0 = p.step(st, t0)                          # MIT start → move begins at p0
        self.assertEqual(a0.motors[0].mode, MODE_MIT)
        self.assertAlmostEqual(a0.motors[0].pos, 0.2, places=5)   # first cmd == current pos
        self.assertAlmostEqual(a0.motors[0].vel, 0.0, places=5)   # zero velocity at start

        # Pure function of t_ns: the same time replays the same command (restart consumed).
        a0b = p.step(st, t0)
        self.assertAlmostEqual(a0b.motors[0].pos, a0.motors[0].pos, places=6)
        self.assertAlmostEqual(a0b.motors[0].vel, a0.motors[0].vel, places=6)

    def test_listen_requests_idle(self):
        from policies.listen_policy import ListenPolicy
        act = ListenPolicy().step(_FakeState(), 123)
        self.assertTrue(act.motors)
        self.assertTrue(all(c.mode == MODE_IDLE for c in act.motors))


class SineStartTraj(unittest.TestCase):
    def _mk(self, amp=0.2, freq=0.4, vmax=0.5):
        from policies.sine_start import SineStart
        return SineStart(amp, 2 * math.pi * freq, vmax)

    def test_first_sample_is_p0_zero_vel(self):
        ss = self._mk(); ss.begin(0.6, 0)
        pos, vel = ss.sample(0)
        self.assertAlmostEqual(pos, 0.6, places=6)       # first command == p0
        self.assertAlmostEqual(vel, 0.0, places=6)

    def test_continuous_at_move_sine_join(self):
        ss = self._mk(); ss.begin(0.6, 0)
        T = ss.T
        self.assertGreater(T, 0.0)
        p_lo, v_lo = ss.sample(int((T - 1e-4) * 1e9))
        p_hi, v_hi = ss.sample(int(T * 1e9))
        self.assertAlmostEqual(p_lo, p_hi, places=3)     # position continuous
        self.assertAlmostEqual(v_lo, v_hi, places=3)     # velocity continuous
        self.assertAlmostEqual(p_hi, ss.peak, places=4)  # join is the peak...
        self.assertAlmostEqual(v_hi, 0.0, places=4)      # ...with zero velocity

    def test_sine_centered_on_zero(self):
        ss = self._mk(amp=0.2, freq=0.4); ss.begin(0.6, 0)
        T = ss.T; half = 0.5 / 0.4                        # half sine period (s)
        p_peak, _ = ss.sample(int(T * 1e9))
        p_opp, _  = ss.sample(int((T + half) * 1e9))
        self.assertAlmostEqual(p_peak, 0.2, places=4)     # +A
        self.assertAlmostEqual(p_opp, -0.2, places=4)     # -A → center (p_peak+p_opp)/2 == 0
        self.assertAlmostEqual(0.5 * (p_peak + p_opp), 0.0, places=5)

    def test_nearest_peak_sign(self):
        ss = self._mk(); ss.begin(-0.6, 0)                # p0 < 0 → nearest peak is -A
        self.assertLess(ss.peak, 0.0)
        pos, _ = ss.sample(0)
        self.assertAlmostEqual(pos, -0.6, places=6)

    def test_refuses_over_soft_limits(self):
        ss = self._mk(amp=0.9)
        with self.assertRaises(ValueError):
            ss.begin(0.0, 0, lo=-0.79, hi=0.79, who="s0.m0")


class BenchSineBehavior(unittest.TestCase):
    def test_arm_then_sine_starts_from_p0(self):
        from policies.bench_sine import BenchSinePolicy
        from master_link.motor_config_gen import MOTORS
        k = (MOTORS[0]["slave"], MOTORS[0]["idx"])
        p = BenchSinePolicy(amp=0.1, freq=0.4, motors="all", arm_s=1.0, move_speed=0.5)
        st = _FakeState(motors={k: _Snap(0.3)})
        p.setup(st, 0)
        a = p.step(st, int(0.1e9))                        # arm → HOLD + fault_reset
        self.assertEqual(a.motors[0].mode, MODE_HOLD)
        self.assertTrue(a.motors[0].fault_reset)
        a = p.step(st, int(1.0e9))                        # MIT start → first cmd == p0
        self.assertEqual(a.motors[0].mode, MODE_MIT)
        self.assertAlmostEqual(a.motors[0].pos, 0.3, places=5)

    def test_refuses_amp_over_soft_limits(self):
        from policies.bench_sine import BenchSinePolicy
        p = BenchSinePolicy(amp=10.0, motors="all", arm_s=1.0)   # absurd amplitude
        with self.assertRaises(ValueError):
            p.setup(_FakeState(), 0)


if __name__ == "__main__":
    unittest.main()
