"""MasterLink tests against a fake serial port (no hardware).

A loopback FakeSerial feeds recorded RX frames — a valid USB_MSG_TELE and a
corrupted run (CRC fail → resync/discard) — and captures TX writes. Asserts:
state store updates (motors/slaves/status/robot), discards are logged,
send_robot_cmd emits a correct, exact-size, decodable USB_MSG_CMD logged as
TX_FRAME (with reset_slaves honoured), and the live detector behaves on a
stale burst vs the paced live stream.
"""
import os
import tempfile
import threading
import time
import unittest
from unittest import mock

import master_link.protocol as P
from master_link import motor_config_gen as mc
from master_link.datalog import BinaryLogReader
from master_link.datalog import format as LOG


class FakeSerial:
    def __init__(self, *a, **k):
        self._rx = bytearray()
        self._lock = threading.Lock()
        self.tx = bytearray()
        self.closed = False

    def feed(self, data: bytes):
        with self._lock:
            self._rx.extend(data)

    @property
    def in_waiting(self):
        with self._lock:
            return len(self._rx)

    def read(self, n):
        with self._lock:
            out = bytes(self._rx[:n])
            del self._rx[:n]
            return out

    def reset_input_buffer(self):
        with self._lock:
            self._rx.clear()

    def write(self, data):
        self.tx.extend(data)
        return len(data)

    def flush(self):
        pass

    def close(self):
        self.closed = True


def _tele_payload(state=P.MOTOR_ENABLED, last_applied_seq=0, pos_raw=5000,
                  time_master_us=0, cycle_id=1):
    """One-slave, one-motor tele_master_t payload (motor at (0,0), slave alive)."""
    tm = P.TeleMaster()
    tm.cycle_id = cycle_id
    tm.time_master_us = time_master_us
    tm.robot_state = P.ROBOT_READY
    tm.n_slaves = 1
    tm.status.usb.rx_frames = 99
    tm.status.spi[0].alive = 1
    sl = tm.slaves[0]
    sl.state = P.SLAVE_READY
    sl.n_motors = 1
    sl.time_slave_us = 7000
    mo = sl.motors[0]
    mo.state = state
    mo.last_applied_seq = last_applied_seq
    mo.rs.pos = pos_raw
    mo.rs.mode = P.RS_MODE_RUN
    mo.rs.time_slave_us = 6500
    return P.pack_tele_master(tm)


def _tele_frame(**kw):
    return P.encode_frame(P.USB_MSG_TELE, _tele_payload(**kw))


def _wait(pred, timeout=2.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(0.005)
    return False


class MasterLinkFakeSerial(unittest.TestCase):
    def _make_link(self, d):
        from master_link import link as linkmod
        fake = FakeSerial()
        with mock.patch.object(linkmod.serial, "Serial", return_value=fake):
            lk = linkmod.MasterLink(port="fakeport", policy_name="test",
                                    log_dir=os.path.join(d, "logs"))
        return lk, fake

    def test_rx_state_and_discard(self):
        with tempfile.TemporaryDirectory() as d:
            lk, fake = self._make_link(d)
            try:
                corrupt = bytearray(_tele_frame(pos_raw=1234))
                corrupt[16] ^= 0xFF                       # break CRC → discarded on resync
                fake.feed(_tele_frame(state=P.MOTOR_ENABLED))   # valid → state store
                fake.feed(bytes(corrupt))                 # garbage → RX_DISCARD
                fake.feed(_tele_frame(state=P.MOTOR_ENABLED))   # resync lands on this one

                self.assertTrue(_wait(lambda: (0, 0) in lk.latest_state().motors),
                                "motor (0,0) never ingested")
                self.assertTrue(_wait(lambda: lk.latest_state().status is not None))
                self.assertTrue(_wait(lambda: lk.stats["discard_bytes"] > 0))

                st = lk.latest_state()
                snap = st.motors[(0, 0)]
                self.assertEqual(snap.state, P.MOTOR_ENABLED)
                self.assertEqual(snap.state_name, "ENABLED")
                self.assertEqual(snap.gidx, 0)
                self.assertEqual(snap.slave_time_us, 7000)
                self.assertEqual(snap.time_slave_us, 6500)
                self.assertEqual(snap.fb_age_us, 500)
                self.assertEqual(st.status["usb"]["rx_frames"], 99)
                self.assertEqual(st.slaves[0]["state"], P.SLAVE_READY)
                self.assertEqual(st.robot["robot_state"], P.ROBOT_READY)
            finally:
                lk.close()

            # Log contains RX_FRAMEs and at least one RX_DISCARD.
            kinds = [rec.kind for rec in BinaryLogReader(lk.log_path)]
            self.assertIn(LOG.RX_DISCARD, kinds, "no RX_DISCARD logged")
            self.assertIn(LOG.RX_FRAME, kinds)

    def test_tx_robot_cmd(self):
        from master_link.link import MotorCommand, REQ_ENABLE, REQ_STOP
        with tempfile.TemporaryDirectory() as d:
            lk, fake = self._make_link(d)
            try:
                lk.send_robot_cmd([
                    MotorCommand(slave=0, local=0, req=REQ_ENABLE),
                    MotorCommand(slave=0, local=1, req=REQ_ENABLE,
                                 pos=1.5, vel=-2.0),
                ], echo_cycle=7, reset_slaves={0})
                self.assertTrue(_wait(lambda: lk.stats["tx_frames"] == 1))
            finally:
                lk.close()

            buf = bytearray(fake.tx)
            frames = []
            while True:
                r = P.decode_frame(buf)
                if r is None:
                    break
                frames.append(r)
                del buf[:r[3]]

            self.assertEqual([f[0] for f in frames], [P.USB_MSG_CMD])
            payload = frames[0][2]
            # Exactly CMD_MASTER_SIZE(N_SLAVES) — the master rejects any other length.
            self.assertEqual(len(payload), P.cmd_master_size(mc.N_SLAVES))
            cm = P.parse_cmd_master(payload)
            self.assertEqual(cm.cmd_seq, 1)   # first tick stamps cmd_seq=1 (skips 0)
            self.assertEqual(cm.cycle_id, 7)
            self.assertEqual(cm.slaves[0].req, P.SLAVE_REQ_RESET)
            motors = cm.slaves[0].motors
            self.assertEqual(motors[0].req, REQ_ENABLE)
            self.assertEqual(motors[1].req, REQ_ENABLE)
            self.assertAlmostEqual(P.dec_i16(motors[1].mit.pos, P.POS_SCALE), 1.5, places=3)
            self.assertAlmostEqual(P.dec_i16(motors[1].mit.vel, P.VEL_SCALE), -2.0, places=2)
            # Uncommanded slots are STOP + zero mit.
            self.assertEqual(motors[2].req, REQ_STOP)
            self.assertEqual(bytes(motors[2].mit), b"\x00" * 10)
            tx = [rec for rec in BinaryLogReader(lk.log_path) if rec.kind == LOG.TX_FRAME]
            self.assertEqual(len(tx), 1)

    def test_wait_robot_coalesces(self):
        with tempfile.TemporaryDirectory() as d:
            lk, fake = self._make_link(d)
            try:
                fno = lk.robot_frame_no()
                for i in range(4):
                    fake.feed(_tele_frame(cycle_id=10 + i))
                got = None
                def _ready():
                    nonlocal got
                    got = lk.wait_robot(fno, timeout=0.05)
                    return got is not None and got[1] >= fno + 4
                self.assertTrue(_wait(_ready))
                state, new_fno, skipped, decode_ns = got
                self.assertEqual(new_fno - fno, 4)
                self.assertGreaterEqual(decode_ns, 0)
                self.assertEqual(state.robot["cycle_id"], 13)   # newest frame wins
                # Timeout path.
                self.assertIsNone(lk.wait_robot(new_fno, timeout=0.05))
            finally:
                lk.close()

    def test_live_detection_stale_then_live(self):
        with tempfile.TemporaryDirectory() as d:
            lk, fake = self._make_link(d)
            try:
                # Stale burst: master ts jumps 100 ms/frame but the whole blob drains
                # to the host at once (Δhost ≪ Δmaster) → never live.
                burst = b"".join(_tele_frame(time_master_us=(500_000 + i * 100) * 1000)
                                 for i in range(15))
                fake.feed(burst)
                time.sleep(0.1)
                self.assertEqual(lk.live_motors(), set(),
                                 "went live on the stale burst")
                # Live stream: master ts advances ~10 ms, fed ~8 ms apart in real time
                # (plus ~2 ms host decode ≈ Δmaster) → latches live.
                for i in range(10):
                    fake.feed(_tele_frame(time_master_us=(1_000_000 + i * 10) * 1000))
                    time.sleep(0.008)
                self.assertTrue(_wait(lambda: (0, 0) in lk.live_motors()),
                                "never went live on the live stream")
                self.assertIn((0, 0), lk.configured_motors())
            finally:
                lk.close()
            live_evs = [r for r in BinaryLogReader(lk.log_path)
                        if r.kind == LOG.EVENT and r.payload == b"live"]
            self.assertEqual(len(live_evs), 1, "EVENT 'live' not logged exactly once")


class LiveDetectorUnit(unittest.TestCase):
    def _det(self, **k):
        from master_link.link import _LiveDetector
        return _LiveDetector(**k)

    def test_stale_burst_never_live(self):
        d = self._det(n=5, tol_ms=3.0)
        # master +5 ms/frame, host +0.1 ms/frame (burst) → inconsistent, never live
        for i in range(50):
            d.update(500000 + i * 5, i * 100_000)
        self.assertFalse(d.live)

    def test_paced_live_latches_after_n(self):
        d = self._det(n=5, tol_ms=3.0)
        d.update(0, 0)                                   # seeds prev, no delta
        for i in range(1, 5):                            # 4 consistent deltas
            d.update(i * 5, i * 5_000_000)
        self.assertFalse(d.live)
        d.update(5 * 5, 5 * 5_000_000)                   # 5th consistent delta → live
        self.assertTrue(d.live)

    def test_gap_resets_then_live(self):
        d = self._det(n=3, tol_ms=3.0)
        d.update(0, 0)
        d.update(5, 5_000_000)                           # count 1
        d.update(10, 10_000_000)                         # count 2
        self.assertFalse(d.live)
        d.update(15, 10_100_000)                         # Δhost≈0.1 ms vs Δmaster 5 → reset
        d.update(20, 15_000_000)                         # count 1
        d.update(25, 20_000_000)                         # count 2
        self.assertFalse(d.live)
        d.update(30, 25_000_000)                         # count 3 → live
        self.assertTrue(d.live)


if __name__ == "__main__":
    unittest.main()
