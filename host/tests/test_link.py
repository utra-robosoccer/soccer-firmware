"""MasterLink tests against a fake serial port (no hardware).

A loopback FakeSerial feeds recorded RX frames — a valid MOTOR_STATE, a corrupted
run (CRC fail → resync/discard), a wrong-version frame, and a MASTER_STATUS — and
captures TX writes. Asserts: state store updates, discards + version frames are
logged, and send_mit/send_control emit correct, decodable frames logged as TX_FRAME.
"""
import os
import struct
import tempfile
import threading
import time
import unittest
from unittest import mock

import master_link.protocol as P
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

    def close(self):
        self.closed = True


def _motor_state(slave, idx, pos_raw=32768, state=3, last_applied_seq=0):
    atom = struct.pack(P.MOTORSTATE_FMT, pos_raw, 40000, 30000, 42, state, 0, 0, 0, 5, 0,
                       last_applied_seq)
    payload = struct.pack(P.FMT_MOTOR_STATE_HDR, slave, idx) + atom
    return P.encode_frame(P.MSG_MOTOR_STATE, P.NODE_MASTER, P.NODE_JETSON, payload)


def _master_status():
    payload = struct.pack(P.FMT_MASTER_STATUS, 1, 0b1, 1234, 0, 99)
    return P.encode_frame(P.MSG_MASTER_STATUS, P.NODE_MASTER, P.NODE_JETSON, payload)


def _motor_state_ts(slave, idx, ts_ms, state=7, pos_raw=32768):
    """MOTOR_STATE frame with an explicit master_ts_ms (encode_frame can't set it)."""
    atom = struct.pack(P.MOTORSTATE_FMT, pos_raw, 40000, 30000, 42, state, 0, 0, 0, 5, 0, 0)
    payload = struct.pack(P.FMT_MOTOR_STATE_HDR, slave, idx) + atom
    fr = bytearray(struct.pack(P.HDR_FMT, P.MSG_MOTOR_STATE, 0, P.NODE_MASTER,
                               P.NODE_JETSON, ts_ms & 0xFFFFFFFF, len(payload),
                               P.PROTO_VERSION, 0)) + payload
    fr[14] = 0; fr[15] = 0
    crc = P.crc16(bytes(fr))
    fr[14] = crc & 0xFF; fr[15] = (crc >> 8) & 0xFF
    return bytes(fr)


def _version_mismatch_frame():
    fr = bytearray(_motor_state(0, 1, pos_raw=1000))
    fr[12] = P.PROTO_VERSION + 1          # ver_flags low byte → wrong version
    fr[14] = 0
    fr[15] = 0
    crc = P.crc16(bytes(fr))              # recompute so it passes CRC but fails the gate
    fr[14] = crc & 0xFF
    fr[15] = (crc >> 8) & 0xFF
    return bytes(fr)


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

    def test_rx_state_discard_and_version(self):
        with tempfile.TemporaryDirectory() as d:
            lk, fake = self._make_link(d)
            try:
                corrupt = bytearray(_motor_state(0, 2))
                corrupt[16] ^= 0xFF                       # break CRC → discarded on resync
                fake.feed(_motor_state(0, 0, state=3))    # valid → state store
                fake.feed(bytes(corrupt))                 # garbage → RX_DISCARD
                fake.feed(_version_mismatch_frame())      # wrong version → RX_FRAME, counted
                fake.feed(_master_status())               # valid master status

                self.assertTrue(_wait(lambda: (0, 0) in lk.latest_state().motors),
                                "motor (0,0) never ingested")
                self.assertTrue(_wait(lambda: lk.latest_state().master is not None))
                self.assertTrue(_wait(lambda: lk.stats["discard_bytes"] > 0))
                self.assertTrue(_wait(lambda: lk.stats["version_frames"] == 1))

                st = lk.latest_state()
                self.assertEqual(st.motors[(0, 0)].state, 3)
                self.assertEqual(st.motors[(0, 0)].gidx, 0)
                self.assertEqual(st.master["rx_frames"], 99)
            finally:
                lk.close()

            # Log contains a wrong-version RX_FRAME and at least one RX_DISCARD.
            kinds = [(rec.kind, rec.payload) for rec in BinaryLogReader(lk.log_path)]
            assert any(k == LOG.RX_DISCARD for k, _ in kinds), "no RX_DISCARD logged"
            wrong_ver = [p for k, p in kinds if k == LOG.RX_FRAME
                         and len(p) >= 13 and p[12] != P.PROTO_VERSION]
            self.assertEqual(len(wrong_ver), 1, "version-mismatch frame not logged as RX_FRAME")

    def test_tx_mit_and_control(self):
        from master_link.link import MitCommand, ControlRequest, ControlKind
        with tempfile.TemporaryDirectory() as d:
            lk, fake = self._make_link(d)
            try:
                lk.send_mit([MitCommand(slave=0, local=1, pos=1.5, vel=-2.0,
                                        kp=15.0, kd=1.0, tau_ff=0.0)])
                lk.send_control(ControlRequest(slave=0, local=0, kind=ControlKind.ARM))
                self.assertTrue(_wait(lambda: lk.stats["tx_frames"] == 2))
            finally:
                lk.close()

            # Decode the captured TX bytes back through the codec.
            buf = bytearray(fake.tx)
            frames = []
            while True:
                r = P.decode_frame(buf)
                if r is None:
                    break
                frames.append(r)
                del buf[:r[4]]      # remove the consumed frame

            types = [f[0] for f in frames]
            self.assertIn(P.MSG_MOTOR_CMD, types)
            self.assertIn(P.MSG_CONTROL_REQ, types)
            mit = next(f for f in frames if f[0] == P.MSG_MOTOR_CMD)
            s, l, pos, vel, kp, kd, tau, cmd_seq = struct.unpack(P.FMT_MOTOR_CMD, mit[3])
            self.assertEqual((s, l), (0, 1))
            self.assertAlmostEqual(pos, 1.5, places=5)
            self.assertAlmostEqual(kp, 15.0, places=5)
            self.assertEqual(cmd_seq, 1)   # first send_mit tick stamps cmd_seq=1 (skips 0)
            # TX_FRAME records present in the log.
            tx = [rec for rec in BinaryLogReader(lk.log_path) if rec.kind == LOG.TX_FRAME]
            self.assertEqual(len(tx), 2)

    def test_live_detection_stale_then_live(self):
        with tempfile.TemporaryDirectory() as d:
            lk, fake = self._make_link(d)
            try:
                # Stale burst: valid frames with old ts fed all at once (Δhost ≈ 0).
                for i in range(15):
                    fake.feed(_motor_state_ts(0, 0, 500000 + i * 5))
                time.sleep(0.1)
                self.assertEqual(lk.live_motors(), set(),
                                 "went live on the stale burst")
                # Live stream: ts advances ~5 ms, fed ~6 ms apart in real time.
                for i in range(8):
                    fake.feed(_motor_state_ts(0, 0, 1_000_000 + i * 5))
                    time.sleep(0.006)
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
