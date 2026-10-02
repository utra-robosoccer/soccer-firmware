"""MasterLink tests against a fake serial port (no hardware).

A loopback FakeSerial feeds recorded RX frames — a valid ROBOT_TELE, a corrupted
run (CRC fail → resync/discard), a wrong-version frame, and a MASTER_STATUS — and
captures TX writes. Asserts: state store updates, discards + version frames are
logged, and send_robot_cmd emits a correct, decodable ROBOT_CMD logged as TX_FRAME.
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

    def flush(self):
        pass

    def close(self):
        self.closed = True


def _tele_payload(state=3, last_applied_seq=0, pos_raw=5000):
    """One-chain, one-motor tele_robot_t payload (motor at (0,0))."""
    motor = dict(pos_raw=pos_raw, vel_raw=0, tau_raw=0, temp_c=42, state=state,
                 cause=0, motor_mode=2, motor_fault=0, flags=0, fb_age_ms=5,
                 fault_word=0, last_applied_seq=last_applied_seq)
    chains = [dict(chain_id=0, spi_seq_echo=0, slave_time_us=0,
                   cmd_crc_errors=0, can_tx_errors=0, motors=[motor])]
    return P.pack_robot_tele(1, 0, 0, 0, 1, chains)


def _robot_tele(state=3, last_applied_seq=0, pos_raw=5000):
    return P.encode_frame(P.MSG_ROBOT_TELE, P.NODE_MASTER, P.NODE_JETSON,
                          _tele_payload(state, last_applied_seq, pos_raw))


def _master_status():
    payload = struct.pack(P.FMT_MASTER_STATUS, 1, 0b1, 1234, 0, 99, 200, 200, 200, 50)
    return P.encode_frame(P.MSG_MASTER_STATUS, P.NODE_MASTER, P.NODE_JETSON, payload)


def _robot_tele_ts(ts_ms, state=7):
    """ROBOT_TELE frame with an explicit master_ts_ms (encode_frame can't set it)."""
    payload = _tele_payload(state=state)
    fr = bytearray(struct.pack(P.HDR_FMT, P.MSG_ROBOT_TELE, 0, P.NODE_MASTER,
                               P.NODE_JETSON, ts_ms & 0xFFFFFFFF, len(payload),
                               P.PROTO_VERSION, 0)) + payload
    fr[14] = 0; fr[15] = 0
    crc = P.crc16(bytes(fr))
    fr[14] = crc & 0xFF; fr[15] = (crc >> 8) & 0xFF
    return bytes(fr)


def _version_mismatch_frame():
    fr = bytearray(_robot_tele(pos_raw=1000))
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
                corrupt = bytearray(_robot_tele(pos_raw=1234))
                corrupt[16] ^= 0xFF                       # break CRC → discarded on resync
                fake.feed(_robot_tele(state=3))           # valid → state store
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

    def test_tx_robot_cmd(self):
        from master_link.link import MotorCommand, MODE_MIT, MODE_HOLD
        with tempfile.TemporaryDirectory() as d:
            lk, fake = self._make_link(d)
            try:
                lk.send_robot_cmd([
                    MotorCommand(slave=0, local=0, mode=MODE_HOLD),
                    MotorCommand(slave=0, local=1, mode=MODE_MIT, pos=1.5, vel=-2.0),
                ])
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
                del buf[:r[4]]

            self.assertEqual([f[0] for f in frames], [P.MSG_ROBOT_CMD])
            d2 = P.parse_robot_cmd(frames[0][3])
            self.assertEqual(d2["cmd_seq"], 1)   # first tick stamps cmd_seq=1 (skips 0)
            motors = d2["chains"][0]["motors"]
            self.assertEqual(motors[0]["mode_req"], MODE_HOLD)
            self.assertEqual(motors[1]["mode_req"], MODE_MIT)
            self.assertAlmostEqual(motors[1]["pos"], 1.5, places=3)
            self.assertAlmostEqual(motors[1]["vel"], -2.0, places=2)
            tx = [rec for rec in BinaryLogReader(lk.log_path) if rec.kind == LOG.TX_FRAME]
            self.assertEqual(len(tx), 1)

    def test_live_detection_stale_then_live(self):
        with tempfile.TemporaryDirectory() as d:
            lk, fake = self._make_link(d)
            try:
                # Stale burst: master ts jumps 100 ms/frame but the whole blob drains
                # to the host at once (Δhost ≪ Δmaster) → never live.
                burst = b"".join(_robot_tele_ts(500000 + i * 100) for i in range(15))
                fake.feed(burst)
                time.sleep(0.1)
                self.assertEqual(lk.live_motors(), set(),
                                 "went live on the stale burst")
                # Live stream: master ts advances ~10 ms, fed ~8 ms apart in real time
                # (plus ~2 ms host decode ≈ Δmaster) → latches live.
                for i in range(10):
                    fake.feed(_robot_tele_ts(1_000_000 + i * 10))
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
