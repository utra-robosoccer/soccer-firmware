"""Tests for the canonical host protocol library (host/master_link/protocol.py),
PROTO_VERSION 3 robot/chain/motor hierarchy.

Covers: fixed-point encode/decode round trips + saturation, cmd/tele pack↔parse
round trips, wrong-version drop+count, wrap-aware seq compare, and a cross-language
fixture that compiles firmware/common/test/gen_fixture.c and byte-compares a full
cmd_robot_t and tele_robot_t against the Python packers.
"""
import os
import shutil
import struct
import subprocess
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))   # host/tests → host → repo root

import master_link.protocol as P  # noqa: E402


class FixedPoint(unittest.TestCase):
    def test_round_trip(self):
        for x, scale in [(0.5, P.POS_SCALE), (-1.25, P.VEL_SCALE), (0.3, P.TAU_SCALE)]:
            r, sat = P.enc_i16(x, scale)
            self.assertFalse(sat)
            self.assertAlmostEqual(P.dec_i16(r, scale), x, places=3)
        for x, scale in [(15.0, P.KP_SCALE), (1.0, P.KD_SCALE), (500.0, P.KP_SCALE)]:
            r, sat = P.enc_u16(x, scale)
            self.assertFalse(sat)
            self.assertAlmostEqual(P.dec_u16(r, scale), x, places=2)

    def test_saturation(self):
        # Position beyond ±π saturates i16 and flags.
        r, sat = P.enc_i16(4.0, P.POS_SCALE)
        self.assertEqual(r, 32767)
        self.assertTrue(sat)
        r, sat = P.enc_i16(-4.0, P.POS_SCALE)
        self.assertEqual(r, -32768)
        self.assertTrue(sat)
        # Large-actuator gains fit (KP×10 covers 0..5000); above that saturates.
        self.assertEqual(P.enc_u16(5000.0, P.KP_SCALE), (50000, False))
        self.assertEqual(P.enc_u16(7000.0, P.KP_SCALE), (65535, True))
        self.assertEqual(P.enc_u16(100.0, P.KD_SCALE), (10000, False))


class CmdRoundTrip(unittest.TestCase):
    def test_pack_parse(self):
        chains = [{"chain_id": 0, "motors": [
            {"mode_req": P.REQ_MIT, "pos": 0.5, "vel": -1.25, "kp": 15.0, "kd": 1.0,
             "tau_ff": 0.3, "flags": P.CMD_FLAG_VALID}]}]
        b = P.pack_robot_cmd(7, 42, chains)
        self.assertEqual(len(b), P.SZ_CMD_ROBOT)
        d = P.parse_robot_cmd(b)
        self.assertEqual((d["cycle_id"], d["cmd_seq"], d["n_chains"]), (7, 42, 1))
        m = d["chains"][0]["motors"][0]
        self.assertEqual(m["mode_req"], P.REQ_MIT)
        self.assertAlmostEqual(m["pos"], 0.5, places=3)
        self.assertAlmostEqual(m["vel"], -1.25, places=2)
        self.assertAlmostEqual(m["kp"], 15.0, places=1)
        self.assertEqual(m["flags"] & P.CMD_FLAG_VALID, P.CMD_FLAG_VALID)


class TeleRoundTrip(unittest.TestCase):
    def test_pack_parse(self):
        pr, _ = P.enc_i16(0.5, P.POS_SCALE)
        vr, _ = P.enc_i16(-1.25, P.VEL_SCALE)
        tr, _ = P.enc_i16(0.3, P.TAU_SCALE)
        motor = dict(pos_raw=pr, vel_raw=vr, tau_raw=tr, temp_c=42, state=P.LIFE_MIT,
                     cause=P.CAUSE_NONE, motor_mode=2, motor_fault=0x0A,
                     flags=P.TELE_FLAG_TO_ZERO_ARRIVED, fb_age_ms=250,
                     fault_word=0xDEADBEEF, last_applied_seq=0x1234)
        chains = [dict(chain_id=0, spi_seq_echo=0x2A, slave_time_us=7777,
                       cmd_crc_errors=3, can_tx_errors=1, spi_tx_arm_fails=4, motors=[motor])]
        b = P.pack_robot_tele(9, 123456, 42, 41, P.ROBOT_STATE_NAMES and 1, chains,
                              cmd_on_time=1000, cmd_late=7, cmd_missing=3, cmd_duplicate=2)
        self.assertEqual(len(b), P.SZ_TELE_ROBOT)
        d = P.parse_robot_tele(b)
        self.assertEqual((d["cycle_id"], d["last_cmd_seq_rx"], d["n_chains"]), (9, 42, 1))
        self.assertEqual((d["cmd_seq_active"], d["cmd_on_time"], d["cmd_late"],
                          d["cmd_missing"], d["cmd_duplicate"]), (41, 1000, 7, 3, 2))
        ch = d["chains"][0]
        self.assertEqual((ch["spi_seq_echo"], ch["cmd_crc_errors"], ch["can_tx_errors"]),
                         (0x2A, 3, 1))
        mo = ch["motors"][0]
        self.assertAlmostEqual(mo.pos, 0.5, places=3)
        self.assertAlmostEqual(mo.vel, -1.25, places=2)
        self.assertEqual(mo.lifecycle_name, "MIT")
        self.assertEqual(mo.motor_mode_name, "NORMAL")
        self.assertEqual(mo.last_applied_seq, 0x1234)
        self.assertTrue(mo.to_zero_arrived)


class FrameCodec(unittest.TestCase):
    def test_round_trip(self):
        payload = P.pack_robot_cmd(1, 1, [{"chain_id": 0, "motors": [
            {"mode_req": P.REQ_HOLD, "flags": P.CMD_FLAG_VALID}]}])
        fr = P.encode_frame(P.MSG_ROBOT_CMD, P.NODE_JETSON, P.NODE_MASTER, payload)
        r = P.decode_frame(bytearray(fr))
        self.assertIsNotNone(r)
        self.assertEqual(r[0], P.MSG_ROBOT_CMD)
        self.assertEqual(r[3], payload)

    def test_bad_crc_resyncs(self):
        fr = bytearray(P.encode_frame(P.MSG_PING, P.NODE_JETSON, P.NODE_MASTER))
        fr[-1] ^= 0x01
        self.assertIsNone(P.decode_frame(bytearray(fr)))

    def test_wrong_version_dropped_and_counted(self):
        wrong = (P.PROTO_VERSION + 1) & 0xFF
        fr = bytearray(P.encode_frame(P.MSG_PING, P.NODE_JETSON, P.NODE_MASTER))
        fr[12] = wrong                  # ver_flags low byte
        fr[14] = 0
        fr[15] = 0
        crc = P.crc16(bytes(fr))
        fr[14] = crc & 0xFF
        fr[15] = (crc >> 8) & 0xFF
        before = P.version_errors
        self.assertIsNone(P.decode_frame(bytearray(fr)))
        self.assertEqual(P.version_errors, before + 1)


class WrapAwareSeq(unittest.TestCase):
    def test_seq_ge_basic_and_wrap(self):
        self.assertTrue(P.seq_ge(5, 5))
        self.assertTrue(P.seq_ge(6, 5))
        self.assertFalse(P.seq_ge(5, 6))
        self.assertTrue(P.seq_ge(2, 65535))
        self.assertFalse(P.seq_ge(65535, 2))
        self.assertTrue(P.seq_ge(0x7FFF, 0))
        self.assertFalse(P.seq_ge(0, 0x7FFF))


class CrossLanguageFixture(unittest.TestCase):
    """Byte-compare the C-packed fixture against the Python-packed equivalent."""

    def _build_and_run(self):
        gcc = shutil.which("gcc") or shutil.which("cc")
        if gcc is None:
            self.skipTest("no C compiler available")
        src = os.path.join(ROOT, "firmware", "common", "test", "gen_fixture.c")
        inc = os.path.join(ROOT, "firmware", "common", "include")
        exe = os.path.join(HERE, "_gen_fixture_bin")
        subprocess.check_call([gcc, "-std=c11", "-Wall", "-Wextra", "-Werror",
                               "-I", inc, src, "-o", exe])
        try:
            out = subprocess.check_output([exe]).decode()
        finally:
            os.remove(exe)
        fields = {}
        for line in out.splitlines():
            parts = line.split()
            fields[parts[0]] = bytes(int(x, 16) for x in parts[1:])
        return fields

    def test_cmd_and_tele_match_c(self):
        got = self._build_and_run()

        chains = [{"chain_id": 0, "motors": [
            {"mode_req": P.REQ_MIT, "pos": 0.5, "vel": -1.25, "kp": 15.0, "kd": 1.0,
             "tau_ff": 0.3, "flags": P.CMD_FLAG_VALID}]}]
        py_cmd = P.pack_robot_cmd(7, 42, chains)
        self.assertEqual(py_cmd, got["ROBOTCMD"], "cmd_robot_t layout drift C↔Python")

        pr, _ = P.enc_i16(0.5, P.POS_SCALE)
        vr, _ = P.enc_i16(-1.25, P.VEL_SCALE)
        tr, _ = P.enc_i16(0.3, P.TAU_SCALE)
        motor = dict(pos_raw=pr, vel_raw=vr, tau_raw=tr, temp_c=42, state=P.LIFE_MIT,
                     cause=P.CAUSE_NONE, motor_mode=2, motor_fault=0x0A,
                     flags=P.TELE_FLAG_TO_ZERO_ARRIVED, fb_age_ms=250,
                     fault_word=0xDEADBEEF, last_applied_seq=0x1234)
        tchains = [dict(chain_id=0, spi_seq_echo=0x2A, slave_time_us=7777,
                        cmd_crc_errors=3, can_tx_errors=1, spi_tx_arm_fails=4, motors=[motor])]
        py_tele = P.pack_robot_tele(9, 123456, 42, 41, 1, tchains,
                                    cmd_on_time=1000, cmd_late=7, cmd_missing=3, cmd_duplicate=2)
        self.assertEqual(py_tele, got["ROBOTTELE"], "tele_robot_t layout drift C↔Python")


if __name__ == "__main__":
    unittest.main()
