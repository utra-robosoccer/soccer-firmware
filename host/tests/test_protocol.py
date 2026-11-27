"""Tests for the canonical host protocol library (host/master_link/protocol.py),
the hand-written ctypes mirror of protocol_wire.h.

Covers: fixed-point encode/decode round trips + saturation, command pack/parse
and telemetry parse round trips with exact truncated sizes, the frame codec
(round trip, bad CRC, unknown type, resync over junk, discard callback), and
the wrap-aware seq compare. The byte-exact C↔Python layout pin lives in
test_wire_fixture.py.
"""
import unittest

import master_link.protocol as P


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
        slaves = [{"req": P.SLAVE_REQ_RESET, "motors": [
            {"req": P.MOTOR_REQ_ENABLE, "pos": 0.5, "vel": -1.25, "kp": 15.0,
             "kd": 1.0, "tau_ff": 0.3}]}]
        b = P.pack_cmd_master(7, 42, slaves, test=0xB0)
        self.assertEqual(len(b), P.cmd_master_size(1))
        cm = P.parse_cmd_master(b)
        self.assertEqual((cm.cycle_id, cm.cmd_seq, cm.test), (7, 42, 0xB0))
        self.assertEqual(cm.slaves[0].req, P.SLAVE_REQ_RESET)
        m = cm.slaves[0].motors[0]
        self.assertEqual(m.req, P.MOTOR_REQ_ENABLE)
        self.assertAlmostEqual(P.dec_i16(m.mit.pos, P.POS_SCALE), 0.5, places=3)
        self.assertAlmostEqual(P.dec_i16(m.mit.vel, P.VEL_SCALE), -1.25, places=2)
        self.assertAlmostEqual(P.dec_u16(m.mit.kp, P.KP_SCALE), 15.0, places=1)
        # Unspecified slots default to STOP + zero mit (the forced-stop shape).
        m1 = cm.slaves[0].motors[1]
        self.assertEqual((m1.req, m1.mit.pos, m1.mit.kp), (P.MOTOR_REQ_STOP, 0, 0))

    def test_exact_sizes(self):
        for n in range(1, P.MAX_SLAVES + 1):
            b = P.pack_cmd_master(0, 1, [{} for _ in range(n)])
            self.assertEqual(len(b), P.cmd_master_size(n))
        self.assertEqual(P.cmd_master_size(P.MAX_SLAVES), 229)
        self.assertIsNone(P.parse_cmd_master(b"\x00"))   # shorter than the prefix


class TeleRoundTrip(unittest.TestCase):
    def _make(self, n_slaves=1):
        tm = P.TeleMaster()
        tm.cycle_id = 9
        tm.time_master_us = 123456
        tm.cmd_seq_active = 41
        tm.robot_state = P.ROBOT_READY
        tm.n_slaves = n_slaves
        tm.status.host.last_cmd_seq_rx = 42
        tm.status.host.on_time = 1000
        tm.status.host.late = 7
        tm.status.spi[0].alive = 1
        sl = tm.slaves[0]
        sl.time_slave_us = 7777
        sl.state = P.SLAVE_READY
        sl.n_motors = 5
        mo = sl.motors[0]
        mo.state = P.MOTOR_ENABLED
        mo.fault = P.MOTOR_FAULT_NONE
        mo.flags = P.MOTOR_FLAG_SATURATED
        mo.last_applied_seq = 0x1234
        mo.rs.pos, _ = P.enc_i16(0.5, P.POS_SCALE)
        mo.rs.vel, _ = P.enc_i16(-1.25, P.VEL_SCALE)
        mo.rs.mode = P.RS_MODE_RUN
        mo.rs.fault_bits = 0x0A
        mo.rs.fault_word = 0xDEADBEEF
        mo.rs.time_slave_us = 7000
        return tm

    def test_pack_parse(self):
        b = P.pack_tele_master(self._make())
        self.assertEqual(len(b), P.tele_master_size(1))
        tm = P.parse_tele_master(b)
        self.assertIsNotNone(tm)
        self.assertEqual((tm.cycle_id, tm.cmd_seq_active, tm.n_slaves), (9, 41, 1))
        self.assertEqual(tm.robot_state, P.ROBOT_READY)
        self.assertEqual(tm.status.host.last_cmd_seq_rx, 42)
        self.assertEqual((tm.status.host.on_time, tm.status.host.late), (1000, 7))
        sl = tm.slaves[0]
        self.assertEqual((sl.state, sl.time_slave_us), (P.SLAVE_READY, 7777))
        mo = sl.motors[0]
        self.assertEqual(P.MOTOR_STATE_NAMES[mo.state], "ENABLED")
        self.assertEqual(P.RS_MODE_NAMES[mo.rs.mode], "RUN")
        self.assertAlmostEqual(P.dec_i16(mo.rs.pos, P.POS_SCALE), 0.5, places=3)
        self.assertEqual(mo.rs.fault_word, 0xDEADBEEF)
        self.assertEqual(mo.last_applied_seq, 0x1234)
        # Truncation: the absent slave slots come back zeroed.
        self.assertEqual(tm.slaves[3].state, 0)

    def test_truncated_short_rejected(self):
        b = P.pack_tele_master(self._make())
        self.assertIsNone(P.parse_tele_master(b[:P.tele_master_size(1) - 1]))
        self.assertIsNone(P.parse_tele_master(b"\x00" * 4))


class FrameCodec(unittest.TestCase):
    def test_round_trip(self):
        payload = P.pack_cmd_master(1, 1, [{}])
        fr = P.encode_frame(P.USB_MSG_CMD, payload, seq=77)
        r = P.decode_frame(bytearray(fr))
        self.assertIsNotNone(r)
        mt, seq, pl, consumed = r
        self.assertEqual((mt, seq, pl, consumed),
                         (P.USB_MSG_CMD, 77, payload, len(fr)))

    def test_bad_crc_resyncs(self):
        fr = bytearray(P.encode_frame(P.USB_MSG_PING))
        fr[-1] ^= 0x01
        self.assertIsNone(P.decode_frame(bytearray(fr)))

    def test_junk_then_frame_recovers_with_discard(self):
        good = P.encode_frame(P.USB_MSG_PING, b"\xAA", seq=5)
        buf = bytearray(b"\x00" * 14 + good)
        discards = []
        r = P.decode_frame(buf, on_reject=lambda why, b: discards.append((why, b)))
        self.assertIsNotNone(r)
        mt, seq, pl, consumed = r
        self.assertEqual((mt, seq, pl), (P.USB_MSG_PING, 5, b"\xAA"))
        self.assertEqual(discards, [("discard", b"\x00" * 14)])

    def test_unknown_type_dropped(self):
        fr = bytearray(P.encode_frame(P.USB_MSG_PING))
        fr[0] = 0x7F       # unknown type; CRC now also wrong — both paths drop
        self.assertIsNone(P.decode_frame(bytearray(fr)))

    def test_incomplete_waits(self):
        fr = P.encode_frame(P.USB_MSG_CMD, b"\x01" * 20)
        buf = bytearray(fr[:-5])
        self.assertIsNone(P.decode_frame(buf))
        self.assertEqual(len(buf), len(fr) - 5)   # nothing consumed while waiting


class WrapAwareSeq(unittest.TestCase):
    def test_seq_ge_basic_and_wrap(self):
        self.assertTrue(P.seq_ge(5, 5))
        self.assertTrue(P.seq_ge(6, 5))
        self.assertFalse(P.seq_ge(5, 6))
        self.assertTrue(P.seq_ge(2, 65535))
        self.assertFalse(P.seq_ge(65535, 2))
        self.assertTrue(P.seq_ge(0x7FFF, 0))
        self.assertFalse(P.seq_ge(0, 0x7FFF))


if __name__ == "__main__":
    unittest.main()
