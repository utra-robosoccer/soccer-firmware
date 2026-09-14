"""Tests for the canonical host protocol library (host/jetson/protocol.py).

Includes a cross-language fixture test: it compiles and runs the C fixture
(firmware/common/test/gen_fixture.c), which packs a known MotorState atom and a
full SPI telemetry frame using protocol.h, and byte-compares against the same
values packed in Python. This catches C<->Python layout / CRC drift in CI.
"""
import os
import shutil
import struct
import subprocess
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "tools"))          # motor_config_gen (transport bounds)
sys.path.insert(0, os.path.join(ROOT, "host", "jetson"))  # protocol

import protocol as P  # noqa: E402

# Known fixture values — MUST match firmware/common/test/gen_fixture.c.
_LIFE0, _CAUSE0 = 3, 1      # MOTOR_ARMED_HOLD, CAUSE_OVERTORQUE
_LIFE1, _CAUSE1 = 7, 0      # MOTOR_ARMED_MIT, CAUSE_NONE
_FLAGS = P.CMDFLAG_CLAMPED_POS | P.CMDFLAG_CMD_STALE


def _atom(pos_raw: int, life: int, cause: int) -> bytes:
    return struct.pack(P.MOTORSTATE_FMT, pos_raw, 40000, 30000, 42,
                       (life & 0x0F) | (cause << 4), 0x05, _FLAGS,
                       0xDEADBEEF, 250, 0)


class MotorStateDecode(unittest.TestCase):
    def test_lifecycle_cause_split_and_flags(self):
        payload = struct.pack(P.FMT_MOTOR_STATE_HDR, 1, 2) + _atom(12345, _LIFE0, _CAUSE0)
        d = P.parse_motor_state(payload)
        self.assertEqual(d["slave_id"], 1)
        self.assertEqual(d["motor_idx"], 2)
        self.assertEqual(d["state"], _LIFE0)
        self.assertEqual(d["cause"], _CAUSE0)
        self.assertEqual(d["motor_fault"], 0x05)
        self.assertEqual(d["fault_word"], 0xDEADBEEF)
        self.assertEqual(d["fb_age"], 250)
        ms = d["motor"]
        self.assertTrue(ms.clamped_pos)
        self.assertFalse(ms.clamped_tau)
        self.assertTrue(ms.cmd_stale)

    def test_raw_decode_uses_transport_bounds(self):
        payload = struct.pack(P.FMT_MOTOR_STATE_HDR, 0, 0) + _atom(0, 0, 0)
        d = P.parse_motor_state(payload)
        # pos_raw=0 -> lower bound; vel_raw=40000, tau_raw=30000 within bounds.
        self.assertAlmostEqual(d["pos"], P.MOTOR_P_MIN, places=4)
        self.assertAlmostEqual(
            d["vel"], P.MOTOR_V_MIN + 40000 * (P.MOTOR_V_MAX - P.MOTOR_V_MIN) / 65535.0,
            places=4)


class FrameCodec(unittest.TestCase):
    def test_frame_round_trip(self):
        payload = struct.pack(P.FMT_MOTOR_STATE_HDR, 0, 1) + _atom(1000, _LIFE1, _CAUSE1)
        fr = P.encode_frame(P.MSG_MOTOR_STATE, P.NODE_MASTER, P.NODE_JETSON, payload)
        buf = bytearray(fr)
        r = P.decode_frame(buf)
        self.assertIsNotNone(r)
        self.assertEqual(r[0], P.MSG_MOTOR_STATE)
        self.assertEqual(r[3], payload)

    def test_bad_crc_resyncs(self):
        fr = bytearray(P.encode_frame(P.MSG_PING, P.NODE_JETSON, P.NODE_MASTER))
        fr[-1] ^= 0x01                      # corrupt the CRC
        self.assertIsNone(P.decode_frame(bytearray(fr)))


class CrossLanguageFixture(unittest.TestCase):
    """Byte-compare the C-packed fixture against the Python-packed equivalent."""

    def _build_and_run(self):
        gcc = shutil.which("gcc") or shutil.which("cc")
        if gcc is None:
            self.skipTest("no C compiler available")
        src = os.path.join(ROOT, "firmware", "common", "test", "gen_fixture.c")
        inc = os.path.join(ROOT, "firmware", "common", "include")
        exe = os.path.join(HERE, "_gen_fixture_bin")
        subprocess.check_call([gcc, "-std=c11", "-I", inc, src, "-o", exe])
        try:
            out = subprocess.check_output([exe]).decode()
        finally:
            os.remove(exe)
        fields = {}
        for line in out.splitlines():
            parts = line.split()
            fields[parts[0]] = bytes(int(x, 16) for x in parts[1:])
        return fields

    def test_atom_and_frame_match_c(self):
        got = self._build_and_run()

        atom0 = _atom(12345, _LIFE0, _CAUSE0)
        self.assertEqual(got["ATOM"], atom0, "MotorState atom layout drift C<->Python")

        atom1 = _atom(1000, _LIFE1, _CAUSE1)
        frame = bytearray(struct.pack("<BB", 0x03, 0x2A) + atom0 + atom1 + b"\x00" * 8 + b"\x00\x00")
        crc = P.crc16(bytes(frame[:-2]))
        frame[-2] = crc & 0xFF
        frame[-1] = crc >> 8
        self.assertEqual(bytes(frame), got["FRAME"], "SPI frame layout/CRC drift C<->Python")


if __name__ == "__main__":
    unittest.main()
