"""Host test for the resynchronizing frame scanner (protocol.h proto_frame_scan),
the fix for the USB command-RX desync.

Compiles test_proto_frame.c — whose `Framer` mirrors the master's
MotorMaster_ProcessUsbRx drain loop — and runs its scenarios (clean back-to-back,
byte-by-byte split, 14-byte leading junk, bad CRC, truncated frame, wrong version).
A nonzero exit means a scenario failed to recover; we also assert the summary line.
"""
import os
import shutil
import subprocess
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))


class ProtoFrameScanner(unittest.TestCase):
    def test_recovers_from_junk_split_and_bad_crc(self):
        gcc = shutil.which("gcc") or shutil.which("cc")
        if gcc is None:
            self.skipTest("no C compiler available")
        src = os.path.join(ROOT, "firmware", "common", "test", "test_proto_frame.c")
        inc = os.path.join(ROOT, "firmware", "common", "include")
        exe = os.path.join(HERE, "_proto_frame_bin")
        subprocess.check_call([gcc, "-std=c11", "-Wall", "-Wextra", "-Werror",
                               "-I", inc, src, "-o", exe])
        try:
            out = subprocess.check_output([exe]).decode()   # nonzero exit ⇒ a CHECK failed
        finally:
            os.remove(exe)
        self.assertIn("OK 6 scenarios", out)


if __name__ == "__main__":
    unittest.main()
