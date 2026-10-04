"""Runs the C host unit tests for the slave's reply-window pairing (cmd_seq_track.c):
normal pairing, dropped reply, dropped first frame of a new seq, and an enable/
mode-change reply not counting. Exit 0 = all pass."""
import os
import shutil
import subprocess
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))


class CmdSeqTrackTests(unittest.TestCase):
    def test_pairing_logic(self):
        gcc = shutil.which("gcc") or shutil.which("cc")
        if gcc is None:
            self.skipTest("no C compiler available")
        slave_inc = os.path.join(ROOT, "firmware", "slave", "slave_general", "Core", "Inc")
        slave_src = os.path.join(ROOT, "firmware", "slave", "slave_general", "Core", "Src")
        test_src = os.path.join(ROOT, "firmware", "common", "test", "test_cmd_seq_track.c")
        mod_src = os.path.join(slave_src, "cmd_seq_track.c")
        exe = os.path.join(HERE, "_test_cmd_seq_track_bin")
        subprocess.check_call([
            gcc, "-std=c11", "-Wall", "-Wextra", "-Werror",
            "-I", slave_inc, test_src, mod_src, "-o", exe,
        ])
        try:
            out = subprocess.run([exe], capture_output=True, text=True)
        finally:
            os.remove(exe)
        self.assertEqual(out.returncode, 0,
                         f"cmd_seq_track C tests failed:\n{out.stdout}\n{out.stderr}")


if __name__ == "__main__":
    unittest.main()
