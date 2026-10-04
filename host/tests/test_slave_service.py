"""Host test for the slave forward/fallback service decision (slave_service.h).

Compiles test_slave_service.c and runs its cases: service on every valid exchange
(ROBOT_CMD or NOP), nothing on a CRC-failed exchange, fallback only after the exchange
gap, no double service within one cycle, and no fallback self-throttling during a gap.
"""
import os
import shutil
import subprocess
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))


class SlaveServiceDecision(unittest.TestCase):
    def test_forward_fallback(self):
        gcc = shutil.which("gcc") or shutil.which("cc")
        if gcc is None:
            self.skipTest("no C compiler available")
        src = os.path.join(ROOT, "firmware", "common", "test", "test_slave_service.c")
        inc = os.path.join(ROOT, "firmware", "common", "include")
        exe = os.path.join(HERE, "_slave_service_bin")
        subprocess.check_call([gcc, "-std=c11", "-Wall", "-Wextra", "-Werror",
                               "-I", inc, src, "-o", exe])
        try:
            out = subprocess.check_output([exe]).decode()   # nonzero exit ⇒ a case failed
        finally:
            os.remove(exe)
        self.assertIn("OK slave_service", out)


if __name__ == "__main__":
    unittest.main()
