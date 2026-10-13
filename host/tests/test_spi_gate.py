"""Host test: compile + run the SPI version-gate C test (firmware/common/test)."""
import os
import shutil
import subprocess
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))


class SpiGate(unittest.TestCase):
    def test_pure(self):
        gcc = shutil.which("gcc") or shutil.which("cc")
        if gcc is None:
            self.skipTest("no C compiler available")
        src = os.path.join(ROOT, "firmware", "common", "test", "test_spi_gate.c")
        inc = os.path.join(ROOT, "firmware", "common", "include")
        exe = os.path.join(HERE, "_spi_gate_bin")
        subprocess.check_call([gcc, "-std=c11", "-Wall", "-Wextra", "-Werror",
                               "-I", inc, src, "-o", exe])
        try:
            out = subprocess.check_output([exe]).decode()
        finally:
            os.remove(exe)
        self.assertIn("OK spi_gate", out)


if __name__ == "__main__":
    unittest.main()
