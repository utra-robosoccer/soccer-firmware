"""Host test for the slave SPI-resync NSS-gating helper (spi_resync.h).

Compiles test_spi_resync.c and runs its cases: re-arm only proceeds while NSS is
high (between exchanges); NSS low within the bound waits; NSS low past the bound
times out (skip, don't re-arm mid-exchange); NSS high wins over the timeout.
"""
import os
import shutil
import subprocess
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))


class SpiResyncGating(unittest.TestCase):
    def test_nss_gating(self):
        gcc = shutil.which("gcc") or shutil.which("cc")
        if gcc is None:
            self.skipTest("no C compiler available")
        src = os.path.join(ROOT, "firmware", "common", "test", "test_spi_resync.c")
        inc = os.path.join(ROOT, "firmware", "common", "include")
        exe = os.path.join(HERE, "_spi_resync_bin")
        subprocess.check_call([gcc, "-std=c11", "-Wall", "-Wextra", "-Werror",
                               "-I", inc, src, "-o", exe])
        try:
            out = subprocess.check_output([exe]).decode()   # nonzero exit ⇒ a case failed
        finally:
            os.remove(exe)
        self.assertIn("OK spi_resync gating", out)


if __name__ == "__main__":
    unittest.main()
