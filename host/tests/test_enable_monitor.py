"""Runs the C host unit tests for the enable monitor (enable_monitor.c).

Compiles firmware/common/test/test_enable_monitor.c together with the pure
decision function and runs it. The C test drives explicit tick sequences (steady
NORMAL, K consecutive not-NORMAL, stale-feedback holds, suspend/resume, counter
saturation) and asserts the verdict + counter. Exit 0 = all pass.
"""
import os
import shutil
import subprocess
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))   # host/tests → host → repo root


class EnableMonitorTests(unittest.TestCase):
    def test_decision_logic(self):
        gcc = shutil.which("gcc") or shutil.which("cc")
        if gcc is None:
            self.skipTest("no C compiler available")
        slave_inc = os.path.join(ROOT, "firmware", "slave", "slave_general", "Core", "Inc")
        slave_src = os.path.join(ROOT, "firmware", "slave", "slave_general", "Core", "Src")
        test_src = os.path.join(ROOT, "firmware", "common", "test", "test_enable_monitor.c")
        mon_src = os.path.join(slave_src, "enable_monitor.c")
        exe = os.path.join(HERE, "_test_enable_monitor_bin")
        subprocess.check_call([
            gcc, "-std=c11", "-Wall", "-Wextra", "-Werror",
            "-I", slave_inc,
            test_src, mon_src, "-o", exe,
        ])
        try:
            out = subprocess.run([exe], capture_output=True, text=True)
        finally:
            os.remove(exe)
        self.assertEqual(out.returncode, 0,
                         f"enable_monitor C tests failed:\n{out.stdout}\n{out.stderr}")


if __name__ == "__main__":
    unittest.main()
