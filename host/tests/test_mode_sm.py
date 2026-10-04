"""Compiles and runs the pure (HAL-free) mode-request state-machine C tests
(mode_sm.c): allowed/rejected transitions, HOLD capture, wound refusal, TO_ZERO
entry, fault latching and both reset paths. Exit 0 = all pass."""
import os
import shutil
import subprocess
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))


class ModeSmTests(unittest.TestCase):
    def test_state_machine(self):
        gcc = shutil.which("gcc") or shutil.which("cc")
        if gcc is None:
            self.skipTest("no C compiler available")
        common_inc = os.path.join(ROOT, "firmware", "common", "include")
        slave_inc = os.path.join(ROOT, "firmware", "slave", "slave_general", "Core", "Inc")
        slave_src = os.path.join(ROOT, "firmware", "slave", "slave_general", "Core", "Src")
        test_src = os.path.join(ROOT, "firmware", "common", "test", "test_mode_sm.c")
        mod_src = os.path.join(slave_src, "mode_sm.c")
        exe = os.path.join(HERE, "_test_mode_sm_bin")
        subprocess.check_call([
            gcc, "-std=c11", "-Wall", "-Wextra", "-Werror",
            "-I", common_inc, "-I", slave_inc, test_src, mod_src, "-o", exe,
        ])
        try:
            out = subprocess.run([exe], capture_output=True, text=True)
        finally:
            os.remove(exe)
        self.assertEqual(out.returncode, 0,
                         f"mode_sm C tests failed:\n{out.stdout}\n{out.stderr}")


if __name__ == "__main__":
    unittest.main()
