"""Runs the C host golden/oracle tests for the SPI frame codec (spi_proto.c).

Compiles firmware/common/test/test_spi_proto.c together with the codec and runs
it. The C test asserts spi_proto's telemetry build + command parse are
byte-identical to the pre-extraction algorithm (embedded as a reference), across
thousands of randomized cases plus a frozen golden frame. Exit 0 = all pass.

This keeps the extraction honest in CI: any drift in the SPI wire framing fails
here (and the sibling test_protocol.py catches C<->Python layout drift).
"""
import os
import shutil
import subprocess
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))   # host/tests → host → repo root


class SpiProtoGolden(unittest.TestCase):
    def test_codec_byte_identical(self):
        gcc = shutil.which("gcc") or shutil.which("cc")
        if gcc is None:
            self.skipTest("no C compiler available")
        common_inc = os.path.join(ROOT, "firmware", "common", "include")
        slave_inc = os.path.join(ROOT, "firmware", "slave", "slave_general", "Core", "Inc")
        slave_src = os.path.join(ROOT, "firmware", "slave", "slave_general", "Core", "Src")
        test_src = os.path.join(ROOT, "firmware", "common", "test", "test_spi_proto.c")
        codec_src = os.path.join(slave_src, "spi_proto.c")
        exe = os.path.join(HERE, "_test_spi_proto_bin")
        subprocess.check_call([
            gcc, "-std=c11", "-Wall", "-Wextra", "-Werror",
            "-I", common_inc, "-I", slave_inc,
            test_src, codec_src, "-o", exe,
        ])
        try:
            out = subprocess.run([exe], capture_output=True, text=True)
        finally:
            os.remove(exe)
        self.assertEqual(out.returncode, 0,
                         f"spi_proto C tests failed:\n{out.stdout}\n{out.stderr}")


if __name__ == "__main__":
    unittest.main()
