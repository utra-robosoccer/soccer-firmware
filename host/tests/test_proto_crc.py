"""Cross-language check for the byte-wise CRC table (task 1b).

Compiles firmware/common/test/test_proto_crc.c (which asserts, in C, that the new
table-driven proto_crc16 == the old bit-by-bit algorithm over 10 000 random buffers
incl. length 0 and the max frame size) and verifies each printed CRC also equals
binascii.crc_hqx(buf, 0xFFFF) — i.e. C-table == C-bitwise == host crc_hqx, which is
what master_link.protocol.crc16 uses. A mismatch means the wire CRC drifted.
"""
import binascii
import os
import shutil
import subprocess
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))

ITERS = 10000
MAXLEN = 512          # must match MAXLEN in test_proto_crc.c


def _lcg():
    """Identical 32-bit LCG to the C harness; yields the same stream forever."""
    s = 0x12345678
    while True:
        s = (s * 1664525 + 1013904223) & 0xFFFFFFFF
        yield s


class CrcTableCrossLang(unittest.TestCase):
    def _build_and_run(self):
        gcc = shutil.which("gcc") or shutil.which("cc")
        if gcc is None:
            self.skipTest("no C compiler available")
        src = os.path.join(ROOT, "firmware", "common", "test", "test_proto_crc.c")
        inc = os.path.join(ROOT, "firmware", "common", "include")
        exe = os.path.join(HERE, "_proto_crc_bin")
        subprocess.check_call([gcc, "-std=c11", "-Wall", "-Wextra", "-Werror",
                               "-I", inc, src, "-o", exe])
        try:
            out = subprocess.check_output([exe]).decode()   # nonzero exit ⇒ C found table!=bitwise
        finally:
            os.remove(exe)
        return [ln.split() for ln in out.splitlines() if ln]

    def test_table_matches_bitwise_and_crc_hqx(self):
        rows = self._build_and_run()
        self.assertEqual(len(rows), ITERS)

        rng = _lcg()
        for i, (len_s, crc_s) in enumerate(rows):
            length = int(len_s)
            c_crc = int(crc_s, 16)
            # Mirror the C length rule, consuming the LCG in the same order.
            if i == 0:
                self.assertEqual(length, 0)
            elif i == 1:
                self.assertEqual(length, MAXLEN)
            else:
                self.assertEqual(length, next(rng) % (MAXLEN + 1))
            buf = bytes(next(rng) & 0xFF for _ in range(length))
            self.assertEqual(
                binascii.crc_hqx(buf, 0xFFFF), c_crc,
                f"CRC drift at i={i} len={length}: crc_hqx != C table")
        # length 0 and the max frame size were both exercised.
        self.assertEqual(int(rows[0][0]), 0)
        self.assertEqual(int(rows[1][0]), MAXLEN)


if __name__ == "__main__":
    unittest.main()
