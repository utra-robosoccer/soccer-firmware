"""Host test for the RobStride ext-id codec (firmware/common/include/robostride_id.h).

Compiles test_robostride_id.c (which round-trips 20 000 random triples and checks
the 29-bit bound in C) and verifies each printed ext id matches the wire formula
mode<<24 | data<<8 | id. Also pins known IDs for the frames we send and the type-2
feedback reply we parse — this is the regression guard for the strict-aliasing /
bitfield-order fix that replaced the (exCanIdInfo*)&ExtId pointer cast.
"""
import os
import shutil
import subprocess
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))


def pack(mode, data, id_):
    return ((mode & 0x1F) << 24) | ((data & 0xFFFF) << 8) | (id_ & 0xFF)


def unpack(ext):
    return ((ext >> 24) & 0x1F, (ext >> 8) & 0xFFFF, ext & 0xFF)   # mode, data, id


class RobostrideIdCodec(unittest.TestCase):
    def test_known_ids(self):
        # Tx frames (mode, data=master_id/payload, target id) → ext id.
        self.assertEqual(pack(3, 0x00FD, 5), 0x0300FD05)   # enable motor id5, master 0xFD
        self.assertEqual(pack(4, 0x00FD, 5), 0x0400FD05)   # disable
        self.assertEqual(pack(1, 0x8000, 1), 0x01800001)   # MIT, torque midscale, id1
        self.assertEqual(pack(0x12, 0x00FD, 2), 0x1200FD02)  # change-mode (type 18)

        # Type-2 feedback reply we parse: data field packs motor_id[0:7],
        # faults[8:13], mode[14:15]; id byte carries the master id (0xFD).
        fb_data = 0x05 | (0x00 << 8) | (0x2 << 14)         # motor 5, no faults, mode 2
        ext = pack(2, fb_data, 0xFD)
        self.assertEqual(ext, 0x028005FD)
        mode, data, idb = unpack(ext)
        self.assertEqual(mode, 2)
        self.assertEqual(data & 0x00FF, 5)                 # feedback motor id
        self.assertEqual((data & 0xC000) >> 14, 2)         # motor mode
        self.assertEqual((data & 0x3F00) >> 8, 0)          # fault byte
        self.assertEqual(idb, 0xFD)

    def test_c_matches_formula(self):
        gcc = shutil.which("gcc") or shutil.which("cc")
        if gcc is None:
            self.skipTest("no C compiler available")
        src = os.path.join(ROOT, "firmware", "common", "test", "test_robostride_id.c")
        inc = os.path.join(ROOT, "firmware", "common", "include")
        exe = os.path.join(HERE, "_rsid_bin")
        subprocess.check_call([gcc, "-std=c11", "-Wall", "-Wextra", "-Werror",
                               "-I", inc, src, "-o", exe])
        try:
            out = subprocess.check_output([exe]).decode()   # nonzero ⇒ C round-trip failed
        finally:
            os.remove(exe)
        rows = [ln.split() for ln in out.splitlines() if ln]
        self.assertEqual(len(rows), 20000)
        for mode_s, data_s, id_s, ext_s in rows:
            mode, data, id_ = int(mode_s), int(data_s), int(id_s)
            ext = int(ext_s, 16)
            self.assertEqual(ext, pack(mode, data, id_))   # C pack == wire formula
            self.assertEqual(unpack(ext), (mode, data, id_))
            self.assertEqual(ext >> 29, 0)


if __name__ == "__main__":
    unittest.main()
