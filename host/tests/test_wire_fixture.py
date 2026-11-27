"""Cross-language wire-layout fixture: protocol_wire.h ⇄ master_link.protocol.

Compiles firmware/common/test/gen_wire_fixture.c, which fills every
host-visible wire struct field by field (declaration order) from a shared
32-bit LCG and prints the raw bytes. This test replays the IDENTICAL LCG over
the ctypes mirrors in master_link.protocol (recursing _fields_ in declaration
order) and compares bytes — so any drift in field order, width, offset,
padding or endianness between the C header and the Python module fails here.
Also pins sizeofs, the truncation macros, a built PING frame against
encode_frame, and the CRC.
"""
import ctypes
import os
import shutil
import subprocess
import unittest

from master_link import protocol as P

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))

# C type name -> ctypes mirror, every struct the fixture dumps.
MIRRORS = {
    "rs_mit_t": P.RsMit, "cmd_motor_t": P.CmdMotor, "cmd_slave_t": P.CmdSlave,
    "cmd_master_t": P.CmdMaster, "rs_feedback_t": P.RsFeedback,
    "tele_motor_t": P.TeleMotor, "boot_result_t": P.BootResult,
    "spi_slave_stats_t": P.SpiSlaveStats, "can_stats_t": P.CanStats,
    "status_slave_t": P.StatusSlave, "tele_slave_t": P.TeleSlave,
    "host_stats_t": P.HostStats, "usb_stats_t": P.UsbStats,
    "spi_master_stats_t": P.SpiMasterStats, "status_master_t": P.StatusMaster,
    "imu_t": P.Imu, "tele_master_t": P.TeleMaster, "usb_header_t": P.UsbHeader,
}


class _Lcg:
    """Identical 32-bit LCG to gen_wire_fixture.c."""

    def __init__(self, seed=0xDEADBEEF):
        self.s = seed

    def next(self):
        self.s = (self.s * 1664525 + 1013904223) & 0xFFFFFFFF
        return self.s


def _fill(obj, lcg):
    """Replay the C filler: assign every field in declaration order, drawing
    one LCG value per scalar, recursing into nested structs and arrays."""
    for name, ftype in obj._fields_:
        if issubclass(ftype, ctypes.Structure):
            _fill(getattr(obj, name), lcg)
        elif issubclass(ftype, ctypes.Array):
            arr = getattr(obj, name)
            if issubclass(ftype._type_, ctypes.Structure):
                for el in arr:
                    _fill(el, lcg)
            else:
                for i in range(len(arr)):
                    arr[i] = _scalar(ftype._type_, lcg)
        else:
            setattr(obj, name, _scalar(ftype, lcg))


def _scalar(ctype, lcg):
    v = lcg.next()
    if ctype is ctypes.c_uint8:
        return v & 0xFF
    if ctype is ctypes.c_uint16:
        return v & 0xFFFF
    if ctype is ctypes.c_uint32:
        return v
    if ctype is ctypes.c_int16:
        v &= 0xFFFF
        return v - 0x10000 if v >= 0x8000 else v
    raise AssertionError(f"unexpected field type {ctype}")


class WireFixture(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        gcc = shutil.which("gcc") or shutil.which("cc")
        if gcc is None:
            raise unittest.SkipTest("no C compiler available")
        src = os.path.join(ROOT, "firmware", "common", "test", "gen_wire_fixture.c")
        inc = os.path.join(ROOT, "firmware", "common", "include")
        exe = os.path.join(HERE, "_wire_fixture_bin")
        subprocess.check_call([gcc, "-std=c11", "-Wall", "-Wextra", "-Werror",
                               "-I", inc, src, "-o", exe])
        try:
            out = subprocess.check_output([exe]).decode()
        finally:
            os.remove(exe)
        cls.sizes = {}
        cls.macros = {}
        cls.dumps = {}
        cls.frame_ping = None
        cls.crc16 = None
        cls.dump_order = []
        for ln in out.splitlines():
            parts = ln.split()
            if not parts:
                continue
            if parts[0] == "SIZEOF":
                cls.sizes[parts[1]] = int(parts[2])
            elif parts[0] == "MACRO":
                cls.macros[parts[1]] = int(parts[2])
            elif parts[0] == "FRAME_PING":
                cls.frame_ping = bytes.fromhex(parts[1])
            elif parts[0] == "CRC16":
                cls.crc16 = int(parts[1], 16)
            else:
                cls.dumps[parts[0]] = bytes.fromhex(parts[1])
                cls.dump_order.append(parts[0])

    def test_sizes(self):
        self.assertEqual(set(self.sizes), set(MIRRORS))
        for name, sz in self.sizes.items():
            self.assertEqual(ctypes.sizeof(MIRRORS[name]), sz, name)

    def test_truncation_macros(self):
        for n in range(P.MAX_SLAVES + 1):
            self.assertEqual(self.macros[f"CMD_MASTER_SIZE_{n}"],
                             P.cmd_master_size(n))
            self.assertEqual(self.macros[f"TELE_MASTER_SIZE_{n}"],
                             P.tele_master_size(n))
        # 138 = sizeof(spi_tele_frame_t); fixed by the SPI DMA setup.
        self.assertEqual(self.macros["SPI_XFER_SIZE"], 138)

    def test_field_layout_lcg_replay(self):
        self.assertEqual(set(self.dumps), set(MIRRORS))
        lcg = _Lcg()
        # Replay in the exact order the C program dumped (one shared stream).
        for name in self.dump_order:
            obj = MIRRORS[name]()
            _fill(obj, lcg)
            self.assertEqual(bytes(obj).hex(), self.dumps[name].hex(),
                             f"{name}: C bytes != ctypes bytes")

    def test_frame_and_crc(self):
        self.assertIsNotNone(self.frame_ping)
        want = P.encode_frame(P.USB_MSG_PING, b"\x11\x22\x33", seq=0xBEEF)
        self.assertEqual(self.frame_ping.hex(), want.hex())
        self.assertEqual(self.crc16, P.crc16(bytes([1, 2, 3, 4, 5])))
        # And the frame decodes back cleanly.
        buf = bytearray(self.frame_ping)
        mt, seq, pl, consumed = P.decode_frame(buf)
        self.assertEqual((mt, seq, pl, consumed),
                         (P.USB_MSG_PING, 0xBEEF, b"\x11\x22\x33", len(self.frame_ping)))


if __name__ == "__main__":
    unittest.main()
