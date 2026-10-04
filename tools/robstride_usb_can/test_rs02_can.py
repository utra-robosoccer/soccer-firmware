"""Unit tests for the parameter (Type-17/18/22) codec in rs02_can.

Run from this directory:  python3 -m unittest test_rs02_can
"""
import struct
import unittest

from rs02_can import (
    COMM_READ_PARAM, COMM_WRITE_PARAM, COMM_SAVE, MASTER_ID,
    encode_read_param, encode_write_param, encode_save, decode_param,
    make_id, parse_id,
)


class ParamCodec(unittest.TestCase):
    def test_read_param_frame(self):
        arb, pl = encode_read_param(1, 0x7029)
        mode, data, node = parse_id(arb)
        self.assertEqual(mode, COMM_READ_PARAM)
        self.assertEqual(node, 1)
        self.assertEqual(data, MASTER_ID)
        self.assertEqual((pl[0], pl[1]), (0x29, 0x70))   # reg LE
        self.assertEqual(pl[2:], bytes(6))

    def test_write_param_int(self):
        arb, pl = encode_write_param(1, 0x7029, 1)
        mode, _data, node = parse_id(arb)
        self.assertEqual(mode, COMM_WRITE_PARAM)
        self.assertEqual(node, 1)
        self.assertEqual((pl[0], pl[1]), (0x29, 0x70))
        self.assertEqual(struct.unpack("<I", pl[4:8])[0], 1)

    def test_write_param_float(self):
        _arb, pl = encode_write_param(2, 0x2024, 1.5, as_float=True)
        self.assertEqual((pl[0], pl[1]), (0x24, 0x20))
        self.assertAlmostEqual(struct.unpack("<f", pl[4:8])[0], 1.5, places=6)

    def test_save_frame(self):
        arb, pl = encode_save(3)
        mode, data, node = parse_id(arb)
        self.assertEqual(mode, COMM_SAVE)
        self.assertEqual(node, 3)
        self.assertEqual(data, MASTER_ID)
        self.assertEqual(pl, bytes(8))

    def test_decode_param_roundtrip(self):
        # a Type-17 reply for reg 0x7029, value 1 (reg in [0:2], value in [4:8])
        data = struct.pack("<H", 0x7029) + bytes(2) + struct.pack("<I", 1)
        arb = make_id(COMM_READ_PARAM, node_id=MASTER_ID, data=0)
        pv = decode_param(arb, data)
        self.assertIsNotNone(pv)
        self.assertEqual(pv.reg, 0x7029)
        self.assertEqual(pv.as_int(), 1)

    def test_decode_param_float_view(self):
        data = struct.pack("<H", 0x2024) + bytes(2) + struct.pack("<f", 2.5)
        pv = decode_param(make_id(COMM_READ_PARAM, node_id=MASTER_ID, data=0), data)
        self.assertAlmostEqual(pv.as_float(), 2.5, places=6)

    def test_decode_param_wrong_mode(self):
        self.assertIsNone(decode_param(make_id(2, node_id=1, data=0), bytes(8)))

    def test_decode_param_short(self):
        self.assertIsNone(decode_param(make_id(COMM_READ_PARAM, node_id=1, data=0), bytes(4)))


if __name__ == "__main__":
    unittest.main()
