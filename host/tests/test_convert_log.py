"""convert_log: binary log -> session folder of split CSVs."""
import csv
import os
import struct
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "host", "analysis"))  # convert_log (not a package)

import convert_log            # noqa: E402
import master_link.protocol as P            # noqa: E402
from master_link.datalog import BinaryLogWriter  # noqa: E402
from master_link.datalog import format as LOG    # noqa: E402


def _rows(path):
    with open(path, newline="") as f:
        r = csv.DictReader(f)
        return r.fieldnames, list(r)


class ConvertLog(unittest.TestCase):
    def _build_log(self, path):
        atom = struct.pack(P.MOTORSTATE_FMT, 32768, 40000, 30000, 42, 3, 0, 0, 0, 5, 0)
        ms = P.encode_frame(P.MSG_MOTOR_STATE, P.NODE_MASTER, P.NODE_JETSON,
                            struct.pack(P.FMT_MOTOR_STATE_HDR, 0, 0) + atom)
        vf = bytearray(ms)               # wrong-version copy
        vf[12] = P.PROTO_VERSION + 1
        vf[14] = 0
        vf[15] = 0
        crc = P.crc16(bytes(vf))
        vf[14] = crc & 0xFF
        vf[15] = crc >> 8

        w = BinaryLogWriter(path, {"config_name": "bench-1-chain"}, proto_version=P.PROTO_VERSION)
        w.write(LOG.RX_FRAME, ms)
        w.write(LOG.RX_FRAME, P.encode_frame(P.MSG_MASTER_STATUS, P.NODE_MASTER, P.NODE_JETSON,
                                             struct.pack(P.FMT_MASTER_STATUS, 1, 1, 1234, 0, 99)))
        w.write(LOG.RX_FRAME, P.encode_frame(P.MSG_SLAVE_STATUS, P.NODE_MASTER, P.NODE_JETSON,
                                             struct.pack(P.FMT_SLAVE_STATUS, 0, 1, 1234, 0, 0, 0)))
        w.write(LOG.RX_FRAME, P.encode_frame(P.MSG_CONTROL_RESP, P.NODE_MASTER, P.NODE_JETSON,
                                             struct.pack(P.FMT_CONTROL_RESP, 0, 0, 1, 0, 3, 7)))
        w.write(LOG.RX_FRAME, bytes(vf))
        w.write(LOG.TX_FRAME, P.encode_frame(P.MSG_MOTOR_CMD, P.NODE_JETSON, P.NODE_MASTER,
                                             struct.pack(P.FMT_MOTOR_CMD, 0, 1, 1.5, -2.0, 15.0, 1.0, 0.0)))
        w.write(LOG.TX_FRAME, P.encode_frame(P.MSG_CONTROL_REQ, P.NODE_JETSON, P.NODE_MASTER,
                                             struct.pack(P.FMT_CONTROL_REQ, 0, 0, P.CTRL_ARM_HOLD, 0)))
        w.write(LOG.RX_DISCARD, b"\xde\xad")
        w.write(LOG.EVENT, b"start")
        w.write(LOG.LOOP_TIMING, LOG.LOOP_TIMING_FMT.pack(0, 20_000_000, 100_000, 50_000, 1000))
        w.close()

    def test_convert_folder_and_split(self):
        with tempfile.TemporaryDirectory() as d:
            binp = os.path.join(d, "20-55-09_listen.bin")
            self._build_log(binp)
            folder, c = convert_log.convert(binp)

            # Folder named after the .bin, next to it, with all six files.
            self.assertEqual(folder, os.path.join(d, "20-55-09_listen"))
            for name in ("motor_state.csv", "motor_cmd.csv", "status.csv",
                         "control_resp.csv", "events.csv", "loop_timing.csv"):
                self.assertTrue(os.path.isfile(os.path.join(folder, name)), name)

            self.assertEqual(c, dict(rx=5, tx=2, discard_records=1, discard_bytes=2,
                                     version_frames=1))

            # motor_state: telemetry only, its own columns (no 'kind').
            f, rows = _rows(os.path.join(folder, "motor_state.csv"))
            self.assertEqual(f, convert_log.MOTOR_STATE_FIELDS)
            self.assertNotIn("kind", f)
            self.assertNotIn("opcode", f)
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["slave"], "0")
            self.assertEqual(rows[0]["state"], "3")
            self.assertNotEqual(rows[0]["master_ts_ms"], "")
            self.assertNotEqual(rows[0]["host_ts"], "")

            # motor_cmd: commands only.
            f, rows = _rows(os.path.join(folder, "motor_cmd.csv"))
            self.assertEqual(f, convert_log.MOTOR_CMD_FIELDS)
            self.assertNotIn("kind", f)
            self.assertEqual(len(rows), 2)
            mit = next(r for r in rows if r["opcode"] == "MIT")
            self.assertAlmostEqual(float(mit["pos"]), 1.5, places=5)
            self.assertAlmostEqual(float(mit["kp"]), 15.0, places=5)
            self.assertAlmostEqual(float(mit["tau_ff"]), 0.0, places=5)
            self.assertTrue(any(r["opcode"] == "ARM_HOLD" for r in rows))

            _, srows = _rows(os.path.join(folder, "status.csv"))
            self.assertEqual({r["type"] for r in srows}, {"MASTER", "SLAVE"})
            _, crows = _rows(os.path.join(folder, "control_resp.csv"))
            self.assertEqual(len(crows), 1)
            _, erows = _rows(os.path.join(folder, "events.csv"))
            self.assertEqual({r["kind"] for r in erows}, {"EVENT", "VERSION_MISMATCH"})
            _, lrows = _rows(os.path.join(folder, "loop_timing.csv"))
            self.assertEqual(len(lrows), 1)
            self.assertAlmostEqual(float(lrows[0]["period_ms"]), 20.0, places=3)

    def test_empty_files_always_written(self):
        with tempfile.TemporaryDirectory() as d:
            binp = os.path.join(d, "empty_listen.bin")
            BinaryLogWriter(binp, {}, proto_version=P.PROTO_VERSION).close()
            folder, c = convert_log.convert(binp)
            for name in ("motor_state.csv", "motor_cmd.csv", "status.csv",
                         "control_resp.csv", "events.csv", "loop_timing.csv"):
                fields, rows = _rows(os.path.join(folder, name))
                self.assertEqual(rows, [])           # header-only
                self.assertTrue(fields)              # but header present

    def test_rerun_overwrites_cleanly(self):
        with tempfile.TemporaryDirectory() as d:
            binp = os.path.join(d, "20-55-09_listen.bin")
            self._build_log(binp)
            folder, _ = convert_log.convert(binp)
            stale = os.path.join(folder, "stale.csv")
            with open(stale, "w") as fh:
                fh.write("junk\n")
            convert_log.convert(binp)                # re-run
            self.assertFalse(os.path.exists(stale), "stale file survived re-convert")


if __name__ == "__main__":
    unittest.main()
