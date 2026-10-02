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


def _tele_frame(state=3, last_applied_seq=7):
    motor = dict(pos_raw=5000, vel_raw=-125, tau_raw=30, temp_c=42, state=state,
                 cause=0, motor_mode=2, motor_fault=0, flags=0, fb_age_ms=5,
                 fault_word=0, last_applied_seq=last_applied_seq)
    chains = [dict(chain_id=0, spi_seq_echo=0, slave_time_us=0,
                   cmd_crc_errors=0, can_tx_errors=0, motors=[motor])]
    payload = P.pack_robot_tele(1, 1000, last_applied_seq, 0, 1, chains)
    return P.encode_frame(P.MSG_ROBOT_TELE, P.NODE_MASTER, P.NODE_JETSON, payload)


def _cmd_frame():
    chains = [{"chain_id": 0, "motors": [
        {"mode_req": P.REQ_MIT, "pos": 1.5, "vel": -2.0, "kp": 15.0, "kd": 1.0,
         "tau_ff": 0.0, "flags": P.CMD_FLAG_VALID}]}]
    payload = P.pack_robot_cmd(1, 1, chains)
    return P.encode_frame(P.MSG_ROBOT_CMD, P.NODE_JETSON, P.NODE_MASTER, payload)


class ConvertLog(unittest.TestCase):
    _FILES = ("motor_state.csv", "motor_cmd.csv", "status.csv",
              "events.csv", "loop_timing.csv")

    def _build_log(self, path):
        vf = bytearray(_tele_frame())    # wrong-version copy
        vf[12] = P.PROTO_VERSION + 1
        vf[14] = 0
        vf[15] = 0
        crc = P.crc16(bytes(vf))
        vf[14] = crc & 0xFF
        vf[15] = crc >> 8

        w = BinaryLogWriter(path, {"config_name": "bench-1-motor"}, proto_version=P.PROTO_VERSION)
        w.write(LOG.RX_FRAME, _tele_frame(state=3))
        w.write(LOG.RX_FRAME, P.encode_frame(P.MSG_MASTER_STATUS, P.NODE_MASTER, P.NODE_JETSON,
                                             struct.pack(P.FMT_MASTER_STATUS, 1, 1, 1234, 0, 99,
                                                         200, 200, 200, 50)))
        w.write(LOG.RX_FRAME, P.encode_frame(P.MSG_SLAVE_STATUS, P.NODE_MASTER, P.NODE_JETSON,
                                             struct.pack(P.FMT_SLAVE_STATUS, 0, 1, 1234, 0, 0, 0)))
        w.write(LOG.RX_FRAME, bytes(vf))
        w.write(LOG.TX_FRAME, _cmd_frame())
        w.write(LOG.RX_DISCARD, b"\xde\xad")
        w.write(LOG.EVENT, b"start")
        w.write(LOG.LOOP_TIMING, LOG.LOOP_TIMING_FMT.pack(0, 20_000_000, 100_000, 50_000, 1000))
        w.close()

    def test_convert_folder_and_split(self):
        with tempfile.TemporaryDirectory() as d:
            binp = os.path.join(d, "20-55-09_listen.bin")
            self._build_log(binp)
            folder, c = convert_log.convert(binp)

            self.assertEqual(folder, os.path.join(d, "20-55-09_listen"))
            for name in self._FILES:
                self.assertTrue(os.path.isfile(os.path.join(folder, name)), name)

            self.assertEqual(c, dict(rx=4, tx=1, discard_records=1, discard_bytes=2,
                                     version_frames=1, dropped_records=0, drop_events=0))

            # motor_state: one per-motor telemetry row.
            f, rows = _rows(os.path.join(folder, "motor_state.csv"))
            self.assertEqual(f, convert_log.MOTOR_STATE_FIELDS)
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["slave"], "0")
            self.assertEqual(rows[0]["state_name"], "HOLD")
            self.assertEqual(rows[0]["last_applied_seq"], "7")
            self.assertNotEqual(rows[0]["master_ts_ms"], "")

            # motor_cmd: one per-motor command row.
            f, rows = _rows(os.path.join(folder, "motor_cmd.csv"))
            self.assertEqual(f, convert_log.MOTOR_CMD_FIELDS)
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["mode_name"], "MIT")
            self.assertAlmostEqual(float(rows[0]["pos"]), 1.5, places=3)
            self.assertAlmostEqual(float(rows[0]["kp"]), 15.0, places=1)
            self.assertEqual(rows[0]["cmd_seq"], "1")

            _, srows = _rows(os.path.join(folder, "status.csv"))
            self.assertEqual({r["type"] for r in srows}, {"ROBOT", "MASTER", "SLAVE"})
            master = next(r for r in srows if r["type"] == "MASTER")
            self.assertEqual(master["slave_tick_hz"], "200")
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
            for name in self._FILES:
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
