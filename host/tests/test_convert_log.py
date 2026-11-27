"""convert_log: binary log -> session folder of split CSVs (protocol_wire era)."""
import csv
import os
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


def _tele_frame(state=P.MOTOR_DISABLED, last_applied_seq=7, imu_seq=1):
    tm = P.TeleMaster()
    tm.cycle_id = 1
    tm.time_master_us = 1000
    tm.cmd_seq_active = last_applied_seq
    tm.robot_state = P.ROBOT_READY
    tm.n_slaves = 1
    tm.status.host.last_cmd_seq_rx = last_applied_seq
    tm.status.usb.rx_frames = 99
    tm.status.spi[0].alive = 1
    tm.imu.seq = imu_seq
    tm.imu.accel[0] = 100
    tm.imu.gyro[2] = -50
    sl = tm.slaves[0]
    sl.state = P.SLAVE_READY
    sl.n_motors = 1
    sl.time_slave_us = 900
    mo = sl.motors[0]
    mo.state = state
    mo.last_applied_seq = last_applied_seq
    mo.rs.pos = 5000
    mo.rs.vel = -125
    mo.rs.tau = 30
    mo.rs.temp = 420
    mo.rs.mode = P.RS_MODE_RUN
    return P.encode_frame(P.USB_MSG_TELE, P.pack_tele_master(tm))


def _cmd_frame():
    slaves = [{"motors": [
        {"req": P.MOTOR_REQ_ENABLE, "pos": 1.5, "vel": -2.0, "kp": 15.0, "kd": 1.0,
         "tau_ff": 0.0}]}]
    return P.encode_frame(P.USB_MSG_CMD, P.pack_cmd_master(1, 1, slaves))


class ConvertLog(unittest.TestCase):
    _FILES = ("motor_state.csv", "motor_cmd.csv", "robot.csv", "slaves.csv",
              "imu.csv", "events.csv", "loop_timing.csv")

    def _build_log(self, path):
        w = BinaryLogWriter(path, {"config_name": "1s_1m"})
        w.write(LOG.RX_FRAME, _tele_frame(state=P.MOTOR_DISABLED, imu_seq=1))
        w.write(LOG.RX_FRAME, _tele_frame(state=P.MOTOR_DISABLED, imu_seq=1))  # imu deduped
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

            self.assertEqual(c, dict(rx=2, tx=1, discard_records=1, discard_bytes=2,
                                     dropped_records=0, drop_events=0))

            # motor_state: one per-motor telemetry row per frame.
            f, rows = _rows(os.path.join(folder, "motor_state.csv"))
            self.assertEqual(f, convert_log.MOTOR_STATE_FIELDS)
            self.assertEqual(len(rows), 2)
            self.assertEqual(rows[0]["slave"], "0")
            self.assertEqual(rows[0]["state_name"], "DISABLED")
            self.assertEqual(rows[0]["rs_mode_name"], "RUN")
            self.assertEqual(rows[0]["last_applied_seq"], "7")
            self.assertAlmostEqual(float(rows[0]["pos"]), 0.5, places=3)
            self.assertAlmostEqual(float(rows[0]["temp"]), 42.0, places=1)
            self.assertNotEqual(rows[0]["master_time_s"], "")

            # motor_cmd: one row per motor slot of the commanded slave.
            f, rows = _rows(os.path.join(folder, "motor_cmd.csv"))
            self.assertEqual(f, convert_log.MOTOR_CMD_FIELDS)
            self.assertEqual(len(rows), P.MAX_MOTORS_PER_SLAVE)
            self.assertEqual(rows[0]["req_name"], "ENABLE")
            self.assertEqual(rows[1]["req_name"], "STOP")   # unspecified slot
            self.assertAlmostEqual(float(rows[0]["pos"]), 1.5, places=3)
            self.assertAlmostEqual(float(rows[0]["kp"]), 15.0, places=1)
            self.assertEqual(rows[0]["cmd_seq"], "1")

            _, rrows = _rows(os.path.join(folder, "robot.csv"))
            self.assertEqual(len(rrows), 2)
            self.assertEqual(rrows[0]["robot_state_name"], "READY")
            self.assertEqual(rrows[0]["usb_rx_frames"], "99")

            _, srows = _rows(os.path.join(folder, "slaves.csv"))
            self.assertEqual(len(srows), 2)
            self.assertEqual(srows[0]["state_name"], "READY")
            self.assertEqual(srows[0]["alive"], "1")

            # imu: identical imu.seq across the two frames → one row.
            _, irows = _rows(os.path.join(folder, "imu.csv"))
            self.assertEqual(len(irows), 1)
            self.assertEqual(irows[0]["ax_raw"], "100")
            self.assertEqual(irows[0]["gz_raw"], "-50")

            _, erows = _rows(os.path.join(folder, "events.csv"))
            self.assertEqual({r["kind"] for r in erows}, {"EVENT"})
            _, lrows = _rows(os.path.join(folder, "loop_timing.csv"))
            self.assertEqual(len(lrows), 1)
            self.assertAlmostEqual(float(lrows[0]["period_ms"]), 20.0, places=3)

    def test_empty_files_always_written(self):
        with tempfile.TemporaryDirectory() as d:
            binp = os.path.join(d, "empty_listen.bin")
            BinaryLogWriter(binp, {}).close()
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

    def test_v1_log_refused_with_pointer(self):
        """A pre-revamp (format v1) log is refused with a message naming the
        commit to convert it with — not silently misdecoded."""
        import struct as _s
        with tempfile.TemporaryDirectory() as d:
            binp = os.path.join(d, "old.bin")
            meta = b'{"git_commit":"cafe1234"}'
            with open(binp, "wb") as fh:
                fh.write(_s.Struct("<4sHHQQI").pack(b"RLOG", 1, 10, 1, 1, len(meta)))
                fh.write(meta)
            with self.assertRaises(SystemExit) as cm:
                convert_log.convert(binp)
            self.assertIn("cafe1234", str(cm.exception))


if __name__ == "__main__":
    unittest.main()
