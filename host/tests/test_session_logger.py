"""Unit tests for tools/session_logger.SessionLogger.

Verifies: one CSV per session created on construction, telemetry + command rows
written with the right schema/values, background flush happens without close(), and
close() flushes remaining buffered rows. Uses a deterministic clock and a tempdir.
"""
import csv
import os
import sys
import tempfile
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))   # host/tests → host → repo root

from master_link.session_logger import SessionLogger, FIELDS  # noqa: E402 (pip install -e host/)


def _read(path):
    with open(path, newline="") as fh:
        return list(csv.DictReader(fh))


class SessionLoggerTest(unittest.TestCase):
    def test_creates_csv_with_header_on_construction(self):
        with tempfile.TemporaryDirectory() as d:
            lg = SessionLogger(log_dir=d, flush_interval=10.0, clock=lambda: 1000.0)
            try:
                self.assertTrue(os.path.isfile(lg.path))
                self.assertTrue(lg.path.endswith(".csv"))
                # One folder per day: <tmpdir>/YYYY-MM-DD/HH-MM-SS.csv
                self.assertEqual(os.path.dirname(os.path.dirname(lg.path)), d)
                self.assertRegex(os.path.basename(os.path.dirname(lg.path)),
                                 r"^\d{4}-\d{2}-\d{2}$")
                with open(lg.path, newline="") as fh:
                    header = next(csv.reader(fh))
                self.assertEqual(header, FIELDS)
            finally:
                lg.close()

    def test_tele_and_cmd_rows_roundtrip(self):
        with tempfile.TemporaryDirectory() as d:
            lg = SessionLogger(log_dir=d, flush_interval=10.0, clock=lambda: 42.5)
            lg.log_tele(motor=2, state=7, cause=1, flags=0x0A,
                        pos=0.5, vel=-1.25, tau=0.3, slave=0, local=2,
                        master_ts_ms=123456)
            lg.log_cmd("MIT", motor=1, slave=0, local=1,
                       pos=1.0, vel=2.0, kp=15.0, kd=1.0, tau=0.0)
            lg.log_cmd("ARM_HOLD", motor=0, slave=0, local=0)
            lg.close()

            rows = _read(lg.path)
            self.assertEqual(len(rows), 3)

            t, c, a = rows
            self.assertEqual(t["kind"], "T")
            self.assertEqual(t["motor"], "2")
            self.assertEqual(t["state"], "7")
            self.assertEqual(t["cause"], "1")
            self.assertEqual(t["flags"], "10")          # 0x0A
            self.assertEqual(float(t["pos"]), 0.5)
            self.assertEqual(float(t["tau"]), 0.3)
            self.assertEqual(t["host_ts"], "42.5")
            self.assertEqual(t["master_ts_ms"], "123456")
            self.assertEqual(t["opcode"], "")           # blank for telemetry
            self.assertEqual(t["kp"], "")

            self.assertEqual(c["master_ts_ms"], "")     # blank for commands

            self.assertEqual(c["kind"], "C")
            self.assertEqual(c["opcode"], "MIT")
            self.assertEqual(float(c["kp"]), 15.0)
            self.assertEqual(c["state"], "")            # blank for command

            self.assertEqual(a["opcode"], "ARM_HOLD")
            self.assertEqual(a["pos"], "")              # control cmd carries no MIT params

    def test_background_flush_without_close(self):
        with tempfile.TemporaryDirectory() as d:
            lg = SessionLogger(log_dir=d, flush_interval=0.05, clock=lambda: 1.0)
            try:
                lg.log_tele(motor=0, state=2, cause=0, flags=0,
                            pos=0.0, vel=0.0, tau=0.0)
                # Wait for the background flusher (not close()) to write it out.
                deadline = time.monotonic() + 2.0
                rows = []
                while time.monotonic() < deadline:
                    rows = _read(lg.path)
                    if rows:
                        break
                    time.sleep(0.02)
                self.assertEqual(len(rows), 1)
                self.assertEqual(rows[0]["kind"], "T")
            finally:
                lg.close()

    def test_buffer_cap_drops_and_counts(self):
        with tempfile.TemporaryDirectory() as d:
            # flush_interval huge so the flusher never runs during the test → the
            # cap is exercised purely in-buffer.
            lg = SessionLogger(log_dir=d, flush_interval=10_000.0, max_rows=2)
            try:
                for _ in range(5):
                    lg.log_cmd("PING")
                self.assertEqual(lg.dropped, 3)          # 2 kept, 3 dropped
            finally:
                lg.close()
            self.assertEqual(len(_read(lg.path)), 2)     # only the buffered 2 written

    def test_close_is_idempotent(self):
        with tempfile.TemporaryDirectory() as d:
            lg = SessionLogger(log_dir=d, flush_interval=10.0)
            lg.log_cmd("PING")
            lg.close()
            lg.close()   # must not raise
            self.assertEqual(len(_read(lg.path)), 1)


if __name__ == "__main__":
    unittest.main()
