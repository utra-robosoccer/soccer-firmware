"""Round-trip + overflow tests for master_link.datalog."""
import os
import tempfile
import time
import unittest

from master_link.datalog import BinaryLogWriter, BinaryLogReader, read_header
from master_link.datalog import format as fmt


class DatalogRoundTrip(unittest.TestCase):
    def test_header_and_all_record_kinds(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "day", "sess.bin")   # nested → exercises makedirs
            meta = {"git_commit": "abc123", "git_dirty": True,
                    "config_name": "1s_5m", "config_hash": "deadbeef",
                    "producer": "listen", "config_yaml": {"slave0.yaml": "slave: slave0\n"}}
            w = BinaryLogWriter(path, meta, proto_version=1)
            recs = [
                (fmt.RX_FRAME,    b"\x01\x02raw-rx"),
                (fmt.TX_FRAME,    b"\x07tx-bytes"),
                (fmt.RX_DISCARD,  b"\xde\xad\xbe\xef"),
                (fmt.EVENT,       b"start"),
                (fmt.ANNOTATION,  "moved joint by hand".encode()),
                (fmt.LOOP_TIMING, fmt.LOOP_TIMING_FMT.pack(3, 20_000_000, 120_000, 80_000, -5000)),
            ]
            for kind, payload in recs:
                w.write(kind, payload)
            w.close()
            self.assertEqual(w.dropped, 0)

            r = BinaryLogReader(path)
            self.assertEqual(r.header["proto_version"], 1)
            self.assertEqual(r.header["log_fmt_version"], fmt.LOG_FMT_VERSION)
            self.assertEqual(r.header["meta"]["config_name"], "1s_5m")
            self.assertEqual(r.header["meta"]["config_yaml"]["slave0.yaml"], "slave: slave0\n")
            self.assertGreater(r.header["wall_start_ns"], 0)

            got = [(rec.kind, rec.payload) for rec in r]
            self.assertEqual(got, recs)
            # timestamps are monotonic and non-decreasing
            ts = [rec.ts_ns for rec in BinaryLogReader(path)]
            self.assertEqual(ts, sorted(ts))

    def test_loop_timing_decode(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "s.bin")
            w = BinaryLogWriter(path, {}, proto_version=1)
            w.write(fmt.LOOP_TIMING, fmt.LOOP_TIMING_FMT.pack(42, 20_000_000, 1000, 2000, -300))
            w.close()
            rec = next(iter(BinaryLogReader(path)))
            seq, period, step, send, late = fmt.LOOP_TIMING_FMT.unpack(rec.payload)
            self.assertEqual((seq, period, step, send, late), (42, 20_000_000, 1000, 2000, -300))

    def test_queue_overflow_drops_and_stays_valid(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "s.bin")
            # Tiny queue + a stalled writer thread would be racy; instead flood fast
            # with a tiny cap so put_nowait overflows before the drainer keeps up.
            w = BinaryLogWriter(path, {}, proto_version=1, queue_max=8, flush_interval=100.0)
            for i in range(5000):
                w.write(fmt.EVENT, b"x" * 32)
            w.close()
            # Some may have dropped; file must still be a valid, readable log, and every
            # dropped record must be self-documented in-band as LOG_DROP records.
            events = drops = drop_recorded = 0
            for rec in BinaryLogReader(path):
                if rec.kind == fmt.EVENT:
                    events += 1
                elif rec.kind == fmt.LOG_DROP:
                    drops += 1
                    cnt, first_ns, last_ns = fmt.LOG_DROP_FMT.unpack(rec.payload)
                    drop_recorded += cnt
                    self.assertLessEqual(first_ns, last_ns)
            self.assertEqual(events + w.dropped, 5000)     # nothing silently vanished
            self.assertEqual(drop_recorded, w.dropped)     # every drop is in the .bin
            if w.dropped:
                self.assertGreater(drops, 0)


if __name__ == "__main__":
    unittest.main()
