"""BinaryLogWriter — background, non-blocking binary log writer.

write() only enqueues; a daemon thread does the file I/O, so the serial hot path
never blocks on disk. The queue is bounded: on overflow the record is dropped and
counted (warn once) rather than blocking the caller. Flushes periodically and on
close().
"""
import json
import os
import queue
import sys
import threading
import time

from . import format as fmt


class BinaryLogWriter:
    def __init__(self, path: str, meta: dict, *, proto_version: int,
                 queue_max: int = 100_000, flush_interval: float = 1.0):
        self.path = path
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        self._flush_interval = flush_interval
        self._q: "queue.Queue[bytes | None]" = queue.Queue(maxsize=queue_max)
        self._dropped = 0
        self._warned = False
        self._lock = threading.Lock()
        # Pending (not-yet-recorded) drop window; the consumer flushes it in-band as a
        # LOG_DROP record once the queue has space, so completeness is provable offline.
        self._drop_pending = 0
        self._drop_first_ns = 0
        self._drop_last_ns = 0

        self._file = open(path, "wb")
        meta_bytes = json.dumps(meta, separators=(",", ":"), sort_keys=True).encode("utf-8")
        self._file.write(fmt.pack_header(proto_version, time.time_ns(),
                                         time.monotonic_ns(), meta_bytes))
        self._file.flush()

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="datalog-writer", daemon=True)
        self._thread.start()

    # ── producer side (serial threads / runner) ───────────────────────────────
    def write(self, kind: int, payload: bytes, ts_ns: int | None = None) -> None:
        rec = fmt.pack_record(kind, time.monotonic_ns() if ts_ns is None else ts_ns, payload)
        try:
            self._q.put_nowait(rec)
        except queue.Full:
            now = time.monotonic_ns()
            with self._lock:
                self._dropped += 1
                if self._drop_pending == 0:
                    self._drop_first_ns = now
                self._drop_last_ns = now
                self._drop_pending += 1
                if not self._warned:
                    self._warned = True
                    sys.stderr.write(
                        f"BinaryLogWriter: queue full ({self._q.maxsize}) — disk too slow; "
                        f"dropping records (recorded as LOG_DROP in the log).\n")

    @property
    def dropped(self) -> int:
        with self._lock:
            return self._dropped

    # ── consumer side ─────────────────────────────────────────────────────────
    def _emit_drop_record(self) -> None:
        """Write a LOG_DROP record for any pending drops (consumer thread only, so it
        writes straight to the file and can't itself be dropped)."""
        with self._lock:
            if self._drop_pending == 0:
                return
            cnt, first, last = self._drop_pending, self._drop_first_ns, self._drop_last_ns
            self._drop_pending = 0
        self._file.write(fmt.pack_record(
            fmt.LOG_DROP, time.monotonic_ns(),
            fmt.LOG_DROP_FMT.pack(cnt, first, last)))

    def _run(self) -> None:
        next_flush = time.monotonic() + self._flush_interval
        while True:
            try:
                rec = self._q.get(timeout=self._flush_interval)
            except queue.Empty:
                rec = None
            if rec is None:
                if self._stop.is_set() and self._q.empty():
                    break
            else:
                self._file.write(rec)
            self._emit_drop_record()   # queue just drained → space exists; record any drops
            now = time.monotonic()
            if now >= next_flush:
                self._file.flush()
                next_flush = now + self._flush_interval

    def close(self) -> None:
        if self._stop.is_set():
            return
        self._stop.set()
        self._q.put(None)          # wake the thread so it can observe stop + drain
        self._thread.join(timeout=5.0)
        # Drain anything still queued (thread may have exited on the sentinel).
        try:
            while True:
                rec = self._q.get_nowait()
                if rec is not None:
                    self._file.write(rec)
        except queue.Empty:
            pass
        self._emit_drop_record()   # record any drops that never got a chance to flush
        self._file.flush()
        self._file.close()
