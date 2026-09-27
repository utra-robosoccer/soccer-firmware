#!/usr/bin/env python3
"""Per-session CSV logger for the motor dashboard (and a future serial bridge).

Owns ONE CSV per session: <log_dir>/YYYY-MM-DD_HH-MM-SS.csv, created on construction
(i.e. when the dashboard connects). Thread-safe and buffered: log_tele()/log_cmd()
only append to an in-memory list under a short lock; a background thread flushes to
disk every ~flush_interval seconds and on close(). So disk I/O never stalls the
serial read/write loop that calls these.

Deliberately free of any serial or UI dependency (pure stdlib) so it can move into
a standalone bridge process unchanged.

Row schema (one row per telemetry frame and per command sent):
  host_ts       epoch seconds (float, host clock at log time)
  master_ts_ms  MsgHeader.ts_ms (master HAL_GetTick at frame emit); the preferred
                analysis time base for telemetry — monotonic, ms resolution, free
                of host receive jitter. Blank for commands (host-originated). NOTE:
                there is NO per-sample slave tick in the telemetry, so this master
                tick is the closest firmware time base.
  kind     'T' telemetry | 'C' command
  motor    global flattened motor index (int) or ''
  slave    slave id | ''
  local    local motor index within the slave | ''
  opcode   command name for 'C' rows (MIT / ARM_HOLD / GOTO_ZERO / DISABLE / PING)
  state    lifecycle+cause byte's lifecycle (T rows)
  cause    fault cause (T rows)
  flags    motor_fault bits (T rows)
  pos vel tau   measured (T) or commanded (C, MIT)
  kp kd    commanded gains (C, MIT)
"""
import csv
import os
import sys
import threading
import time
from datetime import datetime

FIELDS = ["host_ts", "master_ts_ms", "kind", "motor", "slave", "local", "opcode",
          "state", "cause", "flags", "pos", "vel", "tau", "kp", "kd"]

# Buffer size cap: if the background flusher stalls (slow/full disk), the in-memory
# buffer is bounded here rather than growing without limit. Rows over the cap are
# dropped (newest first, preserving already-buffered order) with a one-time warning.
DEFAULT_MAX_ROWS = 200_000


class SessionLogger:
    def __init__(self, log_dir="logs", flush_interval=1.0, clock=time.time,
                 max_rows=DEFAULT_MAX_ROWS):
        os.makedirs(log_dir, exist_ok=True)
        fname = datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + ".csv"
        self.path = os.path.join(log_dir, fname)
        self._clock = clock
        self._flush_interval = flush_interval
        self._max_rows = max_rows
        self._buf = []
        self._dropped = 0
        self._warned_full = False
        self._lock = threading.Lock()
        self._file = open(self.path, "w", newline="")
        self._writer = csv.DictWriter(self._file, fieldnames=FIELDS, restval="")
        self._writer.writeheader()
        self._file.flush()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="session-logger",
                                        daemon=True)
        self._thread.start()

    # ── producer side (called from serial RX / write threads) ──────────────────

    def log_tele(self, motor, state, cause, flags, pos, vel, tau,
                 slave=None, local=None, host_ts=None, master_ts_ms=None):
        self._append({
            "host_ts": self._clock() if host_ts is None else host_ts,
            "master_ts_ms": master_ts_ms,
            "kind": "T", "motor": motor, "slave": slave, "local": local,
            "state": state, "cause": cause, "flags": flags,
            "pos": pos, "vel": vel, "tau": tau,
        })

    def log_cmd(self, opcode, motor=None, slave=None, local=None,
                pos=None, vel=None, kp=None, kd=None, tau=None, host_ts=None):
        self._append({
            "host_ts": self._clock() if host_ts is None else host_ts,
            "kind": "C", "motor": motor, "slave": slave, "local": local,
            "opcode": opcode, "pos": pos, "vel": vel, "tau": tau, "kp": kp, "kd": kd,
        })

    def _append(self, row):
        with self._lock:
            if len(self._buf) >= self._max_rows:
                self._dropped += 1
                if not self._warned_full:
                    self._warned_full = True
                    sys.stderr.write(
                        f"SessionLogger: buffer cap ({self._max_rows} rows) reached — "
                        f"disk flush stalling; dropping new rows.\n")
                return
            self._buf.append(row)

    # ── consumer side (background flusher) ─────────────────────────────────────

    @property
    def dropped(self):
        """Number of rows dropped due to the buffer cap (0 in normal operation)."""
        with self._lock:
            return self._dropped

    def _run(self):
        # _stop.wait returns True when set → loop exits; False on timeout → flush.
        while not self._stop.wait(self._flush_interval):
            self._flush()

    def _flush(self):
        with self._lock:
            if not self._buf:
                return
            rows, self._buf = self._buf, []
        self._writer.writerows(rows)   # heavy I/O done OUTSIDE the lock
        self._file.flush()

    def close(self):
        """Stop the flusher, write any remaining buffered rows, close the file.
        Safe to call more than once."""
        if self._stop.is_set():
            return
        self._stop.set()
        self._thread.join(timeout=2.0)
        self._flush()
        self._file.close()
