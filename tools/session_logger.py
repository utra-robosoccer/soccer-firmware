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
  host_ts  epoch seconds (float, host clock at log time)
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
import threading
import time
from datetime import datetime

FIELDS = ["host_ts", "kind", "motor", "slave", "local", "opcode",
          "state", "cause", "flags", "pos", "vel", "tau", "kp", "kd"]


class SessionLogger:
    def __init__(self, log_dir="logs", flush_interval=1.0, clock=time.time):
        os.makedirs(log_dir, exist_ok=True)
        fname = datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + ".csv"
        self.path = os.path.join(log_dir, fname)
        self._clock = clock
        self._flush_interval = flush_interval
        self._buf = []
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
                 slave=None, local=None, host_ts=None):
        self._append({
            "host_ts": self._clock() if host_ts is None else host_ts,
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
            self._buf.append(row)

    # ── consumer side (background flusher) ─────────────────────────────────────

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
