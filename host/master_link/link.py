"""MasterLink — the one object that owns the serial port to the master MCU.

Opens the port (auto-detecting the master by USB VID:PID), runs an RX thread that
decodes frames and keeps a thread-safe latest-state store, and exposes a small API
to send MIT / control commands. Binary logging (raw TX/RX frames, discards, events,
loop timing) starts the moment the port opens; nothing on the hot path formats or
decodes for logging — it only enqueues raw bytes to a background writer.
"""
import os
import threading
import time
from dataclasses import dataclass
from datetime import datetime

import serial
from serial.tools import list_ports

from . import protocol as P
from . import config_meta
from . import motor_config_gen as _mc
from .timeutil import U32Unwrapper
from .datalog import BinaryLogWriter
from .datalog import format as LOG

# STM32 USB-CDC "Virtual ComPort" default identifiers.
MASTER_VID = 0x0483
MASTER_PID = 0x5740

# Startup liveness: at port open the master's USB buffer drains a stale burst (old
# master_ts_ms arriving faster than real time), then jumps to the live stream. A motor's
# telemetry is "live" once master_ts_ms advances in step with host monotonic time for
# LIVE_N consecutive MOTOR_STATE frames.
LIVE_N = 5
LIVE_TOL_MS = 3.0


class _LiveDetector:
    """Per motor. Feed (master_ts_ms, recv_ns) for each MOTOR_STATE frame; latches live
    once the master tick advances consistently with host monotonic time for N frames.
    The stale burst (Δhost ≪ Δmaster) and the post-burst jump both reset the counter, so
    live latches only on the genuine live stream. Pure/deterministic for testing."""

    def __init__(self, n: int = LIVE_N, tol_ms: float = LIVE_TOL_MS):
        self._n = n
        self._tol = tol_ms
        self._prev_ms: int | None = None
        self._prev_ns: int | None = None
        self._count = 0
        self.live = False

    def update(self, master_ts_ms: int, recv_ns: int) -> bool:
        if self.live:
            return True
        if self._prev_ms is not None:
            dm = master_ts_ms - self._prev_ms                 # master tick delta (ms)
            dh = (recv_ns - self._prev_ns) / 1e6              # host monotonic delta (ms)
            if 0 < dm <= 200 and abs(dm - dh) <= max(self._tol, 0.5 * dm):
                self._count += 1
                if self._count >= self._n:
                    self.live = True
            else:
                self._count = 0
        self._prev_ms = master_ts_ms
        self._prev_ns = recv_ns
        return self.live


# ── command / state types (owned here; policies import them) ──────────────────
# Mode-request constants mirror protocol.MotorModeReq (re-exported for policies).
MODE_IDLE    = P.REQ_IDLE
MODE_HOLD    = P.REQ_HOLD
MODE_MIT     = P.REQ_MIT
MODE_DAMPED  = P.REQ_DAMPED
MODE_TO_ZERO = P.REQ_TO_ZERO

# Armed lifecycles (tele_motor_t.state): HOLD/MIT/DAMPED/TO_ZERO.
_ARMED_STATES = (P.LIFE_HOLD, P.LIFE_MIT, P.LIFE_DAMPED, P.LIFE_TO_ZERO)


@dataclass
class MotorCommand:
    """One motor's level-triggered mode request + targets for a tick. Built by a
    policy and sent (with every other motor's) as one cmd_robot_t per tick."""
    slave: int
    local: int
    mode: int = P.REQ_IDLE            # MotorModeReq
    pos: float = 0.0
    vel: float = 0.0
    kp: float = 0.0
    kd: float = 0.0
    tau_ff: float = 0.0
    use_config_gains: bool = True     # ignore wire kp/kd, use the slave's config
    fault_reset: bool = False         # clear a latched fault this tick


@dataclass(frozen=True)
class MotorSnap:
    slave: int
    local: int
    gidx: int | None
    state: int          # MotorLifecycle (LIFE_*)
    cause: int          # MotorFaultCause
    pos: float
    vel: float
    tau: float
    temp: float
    motor_mode: int     # RS Type-2 run mode (0 reset/1 cal/2 normal)
    motor_fault: int
    flags: int          # TELE_FLAG_*
    fault_word: int
    fb_age: int
    last_applied_seq: int
    master_ts_ms: int   # MsgHeader.ts_ms (master tick at emit)
    recv_ns: int        # host monotonic_ns at receipt

    @property
    def request_rejected(self) -> bool: return bool(self.flags & P.TELE_FLAG_REQUEST_REJECTED)
    @property
    def to_zero_arrived(self) -> bool: return bool(self.flags & P.TELE_FLAG_TO_ZERO_ARRIVED)
    @property
    def saturated(self) -> bool: return bool(self.flags & P.TELE_FLAG_SATURATED)
    @property
    def state_name(self) -> str: return P.LIFECYCLE_NAMES.get(self.state, f"?{self.state}")
    @property
    def cause_name(self) -> str: return P.CAUSE_NAMES.get(self.cause, f"?{self.cause}")


@dataclass(frozen=True)
class LinkState:
    motors: dict            # (slave, local) -> MotorSnap
    master: dict | None     # robot_state, slave_alive, uptime_ms, link_errors, rx_frames, rates
    slaves: dict            # slave_id -> dict(motors_alive, crc_errors, cmd_crc_errors, seq_gaps)
    robot: dict | None      # tele_robot_t meta: cycle_id, last_cmd_seq_rx, missed_deadlines, ...
    stamp_ns: int

    def armed_motors(self):
        """(slave, local) of motors currently in an armed lifecycle."""
        return [k for k, m in self.motors.items() if m.state in _ARMED_STATES]


class MasterLink:
    def __init__(self, port: str | None = None, *, policy_name: str = "listen",
                 log_dir: str = "logs", baud: int = 115200):
        self.port = port or self.find_master_port()
        if not self.port:
            raise RuntimeError(
                "no master serial port found (looked for USB VID:PID "
                f"{MASTER_VID:04x}:{MASTER_PID:04x}); pass --port explicitly")
        self._ser = serial.Serial(self.port, baud, timeout=0)
        try:
            self._ser.reset_input_buffer()   # flush kernel buffer; live-detect handles the rest
        except (OSError, serial.SerialException):
            pass

        now = datetime.now()
        day = os.path.join(log_dir, now.strftime("%Y-%m-%d"))
        self.log_path = os.path.join(day, now.strftime("%H-%M-%S") + f"_{policy_name}.bin")
        meta = config_meta.build_log_meta(policy_name)
        self._log = BinaryLogWriter(self.log_path, meta, proto_version=P.PROTO_VERSION)

        self._gidx = {(m["slave"], m["idx"]): g for g, m in enumerate(_mc.MOTORS)}

        self._lock = threading.Lock()
        self._motors: dict = {}
        self._master: dict | None = None
        self._slaves: dict = {}
        self._robot: dict | None = None  # latest tele_robot_t meta (cycle_id etc.)
        self._cycle_id = 0               # last master cycle_id seen (echoed in commands)
        self._mtu = U32Unwrapper()       # unwrap the 32-bit master_time_us (wraps ~71 min)
        self._live: dict = {}            # (slave, local) -> _LiveDetector
        self._live_logged = False        # EVENT "live" emitted once, on first live motor

        self._tx_lock = threading.Lock()
        self._rx_frames = 0
        self._tx_frames = 0
        self._tx_errors = 0
        # Monotonic command sequence, one per send_robot_cmd tick. Starts at 1; 0 is
        # the CMD_SEQ_NONE sentinel ("none applied"), so wrap skips it (…65535 → 1).
        self._cmd_seq = 0
        self._discard_records = 0
        self._discard_bytes = 0
        self._version_frames = 0

        self._stop = threading.Event()
        self._rx = threading.Thread(target=self._rx_loop, name="master-link-rx", daemon=True)
        self._rx.start()

    # ── discovery ─────────────────────────────────────────────────────────────
    @staticmethod
    def find_master_port() -> str | None:
        for p in list_ports.comports():
            if p.vid == MASTER_VID and p.pid == MASTER_PID:
                return p.device
        return None

    # ── RX path ───────────────────────────────────────────────────────────────
    def _on_reject(self, reason: str, data: bytes) -> None:
        if reason == "discard":
            with self._lock:
                self._discard_records += 1
                self._discard_bytes += len(data)
            self._log.write(LOG.RX_DISCARD, data)
        elif reason == "version":
            with self._lock:
                self._version_frames += 1
            # Logged as a raw RX_FRAME; convert_log flags wrong-version frames.
            self._log.write(LOG.RX_FRAME, data)

    def _on_frame(self, raw: bytes) -> None:
        with self._lock:
            self._rx_frames += 1
        self._log.write(LOG.RX_FRAME, raw)

    def _rx_loop(self) -> None:
        buf = bytearray()
        while not self._stop.is_set():
            try:
                n = self._ser.in_waiting
                if n:
                    buf.extend(self._ser.read(n))
            except (serial.SerialException, OSError):
                break
            while True:
                r = P.decode_frame(buf, on_reject=self._on_reject, on_frame=self._on_frame)
                if r is None:
                    break
                mt, _seq, ts_ms, pl, consumed = r
                del buf[:consumed]      # decode_frame does not remove accepted frames
                self._ingest(mt, ts_ms, pl, time.monotonic_ns())
            # Poll faster than the telemetry period so the host out-drains the
            # master's ~200 Hz full-robot stream and no standing backlog builds
            # (a 1 ms sleep sat just under the emit rate → ~120 ms of buffering).
            time.sleep(0.0002)

    def _ingest(self, mt: int, ts_ms: int, pl: bytes, recv_ns: int) -> None:
        if mt == P.MSG_ROBOT_TELE:
            d = P.parse_robot_tele(pl)
            if not d:
                return
            log_live = False
            with self._lock:
                self._robot = dict(cycle_id=d["cycle_id"], master_time_us=d["master_time_us"],
                                   master_time_us_mono=self._mtu.update(d["master_time_us"]),
                                   last_cmd_seq_rx=d["last_cmd_seq_rx"],
                                   missed_deadlines=d["missed_deadlines"],
                                   n_chains=d["n_chains"], robot_state=d["robot_state"],
                                   recv_ns=recv_ns)
                self._cycle_id = d["cycle_id"]
                for ch in d["chains"]:
                    sid = ch["chain_id"]
                    for local, mo in enumerate(ch["motors"]):
                        key = (sid, local)
                        self._motors[key] = MotorSnap(
                            slave=sid, local=local, gidx=self._gidx.get(key),
                            state=mo.state, cause=mo.cause, pos=mo.pos, vel=mo.vel,
                            tau=mo.tau, temp=mo.temp, motor_mode=mo.motor_mode,
                            motor_fault=mo.motor_fault, flags=mo.flags,
                            fault_word=mo.fault_word, fb_age=mo.fb_age_ms,
                            last_applied_seq=mo.last_applied_seq,
                            master_ts_ms=ts_ms, recv_ns=recv_ns)
                        det = self._live.get(key)
                        if det is None:
                            det = _LiveDetector()
                            self._live[key] = det
                        if det.update(ts_ms, recv_ns) and not self._live_logged:
                            self._live_logged = True
                            log_live = True
            if log_live:
                self._log.write(LOG.EVENT, b"live")
        elif mt == P.MSG_MASTER_STATUS:
            d = P.parse_master_status(pl)
            if d:
                d["recv_ns"] = recv_ns
                with self._lock:
                    self._master = d
        elif mt == P.MSG_SLAVE_STATUS:
            d = P.parse_slave_status(pl)
            if d:
                with self._lock:
                    self._slaves[d["slave_id"]] = d

    # ── snapshot ──────────────────────────────────────────────────────────────
    def latest_state(self) -> LinkState:
        with self._lock:
            return LinkState(
                motors=dict(self._motors),
                master=dict(self._master) if self._master else None,
                slaves={k: dict(v) for k, v in self._slaves.items()},
                robot=dict(self._robot) if self._robot else None,
                stamp_ns=time.monotonic_ns())

    # ── startup liveness ────────────────────────────────────────────────────────
    def configured_motors(self) -> set:
        """(slave, local) keys of every motor in the active config."""
        return set(self._gidx.keys())

    def live_motors(self) -> set:
        """(slave, local) keys whose telemetry has been confirmed live (see _LiveDetector)."""
        with self._lock:
            return {k for k, d in self._live.items() if d.live}

    def wait_until_live(self, keys=None, timeout: float = 2.0):
        """Block until every key in `keys` (default: all configured motors) is live.
        Returns (ok, missing_set)."""
        keys = set(keys) if keys is not None else self.configured_motors()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if keys <= self.live_motors():
                return True, set()
            time.sleep(0.01)
        return False, keys - self.live_motors()

    # ── TX path ───────────────────────────────────────────────────────────────
    def _write_frame(self, frame: bytes) -> bool:
        """Timestamp just before the write; log TX_FRAME on success, an error EVENT
        on failure. Never logs an unsent frame as sent."""
        ts = time.monotonic_ns()
        with self._tx_lock:
            try:
                self._ser.write(frame)
                # No flush(): pyserial flush()=tcdrain() blocks the caller (the
                # runner thread) until the OS drains the TX buffer. The earlier
                # write starvation was the saturated 714 B telemetry stream, now
                # fixed by the master transmitting only the populated tele prefix
                # (~26 KB/s), so the link isn't saturated and writes deliver promptly.
            except (serial.SerialException, OSError) as e:
                with self._lock:
                    self._tx_errors += 1
                self._log.write(LOG.EVENT, f"error:tx_write:{e}".encode("utf-8"))
                return False
        with self._lock:
            self._tx_frames += 1
        self._log.write(LOG.TX_FRAME, frame, ts_ns=ts)
        return True

    def send_robot_cmd(self, cmds) -> None:
        """Send one MSG_ROBOT_CMD (cmd_robot_t) for this tick from a list of
        MotorCommand. Commands are grouped into per-slave chains (chain_id = slave).

        All motors in one call share a single cmd_seq (one per tick): latency.py
        treats a tick as applied once every commanded motor reports
        last_applied_seq >= it. The counter advances once per call, starting at 1
        and skipping 0 on wrap (0 = CMD_SEQ_NONE). cycle_id echoes the last master
        cycle seen (deadline/apply semantics come with the master-clock task)."""
        cmds = list(cmds)
        if not cmds:
            return
        seq = self._cmd_seq + 1
        if seq > 0xFFFF:
            seq = 1
        self._cmd_seq = seq
        with self._lock:
            cycle_id = self._cycle_id

        # Group by slave → chains, motors ordered by local index.
        by_slave: dict = {}
        for c in cmds:
            by_slave.setdefault(c.slave, []).append(c)
        chains = []
        for sid in sorted(by_slave):
            motors = []
            for c in sorted(by_slave[sid], key=lambda m: m.local):
                flags = P.CMD_FLAG_VALID
                if c.use_config_gains:
                    flags |= P.CMD_FLAG_USE_CONFIG_GAINS
                if c.fault_reset:
                    flags |= P.CMD_FLAG_FAULT_RESET
                motors.append(dict(mode_req=c.mode, pos=c.pos, vel=c.vel,
                                   kp=c.kp, kd=c.kd, tau_ff=c.tau_ff, flags=flags))
            chains.append(dict(chain_id=sid, motors=motors))

        payload = P.pack_robot_cmd(cycle_id, seq, chains)
        self._write_frame(P.encode_frame(P.MSG_ROBOT_CMD, P.NODE_JETSON,
                                         P.NODE_MASTER, payload))

    # ── log helpers (for the runner) ──────────────────────────────────────────
    def log_event(self, text: str) -> None:
        self._log.write(LOG.EVENT, text.encode("utf-8"))

    def log_annotation(self, text: str) -> None:
        self._log.write(LOG.ANNOTATION, text.encode("utf-8"))

    def log_loop_timing(self, seq, period_ns, step_ns, send_ns, lateness_ns) -> None:
        self._log.write(LOG.LOOP_TIMING,
                        LOG.LOOP_TIMING_FMT.pack(seq, period_ns, step_ns, send_ns, lateness_ns))

    # ── counters / lifecycle ──────────────────────────────────────────────────
    @property
    def stats(self) -> dict:
        with self._lock:
            return dict(rx_frames=self._rx_frames, tx_frames=self._tx_frames,
                        tx_errors=self._tx_errors, discard_records=self._discard_records,
                        discard_bytes=self._discard_bytes, version_frames=self._version_frames,
                        log_dropped=self._log.dropped)

    def close(self) -> None:
        self._stop.set()
        self._rx.join(timeout=2.0)
        try:
            self._ser.close()
        except Exception:
            pass
        self._log.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
