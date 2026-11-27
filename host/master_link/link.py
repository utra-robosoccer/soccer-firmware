"""MasterLink — the one object that owns the serial port to the master MCU.

Opens the port (auto-detecting the master by USB VID:PID), runs an RX thread that
decodes frames and keeps a thread-safe latest-state store, and exposes a small API
to send command frames. Binary logging (raw TX/RX frames, discards, events,
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
# telemetry arriving faster than real time), then jumps to the live stream. The link
# is "live" once time_master_us advances in step with host monotonic time for
# LIVE_N consecutive telemetry frames.
LIVE_N = 5
LIVE_TOL_MS = 3.0


class _LiveDetector:
    """Feed (master_ms, recv_ns) per telemetry frame (master_ms derived from
    tele_master_t.time_master_us); latches live once the master clock advances
    consistently with host monotonic time for N frames. The stale burst
    (Δhost ≪ Δmaster) and the post-burst jump both reset the counter, so live
    latches only on the genuine live stream. Pure/deterministic for testing."""

    def __init__(self, n: int = LIVE_N, tol_ms: float = LIVE_TOL_MS):
        self._n = n
        self._tol = tol_ms
        self._prev_ms: float | None = None
        self._prev_ns: int | None = None
        self._count = 0
        self.live = False

    def update(self, master_ms: float, recv_ns: int) -> bool:
        if self.live:
            return True
        if self._prev_ms is not None:
            dm = master_ms - self._prev_ms                    # master clock delta (ms)
            dh = (recv_ns - self._prev_ns) / 1e6              # host monotonic delta (ms)
            if 0 < dm <= 200 and abs(dm - dh) <= max(self._tol, 0.5 * dm):
                self._count += 1
                if self._count >= self._n:
                    self.live = True
            else:
                self._count = 0
        self._prev_ms = master_ms
        self._prev_ns = recv_ns
        return self.live


# ── command / state types (owned here; policies import them) ──────────────────
REQ_STOP   = P.MOTOR_REQ_STOP
REQ_ENABLE = P.MOTOR_REQ_ENABLE

# Armed motor states (tele_motor_t.state): the motor may produce torque.
_ARMED_STATES = (P.MOTOR_ENABLING, P.MOTOR_ENABLED, P.MOTOR_STOPPING)


@dataclass
class MotorCommand:
    """One motor's level-triggered request + MIT tuple for a tick. The mit is
    applied every cycle while the motor is ENABLED (all-zero = limp), so
    streaming starts with the ENABLE."""
    slave: int
    local: int
    req: int = P.MOTOR_REQ_STOP      # MotorReq
    pos: float = 0.0
    vel: float = 0.0
    kp: float = 0.0
    kd: float = 0.0
    tau_ff: float = 0.0


@dataclass(frozen=True)
class MotorSnap:
    slave: int
    local: int
    gidx: int | None
    state: int          # MotorState
    fault: int          # MotorFault
    flags: int          # MOTOR_FLAG_*
    pos: float
    vel: float
    tau: float
    temp: float
    rs_mode: int        # RsMode (0 reset / 1 cali / 2 run)
    rs_fault_bits: int
    rs_fault_word: int
    last_applied_seq: int
    time_slave_us: int  # slave clock at this motor's last feedback frame
    slave_time_us: int  # slave clock at the SPI exchange (tele_slave_t stamp)
    recv_ns: int        # host monotonic_ns at receipt

    @property
    def state_name(self) -> str: return P.MOTOR_STATE_NAMES.get(self.state, f"?{self.state}")
    @property
    def fault_name(self) -> str: return P.MOTOR_FAULT_NAMES.get(self.fault, f"?{self.fault}")
    @property
    def rs_mode_name(self) -> str: return P.RS_MODE_NAMES.get(self.rs_mode, f"?{self.rs_mode}")
    @property
    def ready(self) -> bool: return bool(self.flags & P.MOTOR_FLAG_READY)
    @property
    def request_rejected(self) -> bool: return bool(self.flags & P.MOTOR_FLAG_REQ_REJECTED)
    @property
    def cmd_stale(self) -> bool: return bool(self.flags & P.MOTOR_FLAG_CMD_STALE)
    @property
    def saturated(self) -> bool: return bool(self.flags & P.MOTOR_FLAG_SATURATED)
    @property
    def fb_age_us(self) -> int:
        """Age of this motor's feedback at the SPI exchange (slave clock, wrap-safe)."""
        return (self.slave_time_us - self.time_slave_us) & 0xFFFFFFFF


@dataclass(frozen=True)
class LinkState:
    motors: dict            # (slave, local) -> MotorSnap
    slaves: dict            # slave -> dict(alive, state, n_motors, time_slave_us,
                            #   boot {...}, spi {...}, can {...})
    status: dict | None     # status_master_t: host {...}, usb {...},
                            #   spi {slave: {...}}, cycle_overruns
    robot: dict | None      # tele meta: cycle_id, time_master_us(+_mono),
                            #   cmd_seq_active, robot_state, n_slaves, imu, recv_ns
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
        # exclusive=True takes a POSIX advisory lock (TIOCEXCL) so nothing else
        # (ModemManager, a second script) can open the port and inject bytes.
        # Blocking read: a small serial timeout lets _rx_loop park on read() and wake the
        # instant a frame lands (no busy-poll), so each tele_master_t is decoded ASAP.
        self._ser = serial.Serial(self.port, baud, timeout=0.1, exclusive=True)
        try:
            self._ser.reset_input_buffer()   # flush kernel buffer; live-detect handles the rest
        except (OSError, serial.SerialException):
            pass

        now = datetime.now()
        day = os.path.join(log_dir, now.strftime("%Y-%m-%d"))
        self.log_path = os.path.join(day, now.strftime("%H-%M-%S") + f"_{policy_name}.bin")
        meta = config_meta.build_log_meta(policy_name)
        self._log = BinaryLogWriter(self.log_path, meta)

        self._gidx = {(m["slave"], m["idx"]): g for g, m in enumerate(_mc.MOTORS)}

        self._lock = threading.Lock()
        self._motors: dict = {}
        self._slaves: dict = {}
        self._status: dict | None = None
        self._robot: dict | None = None  # latest tele_master_t meta (cycle_id etc.)
        self._cycle_id = 0               # last master cycle_id seen (echoed in commands)
        # Telemetry-driven stepping: a monotonic frame counter + a condition the runner
        # waits on (wait_robot). Coalescing is implicit — a waiter gets the NEWEST frame
        # and the count of frames it skipped. Decode time of the latest frame is recorded
        # for the loop-timing budget. Condition shares self._lock.
        self._robot_cond = threading.Condition(self._lock)
        self._robot_frame_no = 0
        self._robot_decode_ns = 0
        self._mtu = U32Unwrapper()       # unwrap the 32-bit time_master_us (wraps ~71 min)
        self._live = _LiveDetector()     # one per link: time_master_us vs host clock
        self._live_logged = False        # EVENT "live" emitted once, on latch

        self._tx_lock = threading.Lock()
        self._rx_frames = 0
        self._tx_frames = 0
        self._tx_errors = 0
        # Monotonic command sequence, one per send_robot_cmd tick. Starts at 1; 0 is
        # the CMD_SEQ_NONE sentinel ("none applied"), so wrap skips it (…65535 → 1).
        self._cmd_seq = 0
        self._discard_records = 0
        self._discard_bytes = 0

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

    def _on_frame(self, raw: bytes) -> None:
        with self._lock:
            self._rx_frames += 1
        self._log.write(LOG.RX_FRAME, raw)

    def _rx_loop(self) -> None:
        buf = bytearray()
        while not self._stop.is_set():
            try:
                # Block on read() (serial timeout) and wake the instant bytes arrive; drain
                # whatever is buffered. No busy-poll, no standing backlog.
                n = self._ser.in_waiting
                chunk = self._ser.read(n if n else 1)
            except (serial.SerialException, OSError):
                break
            if chunk:
                buf.extend(chunk)
            while True:
                r = P.decode_frame(buf, on_reject=self._on_reject, on_frame=self._on_frame)
                if r is None:
                    break
                mt, _seq, pl, consumed = r
                del buf[:consumed]      # decode_frame does not remove accepted frames
                self._ingest(mt, pl, time.monotonic_ns())

    def _ingest(self, mt: int, pl: bytes, recv_ns: int) -> None:
        if mt != P.USB_MSG_TELE:
            return
        t_dec0 = time.monotonic_ns()
        tm = P.parse_tele_master(pl)
        if tm is None:
            return
        log_live = False
        with self._lock:
            self._robot = dict(
                cycle_id=tm.cycle_id, time_master_us=tm.time_master_us,
                time_master_us_mono=self._mtu.update(tm.time_master_us),
                cmd_seq_active=tm.cmd_seq_active, robot_state=tm.robot_state,
                n_slaves=tm.n_slaves,
                imu=dict(accel=list(tm.imu.accel), gyro=list(tm.imu.gyro),
                         seq=tm.imu.seq, flags=tm.imu.flags, errors=tm.imu.errors),
                recv_ns=recv_ns)
            self._cycle_id = tm.cycle_id
            st = tm.status
            self._status = dict(
                host=dict(lost=st.host.lost, last_cmd_seq_rx=st.host.last_cmd_seq_rx,
                          on_time=st.host.on_time, late=st.host.late,
                          missing=st.host.missing, duplicate=st.host.duplicate),
                usb=dict(rx_frames=st.usb.rx_frames, rx_errors=st.usb.rx_errors,
                         rx_resyncs=st.usb.rx_resyncs,
                         rx_discarded_bytes=st.usb.rx_discarded_bytes,
                         tx_drops=st.usb.tx_drops),
                spi={s: dict(alive=st.spi[s].alive, crc_errors=st.spi[s].crc_errors,
                             seq_gaps=st.spi[s].seq_gaps)
                     for s in range(min(tm.n_slaves, P.MAX_SLAVES))},
                cycle_overruns=st.cycle_overruns)
            for sid in range(min(tm.n_slaves, P.MAX_SLAVES)):
                sl = tm.slaves[sid]
                alive = bool(st.spi[sid].alive)
                self._slaves[sid] = dict(
                    alive=alive, state=sl.state, n_motors=sl.n_motors,
                    time_slave_us=sl.time_slave_us,
                    boot=dict(found_mask=sl.status.boot.found_mask,
                              unexpected_id=sl.status.boot.unexpected_id,
                              zero_sta_fail_mask=sl.status.boot.zero_sta_fail_mask,
                              version_fail_mask=sl.status.boot.version_fail_mask),
                    spi=dict(cmd_crc_errors=sl.status.spi.cmd_crc_errors,
                             resyncs=sl.status.spi.resyncs,
                             tx_arm_fails=sl.status.spi.tx_arm_fails,
                             seq_echo=sl.status.spi.seq_echo),
                    can=dict(tx_errors=sl.status.can.tx_errors,
                             tec=sl.status.can.tec, rec=sl.status.can.rec))
                if not alive:
                    continue   # a dead slave's slot is zeroed — keep the last snaps
                nm = min(sl.n_motors, P.MAX_MOTORS_PER_SLAVE)
                for local in range(nm):
                    mo = sl.motors[local]
                    key = (sid, local)
                    self._motors[key] = MotorSnap(
                        slave=sid, local=local, gidx=self._gidx.get(key),
                        state=mo.state, fault=mo.fault, flags=mo.flags,
                        pos=P.dec_i16(mo.rs.pos, P.POS_SCALE),
                        vel=P.dec_i16(mo.rs.vel, P.VEL_SCALE),
                        tau=P.dec_i16(mo.rs.tau, P.TAU_SCALE),
                        temp=P.dec_i16(mo.rs.temp, P.TEMP_SCALE),
                        rs_mode=mo.rs.mode, rs_fault_bits=mo.rs.fault_bits,
                        rs_fault_word=mo.rs.fault_word,
                        last_applied_seq=mo.last_applied_seq,
                        time_slave_us=mo.rs.time_slave_us,
                        slave_time_us=sl.time_slave_us,
                        recv_ns=recv_ns)
            if (self._live.update(tm.time_master_us / 1000.0, recv_ns)
                    and not self._live_logged):
                self._live_logged = True
                log_live = True
            self._robot_frame_no += 1
            self._robot_decode_ns = time.monotonic_ns() - t_dec0
            self._robot_cond.notify_all()
        if log_live:
            self._log.write(LOG.EVENT, b"live")

    # ── snapshot ──────────────────────────────────────────────────────────────
    def _snapshot_locked(self) -> LinkState:
        """Build a LinkState. Caller MUST hold self._lock."""
        return LinkState(
            motors=dict(self._motors),
            slaves={k: dict(v) for k, v in self._slaves.items()},
            status=dict(self._status) if self._status else None,
            robot=dict(self._robot) if self._robot else None,
            stamp_ns=time.monotonic_ns())

    def latest_state(self) -> LinkState:
        with self._lock:
            return self._snapshot_locked()

    def robot_frame_no(self) -> int:
        """Current telemetry frame counter (monotonic; bumped per tele_master_t)."""
        with self._lock:
            return self._robot_frame_no

    def wait_robot(self, after_frame: int, timeout: float):
        """Block until a tele_master_t newer than `after_frame` arrives, then return
        (state, frame_no, skipped, decode_ns) for the NEWEST frame — coalescing, so a
        caller that was busy gets the latest and `skipped` counts the frames it missed.
        Returns None on timeout (treat as a telemetry stall)."""
        deadline = time.monotonic() + timeout
        with self._robot_cond:
            while self._robot_frame_no <= after_frame:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._robot_cond.wait(remaining)
            fno = self._robot_frame_no
            skipped = fno - after_frame - 1 if after_frame >= 0 else 0
            return (self._snapshot_locked(), fno, skipped, self._robot_decode_ns)

    def measure_cycle_hz(self, min_cycles: int = 64, timeout: float = 1.5):
        """Measure the master's ACTUAL cycle rate from telemetry itself:
        time_master_us deltas over cycle_id deltas, across ~min_cycles cycles.
        The rate is its own evidence — no wire-reported constant to trust.
        Returns Hz (float), or None if telemetry stalls before enough cycles
        accumulate."""
        deadline = time.monotonic() + timeout
        fno = self.robot_frame_no()
        t_first = t_last = None
        cycles = 0
        prev_cid = None
        while time.monotonic() < deadline and cycles < min_cycles:
            got = self.wait_robot(fno, max(0.0, deadline - time.monotonic()))
            if got is None:
                break
            state, fno, _skipped, _decode_ns = got
            r = state.robot
            if r is None:
                continue
            cid = r["cycle_id"]
            t = r["time_master_us_mono"]
            if prev_cid is not None:
                cycles += (cid - prev_cid) & 0xFFFF   # u16 wrap-safe
            prev_cid = cid
            if t_first is None:
                t_first = t
            t_last = t
        if t_first is None or cycles < 8 or t_last <= t_first:
            return None
        return 1e6 * cycles / (t_last - t_first)

    # ── startup liveness ────────────────────────────────────────────────────────
    def configured_motors(self) -> set:
        """(slave, local) keys of every motor in the active config."""
        return set(self._gidx.keys())

    def live_motors(self) -> set:
        """(slave, local) keys considered live: the telemetry stream itself is
        confirmed live (see _LiveDetector) AND the motor's slave is answering
        the master's SPI polls (status.spi[slave].alive)."""
        with self._lock:
            if not self._live.live or self._status is None:
                return set()
            spi = self._status["spi"]
            return {k for k in self._gidx
                    if spi.get(k[0], {}).get("alive")}

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
                # runner thread) until the OS drains the TX buffer; the link is
                # not saturated, so writes deliver promptly on their own.
            except (serial.SerialException, OSError) as e:
                with self._lock:
                    self._tx_errors += 1
                self._log.write(LOG.EVENT, f"error:tx_write:{e}".encode("utf-8"))
                return False
        with self._lock:
            self._tx_frames += 1
        self._log.write(LOG.TX_FRAME, frame, ts_ns=ts)
        return True

    def send_robot_cmd(self, cmds, echo_cycle: int | None = None,
                       reset_slaves=()) -> None:
        """Send one USB_MSG_CMD (cmd_master_t) for this tick from a list of
        MotorCommand. The payload always carries every configured slave at its
        fixed slot (exactly CMD_MASTER_SIZE(N_SLAVES) bytes — the master rejects
        any other length); motors without a command default to STOP with a zero
        mit, so motors a policy does not drive stay off.

        All motors in one call share a single cmd_seq (one per tick): latency.py
        treats a tick as applied once every commanded motor reports
        last_applied_seq >= it. The counter advances once per call, starting at 1
        and skipping 0 on wrap (0 = CMD_SEQ_NONE).

        cycle_id echoes `echo_cycle` — the master cycle the command was COMPUTED FROM
        (the telemetry-driven runner passes the stepped frame's cycle_id) — or the last
        cycle seen if unset. The master classifies on_time/late against this echo.

        reset_slaves: slave indices whose SLAVE_REQ_RESET to assert this tick
        (level-triggered; hold it until telemetry shows the outcome, then drop)."""
        seq = self._cmd_seq + 1
        if seq > 0xFFFF:
            seq = 1
        self._cmd_seq = seq
        if echo_cycle is not None:
            cycle_id = echo_cycle & 0xFFFF
        else:
            with self._lock:
                cycle_id = self._cycle_id

        reset_slaves = set(reset_slaves)
        slaves = [dict(req=(P.SLAVE_REQ_RESET if s in reset_slaves
                            else P.SLAVE_REQ_NONE),
                       motors=[dict(req=P.MOTOR_REQ_STOP)
                               for _ in range(P.MAX_MOTORS_PER_SLAVE)])
                  for s in range(_mc.N_SLAVES)]
        for c in cmds:
            if 0 <= c.slave < _mc.N_SLAVES and 0 <= c.local < P.MAX_MOTORS_PER_SLAVE:
                slaves[c.slave]["motors"][c.local] = dict(
                    req=c.req, pos=c.pos, vel=c.vel, kp=c.kp, kd=c.kd,
                    tau_ff=c.tau_ff)

        payload = P.pack_cmd_master(cycle_id, seq, slaves)
        self._write_frame(P.encode_frame(P.USB_MSG_CMD, payload))

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
                        discard_bytes=self._discard_bytes,
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
