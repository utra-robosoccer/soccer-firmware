#!/usr/bin/env python3
"""
test_client.py — framed-protocol test client for robosoccer firmware.

Usage:  python3 test_client.py [PORT]
        PORT defaults to /dev/ttyACM0

Select-then-act: pick a motor with a digit, then act on it with a letter.

Keys:
  1..N  SELECT active motor (N = configured motor count)
  a     ARM_HOLD   active motor
  z     GOTO_ZERO  active motor  (crawl to zero, then hold)
  s     SINE       active motor  (toggle a sine sized to the motor's soft limits
                                  @ 0.25 Hz via MIT_CMD — arm first; slave clamps)
  A     ARM_HOLD   ALL motors
  Z     GOTO_ZERO  ALL motors
  S     SINE       ALL motors    (starts sine on every motor — arm first)
  D     DISABLE    ALL motors    (also stops all sines)
  p     PING
  q     QUIT       (disable all, then exit)
  ?     this help
"""

import math
import os
import select
import struct
import sys
import termios
import threading
import time
import tty

import serial

# Motor table generated from the active setup's slave YAMLs (single source of truth).
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    from motor_config_gen import (
        N_MOTORS, MOTORS, MOTOR_DEFAULT_KP, MOTOR_DEFAULT_KD,
        MOTOR_SOFT_MIN, MOTOR_SOFT_MAX,
    )
except ImportError:
    sys.exit("motor_config_gen.py not found — run:\n"
             "    python3 scripts/gen_motor_config.py")

# ═══════════════════════════════════════════════════════════════════
#  Protocol — the canonical wire library (host/jetson/protocol.py)
# ═══════════════════════════════════════════════════════════════════
_HOST_JETSON = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "host", "jetson")
sys.path.insert(0, _HOST_JETSON)
from protocol import (                                       # noqa: E402
    NODE_JETSON, NODE_MASTER, NODE_SLAVE_0,
    MSG_PING, MSG_MASTER_STATUS, MSG_SLAVE_STATUS, MSG_MOTOR_STATE,
    MSG_CONTROL_REQ, MSG_CONTROL_RESP, MSG_MOTOR_CMD, MSG_NAMES,
    ROBOT_STATE_NAMES, LIFECYCLE_NAMES, CAUSE_NAMES,
    CTRL_ARM_HOLD, CTRL_DISABLE, CTRL_GOTO_ZERO, CTRL_RESULT_NAMES,
    CMDFLAG_CLAMPED_POS, CMDFLAG_CLAMPED_TAU, CMDFLAG_CMD_STALE,
    HDR_FMT, HDR_SIZE, crc16, encode_frame, decode_frame,
    FMT_MASTER_STATUS, FMT_SLAVE_STATUS, FMT_CONTROL_REQ, FMT_CONTROL_RESP,
    FMT_MOTOR_CMD, MotorState, MOTORSTATE_SIZE,
    parse_master_status, parse_slave_status, parse_control_resp,
    parse_motor_state as _proto_parse_motor_state,
)

# dashboard.py (and this module's display) reference MOTOR_STATE_NAMES — keep it
# as an alias of the library's lifecycle table.
MOTOR_STATE_NAMES = LIFECYCLE_NAMES

# ── Python-side sine parameters ───────────────────────────────────────────────
# The sweep is sized PER MOTOR to its soft limits: amplitude = OVERSHOOT × the
# soft half-range, centred on the limit midpoint. This gently reaches and holds
# each limit (the slave still enforces the clamp) instead of slamming a fixed
# ±90° command through a much smaller clamped range. Lower frequency keeps the
# mid-stroke velocity gentle so the motion is smooth.
SINE_FREQ      = 0.25           # Hz
SINE_OMEGA     = 2.0 * math.pi * SINE_FREQ
SINE_OVERSHOOT = 1.2            # sweep amplitude as a multiple of the soft half-range

# MOTOR_DEFAULT_KP / MOTOR_DEFAULT_KD come from motor_config_gen (per motor).

# Last known motor positions, updated from incoming MOTOR_STATE frames (by
# global flattened index across all slaves).
_motor_pos = [0.0] * N_MOTORS

# Map between the flattened global index (digit keys 1..N) and (slave, local idx)
# used on the wire.
GLOBAL_OF = {(m["slave"], m["idx"]): g for g, m in enumerate(MOTORS)}

def _slave_local(g: int):
    """(slave_id, local_idx) for a global motor index."""
    m = MOTORS[g]
    return m["slave"], m["idx"]

def _global_of(slave_id: int, local_idx: int):
    """Global flattened index for a (slave, local) pair, or None."""
    return GLOBAL_OF.get((slave_id, local_idx))

def _motor_label(idx: int) -> str:
    """Human-readable description of a global motor index, e.g.
    'id=1 s0.m0 (CAN 1, RS02)'."""
    if 0 <= idx < len(MOTORS):
        m = MOTORS[idx]
        return (f"id={idx + 1} s{m['slave']}.m{m['idx']} "
                f"(CAN {m['can_id']}, {m['model']})")
    return f"id={idx + 1}"

# ═══════════════════════════════════════════════════════════════════
#  MOTOR_STATE decode: library wire-parse + app-level global-index glue
# ═══════════════════════════════════════════════════════════════════

def parse_motor_state(p: bytes) -> dict:
    """Wrap the library decoder with this client's flattened global index and
    latest-position cache. Wire decoding lives in host/jetson/protocol.py."""
    d = _proto_parse_motor_state(p)
    if not d:
        return {}
    g = _global_of(d["slave_id"], d["motor_idx"])
    if g is not None:
        _motor_pos[g] = d["pos"]
    d["gidx"] = g
    return d

# ═══════════════════════════════════════════════════════════════════
#  Display helpers
# ═══════════════════════════════════════════════════════════════════

_USE_COLOR = sys.stdout.isatty()

def _c(text: str, code: str) -> str:
    return f"\033[{code}m{text}\033[0m" if _USE_COLOR else text

def _motor_state_str(val: int) -> str:
    name = MOTOR_STATE_NAMES.get(val, f"?{val}")
    if name == "ARMED_HOLD":  return _c(name, "32")    # green
    if name == "ARMED_MIT":   return _c(name, "35")    # magenta
    if name == "ZEROING":     return _c(name, "36")    # cyan
    if name == "IDLE":        return _c(name, "33")    # yellow
    if name == "FAULT":       return _c(name, "31")    # red
    return name

_T0 = time.monotonic()

def _t() -> str:
    return f"t={time.monotonic() - _T0:7.3f}"

def print_frame(msg_type: int, seq: int, payload: bytes) -> None:
    label = MSG_NAMES.get(msg_type, f"0x{msg_type:02x}")

    if msg_type == MSG_MASTER_STATUS:
        d = parse_master_status(payload)
        rs = ROBOT_STATE_NAMES.get(d.get("robot_state", 0), "?")
        print(f"[{_t()}] {label:<16} "
              f"robot_state={rs:<9} "
              f"slave_alive=0b{d.get('slave_alive', 0):b} "
              f"link_errs={d.get('link_errors', 0)}")

    elif msg_type == MSG_SLAVE_STATUS:
        d = parse_slave_status(payload)
        print(f"[{_t()}] {label:<16} "
              f"slave={d.get('slave_id', 0)} "
              f"motors_alive=0b{d.get('motors_alive', 0):b} "
              f"crc_errors={d.get('crc_errors', 0)}")

    elif msg_type == MSG_MOTOR_STATE:
        d = parse_motor_state(payload)
        st_val = d.get("state", 0)
        g = d.get("gidx")
        id_str = f"id={g + 1}" if g is not None else "id=?"
        print(f"[{_t()}] {label:<16} "
              f"{id_str} s{d.get('slave_id', 0)}.m{d.get('motor_idx', 0)} "
              f"state={_motor_state_str(st_val):<20} "
              f"pos={d.get('pos', 0.0):+.3f} "
              f"vel={d.get('vel', 0.0):+.3f} "
              f"tau={d.get('tau', 0.0):+.3f} "
              f"cause={CAUSE_NAMES.get(d.get('cause', 0), '?')} "
              f"fb={d.get('fb_age', 0)}ms "
              f"flags=0x{d.get('cmd_flags', 0):02x}")

    elif msg_type == MSG_CONTROL_RESP:
        d = parse_control_resp(payload)
        result_str = CTRL_RESULT_NAMES.get(d.get("result", 0), "?")
        ns_val = d.get("new_state", 0)
        print(f"[{_t()}] {label:<16} "
              f"s{d.get('slave_id', 0)}.m{d.get('motor_idx', 0)} "
              f"req_seq={d.get('req_seq', 0)} "
              f"result={result_str} "
              f"new_state={_motor_state_str(ns_val)}")

    elif msg_type == MSG_PING:
        print(f"[{_t()}] {label:<16} seq={seq} (PONG)")

    else:
        print(f"[{_t()}] {label:<16} seq={seq} payload={payload.hex()}")

    sys.stdout.flush()

# ═══════════════════════════════════════════════════════════════════
#  Python-side sine wave thread
# ═══════════════════════════════════════════════════════════════════

def _sine_thread(idx: int, ser_ref, ser_lock: threading.Lock,
                 stop_event: threading.Event, center: float) -> None:
    """Sends MIT motor commands at 100 Hz tracing a sine sized to this motor's
    soft limits: centred on the limit midpoint with amplitude SINE_OVERSHOOT ×
    the soft half-range. It overshoots the limit slightly so the slave's clamp
    still engages (the motor reaches and holds the limit), but at a low
    mid-stroke velocity so the motion is smooth rather than slamming.

    `idx` is the global flattened index; it is mapped to (slave, local) for the
    wire. The `center` argument is accepted for call-compatibility but ignored;
    the sweep is always centred on the soft-limit midpoint."""
    slave_id, local_idx = _slave_local(idx)
    kp = MOTOR_DEFAULT_KP[idx]
    kd = MOTOR_DEFAULT_KD[idx]
    lo, hi   = MOTOR_SOFT_MIN[idx], MOTOR_SOFT_MAX[idx]
    mid      = 0.5 * (lo + hi)
    amp      = SINE_OVERSHOOT * 0.5 * (hi - lo)
    t0 = time.monotonic()
    while not stop_event.is_set():
        t   = time.monotonic() - t0
        pos = mid + amp * math.sin(SINE_OMEGA * t)
        vel =       amp * SINE_OMEGA * math.cos(SINE_OMEGA * t)
        payload = struct.pack(FMT_MOTOR_CMD, slave_id, local_idx, pos, vel, kp, kd, 0.0)
        frame   = encode_frame(MSG_MOTOR_CMD, NODE_JETSON, NODE_MASTER, payload)
        with ser_lock:
            try:
                ser_ref.write(frame)
            except serial.SerialException:
                break
        time.sleep(0.01)  # 100 Hz

# ═══════════════════════════════════════════════════════════════════
#  Main loop
# ═══════════════════════════════════════════════════════════════════

def _build_help() -> str:
    lines = [f"  1..{N_MOTORS}  SELECT active motor:"]
    for m in MOTORS:
        lines.append(f"          {m['idx'] + 1} = {_motor_label(m['idx'])}")
    lines += [
        "  a   ARM_HOLD   active        z   GOTO_ZERO  active",
        "  s   SINE       active (sine sized to soft limits @ 0.25 Hz via MIT — arm first; slave clamps)",
        "  A   ARM_HOLD   ALL    Z   GOTO_ZERO ALL    S   SINE ALL    D   DISABLE ALL",
        "  p   PING       check link    q   QUIT (disable all)    ?   this help",
    ]
    return "\n".join(lines) + "\n"

HELP = _build_help()

def main() -> None:
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"

    try:
        ser = serial.Serial(port, 115200, timeout=0)
    except serial.SerialException as e:
        sys.exit(f"Cannot open {port}: {e}")

    fd_ser   = ser.fileno()
    fd_stdin = sys.stdin.fileno()
    old_term = termios.tcgetattr(fd_stdin)
    rx_buf: bytearray = bytearray()

    ser_lock     = threading.Lock()
    sine_stop    = [threading.Event() for _ in range(N_MOTORS)]
    sine_active  = [False] * N_MOTORS
    sine_threads = [None] * N_MOTORS
    active       = 0  # currently selected motor index

    print("─" * 60)
    print(f"  soccer-firmware test client  |  {port}")
    print("─" * 60)
    print(HELP)
    print("─" * 60)
    print("Waiting for boot…  (power-cycle the board if needed)\n")
    sys.stdout.flush()

    def _stop_sine(idx: int) -> None:
        if sine_active[idx]:
            sine_stop[idx].set()
            sine_active[idx] = False

    def _ser_write(data: bytes) -> None:
        with ser_lock:
            ser.write(data)

    def _send_ctrl(idx: int, cmd: int, name: str) -> None:
        slave_id, local_idx = _slave_local(idx)
        payload = struct.pack(FMT_CONTROL_REQ, slave_id, local_idx, cmd, 0)
        _ser_write(encode_frame(MSG_CONTROL_REQ, NODE_JETSON, NODE_MASTER, payload))
        print(f"[{_t()}] → CONTROL_REQ {name:<9} {_motor_label(idx)} seq={(_seq-1)&0xFFFF}")
        sys.stdout.flush()

    def _toggle_sine(idx: int) -> None:
        if sine_active[idx]:
            _stop_sine(idx)
            print(f"[{_t()}] → SINE {_motor_label(idx)} STOPPED")
        else:
            sine_stop[idx].clear()
            sine_active[idx] = True
            th = threading.Thread(
                target=_sine_thread,
                args=(idx, ser, ser_lock, sine_stop[idx], _motor_pos[idx]),
                daemon=True,
            )
            sine_threads[idx] = th
            th.start()
            lo, hi = MOTOR_SOFT_MIN[idx], MOTOR_SOFT_MAX[idx]
            amp = SINE_OVERSHOOT * 0.5 * (hi - lo)
            print(f"[{_t()}] → SINE {_motor_label(idx)} STARTED  "
                  f"mid={0.5*(lo+hi):+.3f}  amp=±{amp:.3f} rad @ {SINE_FREQ} Hz "
                  f"(soft limits [{lo:+.3f}, {hi:+.3f}])")
        sys.stdout.flush()

    def _disable_all() -> None:
        for i in range(N_MOTORS):
            _stop_sine(i)
        for i in range(N_MOTORS):
            _send_ctrl(i, CTRL_DISABLE, "DISABLE")

    try:
        tty.setcbreak(fd_stdin)

        while True:
            try:
                rlist, _, _ = select.select([fd_ser, fd_stdin], [], [], 0.05)
            except (KeyboardInterrupt, SystemExit):
                break

            # ── serial RX ─────────────────────────────────────────────
            if fd_ser in rlist:
                try:
                    chunk = ser.read(512)
                except serial.SerialException as e:
                    print(f"\n[serial error: {e}]")
                    break
                if chunk:
                    rx_buf.extend(chunk)

            # Decode all complete frames currently in the buffer
            while True:
                result = decode_frame(rx_buf)
                if result is None:
                    break
                msg_type, seq, ts_ms, payload, consumed = result
                del rx_buf[:consumed]
                print_frame(msg_type, seq, payload)

            # ── keyboard ──────────────────────────────────────────────
            if fd_stdin in rlist:
                try:
                    ch = sys.stdin.read(1)
                except OSError:
                    break

                if ch.isdigit() and ch != "0" and int(ch) <= N_MOTORS:
                    active = int(ch) - 1
                    print(f"[{_t()}] ▸ selected {_motor_label(active)}")
                    sys.stdout.flush()

                elif ch == "a":
                    _send_ctrl(active, CTRL_ARM_HOLD, "ARM_HOLD")

                elif ch == "z":
                    _send_ctrl(active, CTRL_GOTO_ZERO, "GOTO_ZERO")

                elif ch == "s":
                    _toggle_sine(active)

                elif ch == "A":
                    for i in range(N_MOTORS):
                        _send_ctrl(i, CTRL_ARM_HOLD, "ARM_HOLD")

                elif ch == "Z":
                    for i in range(N_MOTORS):
                        _send_ctrl(i, CTRL_GOTO_ZERO, "GOTO_ZERO")

                elif ch == "S":
                    for i in range(N_MOTORS):
                        if not sine_active[i]:
                            _toggle_sine(i)

                elif ch == "D":
                    _disable_all()
                    print(f"[{_t()}] → DISABLE all")
                    sys.stdout.flush()

                elif ch == "p":
                    frame = encode_frame(MSG_PING, NODE_JETSON, NODE_MASTER, b"")
                    _ser_write(frame)
                    print(f"[{_t()}] → PING seq={(_seq-1)&0xFFFF}")
                    sys.stdout.flush()

                elif ch in ("q", "\x03", "\x04"):
                    print(f"\n[{_t()}] Disabling all and exiting…")
                    _disable_all()
                    time.sleep(0.05)
                    break

                elif ch == "?":
                    print(HELP)
                    sys.stdout.flush()

    finally:
        for i in range(N_MOTORS):
            _stop_sine(i)
        termios.tcsetattr(fd_stdin, termios.TCSADRAIN, old_term)
        ser.close()
        print("Exited.")


if __name__ == "__main__":
    main()
