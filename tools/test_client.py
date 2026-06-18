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
  s     SINE       active motor  (toggle ±30° at 0.3 Hz via MIT_CMD — arm first)
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

# Motor table generated from configs/slave0.yaml (single source of truth).
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    from motor_config_gen import (
        N_MOTORS, MOTORS, MOTOR_DEFAULT_KP, MOTOR_DEFAULT_KD,
    )
except ImportError:
    sys.exit("motor_config_gen.py not found — run:\n"
             "    python3 scripts/gen_motor_config.py configs/slave0.yaml")

# ═══════════════════════════════════════════════════════════════════
#  Protocol constants  (mirror firmware/common/include/protocol.h)
# ═══════════════════════════════════════════════════════════════════

# NodeId
NODE_JETSON  = 1
NODE_MASTER  = 2
NODE_SLAVE_0 = 3

# MsgType
MSG_PING          = 0x01
MSG_MASTER_STATUS = 0x02
MSG_SLAVE_STATUS  = 0x03
MSG_MOTOR_STATE   = 0x04
MSG_CONTROL_REQ   = 0x05
MSG_CONTROL_RESP  = 0x06
MSG_MOTOR_CMD     = 0x07

MSG_NAMES = {
    MSG_PING:          "PING",
    MSG_MASTER_STATUS: "MASTER_STATUS",
    MSG_SLAVE_STATUS:  "SLAVE_STATUS",
    MSG_MOTOR_STATE:   "MOTOR_STATE",
    MSG_CONTROL_REQ:   "CONTROL_REQ",
    MSG_CONTROL_RESP:  "CONTROL_RESP",
    MSG_MOTOR_CMD:     "MOTOR_CMD",
}

# RobotState
ROBOT_STATE_NAMES = {0: "INIT", 1: "READY", 2: "DEGRADED"}

# MotorLifecycle
MOTOR_STATE_NAMES = {
    0: "BOOT", 1: "DISCOVERING", 2: "IDLE",
    3: "ARMED_HOLD", 4: "FAULT", 5: "DISABLED", 6: "ZEROING",
    7: "ARMED_MIT",
}

# ControlCmd
CTRL_ARM_HOLD  = 0x01
CTRL_DISABLE   = 0x02
CTRL_GOTO_ZERO = 0x04

# ControlResult
CTRL_RESULT_NAMES = {0: "OK", 1: "ERR_STATE", 2: "ERR_STUB", 3: "ERR_MOTOR"}

# ── MsgHeader wire layout (little-endian, packed, 16 bytes) ───────────────
HDR_FMT  = "<HHBBIHHH"
HDR_SIZE = struct.calcsize(HDR_FMT)  # 16

# Payload formats (little-endian, packed)
FMT_MASTER_STATUS = "<BBBIII"   # 15 bytes
FMT_SLAVE_STATUS  = "<BBI"      #  6 bytes
FMT_MOTOR_STATE   = "<BBffffHH" # 22 bytes
FMT_CONTROL_REQ   = "<BBH"      #  4 bytes
FMT_CONTROL_RESP  = "<BBBBH"    #  6 bytes
FMT_MOTOR_CMD     = "<Bfffff"   # 21 bytes: motor_idx, pos, vel, kp, kd, tau_ff

assert HDR_SIZE == 16,                            f"HDR {HDR_SIZE}"
assert struct.calcsize(FMT_MASTER_STATUS) == 15,  "MasterStatus"
assert struct.calcsize(FMT_SLAVE_STATUS)  == 6,   "SlaveStatus"
assert struct.calcsize(FMT_MOTOR_STATE)   == 22,  "MotorState"
assert struct.calcsize(FMT_CONTROL_REQ)   == 4,   "ControlReq"
assert struct.calcsize(FMT_CONTROL_RESP)  == 6,   "ControlResp"
assert struct.calcsize(FMT_MOTOR_CMD)     == 21,  "MotorCmd"

# ── Python-side sine parameters ───────────────────────────────────────────────
SINE_AMP   = math.pi / 2.0      # 90° in rad
SINE_FREQ  = 0.5                # Hz
SINE_OMEGA = 2.0 * math.pi * SINE_FREQ

# MOTOR_DEFAULT_KP / MOTOR_DEFAULT_KD come from motor_config_gen (per motor).

# Last known motor positions, updated from incoming MOTOR_STATE frames
_motor_pos = [0.0] * N_MOTORS

def _motor_label(idx: int) -> str:
    """Human-readable description of a motor index, e.g. 'id=1 (CAN 1, RS02)'."""
    if 0 <= idx < len(MOTORS):
        m = MOTORS[idx]
        return f"id={idx + 1} (CAN {m['can_id']}, {m['model']})"
    return f"id={idx + 1}"

# ═══════════════════════════════════════════════════════════════════
#  CRC16-CCITT  (poly=0x1021, init=0xFFFF — must match firmware)
# ═══════════════════════════════════════════════════════════════════

def _crc16_update(crc: int, data: bytes) -> int:
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc

def crc16(data: bytes) -> int:
    return _crc16_update(0xFFFF, data)

# ═══════════════════════════════════════════════════════════════════
#  Frame encode / decode
# ═══════════════════════════════════════════════════════════════════

_seq = 0

def encode_frame(msg_type: int, src: int, dst: int,
                 payload: bytes = b"") -> bytes:
    global _seq
    ts_ms = (time.monotonic_ns() // 1_000_000) & 0xFFFFFFFF
    hdr = struct.pack(HDR_FMT,
                      msg_type, _seq, src, dst,
                      ts_ms, len(payload), 0, 0)
    frame = bytearray(hdr) + payload
    checksum = crc16(bytes(frame))
    frame[14] = checksum & 0xFF
    frame[15] = (checksum >> 8) & 0xFF
    _seq = (_seq + 1) & 0xFFFF
    return bytes(frame)


_MAX_PAYLOAD = 256

def decode_frame(buf: bytearray):
    """
    Try to decode one frame from buf (mutated in place on resync).

    Returns (msg_type, seq, ts_ms, payload_bytes, consumed) on success.
    Returns None if buf doesn't yet contain a complete valid frame.
    Drops one byte and retries on bad CRC or implausible pay_len.
    """
    while len(buf) >= HDR_SIZE:
        msg_type, seq, src, dst, ts_ms, pay_len, flags, crc_wire = \
            struct.unpack_from(HDR_FMT, buf, 0)

        if pay_len > _MAX_PAYLOAD:
            del buf[0]
            continue

        total = HDR_SIZE + pay_len
        if len(buf) < total:
            return None

        check = bytearray(buf[:total])
        check[14] = 0
        check[15] = 0
        if crc16(bytes(check)) != crc_wire:
            del buf[0]
            continue

        payload = bytes(buf[HDR_SIZE:total])
        return (msg_type, seq, ts_ms, payload, total)

    return None

# ═══════════════════════════════════════════════════════════════════
#  Payload parsers
# ═══════════════════════════════════════════════════════════════════

def parse_master_status(p: bytes) -> dict:
    if len(p) < struct.calcsize(FMT_MASTER_STATUS):
        return {}
    rs, sa, ma, up, le, rf = struct.unpack_from(FMT_MASTER_STATUS, p)
    return dict(robot_state=rs, slave_alive=sa, motors_alive=ma,
                uptime_ms=up, link_errors=le, rx_frames=rf)

def parse_slave_status(p: bytes) -> dict:
    if len(p) < struct.calcsize(FMT_SLAVE_STATUS):
        return {}
    ma, ms, up = struct.unpack_from(FMT_SLAVE_STATUS, p)
    return dict(motors_alive=ma, motor_state=ms, uptime_ms=up)

def parse_motor_state(p: bytes) -> dict:
    if len(p) < struct.calcsize(FMT_MOTOR_STATE):
        return {}
    idx, st, pos, vel, tau, temp, ff, lcs = struct.unpack_from(FMT_MOTOR_STATE, p)
    if 0 <= idx < len(_motor_pos):
        _motor_pos[idx] = pos
    return dict(motor_idx=idx, state=st, pos=pos, vel=vel,
                tau=tau, temp=temp, fault_flags=ff, last_cmd_seq=lcs)

def parse_control_resp(p: bytes) -> dict:
    if len(p) < struct.calcsize(FMT_CONTROL_RESP):
        return {}
    idx, cmd, result, new_st, req_seq = struct.unpack_from(FMT_CONTROL_RESP, p)
    return dict(motor_idx=idx, cmd=cmd, result=result,
                new_state=new_st, req_seq=req_seq)

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
              f"slave_alive={d.get('slave_alive', 0)} "
              f"motors_alive=0b{d.get('motors_alive', 0):b} "
              f"link_errs={d.get('link_errors', 0)}")

    elif msg_type == MSG_SLAVE_STATUS:
        d = parse_slave_status(payload)
        ms_val = d.get("motor_state", 0)
        print(f"[{_t()}] {label:<16} "
              f"slave_state={_motor_state_str(ms_val):<20} "
              f"motors_alive=0b{d.get('motors_alive', 0):b}")

    elif msg_type == MSG_MOTOR_STATE:
        d = parse_motor_state(payload)
        st_val = d.get("state", 0)
        print(f"[{_t()}] {label:<16} "
              f"id={d.get('motor_idx', 0) + 1} "
              f"state={_motor_state_str(st_val):<20} "
              f"pos={d.get('pos', 0.0):+.3f} "
              f"vel={d.get('vel', 0.0):+.3f} "
              f"tau={d.get('tau', 0.0):+.3f} "
              f"lcs={d.get('last_cmd_seq', 0)}")

    elif msg_type == MSG_CONTROL_RESP:
        d = parse_control_resp(payload)
        result_str = CTRL_RESULT_NAMES.get(d.get("result", 0), "?")
        ns_val = d.get("new_state", 0)
        print(f"[{_t()}] {label:<16} "
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
    """Sends MIT motor commands at 20 Hz tracing a ±30° sine wave."""
    kp = MOTOR_DEFAULT_KP[idx]
    kd = MOTOR_DEFAULT_KD[idx]
    t0 = time.monotonic()
    while not stop_event.is_set():
        t   = time.monotonic() - t0
        pos = center + SINE_AMP * math.sin(SINE_OMEGA * t)
        vel = SINE_AMP * SINE_OMEGA * math.cos(SINE_OMEGA * t)
        payload = struct.pack(FMT_MOTOR_CMD, idx, pos, vel, kp, kd, 0.0)
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
        "  s   SINE       active (toggle ±90° @ 0.5 Hz via MIT — arm first)",
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
        payload = struct.pack(FMT_CONTROL_REQ, idx, cmd, 0)
        _ser_write(encode_frame(MSG_CONTROL_REQ, NODE_JETSON, NODE_MASTER, payload))
        print(f"[{_t()}] → CONTROL_REQ {name:<9} {_motor_label(idx)} seq={(_seq-1)&0xFFFF}")
        sys.stdout.flush()

    def _toggle_sine(idx: int) -> None:
        if sine_active[idx]:
            _stop_sine(idx)
            print(f"[{_t()}] → SINE {_motor_label(idx)} STOPPED")
        else:
            center = _motor_pos[idx]
            sine_stop[idx].clear()
            sine_active[idx] = True
            th = threading.Thread(
                target=_sine_thread,
                args=(idx, ser, ser_lock, sine_stop[idx], center),
                daemon=True,
            )
            sine_threads[idx] = th
            th.start()
            print(f"[{_t()}] → SINE {_motor_label(idx)} STARTED  center={center:+.3f} rad")
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
