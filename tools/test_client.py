#!/usr/bin/env python3
"""
test_client.py — framed-protocol test client for robosoccer firmware.

Usage:  python3 test_client.py [PORT]
        PORT defaults to /dev/ttyACM0

Keys:
  1/2   ARM_HOLD  motor 1/2
  3/4   GOTO_ZERO motor 1/2
  5/6   SINE toggle motor 1/2  (±30° at 0.3 Hz via MIT_CMD — arm first)
  d     DISABLE all motors (also stops active sines)
  p     PING
  q     QUIT
  ?     this help
"""

import math
import select
import struct
import sys
import termios
import threading
import time
import tty

import serial

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
SINE_AMP   = 0.5236             # 30° in rad
SINE_FREQ  = 0.3                # Hz
SINE_OMEGA = 2.0 * math.pi * SINE_FREQ

# Gains matching motor_config.h defaults
MOTOR_DEFAULT_KP = [15.0, 15.0]
MOTOR_DEFAULT_KD = [1.0,  1.0]

# Last known motor positions, updated from incoming MOTOR_STATE frames
_motor_pos = [0.0, 0.0]

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
        time.sleep(0.05)  # 20 Hz

# ═══════════════════════════════════════════════════════════════════
#  Main loop
# ═══════════════════════════════════════════════════════════════════

HELP = """\
  1   ARM_HOLD   motor id=1  (motor_idx=0, CAN 2)
  2   ARM_HOLD   motor id=2  (motor_idx=1, CAN 1)
  3   GOTO_ZERO  motor id=1  — crawl to zero, then hold
  4   GOTO_ZERO  motor id=2  — crawl to zero, then hold
  5   SINE       motor id=1  — toggle ±30° @ 0.3 Hz  (arm first)
  6   SINE       motor id=2  — toggle ±30° @ 0.3 Hz  (arm first)
  d   DISABLE    all motors  (also stops active sines)
  p   PING       check link
  q   QUIT       disable all then exit
  ?   this help
"""

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
    sine_stop    = [threading.Event(), threading.Event()]
    sine_active  = [False, False]
    sine_threads = [None, None]

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

                if ch in ("1", "2"):
                    motor_idx = int(ch) - 1
                    payload = struct.pack(FMT_CONTROL_REQ, motor_idx, CTRL_ARM_HOLD, 0)
                    _ser_write(encode_frame(MSG_CONTROL_REQ, NODE_JETSON, NODE_MASTER, payload))
                    print(f"[{_t()}] → CONTROL_REQ ARM_HOLD   id={ch} (idx={motor_idx}) seq={(_seq-1)&0xFFFF}")
                    sys.stdout.flush()

                elif ch in ("3", "4"):
                    motor_idx = int(ch) - 3   # '3'→idx 0, '4'→idx 1
                    payload = struct.pack(FMT_CONTROL_REQ, motor_idx, CTRL_GOTO_ZERO, 0)
                    _ser_write(encode_frame(MSG_CONTROL_REQ, NODE_JETSON, NODE_MASTER, payload))
                    print(f"[{_t()}] → CONTROL_REQ GOTO_ZERO  id={motor_idx+1} (idx={motor_idx}) seq={(_seq-1)&0xFFFF}")
                    sys.stdout.flush()

                elif ch in ("5", "6"):
                    idx = int(ch) - 5   # '5'→idx 0, '6'→idx 1
                    if sine_active[idx]:
                        _stop_sine(idx)
                        print(f"[{_t()}] → SINE id={idx+1} STOPPED")
                    else:
                        center = _motor_pos[idx]
                        sine_stop[idx].clear()
                        sine_active[idx] = True
                        t = threading.Thread(
                            target=_sine_thread,
                            args=(idx, ser, ser_lock, sine_stop[idx], center),
                            daemon=True,
                        )
                        sine_threads[idx] = t
                        t.start()
                        print(f"[{_t()}] → SINE id={idx+1} STARTED  center={center:+.3f} rad")
                    sys.stdout.flush()

                elif ch == "d":
                    for i in range(2):
                        _stop_sine(i)
                    for motor_idx in range(2):
                        payload = struct.pack(FMT_CONTROL_REQ, motor_idx, CTRL_DISABLE, 0)
                        _ser_write(encode_frame(MSG_CONTROL_REQ, NODE_JETSON, NODE_MASTER, payload))
                    print(f"[{_t()}] → CONTROL_REQ DISABLE all")
                    sys.stdout.flush()

                elif ch == "p":
                    frame = encode_frame(MSG_PING, NODE_JETSON, NODE_MASTER, b"")
                    _ser_write(frame)
                    print(f"[{_t()}] → PING seq={(_seq-1)&0xFFFF}")
                    sys.stdout.flush()

                elif ch in ("q", "\x03", "\x04"):
                    print(f"\n[{_t()}] Disabling all and exiting…")
                    for i in range(2):
                        _stop_sine(i)
                    for motor_idx in range(2):
                        payload = struct.pack(FMT_CONTROL_REQ, motor_idx, CTRL_DISABLE, 0)
                        _ser_write(encode_frame(MSG_CONTROL_REQ, NODE_JETSON, NODE_MASTER, payload))
                    time.sleep(0.05)
                    break

                elif ch == "?":
                    print(HELP)
                    sys.stdout.flush()

    finally:
        for i in range(2):
            _stop_sine(i)
        termios.tcsetattr(fd_stdin, termios.TCSADRAIN, old_term)
        ser.close()
        print("Exited.")


if __name__ == "__main__":
    main()
