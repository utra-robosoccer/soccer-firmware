#!/usr/bin/env python3
"""
ARM_HOLD interactive controller.

Keys:
  e        - ARM motor (safe activation: capture current pos, hold it)
  d / SPC  - DISARM motor
  q        - DISARM and quit

Sends keepalive 'k' every 100 ms while armed so the master USB watchdog
(future) has a heartbeat, and displays live position + slave state.

Usage:
    python3 arm_hold.py [PORT]
    PORT defaults to /dev/ttyACM1
"""

import re
import sys
import termios
import threading
import time
import tty

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM1"
BAUD = 115200

LINE_RE = re.compile(
    r"motor=(\d+)\s+pos=([+-]?\d+\.\d+)\s+state=(\w+)"
)

armed = False
running = True
last_pos = 0.0
last_state = "IDLE"


def reader_thread(ser: serial.Serial) -> None:
    global last_pos, last_state
    while running:
        try:
            raw = ser.readline()
        except serial.SerialException:
            break
        if not raw:
            continue
        line = raw.decode("ascii", errors="replace").strip()
        m = LINE_RE.search(line)
        if m:
            last_pos = float(m.group(2))
            last_state = m.group(3)
            arm_indicator = "\033[32mARMED\033[0m" if last_state == "ARMED" else "\033[90mIDLE \033[0m"
            print(
                f"\r  [{arm_indicator}]  motor={m.group(1)}  pos={last_pos:>9.4f} rad   ",
                end="",
                flush=True,
            )
        elif line:
            print(f"\n[MCU] {line}", flush=True)


def keepalive_thread(ser: serial.Serial) -> None:
    while running:
        if armed:
            try:
                ser.write(b"k")
            except serial.SerialException:
                break
        time.sleep(0.1)


def main() -> None:
    global armed, running

    print(f"Opening {PORT} ...")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.5)
    except serial.SerialException as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

    threading.Thread(target=reader_thread, args=(ser,), daemon=True).start()
    threading.Thread(target=keepalive_thread, args=(ser,), daemon=True).start()

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    print(f"Connected.  e=ARM  d/SPACE=DISARM  q=quit\n")

    try:
        tty.setraw(fd)
        while True:
            ch = sys.stdin.read(1)

            if ch == "e" and not armed:
                armed = True
                ser.write(b"e")

            elif ch in ("d", " ") and armed:
                armed = False
                ser.write(b"d")

            elif ch == "q":
                if armed:
                    armed = False
                    ser.write(b"d")
                    time.sleep(0.06)
                break

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        running = False
        try:
            ser.close()
        except Exception:
            pass
        print("\nExited.")


if __name__ == "__main__":
    main()
