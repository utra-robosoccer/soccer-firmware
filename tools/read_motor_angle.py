#!/usr/bin/env python3
"""
Read motor angle (and velocity) forwarded from master MCU over USB CDC.
Usage:
    python3 read_motor_angle.py [PORT]
    PORT defaults to /dev/ttyACM1
"""

import sys
import re
import serial
from datetime import datetime

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM1"
BAUD = 115200  # CDC ignores baud, but pyserial needs a value

LINE_RE = re.compile(
    r"motor=(\d+)\s+pos=([+-]?\d+\.\d+)\s+vel=([+-]?\d+\.\d+)"
)

def main():
    print(f"Opening {PORT} ...")
    with serial.Serial(PORT, BAUD, timeout=2) as ser:
        print(f"Connected. Waiting for data...\n")
        print(f"{'TIME':12}  {'MOTOR':5}  {'POS (rad)':>12}  {'VEL (rad/s)':>12}")
        print("-" * 50)
        while True:
            try:
                raw = ser.readline()
            except serial.SerialException as e:
                print(f"Serial error: {e}", file=sys.stderr)
                break

            if not raw:
                continue

            try:
                line = raw.decode("ascii", errors="replace").strip()
            except Exception:
                continue

            m = LINE_RE.search(line)
            if m:
                motor_id = int(m.group(1))
                pos      = float(m.group(2))
                vel      = float(m.group(3))
                ts       = datetime.now().strftime("%H:%M:%S.%f")[:12]
                print(f"{ts:12}  {motor_id:<5}  {pos:>12.4f}  {vel:>12.4f}")
            else:
                # Pass through any other text from the MCU (debug, errors)
                if line:
                    print(f"[MCU] {line}")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")
