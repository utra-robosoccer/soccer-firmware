"""Passively sniff the CAN bus via the CH341 USB-CAN adapter.

Prints every frame seen, decoded as RobStride private protocol
(mode / data / node_id), with Type 1 (MIT) and Type 2 (feedback) annotated.

Usage:
    python sniff.py                  # listen forever, default port
    python sniff.py --port /dev/ttyCH341USB0
    python sniff.py --seconds 5      # listen for 5 s, then exit with a summary
    python sniff.py --ping 2         # also send one Type 1 ping to motor 2 each second
"""
import argparse
import time
from collections import Counter

import serial

from rs02_can import (
    COMM_FEEDBACK,
    COMM_MIT,
    decode_feedback,
    decode_raw_frame,
    encode_mit,
    encode_raw_frame,
    parse_id,
)

DEFAULT_PORT = "/dev/ttyCH341USB0"
DEFAULT_BAUD = 921600


def _fmt_frame(arb_id: int, data: bytes) -> str:
    mode, data_field, node_id = parse_id(arb_id)
    base = f"id=0x{arb_id:08X} mode={mode:2d} data=0x{data_field:04X} node=0x{node_id:02X} dlc={len(data)} payload={data.hex()}"
    if mode == COMM_MIT:
        return base + "  <- Type1 MIT cmd"
    if mode == COMM_FEEDBACK:
        fb = decode_feedback(arb_id, data)
        if fb is not None:
            return base + f"  <- Type2 fb motor={fb.motor_id} pos={fb.pos:.3f} vel={fb.vel:.3f} torq={fb.torque:.3f} temp={fb.temp:.1f} status={fb.status_str()} faults={fb.fault_str()}"
        return base + "  <- Type2 (decode failed)"
    return base


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--seconds", type=float, default=0.0, help="Stop after N seconds (0 = forever)")
    ap.add_argument("--ping", type=int, default=None, help="Also send Type 1 (zero MIT) to this motor ID once per second")
    args = ap.parse_args()

    print(f"opening {args.port} @ {args.baud} baud")
    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    ser.reset_input_buffer()

    buf = b""
    by_mode: Counter = Counter()
    by_node: Counter = Counter()
    total = 0
    t_start = time.monotonic()
    next_ping = t_start

    try:
        while True:
            now = time.monotonic()
            if args.seconds > 0 and (now - t_start) >= args.seconds:
                break

            if args.ping is not None and now >= next_ping:
                next_ping = now + 1.0
                arb, payload = encode_mit(args.ping, 0.0, 0.0, 0.0, 0.0, 0.0)
                ser.write(encode_raw_frame(arb, payload))
                ser.flush()
                print(f"[t={now-t_start:5.2f}] >> ping motor {args.ping} (Type1 zero-MIT)")

            chunk = ser.read(256)
            if chunk:
                buf += chunk
            while True:
                frame = decode_raw_frame(buf)
                if frame is None:
                    break
                buf = buf[frame.end:]
                mode, _, node_id = parse_id(frame.arb_id)
                by_mode[mode] += 1
                by_node[node_id] += 1
                total += 1
                print(f"[t={now-t_start:5.2f}] {_fmt_frame(frame.arb_id, frame.data)}")
    except KeyboardInterrupt:
        print()

    elapsed = time.monotonic() - t_start
    print(f"\n--- summary: {total} frames in {elapsed:.2f}s ({total/max(elapsed,1e-6):.1f} fps) ---")
    if by_mode:
        print("by mode: " + ", ".join(f"mode{m}={n}" for m, n in sorted(by_mode.items())))
    if by_node:
        print("by node: " + ", ".join(f"0x{n:02X}={c}" for n, c in sorted(by_node.items())))
    if total == 0:
        print("** no frames seen — check that USB-CAN is at 1 Mbps and is on the same bus as the slave **")

    ser.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
