"""
motor_sine_sweep.py

Sends a sine-sweep position command over USB CDC to STM32F446 SPI master board.
Firmware expects a flat array of 20 x motor_cmd_t (4 slaves × 5 motors each).

C struct (__attribute__((packed))):
    typedef struct __attribute__((packed)) {
        uint8_t motor_id;   // 1 byte
        float   position;   // 4 bytes  (raw IEEE 754, little-endian)
        float   speed;      // 4 bytes
    } motor_cmd_t;           // total: 9 bytes

USB frame = 20 × 9 = 180 bytes.

Motor ID layout per the firmware LUT:
    Slave 0: motors  1, 2, 3, 4, 0
    Slave 1: motors  5, 6, 7, 8, 0
    Slave 2: motors  9,10,11,12, 0
    Slave 3: motors 13,14,15,16, 0
"""

import struct
import math
import time
import serial

# ── Configuration ────────────────────────────────────────────────────────────

SERIAL_PORT = "/dev/ttyACM0"   # adjust to your USB device
BAUD_RATE   = 115200
SEND_HZ     = 500               # how often we send a frame (Hz)

NUM_MOTORS  = 2

# Per-motor configuration: (motor_id, pos_min, pos_max)
MOTORS = [
    {"id": 2, "pos_min": -0.5, "pos_max":  0.0},
    {"id": 1, "pos_min": -0.5, "pos_max":  0.5},
]

SWEEP_SPEED =   0.1            # speed field sent alongside position
SWEEP_FREQ  =   1.2           # Hz of the sine wave

# ── Struct format ────────────────────────────────────────────────────────────
# __attribute__((packed)):  B = uint8_t, ff = two floats, little-endian
STRUCT_FMT = "<Bff"  # 9 bytes per motor_cmd_t

MOTOR_CMD_SIZE = struct.calcsize(STRUCT_FMT)
FRAME_SIZE     = MOTOR_CMD_SIZE * NUM_MOTORS

assert MOTOR_CMD_SIZE == 9, f"Struct size mismatch: expected 9, got {MOTOR_CMD_SIZE}"
assert len(MOTORS) == NUM_MOTORS, "MOTORS config length must match NUM_MOTORS"
print(f"Struct size: {MOTOR_CMD_SIZE} bytes per motor")
print(f"Frame size:  {FRAME_SIZE} bytes total ({NUM_MOTORS} motors)")


def sine_position(t: float, pos_min: float, pos_max: float) -> float:
    """Map a sine wave into [pos_min, pos_max]."""
    mid = (pos_max + pos_min) / 2.0
    amp = (pos_max - pos_min) / 2.0
    return mid + amp * math.sin(2.0 * math.pi * SWEEP_FREQ * t)


def build_frame(t: float) -> bytes:
    """Pack NUM_MOTORS motor_cmd_t structs."""
    parts = []
    for m in MOTORS:
        pos = sine_position(t, m["pos_min"], m["pos_max"])
        parts.append(struct.pack(STRUCT_FMT, m["id"], pos, SWEEP_SPEED))
    frame = b"".join(parts)
    assert len(frame) == FRAME_SIZE
    return frame


def main():
    # Sanity check: print one frame for inspection
    test_frame = build_frame(0.0)
    print(f"\nTest frame (t=0), {len(test_frame)} bytes:")
    print(test_frame.hex(" "))

    for i, m in enumerate(MOTORS):
        unpacked = struct.unpack_from(STRUCT_FMT, test_frame, i * MOTOR_CMD_SIZE)
        print(f"  motor[{i}]: id={unpacked[0]}, pos={unpacked[1]:.4f}, spd={unpacked[2]:.4f}")

    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"\nOpened {SERIAL_PORT} @ {BAUD_RATE}")
    for m in MOTORS:
        print(f"  Motor {m['id']}: sweep [{m['pos_min']}, {m['pos_max']}]")
    print(f"Frequency: {SWEEP_FREQ} Hz, sending at {SEND_HZ} Hz")
    print("Ctrl+C to stop\n")

    dt = 1.0 / SEND_HZ
    t  = 0.0

    try:
        while True:
            frame = build_frame(t)
            ser.write(frame)

            # Print every ~1 second
            if int(t * SEND_HZ) % SEND_HZ == 0:
                positions = [sine_position(t, m["pos_min"], m["pos_max"]) for m in MOTORS]
                pos_str = "  ".join(f"m{m['id']}={p:+.4f}" for m, p in zip(MOTORS, positions))
                print(f"t={t:6.2f}s  {pos_str}  [{len(frame)}B sent]")

            t += dt
            time.sleep(dt)

    except KeyboardInterrupt:
        print(f"\nStopped at t={t:.2f}s. Last frame held, closing port.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()