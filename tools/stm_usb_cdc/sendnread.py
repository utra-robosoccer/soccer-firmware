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
SWEEP_FREQ  =   0.8           # Hz of the sine wave

# ── Receive config ───────────────────────────────────────────────────────────
# Firmware sends back 5 x motor_cmd_t feedback
RX_NUM_MOTORS = 2

# ── Struct format ────────────────────────────────────────────────────────────
# __attribute__((packed)):  B = uint8_t, ff = two floats, little-endian
STRUCT_FMT = "<Bff"  # 9 bytes per motor_cmd_t

MOTOR_CMD_SIZE = struct.calcsize(STRUCT_FMT)
FRAME_SIZE     = MOTOR_CMD_SIZE * NUM_MOTORS

assert MOTOR_CMD_SIZE == 9, f"Struct size mismatch: expected 9, got {MOTOR_CMD_SIZE}"
assert len(MOTORS) == NUM_MOTORS, "MOTORS config length must match NUM_MOTORS"

RX_FRAME_SIZE = MOTOR_CMD_SIZE * RX_NUM_MOTORS  # 180 bytes back from firmware

print(f"Struct size: {MOTOR_CMD_SIZE} bytes per motor")
print(f"TX frame:    {FRAME_SIZE} bytes ({NUM_MOTORS} motors)")
print(f"RX frame:    {RX_FRAME_SIZE} bytes ({RX_NUM_MOTORS} motors)")


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


def parse_rx_frame(data: bytes) -> list:
    """Unpack RX_NUM_MOTORS motor_cmd_t structs from raw bytes."""
    motors = []
    for i in range(RX_NUM_MOTORS):
        motor_id, pos, spd = struct.unpack_from(STRUCT_FMT, data, i * MOTOR_CMD_SIZE)
        motors.append({"id": motor_id, "pos": pos, "spd": spd})
    return motors


def main():
    # Sanity check: print one frame for inspection
    test_frame = build_frame(0.0)
    print(f"\nTest frame (t=0), {len(test_frame)} bytes:")
    print(test_frame.hex(" "))

    for i, m in enumerate(MOTORS):
        unpacked = struct.unpack_from(STRUCT_FMT, test_frame, i * MOTOR_CMD_SIZE)
        print(f"  motor[{i}]: id={unpacked[0]}, pos={unpacked[1]:.4f}, spd={unpacked[2]:.4f}")

    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0)  # non-blocking
    print(f"\nOpened {SERIAL_PORT} @ {BAUD_RATE}")
    for m in MOTORS:
        print(f"  Motor {m['id']}: sweep [{m['pos_min']}, {m['pos_max']}]")
    print(f"Frequency: {SWEEP_FREQ} Hz, sending at {SEND_HZ} Hz")
    print("Ctrl+C to stop\n")

    dt = 1.0 / SEND_HZ
    t  = 0.0

    # RX state
    rx_buf = b""
    rx_count = 0
    rx_freq_t0 = time.monotonic()
    last_rx_print = time.monotonic()
    rx_hz = 0.0

    try:
        while True:
            # ── TX ──
            frame = build_frame(t)
            ser.write(frame)

            # Print TX every ~1 second
            if int(t * SEND_HZ) % SEND_HZ == 0:
                positions = [sine_position(t, m["pos_min"], m["pos_max"]) for m in MOTORS]
                pos_str = "  ".join(f"m{m['id']}={p:+.4f}" for m, p in zip(MOTORS, positions))
                print(f"TX  t={t:6.2f}s  {pos_str}  [{len(frame)}B]")

            # ── RX ──
            incoming = ser.read(ser.in_waiting or 0)
            if incoming:
                rx_buf += incoming

            # Process complete frames
            while len(rx_buf) >= RX_FRAME_SIZE:
                rx_data = rx_buf[:RX_FRAME_SIZE]
                rx_buf = rx_buf[RX_FRAME_SIZE:]
                rx_count += 1
                motors = parse_rx_frame(rx_data)

                # Update RX frequency
                now = time.monotonic()
                elapsed = now - rx_freq_t0
                if elapsed >= 1.0:
                    rx_hz = rx_count / elapsed
                    rx_count = 0
                    rx_freq_t0 = now

                # Print RX every ~1 second
                if now - last_rx_print >= 1.0:
                    fb_str = "  ".join(f"m{m['id']}={m['pos']:+.4f}/{m['spd']:+.4f}" for m in motors)
                    print(f"RX  {fb_str}  [{RX_FRAME_SIZE}B @ {rx_hz:.1f}Hz]")
                    last_rx_print = now

            t += dt
            time.sleep(dt)

    except KeyboardInterrupt:
        print(f"\nStopped at t={t:.2f}s. Last frame held, closing port.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()