"""
motor_sine_sweep.py

Sends a sine-sweep position command over USB CDC to STM32F446 SPI master board.
Live matplotlib plot shows CMD vs RX position for motors 1 and 2 over a rolling 5s window.

C struct (__attribute__((packed))):
    typedef struct __attribute__((packed)) {
        uint8_t motor_id;   // 1 byte
        float   position;   // 4 bytes  (raw IEEE 754, little-endian)
        float   speed;      // 4 bytes
    } motor_cmd_t;           // total: 9 bytes
"""

import struct
import math
import time
from collections import deque
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ── Configuration ────────────────────────────────────────────────────────────

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE   = 115200
SEND_HZ     = 1000

NUM_MOTORS  = 2

MOTORS = [
    {"id": 2, "pos_min": -0.5, "pos_max":  0.0},
    {"id": 1, "pos_min": -0.5, "pos_max":  0.5},
]

SWEEP_SPEED = 0.1
SWEEP_FREQ  = 1.0

RX_NUM_MOTORS = 5

# Plot config
PLOT_WINDOW_S  = 5.0   # rolling window in seconds
PLOT_INTERVAL  = 50     # ms between plot refreshes

# ── Struct format ────────────────────────────────────────────────────────────

STRUCT_FMT     = "<Bff"
MOTOR_CMD_SIZE = struct.calcsize(STRUCT_FMT)
FRAME_SIZE     = MOTOR_CMD_SIZE * NUM_MOTORS
RX_FRAME_SIZE  = MOTOR_CMD_SIZE * RX_NUM_MOTORS

assert MOTOR_CMD_SIZE == 9
assert len(MOTORS) == NUM_MOTORS

# Motor IDs we care about for plotting
PLOT_IDS = [m["id"] for m in MOTORS]

print(f"Struct size: {MOTOR_CMD_SIZE} bytes per motor")
print(f"TX frame:    {FRAME_SIZE} bytes ({NUM_MOTORS} motors)")
print(f"RX frame:    {RX_FRAME_SIZE} bytes ({RX_NUM_MOTORS} motors)")


# ── Helpers ──────────────────────────────────────────────────────────────────

def sine_position(t: float, pos_min: float, pos_max: float) -> float:
    mid = (pos_max + pos_min) / 2.0
    amp = (pos_max - pos_min) / 2.0
    return mid + amp * math.sin(2.0 * math.pi * SWEEP_FREQ * t)


def build_frame(t: float) -> bytes:
    parts = []
    for m in MOTORS:
        pos = sine_position(t, m["pos_min"], m["pos_max"])
        parts.append(struct.pack(STRUCT_FMT, m["id"], pos, SWEEP_SPEED))
    return b"".join(parts)


def parse_rx_frame(data: bytes) -> dict:
    """Returns dict keyed by motor_id -> {pos, spd}."""
    result = {}
    for i in range(RX_NUM_MOTORS):
        motor_id, pos, spd = struct.unpack_from(STRUCT_FMT, data, i * MOTOR_CMD_SIZE)
        result[motor_id] = {"pos": pos, "spd": spd}
    return result


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0)
    print(f"Opened {SERIAL_PORT} @ {BAUD_RATE}")
    print("Ctrl+C or close plot window to stop\n")

    # Rolling data buffers: time + cmd/rx per motor
    ts_cmd   = deque()
    cmd_pos  = {mid: deque() for mid in PLOT_IDS}

    ts_rx    = deque()
    rx_pos   = {mid: deque() for mid in PLOT_IDS}

    # RX byte accumulator + frequency tracking
    rx_buf     = b""
    rx_count   = 0
    rx_freq_t0 = time.monotonic()
    rx_hz      = 0.0
    last_print = time.monotonic()

    t  = 0.0
    dt = 1.0 / SEND_HZ
    t0 = time.monotonic()  # wall-clock reference for plot x-axis

    # ── Set up matplotlib ────────────────────────────────────────────────
    plt.style.use("dark_background")
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
    fig.suptitle("Motor Position: CMD vs RX")

    # Motor 1 (MOTORS[0])
    line_cmd1, = ax1.plot([], [], "-",  color="cyan",   linewidth=1.5, label=f"m{PLOT_IDS[0]} cmd")
    line_rx1,  = ax1.plot([], [], "--", color="orange",  linewidth=1.5, label=f"m{PLOT_IDS[0]} rx")
    ax1.set_ylabel("Position (rad)")
    ax1.legend(loc="upper right")
    ax1.grid(True, alpha=0.3)

    # Motor 2 (MOTORS[1])
    line_cmd2, = ax2.plot([], [], "-",  color="cyan",   linewidth=1.5, label=f"m{PLOT_IDS[1]} cmd")
    line_rx2,  = ax2.plot([], [], "--", color="orange",  linewidth=1.5, label=f"m{PLOT_IDS[1]} rx")
    ax2.set_ylabel("Position (rad)")
    ax2.set_xlabel("Time (s)")
    ax2.legend(loc="upper right")
    ax2.grid(True, alpha=0.3)

    freq_text = fig.text(0.01, 0.01, "", fontsize=9, color="gray")

    def trim_deque(d, cutoff):
        while d and d[0] < cutoff:
            d.popleft()

    def update_plot(_frame_num):
        nonlocal t, rx_buf, rx_count, rx_freq_t0, rx_hz, last_print

        now_wall = time.monotonic()

        # Run several TX/RX cycles per plot refresh to keep up with SEND_HZ
        cycles = max(1, int(SEND_HZ * PLOT_INTERVAL / 1000))
        for _ in range(cycles):
            # ── TX ──
            frame = build_frame(t)
            ser.write(frame)

            wall = time.monotonic() - t0
            ts_cmd.append(wall)
            for m in MOTORS:
                cmd_pos[m["id"]].append(sine_position(t, m["pos_min"], m["pos_max"]))

            # ── RX ──
            incoming = ser.read(ser.in_waiting or 0)
            if incoming:
                rx_buf += incoming

            while len(rx_buf) >= RX_FRAME_SIZE:
                rx_data = rx_buf[:RX_FRAME_SIZE]
                rx_buf  = rx_buf[RX_FRAME_SIZE:]
                rx_count += 1
                fb = parse_rx_frame(rx_data)

                rx_wall = time.monotonic() - t0
                ts_rx.append(rx_wall)
                for mid in PLOT_IDS:
                    rx_pos[mid].append(fb.get(mid, {}).get("pos", float("nan")))

            # Frequency measurement
            elapsed = time.monotonic() - rx_freq_t0
            if elapsed >= 1.0:
                rx_hz = rx_count / elapsed
                rx_count = 0
                rx_freq_t0 = time.monotonic()

            # Terminal print every ~1s
            if time.monotonic() - last_print >= 1.0:
                positions = [sine_position(t, m["pos_min"], m["pos_max"]) for m in MOTORS]
                pos_str = "  ".join(f"m{m['id']}={p:+.4f}" for m, p in zip(MOTORS, positions))
                print(f"TX  t={t:6.2f}s  {pos_str}  |  RX @ {rx_hz:.1f}Hz")
                last_print = time.monotonic()

            t += dt
            time.sleep(dt)

        # ── Trim to rolling window ──
        cutoff = (time.monotonic() - t0) - PLOT_WINDOW_S
        trim_deque(ts_cmd, cutoff)
        for mid in PLOT_IDS:
            while len(cmd_pos[mid]) > len(ts_cmd):
                cmd_pos[mid].popleft()

        trim_deque(ts_rx, cutoff)
        for mid in PLOT_IDS:
            while len(rx_pos[mid]) > len(ts_rx):
                rx_pos[mid].popleft()

        # ── Update lines ──
        t_cmd_list = list(ts_cmd)
        t_rx_list  = list(ts_rx)

        line_cmd1.set_data(t_cmd_list, list(cmd_pos[PLOT_IDS[0]]))
        line_rx1.set_data(t_rx_list,  list(rx_pos[PLOT_IDS[0]]))
        line_cmd2.set_data(t_cmd_list, list(cmd_pos[PLOT_IDS[1]]))
        line_rx2.set_data(t_rx_list,  list(rx_pos[PLOT_IDS[1]]))

        # Rescale axes
        now_rel = time.monotonic() - t0
        for ax in (ax1, ax2):
            ax.set_xlim(max(0, now_rel - PLOT_WINDOW_S), now_rel)
            ax.relim()
            ax.autoscale_view(scalex=False)

        freq_text.set_text(f"RX: {rx_hz:.1f} Hz  |  TX: {SEND_HZ} Hz")

        return line_cmd1, line_rx1, line_cmd2, line_rx2, freq_text

    ani = animation.FuncAnimation(fig, update_plot, interval=PLOT_INTERVAL, blit=False, cache_frame_data=False)

    try:
        plt.tight_layout()
        plt.show()
    except KeyboardInterrupt:
        print(f"\nStopped at t={t:.2f}s. Last frame held.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()