# Telemetry Protocol

How a motor's state travels from a slave to the master to the Jetson, and how to
read every field. The one unit that carries it all is the **`MotorState` atom** —
16 bytes per motor.

Source of truth: [`firmware/common/include/protocol.h`](../firmware/common/include/protocol.h)
(C) and [`host/jetson/protocol.py`](../host/jetson/protocol.py) (Python). If you
change one, change both — a test checks they agree byte-for-byte.

## The MotorState atom (16 bytes)

Everything we know about one motor, packed tight:

| bytes | field | type | what it means |
|---|---|---|---|
| 0–1 | `pos_raw` | u16 | position |
| 2–3 | `vel_raw` | u16 | velocity |
| 4–5 | `tau_raw` | u16 | measured torque |
| 6 | `temp_c` | u8 | temperature, whole °C |
| 7 | `state` | u8 | lifecycle + fault cause (see below) |
| 8 | `motor_fault` | u8 | the motor's own fault bits |
| 9 | `cmd_flags` | u8 | what happened to the command this tick |
| 10–13 | `fault_word` | u32 | detailed fault code (or 0) |
| 14 | `fb_age` | u8 | ms since the motor last reported |
| 15 | `_rsvd` | u8 | reserved (0) |

### Reading pos / vel / tau

These are stored as **scaled integers**, not real numbers, to save space. `0`
means the low end of the range, `65535` the high end. The host converts back to
radians / rad·s⁻¹ / N·m using the ranges generated from the active config
(so it's always right for whichever motors are plugged in). You never do this by
hand — `protocol.py` does it. Two things worth knowing:

- **Position is "home-frame" wrapped to ±π** — the angle relative to the joint's
  set zero, *not* a raw multi-turn count. So a joint sitting at its zero reads
  ~0, whichever way it powered up.
- The full scale is **±4π (±12.57 rad)**; velocity/torque scales are the widest
  motor model on the bus.

### Reading `state` (byte 7)

One byte holds **two** things — the low half is the lifecycle, the high half is
*why* it last left an active state:

```
  state = [ 7 6 5 4 | 3 2 1 0 ]
             cause      lifecycle
```

| lifecycle (low nibble) | meaning |
|---|---|
| 0 BOOT / 1 DISCOVERING | starting up / finding the motor |
| 2 IDLE | powered but not holding |
| 3 ARMED_HOLD | holding position |
| 6 ZEROING | creeping to its zero |
| 7 ARMED_MIT | following live commands |
| 4 FAULT / 5 DISABLED | faulted / off |

| cause (high nibble) | why it tripped |
|---|---|
| 0 NONE | normal |
| 1 OVERTORQUE | pushed past its torque limit |
| 2 CAN_TIMEOUT | stopped reporting (feedback went stale) |
| 3 WATCHDOG | master stopped commanding it |
| 4 MOTOR_FAULT | the motor's own protection fired |

The cause is **latched** — it stays set after a trip so you can see *what
happened* even after the motor dropped to IDLE. It clears when you re-arm.

### Reading `motor_fault` (byte 8)

Six flag bits straight from the motor, `1` = active:

| bit | 0 | 1 | 2 | 3 | 4 | 5 |
|---|---|---|---|---|---|---|
| meaning | undervoltage | driver | overheat | encoder | stall/overload | uncalibrated |

`0x00` is healthy. Example: `0x04` = overheat.

### Reading `cmd_flags` (byte 9)

Recomputed every tick — tells you what the slave did with the command *right
now*:

| bit | name | meaning |
|---|---|---|
| 0 | CLAMPED_POS | commanded angle was clipped to the joint's soft limit |
| 1 | CLAMPED_TAU | velocity feed-forward was cut at the limit |
| 2 | CMD_STALE | holding an old/watchdog command, not a fresh one |

Handy for tuning: if you command a big sweep and see `CLAMPED_POS`, the joint hit
its configured limit — that's the slave protecting the mechanism, working as
intended.

### Reading `fault_word` (bytes 10–13)

The motor's detailed fault register, fetched only when a `MOTOR_FAULT` happens:

- `0x00000000` — no fault.
- `0xFFFFFFFF` — fault detected, detail read is still pending (or failed).
- anything else — the raw fault code (see the motor reference for bit meanings).

### Reading `fb_age` (byte 14)

**Milliseconds since this motor last sent feedback**, capped at 255. A healthy
motor sits around a handful of ms. If it climbs toward 100 the motor is going
quiet, and at 100 ms the slave trips it (`CAN_TIMEOUT`). A parked value of **255
means "never heard from"** — usually a motor that isn't plugged in.

## The frame around the atoms

Atoms don't travel alone. A slave sends the master one **frame** holding every
motor plus a wrapper:

```
[ alive_mask ][ echo_seq ][ MotorState × N ][ health_rsvd (8) ][ crc16 (2) ]
      1 byte      1 byte      16·N bytes          reserved         checksum
```

- **`alive_mask`** — one bit per motor: is it responding?
- **`echo_seq`** — echoes the sequence number of the last command, so the master
  can tell its commands are getting through.
- **`health_rsvd`** — 8 spare bytes reserved for future per-slave health data.
- **`crc16`** — the integrity check (next section).

The master→Jetson hop wraps each atom in a small USB message with its own
16-byte header (type, sequence, timestamp, length, and its own CRC) — but the
atom inside is forwarded **unchanged**. The master doesn't reinterpret motor
data; the Jetson decodes it.

## The CRC: trust, but verify

A **CRC** is a checksum — a short number computed from all the bytes. The sender
computes it and appends it; the receiver recomputes it and compares. If even one
bit got corrupted in transit, the numbers won't match.

- The slave computes `crc16` over the *whole* frame (everything except the CRC
  itself) and appends it.
- The master recomputes it on arrival. **Match → trust the frame. Mismatch →
  throw the whole frame away** and count it (`crc_errors`); that motor's data
  just isn't updated this tick.

It's the same algorithm (CRC16-CCITT) on the USB hop, so a motor's data is
checked twice on its way to you.

Why it matters: the CRC turns *any* glitch — electrical noise, a mis-timed
buffer, an absent slave clocking back garbage — into a **dropped frame, never
wrong data**. If `crc_errors` is climbing fast, suspect wiring or a board that
isn't running; if it's parked at a fixed number, that's just start-up noise from
before both boards were in sync.

## Where to see it live

`python3 tools/telemetry.py <port>` shows one row per motor with all of the
above decoded — state, cause, pos/vel/tau, faults, flags, and fb_age — reading
the motor table straight from the active config.
