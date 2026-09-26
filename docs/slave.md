# The Slave Board

Plain-language guide to what a slave MCU does and the few ideas that make it
reliable. If you touch motors, wiring, or bring-up, this is the one to read.

## What it is

Each **slave** is an STM32 that sits between the **master** and a **chain of
RobStride motors**. It has two jobs:

```
        SPI (to/from master)                 CAN (to/from motors)
 master ───────────────────▶  SLAVE  ───────────────────▶  motor 0, 1, 2, ...
        ◀───────────────────         ◀───────────────────
          commands / telemetry           commands / feedback
```

- **Down to the motors:** it sends each motor a command and reads back what the
  motor is actually doing.
- **Up to the master:** it receives high-level commands and reports a tidy
  snapshot of every motor's state.

A control loop runs at **200 Hz** (every 5 ms). Every tick it talks to each
motor once.

## Talking to a motor: Type-1 out, Type-2 back

RobStride motors speak a CAN protocol. The slave uses two message types in the
hot loop:

- **Type 1 — the command (a.k.a. "MIT" control).** Each tick the slave tells a
  motor five things at once:
  | field | meaning |
  |---|---|
  | position | where to go (radians) |
  | velocity | how fast (feed-forward) |
  | Kp | position stiffness (spring) |
  | Kd | velocity damping (shock absorber) |
  | torque | extra push (feed-forward) |

  Think of Kp/Kd as a tunable spring-and-damper: high Kp holds position firmly,
  Kd smooths it out. This is how the slave holds a joint, creeps it to zero, or
  streams a live trajectory.

- **Type 2 — the feedback.** The motor answers every command with its real
  state: **position, velocity, torque, temperature, fault bits, and mode**. This
  is the ground truth the slave reports upward and uses to protect the motor
  (e.g. cut power if measured torque exceeds the limit).

So each motor, each tick: **one Type-1 command → one Type-2 reply.** At 200 Hz
that's ~200 command/feedback exchanges per motor per second.

For the full CAN command set (zeroing, setting IDs, fault registers, per-model
ranges) see [`robostride-motor-reference.md`](robostride-motor-reference.md).

## The CAN interrupt (ISR): catching replies

The motors' Type-2 replies arrive **whenever the motor decides to send them** —
not on the slave's schedule. To never miss one, the slave uses an **interrupt**:
the moment a CAN message lands, the hardware pauses the main program and runs a
short handler that:

1. decodes the reply into that motor's slot in a private array (`motors[]`),
2. stamps the arrival time (used for the "feedback age" health signal),
3. latches a motor's fault register if it reported one.

Then it hands control back. The main loop never has to "wait" for a motor.

## The snapshot: reading motor state without tearing

Here's the subtle part. The ISR **writes** `motors[]` at random times. The main
loop **reads** it to run control and build telemetry. If the ISR fires *in the
middle* of the main loop reading a motor — say, after it read `position` but
before `velocity` — you'd get a **position from one instant and a velocity from
the next**. Mixed, incoherent data. Control decisions and reported telemetry
would be built on a state that never actually existed.

The fix is a **snapshot**: `motor_get_snapshot()` briefly disables interrupts,
copies the *whole* motor struct in one go, then re-enables them. The copy can't
be interrupted, so main always gets a clean, single-instant picture.

Two rules keep it honest:

- **One snapshot per motor per tick.** The control logic and the telemetry are
  built from the *same* snapshot — so **what we report is exactly what we acted
  on** (down to the feedback-age).
- `motors[]` is **private**; the only way for main-loop code to read it is the
  snapshot. This makes tearing impossible by construction.

(The interrupt-off window is a whole-struct copy — under a microsecond — far too
short to disturb CAN timing.)

## The ping-pong buffers: SPI without stalls

The master clocks the SPI link on *its* schedule; the slave's control loop runs
on *its own*. Neither should wait for the other, and a half-written telemetry
frame must never go out on the wire. The slave solves this with **two buffers
that take turns** ("ping-pong", a.k.a. double-buffering):

```
   TX-A ──▶ hardware is clocking this one out to the master
   TX-B ◀── software is filling this one with fresh telemetry
        (when the frame is complete, the two swap roles)
```

- The SPI hardware (via DMA — a data-mover that runs without the CPU) is always
  sending from one buffer.
- The main loop fills the *other* buffer with the next telemetry frame and only
  then flips a "ready" flag.
- At the end of each SPI transfer, the buffers swap **only if** a new frame is
  ready.

So the hardware is never blocked, the software is never rushed, and the master
only ever clocks out a fully-formed frame. The same trick runs on the receive
side for incoming commands. (This handshake is deliberately left untouched by
the snapshot work — it's a separate mechanism guarding a different seam.)

## Why this all matters

- **ISR** → never miss a motor reply.
- **Snapshot** → control and telemetry see coherent, real motor state.
- **Ping-pong** → the SPI link and the control loop run at full speed without
  tripping over each other, and no torn frames escape.

Every telemetry frame is also CRC-checked end-to-end (see
[`protocol.md`](protocol.md)), so even a rare glitch is *dropped*, never trusted.
