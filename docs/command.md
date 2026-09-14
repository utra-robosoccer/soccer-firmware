# Command Path — Jetson → Motor

Field-level walkthrough of the **downstream** (command) path, hardened to the same
standard as telemetry v1. Telemetry (upstream) is covered in
[`telemetry-path.md`](telemetry-path.md); this is its mirror.

> **Hardening applied (this pass):** ① slave command parse is now a coherent
> snapshot (no torn parse); ② master command-staging read/clear is masked (no torn
> setpoint, no lost one-shot bit); ③ the SPI **command frame is CRC-protected**
> (slave rejects bad commands, counts them); ④ the USB TX ring is single-producer
> again (ISR responses queued); ⑤ **seq/echo** gap counting is active as
> diagnostics. Telemetry v1 is **byte-identical** (fixture test unchanged).

## Layered diagram

```
 JETSON ── USB MsgHeader ──▶ MASTER ── SPI cmd frame ──▶ SLAVE ── CAN Type-1 ──▶ MOTOR
 (python)                    (STM32)                     (STM32)                (RobStride)

 encode_frame   CDC ISR:        pending_* staging      snapshot-parse (fix 1)   send_mit()
 (v1 + CRC)     accum + CRC     (ISR writes,           + cmd CRC (fix 3)        MIT: pos/vel/
 ControlReq /   + dispatch      main reads MASKED)     → motor_runtime_*        kp/kd/tau
 MotorCmd       (USB-ISR)       (fix 2)                → Type-1 CAN             + soft clamp
```

Every hop is now integrity-checked: **USB CRC + version** (host→master), **SPI
command CRC** (master→slave), and the two cross-context seams are exclusion-safe.

---

## Layer 1 — Jetson command API

The host sends two message types (`protocol.py`, used by `test_client.py` /
`dashboard.py`):

| msg | struct | when | rate |
|---|---|---|---|
| `MSG_CONTROL_REQ` (0x05) | `ControlReq{slave_id, motor_idx, cmd, reserved}` | on keypress | event |
| `MSG_MOTOR_CMD` (0x07) | `MotorCmd{slave_id, motor_idx, pos, vel, kp, kd, tau_ff}` | streamed | `SINE_RATE_HZ` = 200 Hz, deadline-paced (= the master poll rate; the useful ceiling — it coalesces above that) |

`cmd` values: `CTRL_ARM_HOLD=1`, `CTRL_DISABLE=2`, `CTRL_GOTO_ZERO=4`.
Every frame is wrapped by `encode_frame` → `MsgHeader` with **`ver_flags`=1** and a
CRC16. `kp/kd/tau_ff` in `MotorCmd` are currently ignored downstream (the slave
uses per-motor `default_kp/kd`); only `pos`/`vel` drive the motor.

---

## Layer 2 — USB ingress (master, USB-ISR context)

`CDC_Receive_FS` (`usbd_cdc_if.c`) runs in the **USB interrupt**:

1. Appends the packet to **`accum[128]`** (`:267`), a single-context reassembly
   buffer, and pulls complete `MsgHeader`+payload frames.
2. **CRC-checks** each (`proto_frame_crc`, `:287`); a bad frame is dropped
   (`master_link_errors++`). Overflow resets `accum` and resyncs.
3. Dispatches: `MSG_CONTROL_REQ` → `MotorMaster_HandleControlReq`;
   `MSG_MOTOR_CMD` → `MotorMaster_SetMitCmd`; `MSG_PING` → PONG.
4. NAKs further OUT packets until it returns → real USB RX backpressure.

**Note (known gap):** ingress validates CRC but does **not** enforce `ver_flags`
(the host stamps v1; the master accepts any CRC-valid version). Same-version by
deployment; a master-side version gate is a later add.

---

## Layer 3 — Master command staging

`HandleControlReq` / `SetMitCmd` (USB-ISR) write staging state that
`poll_one_slave` (main) consumes. **Fix 2:** main reads-and-clears staging inside
a short `__disable_irq()`/`__enable_irq()` critical section, and **snapshots the
MIT setpoints into a main-owned local** (`mit_local`) — so an ISR write mid-read
can't split a setpoint or lose a bitmask bit.

| variable (`spi_master.c:21–29`) | written by | read/cleared by | semantics |
|---|---|---|---|
| `pending_mit[s][m]` + `mit_pending[s]` | ISR (`SetMitCmd`) | main (masked) | **latest MIT wins** (overwrite = coalesce) |
| `pending_arm_bits[s]` | ISR (`HandleControlReq`) | main (masked read; masked clear) | one-shot; **cleared only on telemetry confirm** → retry until armed |
| `pending_goto_zero_bits[s]` | ISR | main (masked) | one-shot; retry until ZEROING/ARMED_HOLD |
| `master_armed[s]` / `send_disarm[s]` | ISR | main (masked) | arm latch / one-shot disarm |

**Priority per poll** (`poll_one_slave`): `DISARM > GOTO_ZERO > ARM > MIT > HOLD >
NOP`. One command per slave per 5 ms tick. If armed with no fresh MIT → `HOLD`
(refreshes the slave watchdog); if disarmed → `NOP`.

---

## Layer 4 — SPI command frame

Rides in the prefix of the same 60 B full-duplex transfer that returns telemetry
(`protocol.h`):

```
[ cmd u8 ][ seq u8 ][ SpiMitCmd × N ][ crc16 u16 ]   ... then zero-pad to the transfer length
```

- **`cmd`** — low nibble = op (`NOP 0 / ARM 1 / HOLD 2 / DISARM 3 / GOTO_ZERO 4 /
  MIT 5`), high nibble = motor index (for ARM / GOTO_ZERO).
- **`seq`** — increments every poll (`spi_seq[s]++`); echoed back in telemetry for
  gap detection.
- **`SpiMitCmd`** — `{float pos, float vel, uint8_t valid}` per motor (9 B); only
  used when `cmd == MIT`.
- **`crc16`** — **new (fix 3):** `proto_crc16` over `[cmd … last SpiMitCmd byte]`
  at offset `SPI_CMD_CRC_OFF(N)`. Built for *every* command (HOLD/NOP included).

**HOLD** = "stay armed, no new setpoint" (refreshes the slave watchdog so an armed
motor keeps holding). **NOP** = "nothing to say" (disarmed/idle). Neither moves a
motor; both still carry a valid CRC and seq.

---

## Layer 5 — Slave apply path

On `data_receive_flag` (`main.c`):

1. **Snapshot-parse (fix 1):** under a brief IRQ mask, `memcpy` the current
   `cmd_inbox_buf` into a main-owned `cmd_local`, clear the flag, unmask. Parse
   only `cmd_local` — the SPI ISR can reswap the buffer pointer, so parsing in
   place could split a command across two transfers. (Copy, not latch: after a
   swap the old buffer is the DMA's next write target.)
2. **CRC verify (fix 3):** `proto_crc16(cmd_local, SPI_CMD_CRC_OFF(N))` vs the
   trailer. **On failure: apply nothing, `cmd_crc_errors++`, leave `echo_seq`
   unchanged** (so the master sees a seq gap). No recovery logic — the master
   re-sends in 5 ms, the watchdog covers sustained loss.
3. On pass: `echo_seq = seq`; refresh every motor's watchdog (a valid command
   proves the link); dispatch:
   - `ARM` → `motor_runtime_arm(idx)` — Type-3 enable (+ Type-4 fault-clear if
     needed) → ARMED_HOLD.
   - `GOTO_ZERO` → `motor_runtime_goto_zero(idx)` — creep to home, then ARMED_HOLD.
   - `MIT` → per motor, `motor_runtime_apply_mit(i, pos, vel)`.
   - `HOLD` → refresh watchdogs. `DISARM` → `motor_runtime_disable(i)`.
4. **Type-1 CAN:** `send_mit` → `can_mit_control_set(pos+offset, vel, default_kp,
   default_kd)`.

**Clamp + feedback:** `apply_mit` clamps commanded `pos` to the motor's
`[soft_min, soft_max]` and applies a one-sided velocity clamp, setting
`cmd_flags` (`CLAMPED_POS` / `CLAMPED_TAU`). Those flags ride back to the host in
the telemetry atom — so the host sees, per tick, whether its command was clipped.

---

## Layer 6 — Safety semantics (end-to-end)

**The watchdog chain** (why nothing runs away):

| event | what happens | motor ends up |
|---|---|---|
| Jetson stops streaming MIT | master's `mit_pending` clears → master sends **HOLD** every poll → slave watchdog stays refreshed | **holds last setpoint** (does NOT idle) |
| Master dies / SPI stops | slave gets no commands → per-motor watchdog (200 ms) expires | **IDLE** (`CAUSE_WATCHDOG`) |
| Bad command CRC (fix 3) | slave applies nothing, counts it; seq gap at master | last good state; re-sent next tick |
| Bad USB CRC | master drops the frame (`master_link_errors`) | host retransmits (one-shots retry; MIT via next sample) |
| Motor overload | slave torque trip | **IDLE** (`CAUSE_OVERTORQUE`) |
| CAN feedback lost | slave `fb_age ≥ 100 ms` | **IDLE** (`CAUSE_CAN_TIMEOUT`) |

**Runaway is excluded by construction:** a setpoint is applied only if it is
(a) CRC-valid *and* (b) parsed coherently from one transfer; every applied
position is (c) clamped to soft limits on the slave; (d) MIT is latest-value, so a
stale command is *superseded*, never accumulated; and (e) any loss of commands
degrades to hold → watchdog → IDLE. There is no path that applies an unverified,
torn, unbounded, or accumulating command.

> **motor_cmd caveat:** Jetson silence *holds the last setpoint* (master HOLD), it
> does not stop the motor. Command logic must send an explicit `DISABLE` (or a
> zeroed MIT) to stop — never rely on silence.

---

## Assessment

**Now guaranteed at each seam** (excluded vs merely detected):

| seam | before | now |
|---|---|---|
| Slave command parse | torn parse possible (no exclusion) | **excluded** — snapshot copy under mask (fix 1) |
| Master staging | torn MIT / lost arm bit (ISR↔main RMW) | **excluded** — masked read-and-clear + local snapshot (fix 2) |
| SPI command integrity | unprotected (only telemetry CRC'd) | **detected + rejected** — command CRC, counted (fix 3) |
| USB TX ring | two-writer race (main + ISR) | **excluded** — SPSC via response queue (fix 4) |
| Rejected command | invisible | **detected** — `cmd_crc_errors` (per-command) + `seq_gaps` (link stall) in `SlaveStatus` (fixes 3, 5) |

**Diagnostics surfaced to the host** (`SlaveStatus`, 20 Hz):
- `crc_errors` — telemetry frames the master rejected (tele CRC).
- `cmd_crc_errors` — command frames the *slave* rejected on CRC, relayed via
  `slave_debug_rsvd[0..3]`. **This is the definitive per-command integrity signal**
  (0 = every command passed CRC).
- `seq_gaps` — a **command-link stall detector**, not a per-command counter. The
  echoed seq rides the coalesced telemetry stream, so it jitters ±1 poll in normal
  operation; a gap is counted only when the echo stays **frozen** for several polls
  while the master keeps sending (a blocking slave op or a dead link). Idle reads 0;
  single dropped commands are caught by `cmd_crc_errors`, not here.

The dashboard shows all three per slave.

**Remaining known-bounded gaps:**
- Master ingress checks CRC but **not** `ver_flags` on incoming commands
  (host/master are same-version by deployment).
- The main-side critical sections briefly disable *all* interrupts (~µs each) —
  negligible vs the 5 ms tick, but they exist.
- USB command layer has no idempotency token; a *duplicated* CRC-valid command
  would apply twice. Harmless today (MIT latest-value; ARM/DISARM/GOTO_ZERO
  idempotent), but motor_cmd should not assume exactly-once.

**Transport contract motor_cmd may assume:**
1. A CRC-valid v1 command either reaches the motor as a **coherent Type-1 within
   ~1–2 ticks**, or is **provably rejected** (visible in `cmd_crc_errors` /
   `seq_gaps`) — never silently mis-applied.
2. MIT is **latest-value**; stream continuously (≤ 200 Hz). **Silence = hold**, not
   stop.
3. Commanded positions are **clamped to soft limits on the slave**; clamping is
   reported back per-tick in the atom's `cmd_flags`.
4. Any hop failure degrades to **hold → watchdog → IDLE**; no runaway.
5. Both firmware images **must be built from the same active config** (N is
   compile-time on both sides; a mismatch presents as a dead slave).

## Verification

- **Both firmwares** build 0/0 (`scripts/build.sh --config slave0 …` and master).
- **Host tests** (`python3 -m unittest discover -s host/jetson/tests`): the
  cross-language fixture passes **byte-identical for telemetry** (unchanged) and
  now also validates the **command-frame CRC layout** (`CMDFRAME`); the
  status-struct sizes assert (14 / 18).
- **Command smoke test (run on hardware, zero-motion):** flashed both boards
  (the command-frame CRC is a wire change — both must run this firmware together),
  then observed `SlaveStatus` while idle. The master streams a CRC'd `NOP` every
  5 ms that the slave verifies each tick, so this exercises the full hardened
  command path — CRC build (master) → snapshot-parse + CRC verify (slave) → echo —
  without moving a motor. Result: **`cmd_crc_errors` delta 0, `seq_gaps` delta 0
  over 6 s, `version_errors` 0** — the command CRC round-trips cleanly and the
  stall detector is quiet. (A finding caught here: an initial per-poll seq-gap
  metric false-counted ~200/s because it assumed a fixed echo latency; replaced
  with the coalescing-robust stall detector above.)
- **Full motion test (host-side, needs an operator watching):** drive
  `arm → MIT sine → disarm` from `dashboard.py`; confirm the motor holds (not
  idles) when the sine stops and idles on `DISABLE`, and that `cmd_crc_errors` /
  `seq_gaps` stay flat throughout.
