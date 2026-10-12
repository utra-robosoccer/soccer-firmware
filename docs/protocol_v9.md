# Protocol v9 — drive-state × streamed-tuple split

Status: **draft for agreement** (firmware ↔ software leads). Nothing wire-breaking merges
until this document is agreed. Field renames that don't change bytes (see §6) may merge
early under v8.

v9 replaces the per-motor *mode request* (IDLE/HOLD/MIT/DAMPED/TO_ZERO) with a CiA-402-style
split:

- a **drive request** says whether the motor should be on (`DISABLE / ENABLE / STOP / RESET`),
- the **MIT tuple** (pos, vel, kp, kd, tau_ff), streamed every cycle, says what an enabled
  motor should do.

Firmware stops making motion decisions. HOLD-as-a-command, TO_ZERO, and DAMPED-as-a-command
are deleted; holding a pose, going to zero, and damping are host behaviors expressed as
tuples. Firmware keeps exactly two autonomous jobs: the **enable/disable lifecycle** (with
fault latching) and the **failure-path stop ramp** when its command source dies.

## 1. Command (per motor, 12 bytes — layout unchanged)

| field | type | v8 name | meaning |
|---|---|---|---|
| `drive_req` | u8 | `mode_req` | one of `DriveReq` below |
| `pos, vel` | i16 | — | MIT tuple (scales unchanged: ×10000, ×100) |
| `kp, kd` | u16 | — | MIT tuple (×10, ×100) |
| `tau_ff` | i16 | — | MIT tuple (×100) |
| `flags` | u8 | — | bit0 `VALID`, bit1 `USE_CONFIG_GAINS` (bit2 `FAULT_RESET` **deleted** — RESET is a request) |

`DriveReq`: `DISABLE=0, ENABLE=1, STOP=2, RESET=3`. Requests are **level-triggered** (safe to
repeat every cycle; a repeated request in its target/transit state is a no-op).

The tuple is independent of the request: a frame can carry `ENABLE` *and* a VALID tuple.
`VALID=0` means "no tuple this frame" — the request byte still applies. There is no MIT
request: in ENABLED, every VALID tuple is simply applied.

## 2. Drive states and faults (telemetry)

`DriveState`: `BOOT=0, DISCOVERING=1, ABSENT=2, DISABLED=3, ENABLING=4, ENABLED=5,
STOPPING=6, CLEARING=7, FAULT=8`.

`DriveFault`: `NONE=0, CAN_TIMEOUT=1, MASTER_LOST=2, ENABLE_FAILED=3, NOT_ENABLED=4,
OVERTORQUE=5, MOTOR_REPORTED=6, WOUND=7`.

Deleted from v8: `WATCHDOG` (was never raised), `ZERO_TIMEOUT` (TO_ZERO is gone).
Renamed: `MOTOR_FAULT → MOTOR_REPORTED`. New: `ENABLE_FAILED`.

### State semantics

- **BOOT / DISCOVERING** — power-up and the boot CAN discovery. Unchanged.
- **ABSENT** — no feedback. Pinged (Type-0) every `ABSENT_PING_MS` (~100 ms); any reply →
  DISABLED. Replaces the old `alive` flag: *alive = drive_state ≠ ABSENT*.
- **DISABLED** — motor off, polled (all-zero Type-1 state poll) every cycle. Feedback
  timeout → ABSENT. Vendor fault bits or `rs_mode == Cali` → FAULT(MOTOR_REPORTED)
  *(behind a config mask, default off, until bench boot-rest data shows motors report
  clean at rest — see §8)*.
- **ENABLING** — the async handshake: run-mode write → Type-3 enable → wait for
  `rs_mode == Run` in feedback; `ENABLE_TIMEOUT_MS` without it → FAULT(ENABLE_FAILED).
  A VALID tuple streamed during ENABLING is **latched and applied on the ENABLING→ENABLED
  edge**, so a streaming host sees a limp window of zero cycles.
- **ENABLED** — **limp until the first VALID tuple.** No firmware hold-at-capture. With no
  tuple yet, firmware sends an all-zero Type-1 each cycle (a pure state poll per the RS
  reference), so the enable monitor, CAN feedback timeout, and telemetry keep working
  while the motor is limp. Every VALID tuple is clamped to the config envelope and sent.
- **STOPPING** — the ramped stop (grace at current velocity-zero MIT, then config-Kd
  damping, then Type-4 disable; reuses the proven master-loss ramp). A *requested* stop
  ends in DISABLED; a *fault-triggered* stop ends in FAULT (see latch rule). A
  comms-broken fault mid-ramp abandons the ramp with an immediate Type-4.
- **CLEARING** — entered only from FAULT via RESET. On entry: `drive_fault` is cleared and
  the Type-4 Byte0=1 fault-clear frame is sent. A fresh, clean Type-2 reply → DISABLED;
  a reply with fault bits still set → FAULT(MOTOR_REPORTED, re-latched); no reply within
  `CLEAR_TIMEOUT_MS` → ABSENT. **Never enables.**
- **FAULT** — latched; only RESET leaves it. The motor was Type-4-disabled on entry (or
  the stop ramp finished).

### Fault latch rule

`drive_fault` latches at fault **detection**, not at FAULT entry. First fault wins. If
`drive_fault ≠ NONE` while STOPPING, the stop ends in FAULT regardless of requests — a
DISABLE mid-ramp or ramp completion must not launder a fault-triggered stop into DISABLED.
`drive_fault` is cleared on CLEARING **entry**, so a re-latch reports the *current* fault,
not the original one.

## 3. Request × state matrix

"rej" = ignored + `REQUEST_REJECTED` telemetry flag. "no-op" = ignored silently (already
there / nothing to do).

| state \ req | DISABLE | ENABLE | STOP | RESET |
|---|---|---|---|---|
| BOOT / DISCOVERING | rej | rej | rej | rej |
| ABSENT | no-op | rej | no-op | rej |
| DISABLED | no-op | → ENABLING *(wound shaft → FAULT(WOUND), never enables)* | no-op | no-op |
| ENABLING | abort handshake, Type-4 → DISABLED | no-op | same as DISABLE (nothing to ramp yet) | rej |
| ENABLED | Type-4 now → DISABLED | no-op | → STOPPING | rej |
| STOPPING | immediate Type-4; end state per latch rule | no-op | no-op | rej |
| CLEARING | ignored until CLEARING resolves | ignored | ignored | ignored |
| FAULT | no-op (stays latched) | rej | no-op | → CLEARING |

## 4. Fault reactions

| fault | comms | reaction |
|---|---|---|
| OVERTORQUE | intact | → STOPPING ramp → FAULT |
| MASTER_LOST (slave stops hearing master) | intact | → STOPPING ramp → **FAULT(MASTER_LOST)** |
| CAN_TIMEOUT | broken | immediate Type-4 → FAULT |
| NOT_ENABLED (enable monitor) | broken | immediate Type-4 → FAULT |
| MOTOR_REPORTED (vendor bits / Cali) | broken | immediate Type-4 → FAULT |
| ENABLE_FAILED (ENABLING timeout) | broken | immediate Type-4 → FAULT |
| WOUND (shaft wound at ENABLE) | n/a | FAULT without ever enabling |

### Host loss vs master loss — deliberately different outcomes

- **Master loss** (slave stops hearing the master over SPI): slave-autonomous ramp →
  **FAULT(MASTER_LOST)** per motor. This is a change from v8, where the ramp ended in IDLE.
- **Host loss** (master stops hearing the host over USB): the master forces **STOP**
  requests into every slot. Slave-side this is an ordinary requested stop ending in
  **DISABLED with `drive_fault = NONE`**; the record of what happened is
  `robot_state = HOST_LOST` plus the master's latched `HOST_LINK_STOPPED`, which keeps
  forcing STOP (blocking re-enable) until a fresh command stream containing any RESET
  arrives.
- **Open question for the software lead:** is a per-motor `HOST_LOST` drive_fault wanted
  instead of the fault-free DISABLED + robot-level flag? The fault-free version keeps
  "fault" meaning "something broke at this motor"; the per-motor version makes every
  telemetry row self-describing.

The master's host-death watchdog arms when any motor has
`drive_state ∈ {ENABLING, ENABLED, STOPPING}` (replaces the v8 HOLD/MIT/DAMPED/TO_ZERO scan).

## 5. Timing constants (generated via gen_motor_config.py)

| constant | default | role |
|---|---|---|
| `ENABLE_TIMEOUT_MS` | 100 | ENABLING without `rs_mode == Run` → ENABLE_FAILED (new — v8 had no timer) |
| `CLEAR_TIMEOUT_MS` | 100 | CLEARING without a fresh reply → ABSENT (new) |
| `ABSENT_FB_TIMEOUT_MS` | 100 | DISABLED without feedback → ABSENT (reuses `MOTOR_CAN_FB_TIMEOUT_MS`) |
| `ABSENT_PING_MS` | 100 | ping period while ABSENT (new) |
| `STOP_GRACE_MS` | 50 | stop-ramp phase 1 (reuses `MASTER_LOST_GRACE_MS`) |
| `STOP_DAMP_MS` | 300 | stop-ramp phase 2 (reuses `MASTER_LOST_DAMP_MS`) |

Deleted: `MOTOR_WATCHDOG_MS` (never used), all `MOTOR_ZERO_*` constants.

## 6. Telemetry (21 bytes/motor — layout unchanged, fields renamed)

| v8 | v9 | note |
|---|---|---|
| `state` | `drive_state` | `DriveState` above |
| `cause` | `drive_fault` | `DriveFault` above |
| `motor_mode` | `rs_mode` | raw RobStride mode (Reset/Cali/Run) |
| `motor_fault` | `rs_fault_bits` | packed Type-2 fault bits |
| `fault_word` | `rs_fault_word` | raw 0x3022 fault register |

Telemetry flags: `TO_ZERO_ARRIVED` (bit1) deleted; the rest (`REQUEST_REJECTED`,
`SATURATED`, `CLAMPED_POS`, `CLAMPED_TAU`, `CMD_STALE`, `CLAMPED_GAIN`) unchanged.

Because no sizes change, the renames alone are wire-compatible with v8 and ship first;
the version bump to 9 lands with the semantic change.

## 7. What the host must now do (ships in the same release)

With HOLD gone and enable ending limp, a host that doesn't stream is a limp robot. The
reference flow (host tools and ros2_control alike):

1. **RESET** every motor until all read DISABLED with `drive_fault == NONE` (bounded
   retries; RESET is an operator/activation action, never an automatic mid-run reaction).
2. **ENABLE** every motor **while already streaming the hold tuple** (captured position,
   qualified gains) so the ENABLING→ENABLED edge applies it with zero limp cycles.
3. Stream the controller's tuples. Protective/fallback tuples are real damping/hold
   profiles from the safety manifest — never all-zeros.
4. **DISABLE** (or STOP) on shutdown.

## 8. Open items

- HOST_LOST per-motor fault vs robot-level only (§4).
- The DISABLED vendor-bit/Cali → FAULT check ships masked off until boot-rest telemetry
  from all bench motors over several power-ups shows no spurious bits (an
  uncalibrated-at-rest bit would otherwise fault every boot).
- Boot `discover()` stays blocking for now; ABSENT covers runtime presence.
