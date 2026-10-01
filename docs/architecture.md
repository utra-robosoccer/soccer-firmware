# Robosoccer motor-control system — architecture

**Snapshot anchor:** commit `7622aee` (2026-09-28), branch `akp/single_motor_rework`.
**Doc date:** 2026-09-29.

> The working tree has uncommitted changes finalized around this snapshot; features
> that are staged-but-uncommitted are marked **(uncommitted)**: the enable monitor +
> `CAUSE_NOT_ENABLED`, the startup live-gate, in-band log-drop recording, and the
> policy `t_ns` time interface. Where the code and older docs disagreed, this doc
> follows the **code**; discrepancies are called out inline.

This is the overarching system doc. It replaces `protocol.md`, `telemetry-path.md`,
`slave.md`, and `command.md` (removed). The manufacturer CAN reference lives in
`robostride-motor-reference.md`; a specific study lives in `can-investigation.md`.

Active config for this snapshot: **`bench-1-motor`** — one slave (`slave0`), one RS02,
`N_MOTORS = 1`, `can_id = 1`.

---

## 0. The hops

```
 ┌─────────┐  policy    ┌───────────┐  USB CDC  ┌─────────┐   SPI    ┌─────────┐   CAN    ┌────────┐
 │ policy  │──────────▶ │  runner   │─────────▶ │ master  │────────▶ │ slave   │────────▶ │ RS02   │
 │ .step() │   Action   │run_policy │ MsgHeader │ STM32   │ full-dup │ STM32   │ Type-1   │ motor  │
 │         │◀────────── │ MasterLink│◀───────── │ F446    │◀──────── │ F446    │◀──────── │        │
 └─────────┘ LinkState  └────┬──────┘  frames   └─────────┘  frame   └─────────┘ Type-2   └────────┘
                             │
                             │ raw TX/RX bytes
                             ▼
                        ┌─────────┐   convert_log    ┌──────┐   plot_motor_state / latency
                        │ .bin log│───────────────▶  │ CSVs │──────────────────────────────▶ figures / stats
                        └─────────┘                  └──────┘
```

Encoding boundaries: the RobStride speaks **big-endian 16-bit** CAN fields; the slave
decodes those to `float`, then re-encodes to **little-endian 16-bit** `MotorState` for
SPI. From the slave's SPI output all the way to the host it is **little-endian,
pass-through** — the master never reinterprets motor data.

Wire contract source of truth: `firmware/common/include/protocol.h` (C, included by both
MCUs) mirrored by `host/master_link/protocol.py` (Python). A cross-language fixture test
(`firmware/common/test/gen_fixture.c` ↔ `host/tests/test_protocol.py`) byte-compares them.

Common integrity primitive: **CRC16-CCITT** (`poly 0x1021`, `init 0xFFFF`), `proto_crc16`.

---

## 1. Hop: policy → runner → MasterLink (host)

Pure-Python, on the Jetson/PC. Files: `host/policies/`, `host/apps/run_policy.py`,
`host/master_link/link.py`.

### Messages (in-process, not on a wire)
- **`Action`** (`policies/base.py`) — `mit: list[MitCommand]`, `control: list[ControlRequest]`.
  - `MitCommand{slave, local, pos, vel, kp, kd, tau_ff}` — floats, SI (rad, rad/s, N·m).
  - `ControlRequest{slave, local, kind}` — `kind ∈ {ARM, GOTO_ZERO, DISABLE}` (`ControlKind`).
- **`LinkState`** (`link.py`) — snapshot passed to the policy: `motors {(slave,local)→MotorSnap}`,
  `master`, `slaves`, `last_control_resp`, `stamp_ns`.
  - `MotorSnap` — decoded SI fields (`pos/vel/tau/temp`), `state`, `cause`,
    `motor_fault`, `cmd_flags`, `fault_word`, `fb_age`, `master_ts_ms` (frame header tick),
    `recv_ns` (host monotonic at receipt).

### Built / parsed
- Policy produces `Action` in `Policy.step(self, state, t_ns)`; the base class is
  `policies/base.py`. **(uncommitted)** `step`/`setup` take `t_ns` (runner's
  `time.monotonic_ns()`); policies must derive all timing from it, never read a clock.
- `run_policy.main` (`apps/run_policy.py`) is the loop; it calls `link.latest_state()` →
  `policy.step(state, t_ns)` → `link.send_mit()/send_control()`.

### Triggers / rate / buffering
- Deadline-scheduled loop at `--rate` Hz (default 50). Each tick: snapshot → step → send →
  `log_loop_timing`.
- `MasterLink` runs a background RX thread (`_rx_loop`) that owns the serial port; the
  policy only sees the latest snapshot (`latest_state()` copies under a lock). Intermediate
  frames are in the log but not in the snapshot.

### Checks / failure
- **(uncommitted) startup live-gate:** `run_policy` calls `link.wait_until_live(timeout=2.0)`
  before `setup()`/first `step()`. A motor is "live" once `master_ts_ms` advances in step
  with host monotonic time for **N=5** consecutive `MOTOR_STATE` frames
  (`|Δmaster−Δhost| ≤ max(3 ms, 0.5·Δmaster)`; `_LiveDetector`). On open, `MasterLink`
  also calls `reset_input_buffer()`. Timeout → exit naming the dead motors.
- TX write failures are logged as an `EVENT`, never counted as sent.

### Timeouts
- Live-gate timeout **2.0 s** → clean exit.
- No host-side command watchdog (see §9).

---

## 2. Hop: host ↔ master (USB CDC serial)

Raw `[MsgHeader(16 B)][payload]` frames over the STM32 "Virtual ComPort"
(VID:PID `0483:5740`, 115200 baud nominal — CDC ignores baud). Not COBS-framed;
boundaries are found by CRC resync.

### `MsgHeader` (16 B, LE, packed)
| field | type | meaning |
|---|---|---|
| `type` | u16 | `MsgType` |
| `seq` | u16 | per-message counter |
| `src`,`dst` | u8,u8 | `NodeId` (JETSON=1, MASTER=2, SLAVE_0=3) |
| `ts_ms` | u32 | sender `HAL_GetTick()` (master) / `monotonic_ns//1e6` (host) |
| `len` | u16 | payload bytes |
| `ver_flags` | u16 | low byte = `PROTO_VERSION` (=2), high byte reserved 0 |
| `crc16` | u16 | CRC16-CCITT over header(crc=0)+payload |

### Messages — host → master
| type | struct | fields / encoding | built | parsed | trigger/rate |
|---|---|---|---|---|---|
| `MSG_CONTROL_REQ` 0x05 | `ControlReq{slave_id,motor_idx,cmd,reserved}` u8×4 | `cmd`=`ControlCmd` (ARM_HOLD 1, DISABLE 2, GOTO_ZERO 4; SET_ZERO 3 = stub) | `link.send_control` (`FMT_CONTROL_REQ`) | `usbd_cdc_if.c CDC_Receive_FS` → `MotorMaster_HandleControlReq` | on event |
| `MSG_MOTOR_CMD` 0x07 | `MotorCmd{slave_id,motor_idx,pos,vel,kp,kd,tau_ff,cmd_seq}` u8,u8,f32×5,u16 | SI floats; **`kp/kd/tau_ff` ignored downstream** — slave uses per-motor `default_kp/kd`. `cmd_seq` = one per policy tick (≥1, 0 reserved), echoed back as `last_applied_seq` (§5, §6) | `link.send_mit` (`FMT_MOTOR_CMD`) | `CDC_Receive_FS` → `MotorMaster_SetMitCmd` | streamed at loop rate (≤200 Hz useful — coalesced above) |
| `MSG_PING` 0x01 | (empty) | — | `link._write_frame(encode_frame(PING))` | `CDC_Receive_FS` → PONG | on request |

> **Discrepancy:** `protocol.h` labels `MSG_MOTOR_CMD` "stub – not wired" — **wrong**; it is
> fully wired (host → master → slave). Trust the code.

### Messages — master → host
| type | struct | fields / encoding | built | parsed | trigger/rate |
|---|---|---|---|---|---|
| `MSG_MOTOR_STATE` 0x04 | `MotorStatePayload{slave_id,motor_idx,MotorState atom}` | atom = 18 B, see §5 | `emit_motor_state` (`spi_master.c`) | `MasterLink._ingest`→`parse_motor_state` | 200 Hz, **emission-gated** (only when the slave's poll passed CRC that tick) |
| `MSG_MASTER_STATUS` 0x02 | `MasterStatus{robot_state,slave_alive,uptime_ms,link_errors,rx_frames}` | `robot_state`=`RobotState`; `slave_alive` bitmask | `emit_master_status` | `parse_master_status` | 20 Hz |
| `MSG_SLAVE_STATUS` 0x03 | `SlaveStatus{slave_id,motors_alive,uptime_ms,crc_errors,cmd_crc_errors,seq_gaps}` | counters u32 | `emit_slave_status` | `parse_slave_status` | 20 Hz |
| `MSG_PING` 0x01 (PONG) | (empty) | **echoes request `seq`** | `usbd_cdc_if.c` → posted to TX ring | logged as RX_FRAME | on PING |

### Encode / decode
- Encode: `proto_build` (C) / `encode_frame` (Py). Decode: `decode_frame` (Py, resync on
  bad CRC by dropping one byte); master ingress reassembles in `accum[128]`.

### Checks / failure
- **CRC** both directions. Host: `decode_frame` drops one byte & retries on mismatch
  (logged coalesced as `RX_DISCARD`). Master ingress: bad CRC dropped, `master_link_errors++`.
- **Version:** host **drops + counts** any frame with `ver_flags` low byte ≠ `PROTO_VERSION`
  (logged as a raw `RX_FRAME`, flagged by `convert_log`). **Master ingress also gates now:**
  `CDC_Receive_FS` checks the low byte of `ver_flags` on each CRC-valid host frame and, on a
  mismatch, skips dispatch and increments `master_proto_ver_mismatch` (added with
  `PROTO_VERSION`=2). Both ends therefore reject a wrong-version peer rather than
  misparsing the grown structs.
- USB RX backpressure: `CDC_Receive_FS` NAKs further OUT packets until it returns.

### Buffering
- Master egress: single-producer TX **ring** (`usb_tx.c`); ISR-built responses
  (ControlResp/PONG) are posted and drained by main into the ring.
- Master ingress: `accum[128]` reassembly (USB-ISR context).

### Latency (measured)
- `MSG_PING` host↔master RTT: **median 1.56 ms** (min 0.67, p95 2.64, max 3.34) —
  `logs/2026-09-29/21-25-09_latency.bin`. This is the pure link, no motor/CAN.
- **cmd_seq latency** (`host/analysis/latency.py`, primary): TX of a command's `cmd_seq`
  → the first telemetry whose `last_applied_seq ≥` it (wrap-aware). This **includes the full
  return path** — host→master→SPI→slave→CAN→motor to apply, then
  motor→CAN→SPI→master→USB→host for the echo to come back — so it is strictly larger than
  the one-way command delay. Reported per-tick (all commanded motors applied) and per-motor,
  each with a never-applied count.
  - **Measured (2026-10-01):** **median 19.17 ms** (min 17.67, p95 20.58, max 21.17),
    **0 never-applied of 1500** — `logs/2026-10-01/10-35-16_man_1s_1m.bin`, a 30 s in-range
    MIT sine at 50 Hz on the `bench-1-motor` config. The command traverses, in order:
    host `send_mit` → USB-CDC → master `pending_mit` → SPI poll → slave `apply_mit` →
    Type-1 MIT frame → RS02 controller **(applied)**; the echo then returns Type-2 feedback →
    slave pairs it into `last_applied_seq` → SPI telemetry → master pass-through → USB-CDC →
    host. The tight ~17–21 ms band reflects 200 Hz SPI poll quantization (±5 ms) plus the
    CAN and telemetry-emission timing. It lands **below** the ~29 ms torque-onset step
    latency because `last_applied_seq` flips when the motor's feedback *acknowledges* the
    command, which precedes a physically measurable torque departure.
  - Compare against the physical torque-onset step latency (~29 ms median from
    `logs/2026-09-29/21-25-09_latency.bin`).

---

## 3. Master internals

STM32F446 (`firmware/master/`). No motor logic — it is a USB↔SPI bridge + command stager +
telemetry forwarder. Main loop drives three timers.

### Loops / rates
| loop | period | does |
|---|---|---|
| command poll | `MASTER_POLL_PERIOD_MS = 5 ms` (200 Hz) | `poll_one_slave` — one SPI full-duplex exchange per slave |
| telemetry emit | `MASTER_TELE_PERIOD_MS = 5 ms` (200 Hz) | `emit_motor_state` per motor (gated on last poll CRC) |
| status emit | `MASTER_STATUS_PERIOD_MS = 50 ms` (20 Hz) | `emit_master_status` + `emit_slave_status` |

Poll and tele run in lockstep: a CRC-passed poll updates `latest_atom[s][]` and enables that
tick's emit; a failed/absent poll emits nothing for that slave (silence = dead).

### Command staging (USB-ISR writes, main reads)
`spi_master.c` variables, consumed by `poll_one_slave` under a brief `__disable_irq()`:
- `pending_mit[s][m]` + `mit_pending[s]` — **latest MIT wins** (overwrite = coalesce). Cleared when sent.
- `pending_arm_bits[s]`, `pending_goto_zero_bits[s]` — one-shot bitmasks; **cleared on telemetry confirmation** (retry each poll until the motor reports the target state).
- `master_armed[s]`, `send_disarm[s]` — arm latch / one-shot disarm.
- **Per-poll priority:** `DISARM > GOTO_ZERO > ARM > MIT > HOLD > NOP`. One command per slave per tick. Armed + no fresh MIT → `HOLD` (refreshes the slave watchdog). Disarmed → `NOP`. ARM/GOTO_ZERO send **one motor index per poll** (`__builtin_ctz` of the pending bits).

### `CONTROL_RESP` — built immediately, optimistic
`MotorMaster_HandleControlReq` (USB-ISR) builds `ControlResp{slave_id,motor_idx,cmd,result,new_state,req_seq}` **right away** and posts it to the TX ring — reporting the *intended*
`new_state` (e.g. `MOTOR_ARMED_HOLD` for a valid ARM, `MOTOR_ZEROING` for GOTO_ZERO,
`CTRL_ERR_STUB` for SET_ZERO). It also stages the `pending_*` bit.
> **Discrepancy:** older docs implied `CONTROL_RESP` waits for telemetry confirmation. It
> does not — `result=OK` means "accepted & staged," **not** "motor physically armed."
> Physical arming happens later on the slave; observe the motor's `state` in telemetry to
> confirm.

### Checks / failure
- SPI telemetry CRC verified in `spi_exchange`; fail → `crc_errors++`, no emit that tick.
- Echoed command `seq` gap detection (`echo_stall`, `SEQ_STALL_POLLS = 5`) → diagnostics
  (`seq_gaps`), not a hard fault.

### Timeouts
- None that safe the robot. If the host stops, the master keeps polling and sending `HOLD`
  (motor stays armed) — see §9 (no host-death timeout).

---

## 4. Hop: master ↔ slave (SPI)

One **full-duplex** transfer per poll (200 Hz): the master clocks out a command frame while
the slave clocks out its telemetry frame simultaneously. Transfer length = the (larger)
telemetry frame size, `SPI_TELE_FRAME_SIZE(N) = 2 + 18·N + 8 + 2` (N=1 → 30 B).
Master SPI: `SPI_MODE_MASTER`, CPOL=0/CPHA=0, MSB-first, prescaler 64.

### Command frame (master → slave), CRC-protected
```
[ cmd u8 ][ seq u8 ][ SpiMitCmd × N ][ crc16 u16 ]  (zero-padded to transfer length)
```
- `cmd`: low nibble = opcode (`NOP 0 / ARM 1 / HOLD 2 / DISARM 3 / GOTO_ZERO 4 / MIT 5`),
  high nibble = motor index (ARM/GOTO_ZERO).
- `seq`: `spi_seq[s]++` each poll; echoed in telemetry. **This is the SPI link-health seq —
  distinct from the host `cmd_seq`** carried inside `SpiMitCmd` below.
- `SpiMitCmd{float pos, float vel, uint8_t valid, uint16_t cmd_seq}` (11 B) — per motor; used
  only for MIT. `cmd_seq` is the host command sequence (§2), forwarded verbatim so the slave
  can echo it back as `last_applied_seq` (§5, §6).
- `crc16`: `proto_crc16` over `[cmd … last SpiMitCmd byte]` at `SPI_CMD_CRC_OFF(N)`; built
  for **every** command (HOLD/NOP included).
- Built: `poll_one_slave`/`spi_build_cmd` (`spi_master.c`). Parsed: slave `main.c` +
  `spi_proto_parse` (`spi_proto.c`).

### Telemetry frame (slave → master), CRC-protected
```
[ alive_mask u8 ][ echo_seq u8 ][ MotorState × N ][ slave_debug_rsvd[8] ][ crc16 u16 ]
```
- `alive_mask` bit i = motor i alive; `echo_seq` echoes the last command seq (gap detect).
- `slave_debug_rsvd[8]`: `[0..3] = cmd_crc_errors (u32 LE)`, `[4..7]` reserved 0 — CRC-covered.
- `crc16` over all preceding bytes.
- Built: `spi_proto_build_tele` (`spi_proto.c`). Parsed/verified: master `spi_exchange`.

### Triggers / buffering
- Triggered by the master's 200 Hz poll clock. The slave uses **ping-pong (double-buffered)
  DMA**: hardware clocks one buffer while the main loop fills the other; buffers swap at
  end-of-transfer only if a fresh frame is ready → no torn frames, neither side stalls.
- The slave parses the received command from a main-owned copy (`cmd_local`) taken under a
  brief IRQ mask (the SPI ISR can reswap the inbox pointer mid-parse).

### Checks / failure
- **Command CRC** (slave): fail → apply nothing, `cmd_crc_errors++`, leave `echo_seq`
  unchanged so the master sees a seq gap. Recovery = the master re-sends in 5 ms; the
  watchdog covers sustained loss.
- **Telemetry CRC** (master): fail → drop the whole frame, `crc_errors++`, no emit.
- **N agreement is compile-time only**, from the same generated config on both sides. A
  mismatch is **undetectable in-band** — presents as a permanently dead slave with climbing
  `crc_errors` (see §8, §9).

### Timeouts
- None at the SPI layer itself; the *slave's* per-motor command watchdog (§6) is refreshed by
  every valid SPI command (incl. HOLD), so it is really the "master-link alive" watchdog.

---

## 5. The `MotorState` atom (18 B) — the telemetry unit

Built in `motor_runtime_sample` + `spi_proto_build_tele` (slave); consumed by
`protocol.py parse_motor_state` (host). Forwarded byte-identical by the master.

| off | field | type | units | encoding / precision |
|---|---|---|---|---|
| 0 | `pos_raw` | u16 | rad | **home-frame wrapped [−π,π]**, quantized over ±12.57 (±4π) → 0..65535; res ≈ **0.00038 rad** |
| 2 | `vel_raw` | u16 | rad/s | over global ±`MOTOR_V` (widest model, RS02 ±44) → 0..65535; res ≈ 0.0013 |
| 4 | `tau_raw` | u16 | N·m | over global ±`MOTOR_T` (RS02 ±17) → 0..65535; res ≈ 0.0005 |
| 6 | `temp_c` | u8 | °C | integer degrees (CAN ×10 → ÷10 → truncated) |
| 7 | `state` | u8 | enum | `[3:0]` `MotorLifecycle` · `[7:4]` `MotorFaultCause` |
| 8 | `motor_fault` | u8 | bits | b0 undervolt, b1 driver, b2 overheat, b3 encoder, b4 stall/overload, b5 uncalibrated |
| 9 | `cmd_flags` | u8 | bits | b0 `CLAMPED_POS`, b1 `CLAMPED_TAU`, b2 `CMD_STALE` — recomputed each tick |
| 10 | `fault_word` | u32 | code | `0` clear · `0xFFFFFFFF` read pending/fail · else raw `0x3022` register |
| 14 | `fb_age` | u8 | ms | ms since this motor's last Type-2, **saturating 255** (CAN hop only; frozen at pack time) |
| 15 | `reserved_v2` | u8 | — | 0, append-only growth slot (one byte still spare after v2) |
| 16 | `last_applied_seq` | u16 | — | host `cmd_seq` the motor last confirmed applied; `0` (`CMD_SEQ_NONE`) = none since arm; pairing in §6 |

**`last_applied_seq` semantics (added v2):** the slave pairs each MIT (Type-1) frame it sends
with the next fresh Type-2 feedback and, on that reply, promotes the pending `cmd_seq` to
`last_applied_seq` (the reply-window pairing in `cmd_seq_track.c`). **Pairing is MIT-only** —
any non-MIT frame to the motor (enable, mode-change, disable, set-zero) closes the window
*without* updating `last_applied_seq`, so an enable/mode reply is never miscounted as a
command being applied. It is **frozen** (holds its last value) while merely holding or idle and
**reset to 0** on arm / goto-zero / disable; zeroing is not a host command so it stays 0
throughout. This is the field `latency.py` matches against (§2, wrap-aware); the pairing
mechanics are in §6.

**Encoding notes / where precision is lost:** float→u16 quantization on pos/vel/tau (clamped
to the global bound; overshoot saturates); temperature truncated to whole °C. `pos_raw` is the
**wrapped home-frame** angle — multi-turn winding is *not* represented (deliberate; see the
position-wrap handling in §6). The atom is **append-only**: never insert a field mid-struct
(offsets shift → silent host corruption); grow at the end + bump `PROTO_VERSION` (v1→v2 did
exactly this, appending `last_applied_seq` after `reserved_v2`, which stays a spare byte).
Guards: `_Static_assert(sizeof==18)` + the cross-language fixture test.

---

## 6. Slave internals

STM32F446 (`firmware/slave/slave_general/`). 200 Hz control loop
(`LOOP_POLL_PERIOD_MS = 5 ms`) driving `motor_runtime_update`.

### Pieces
- **CAN RX ISR** (`motor_chain.c HAL_CAN_RxFifo0MsgPendingCallback`): decodes each Type-2
  into a private `motors[]` slot, stamps `last_fb_ms` and per-motor `fb_count`, latches the
  `0x3022` fault register on a Type-2 fault. Never blocks the main loop.
- **Snapshot** (`motor_get_snapshot`): IRQ-masked whole-struct copy — the *only* read path
  into `motors[]`. One snapshot per motor per tick → control and telemetry act on the same
  coherent state.
- **Per-tick control** (`motor_runtime_update`): fault checks, then a per-state action; sends
  Type-1 via `send_mit`→`can_mit_control_set`→`can_tx`.
- **SPI command handler** (`main.c`): CRC-verify `cmd_local`, dispatch to
  `motor_runtime_arm/goto_zero/apply_mit/disable`.
- **Telemetry build** (`spi_proto_build_tele`) on each new CAN feedback → staged into the
  ping-pong TX buffer.

### Per-tick MIT/HOLD path is non-blocking
`ARMED_HOLD`/`ARMED_MIT` only call `send_mit` (queue-and-return) + a watchdog check. No
`HAL_Delay`, no reply-wait. The only bounded wait is inside `can_tx`: a busy-wait for a free
TX mailbox capped at **~1 ms** (frees in ~110 µs). Blocking waits exist only in `discover()`
(startup), the entry handshakes (`motor_runtime_arm`, `motor_runtime_goto_zero`, ~40 ms each),
and the ZEROING-arrival re-enable — **not** the steady-state path (see §9).

### Soft limits + clamp
`motor_runtime_apply_mit` clamps commanded `pos` to `[soft_min, soft_max]` and applies a
**one-sided velocity clamp** (only cancels feed-forward driving *further into* the limit),
setting `CLAMPED_POS`/`CLAMPED_TAU` in `cmd_flags`. Host sends the full command; the slave
enforces the clamp.

### cmd_seq reply-window pairing
`cmd_seq_track.c` (pure, host-tested) turns each host `cmd_seq` into the echoed
`last_applied_seq` (§5). `apply_mit` stores the command's `cmd_seq` as the motor's current
target; the next `send_mit` stamps it onto the Type-1 frame and **opens a reply window**
(`cmd_seq_on_mit_frame`); the next fresh Type-2 feedback **closes it** and promotes the target
to `last_applied_seq` (`cmd_seq_on_reply`, driven by the same `fb_count`-change signal the
enable monitor uses, evaluated *before* the per-state action so it pairs with the previous
tick's frame). A non-MIT frame calls `cmd_seq_on_other_frame` to drop the window without
crediting it (so the ZEROING-arrival enable/mode replies don't count); arm/goto-zero/disable
`cmd_seq_reset` back to 0. The reader/host comparison is **wrap-aware** (`protocol.seq_ge`,
RFC-1982).

### Enable monitor **(uncommitted)**
Per tick, `enable_monitor_step` (pure, `enable_monitor.c`) checks that an armed motor reports
running mode (`RS_MODE_NORMAL`) on fresh feedback frames; after **K=`MOTOR_ENABLE_MON_K`=5**
consecutive not-running fresh frames it faults `CAUSE_NOT_ENABLED`. Suspended during the
ZEROING-arrival re-enable window.

---

## 7. Hop: slave ↔ motor (CAN, RobStride)

1 Mbps, sample point ~81% (APB1 42 MHz, presc 2, BS1 16TQ, BS2 4TQ, SJW 1TQ),
**`AutoRetransmission = DISABLE` (one-shot)**, `AutoBusOff = DISABLE`. 29-bit extended IDs:
`mode[28:24] | data[23:8] | node_id[7:0]`. Full command set: `robostride-motor-reference.md`.

### Messages the hot loop uses (`robostride.c`)
| type | dir | fields / encoding | built | parsed |
|---|---|---|---|---|
| **Type 1** (MIT op-control) | slave→motor | ID data[23:8]=torque_ff (u16 over ±T); payload BE u16×4 = pos(±4π), vel(±V), Kp(0–500), Kd(0–5) | `can_mit_control_set` | motor |
| **Type 2** (feedback) | motor→slave | ID: `[7:0]` id, `[13:8]` 6 fault bits, `[15:14]` mode (0 reset/1 cal/2 normal); payload BE u16×4 = pos,vel,torque,temp×10 | motor | `can_unpack_motor_feedback` |
| **Type 3** enable, **Type 4** stop (`Byte0=1` clears faults), **Type 18** write run-mode (0x7005), **Type 6** set-zero, **Type 0/7/17** id/param | slave→motor | per manual | `can_enable_motor` / `can_disable_motor` / `can_clear_fault` / `can_change_motor_mode` / `can_set_mech_zero` / … | motor |

Encoding: `float_to_uint`/`uint_to_float` over per-model ranges (RS00 vs RS02 differ on V/T;
pos/Kp/Kd shared). The slave then re-encodes to the **global** bounds for the SPI atom.

### Triggers / rate
- Type-1 sent once per motor per 200 Hz tick (in `ARMED_*`/`ZEROING`); Type-2 arrives async
  (reply to each command) → RX ISR. Zero-order-hold: between host updates the slave re-sends
  the last `hold_pos/hold_vel` every tick.

### Checks / failure
- No CRC (CAN has its own). A disabled motor still replies to Type-1 with `mode=reset`.
- **One-shot TX:** a Type-3 enable (or any frame) that loses arbitration/errors is **not
  retransmitted** — the root cause of the rare (~0.2%, not 4%) enable loss (see
  `can-investigation.md`).

### Timeouts
- `MOTOR_CAN_FB_TIMEOUT_MS = 100 ms`: a driving motor whose Type-2 goes stale ≥100 ms →
  `CAUSE_CAN_TIMEOUT`.

---

## 8. Host logging path (log → convert → plot)

`MasterLink` logs **raw wire bytes** on the hot path (no decode); everything is decoded
offline.

### Binary log (`.bin`) — `datalog/format.py`, `writer.py`
Header `RLOG` + `{log_fmt_version, proto_version, wall_start_ns, mono_start_ns, JSON meta}`
(meta = git commit/dirty, config name+hash+YAML). Records `[kind u8][mono_ns u64][len u32][payload]`:
`RX_FRAME`, `TX_FRAME`, `RX_DISCARD`, `EVENT`, `ANNOTATION`, `LOOP_TIMING`, and **(uncommitted)
`LOG_DROP`** (`{count, first_ns, last_ns}` — emitted in-band when the bounded writer queue
overflows, so a log's completeness is provable). `TX_FRAME` is timestamped just before the
write and only logged on write success.
- Writer: background thread, bounded queue (100 000), flush every 1 s. Real captures run
  270–520 rec/s — the queue never fills; drops seen in tests were the `queue_max=8` unit test.

### `convert_log.py`
`.bin` → a session folder of CSVs (`motor_state`, `motor_cmd`, `status`, `control_resp`,
`events`, `loop_timing`). Decodes each frame; `motor_state.csv` has `cause` + **`cause_name`**
(from the single `CAUSE_NAMES` in `protocol.py`) and the echoed **`last_applied_seq`**, and
`motor_cmd.csv` carries the per-tick **`cmd_seq`** — the two columns `latency.py` pairs. Flags
wrong-version frames; prints **`log complete: 0 records dropped`** or an INCOMPLETE warning
from `LOG_DROP` records.

### `plot_motor_state.py`
Session folder or `.bin` → per-motor pos/vel/tau figures (measured `x`, commanded `+`),
fault onsets as red verticals. Anchors on `master_ts_ms`, trimming the stale pre-open prefix.

### `latency.py`
`.bin` → **cmd_seq latency** (per-tick and per-motor: TX of a `cmd_seq` → first telemetry with
`last_applied_seq ≥` it, wrap-aware, **including the return path** — see §2 — each with a
never-applied count), plus step-latency (cmd TX → first torque response beyond noise) and
ping-RTT distributions for comparison.

---

## 9. Slave motor state machine

`MotorLifecycle` (low nibble of `state`), driven by `motor_runtime_update` + the command
handlers:

```
 BOOT ─▶ DISCOVERING ─▶ IDLE ──ARM──────────────▶ ARMED_HOLD ──(fresh MIT)──▶ ARMED_MIT
                         │  ▲                         │  ▲                         │
                         │  │   watchdog / disable    │  └── watchdog (200ms) ─────┘
                         │  └─────────────────────────┤       falls back to HOLD
                         │                            │
                         ├──GOTO_ZERO──▶ ZEROING ──arrival──▶ ARMED_HOLD
                         │                  │
                         │                  └── stall / host-stop / fault
                         ▼
                       FAULT ◀── any fault trip (idles motor, latches cause)
```

Transitions & triggers:
- **BOOT→DISCOVERING→IDLE:** startup probe (`discover()`); a found, responsive motor → IDLE.
- **IDLE→ARMED_HOLD:** `SPI_CMD_ARM` → `motor_runtime_arm` (mode-change + enable; clears
  whitelisted latched faults). Gate: must be IDLE + alive. (Block action if motors not equal config)
- **IDLE→ZEROING→ARMED_HOLD:** `SPI_CMD_GOTO_ZERO` → leashed creep to home → set-zero +
  re-enable → hold. Gates: IDLE, `0 ∈ [soft_min,soft_max]`, latched cause ∉ {OVERTORQUE,
  MOTOR_FAULT} (those need an explicit ARM).
- **ARMED_HOLD↔ARMED_MIT:** a fresh MIT command → ARMED_MIT; loss of fresh MIT for the
  watchdog window → back to ARMED_HOLD (holds last position).
- **any ARMED/ZEROING→FAULT/IDLE:** a fault trip idles the motor and latches the cause.
- **→DISABLED/IDLE:** `SPI_CMD_DISARM` → `motor_runtime_disable`.

### Fault causes (`MotorFaultCause`, high nibble; latched until re-arm/disable)
| cause | value | raised by |
|---|---|---|
| `CAUSE_NONE` | 0 | — |
| `CAUSE_OVERTORQUE` | 1 | measured `|tau| > cfg->max_tau` while driving (`motor_runtime_update`) |
| `CAUSE_CAN_TIMEOUT` | 2 | Type-2 feedback stale ≥ `MOTOR_CAN_FB_TIMEOUT_MS` (100 ms) while driving |
| `CAUSE_WATCHDOG` | 3 | SPI-command watchdog expired ≥ `MOTOR_WATCHDOG_MS` (200 ms) — master link stopped |
| `CAUSE_MOTOR_FAULT` | 4 | RS motor's own Type-2 fault bits set (`motor_fault != 0`); triggers a `0x3022` read |
| `CAUSE_ZERO_TIMEOUT` | 5 | ZEROING made no progress toward home for `MOTOR_ZERO_STALL_MS` (1500 ms) |
| `CAUSE_NOT_ENABLED` **(uncommitted)** | 6 | armed motor reported not-running for K=5 fresh feedback frames (enable didn't take/dropped) |

### Timeouts / watchdogs (values + effect)
| name | value | effect |
|---|---|---|
| `MOTOR_WATCHDOG_MS` | 200 ms | no valid SPI command in window → armed motor → IDLE (`CAUSE_WATCHDOG`); ARMED_MIT first falls back to ARMED_HOLD |
| `MOTOR_CAN_FB_TIMEOUT_MS` | 100 ms | stale feedback → `CAUSE_CAN_TIMEOUT` |
| `MOTOR_ZERO_STALL_MS` | 1500 ms | no homing progress → damp + `CAUSE_ZERO_TIMEOUT` |
| `can_tx` mailbox wait | ≤1 ms | bounded busy-wait for a free TX mailbox (not a reply wait) |

---

## 10. Config flow

```
 configs/<setup>/slave*.yaml  ──(scripts/gen_motor_config.py)──▶ generated files
```
`gen_motor_config.py` reads `configs/active` (a pointer file; `$SOCCER_SETUP` overrides the
*generator only*) and the setup's `slave*.yaml`, then emits:

| generated file | flag | consumed by |
|---|---|---|
| `firmware/common/include/motor_config.h` | `--slave slaveN` | **slave** build: `N_MOTORS`, `MotorConfig[]` (can_id, model, soft_min/max, max_vel, **max_tau**, default_kp/kd), transport bounds `MOTOR_P/V/T_MIN/MAX`, `MOTOR_ZERO_*`, **`MOTOR_ENABLE_MON_K`** |
| `firmware/common/include/system_config.h` | `--system` | **master** build: `NUM_SLAVES`, `MAX_MOTORS_PER_SLAVE`, `TOTAL_MOTORS`, per-slave counts, transport bounds |
| `host/master_link/motor_config_gen.py` | `--system` | **host**: `MOTORS`, `N_MOTORS`, `MOTOR_DEFAULT_KP/KD`, `MOTOR_SOFT_MIN/MAX`, transport bounds, `CONFIG_NAME`, `CONFIG_HASH` (sha256 over the setup's YAMLs) |

The host `config_meta.check_config_fresh()` compares the generated `CONFIG_NAME/HASH` against
the live `configs/active` YAMLs and warns loudly on staleness or a `$SOCCER_SETUP` divergence.
The wire motor count `N` is baked into all three at build time; there is **no in-band N check**.

---

## 11. End-to-end traces

### A. MIT command → torque response (~29 ms median; `logs/2026-09-29/21-25-09_latency.bin`)
1. `policy.step` → `MitCommand` → `MasterLink.send_mit` → `struct.pack(FMT_MOTOR_CMD)` →
   `encode_frame` → `serial.write`. **[USB ≈ 0.8 ms]**
2. master `CDC_Receive_FS` (USB-ISR) → CRC → `MotorMaster_SetMitCmd` → `pending_mit`.
   **[waits for next poll ≤ 5 ms]**
3. master `poll_one_slave` (200 Hz) → `SPI_CMD_MIT` + `SpiMitCmd` → `spi_exchange` (SPI DMA).
4. slave SPI ISR → `cmd_inbox` (ping-pong); main → CRC → `motor_runtime_apply_mit` → clamp →
   `hold_pos/hold_vel`, `ARMED_MIT`.
5. slave `motor_runtime_update` (200 Hz) → `send_mit` → `can_mit_control_set` → **Type-1 CAN**.
   **[CAN ≈ 0.11 ms + motor torque onset]**
6. motor applies MIT (torque rises) → **Type-2** reply → slave `can_unpack_motor_feedback`
   (`motors[].torq`, `last_fb_ms`). **[fb_age ≤ 5 ms]**
7. slave next tick → `motor_runtime_sample` → `MotorState` → `spi_proto_build_tele`.
8. master next poll → `spi_exchange` reads tele → CRC → `latest_atom`; `emit_motor_state`
   (gated) → `proto_build` → TX ring → USB. **[USB ≈ 0.8 ms]**
9. host RX thread → `decode_frame` → `parse_motor_state` → `MotorSnap`; `latency.py` sees
   `tau` cross the noise threshold.
- **Where the ~29 ms goes:** ~1.6 ms USB RTT + **two 200 Hz poll quantizations** (command
  poll + telemetry poll, up to ~5 ms each) + CAN hop + `fb_age` + motor torque-onset. The
  ~5 ms measured spread matches the SPI poll phase; the USB link is a small fraction.

### B. ARM request → CONTROL_RESP
1. `policy`/user → `MasterLink.send_control(ControlRequest ARM)` →
   `struct.pack(FMT_CONTROL_REQ)` → `encode_frame` → USB.
2. master `CDC_Receive_FS` → CRC → `MotorMaster_HandleControlReq`: builds `ControlResp`
   (`result=CTRL_OK`, `new_state=MOTOR_ARMED_HOLD`, echo `req_seq`) → posts to TX ring
   **immediately**, and sets `pending_arm_bits`. → host receives `CONTROL_RESP` in ~1 link RTT.
3. (asynchronously) master `poll_one_slave` → `SPI_CMD_ARM_IDX(idx)` → slave
   `motor_runtime_arm(idx)`: fault-clear (if latched) → Type-18 MIT-mode → Type-3 enable →
   settle (**~40 ms blocking**) → `ARMED_HOLD`.
4. master clears `pending_arm_bits[s]` only when telemetry shows the armed state.
- **Note:** the `CONTROL_RESP` in step 2 is optimistic — real arming is confirmed by watching
  the motor's `state` reach `ARMED_HOLD` in `MOTOR_STATE` telemetry, not by the resp.

### C. Fault → convert_log output
1. slave `motor_runtime_update` fault check (e.g. `|tau| > max_tau`) → `idle_motor` +
   `cause = CAUSE_OVERTORQUE`; `state = (IDLE | OVERTORQUE<<4)`.
2. `motor_runtime_sample` → `MotorState.state` → `spi_proto_build_tele` → SPI.
3. master `spi_exchange` → `latest_atom` → `emit_motor_state` → USB.
4. host RX thread → `parse_motor_state` (`cause=1`); the raw frame is logged as `RX_FRAME`.
5. `convert_log.py` → `motor_state.csv` row with `cause=1`, `cause_name=OVERTORQUE`;
   `plot_motor_state.py` draws a red fault-onset line.

---

## 12. Known limitations

- **Blocking entry handshakes.** `motor_runtime_arm` / `motor_runtime_goto_zero` and the
  ZEROING-arrival re-enable block the slave loop ~40 ms each; arming N motors is N separate
  SPI commands, each a ~40 ms blocking call (serialized). Steady-state MIT/HOLD is
  non-blocking. (Async confirm-and-retry handshake is planned, not built.)
- **No host-death timeout.** If the host stops, the master keeps polling and sending `HOLD`,
  so an armed motor keeps holding indefinitely. `kill -9` of the runner does not safe the
  motor — only a clean Ctrl-C (or an explicit DISABLE) does.
- **One-shot CAN TX** (`AutoRetransmission = DISABLE`): a lost enable/command frame is not
  retransmitted (rare enable-loss, ~0.2%). Fix (auto-retransmit + abort-on-timeout, or
  confirm-and-retry) is planned.
- **`max_tau` trip sensitivity.** `max_tau = 0.8 N·m` on the RS02 is easily hit by a position
  step: `kp·Δpos` (default `kp=15`) reaches the trip at only ~0.053 rad, and a full-amplitude
  sine setpoint applied to a motor not at that position steps `kp` straight into `OVERTORQUE`.
  Zero/ramp to the trajectory start first.
- **No in-band SPI `N` check.** Slave/master built from different configs → silent "dead
  slave" with climbing `crc_errors`. Both must build from the same active config.
- **Version gating is now on both ends** (as of PROTO_VERSION 2): the host drops+counts
  wrong-version RX, and the master's `CDC_Receive_FS` drops+counts wrong-version host frames
  (`master_proto_ver_mismatch`). (Older docs said the master gated only on CRC — no longer true.)
- **`MSG_MOTOR_CMD kp/kd/tau_ff` are ignored** downstream — the slave always uses per-motor
  `default_kp/kd`; only `pos/vel` drive the motor.
- **`CTRL_SET_ZERO` is a stub** (`CTRL_ERR_STUB`); set-zero happens only inside GOTO_ZERO
  arrival.
- **`pos_raw` is home-frame wrapped** — multi-turn absolute angle is not on the wire.

---

## 13. Timing & dataflow model

**Snapshot:** commit `7b3556b` + uncommitted cmd_seq work (PROTO_VERSION 2). A scheduling/
dataflow picture of the whole path, as the code is now. Where code and prose disagree, the
code wins — noted inline.

### 13.1 Clock domains (independently timed loops)

| domain | rate | driven by | jitter |
|---|---|---|---|
| host runner loop | 50 Hz (`--rate`, default) | sleep-to-deadline on `time.monotonic_ns` (`run_policy.main`) | sub-ms (bench p99 ~0.14 ms); resyncs grid if it falls a full period behind |
| MasterLink RX thread | ~1 kHz poll | `in_waiting` read + `time.sleep(0.001)` (`link._rx_loop`) | up to ~1 ms before bytes are ingested |
| log writer thread | flush ~1 Hz | background thread draining a bounded queue (100 000) | opportunistic; overflow recorded as `LOG_DROP` |
| master main loop | free-running | `HAL_GetTick()` (1 ms) deadline checks in `MotorMaster_ProcessLoop` | ~1 ms (tick granularity) |
| ├ SPI poll sub-rate | 200 Hz (5 ms) | `next_poll_ms` deadline | |
| ├ telemetry emit | 200 Hz (5 ms), gated on `slave_alive` | `next_tele_ms` deadline | |
| └ status emit | 20 Hz (50 ms) | `next_status_ms` deadline | |
| slave main loop | free-running | `HAL_GetTick()` deadline checks (`main.c` while(1)) | ~1 ms |
| ├ control tick | 200 Hz (5 ms) | `loop_next_poll_ms` → `motor_runtime_update` | |
| └ telemetry rebuild | event (per new CAN feedback) | `can_feedback_count` change | tracks feedback rate |
| slave SPI transfer | master-paced | DMA + `SPI1` TxRxCplt ISR (re-arms DMA) | set by master clock |
| motor (RS02) | ~200 Hz feedback while armed | motor firmware; one Type-2 per received Type-1 | motor-internal |

Interrupt priorities (lower = higher): **master** OTG_FS=0, SPI1=0, DMA2=0; **slave** DMA2
streams=0, SPI1=1, CAN1_RX0=1. Both MCUs run the control/scheduling work in the main loop;
ISRs only move bytes/frames and set flags.

### 13.2 Stage table (one row per stage)

Forward path (command), then return path (telemetry). "latest-value mailbox" = a single slot,
newest write wins; "ping-pong" = two halves, one DMA-owned, one main-owned, swapped at
transfer end.

| # | stage | trigger | context | input buffer (full policy) | output | code |
|---|---|---|---|---|---|---|
| F1 | policy → Action | time, 50 Hz | host main | `latest_state()` snapshot (latest-value, lock-copied) | `Action{mit,control}` | `run_policy.main`, `Policy.step` |
| F2 | send → USB | event (step w/ mit) | host main | Action list | `MSG_MOTOR_CMD` frames; **cmd_seq stamped once per `send_mit` call** | `link.send_mit`/`_write_frame` |
| F3 | master USB RX | event (USB OUT) | OTG_FS ISR (0) | `accum[128]` reassembly; CRC + version gate | `pending_mit[s][idx]` (latest-value mailbox) | `CDC_Receive_FS`→`MotorMaster_SetMitCmd` |
| F4 | master SPI poll | time, 200 Hz | master main | `pending_mit` (latest-value); priority DISARM>GOTO_ZERO>ARM>MIT>HOLD/NOP | one SPI cmd frame (**blocking** `HAL_SPI_TransmitReceive`) | `ProcessLoop`→`poll_one_slave`→`spi_exchange` |
| F5 | slave SPI RX | event (master clock) | DMA2 (0) + SPI1 TxRxCplt ISR (1) | ping-pong RX halves | `cmd_inbox_buf` + `data_receive_flag` | `slave_spi.HAL_SPI_TxRxCpltCallback` |
| F6 | slave dispatch | event (`data_receive_flag`) | slave main | `cmd_local` copy (IRQ-masked), CRC-checked | `apply_mit`: stores `hold_pos`/`target_cmd_seq` | `main.c` SPI handler→`motor_runtime_apply_mit` |
| F7 | control tick → CAN | time, 200 Hz | slave main | `motors_rt[]` | Type-1 MIT frame; **opens cmd_seq reply window** | `motor_runtime_update`→`send_mit`→`can_mit_control_set` |
| F8 | motor apply | event (Type-1 on bus) | RS02 firmware | — | **[APPLIED]**, emits Type-2 feedback | motor |
| R1 | slave CAN RX | event (Type-2 on bus) | CAN1_RX0 ISR (1) | decode into `motors[]` slot | `motors[]`, `fb_count++`, `can_feedback_count++` | `motor_chain.HAL_CAN_RxFifo0MsgPendingCallback` |
| R2 | telemetry rebuild | event (`can_feedback_count` Δ) | slave main | `motor_get_snapshot` (IRQ-masked) + `cmd_track.last_applied` | staged into `tele_stage_buf` (ping-pong) | `main.c`→`motor_runtime_sample`→`spi_proto_build_tele`→`spi_write_next_tx_buf` |
| R3 | slave→master SPI | event (next master poll) | DMA + ISR | ping-pong TX | telemetry frame into master RX | `spi_exchange` (RX half) |
| R4 | master telemetry emit | time, 200 Hz, **gated on this tick's poll CRC** | master main | `latest_atom[s][]` (latest-value) | `MSG_MOTOR_STATE` into TX ring | `emit_motor_state`→`usb_tx` |
| R5 | master USB TX | event (ring non-empty) | master main (`usb_tx_pump`) | ring buffer (4096 B) | bytes to host | `usb_tx.c` |
| R6 | host RX ingest | event (bytes), ~1 kHz poll | host RX thread | `bytearray` reassembly | `_motors[key]=MotorSnap` (latest-value, lock); log `RX_FRAME` | `link._rx_loop`→`_ingest` |
| R7 | host log write | event (per frame/record) | queued → writer thread | bounded queue (100 000) | `.bin` on disk | `datalog` writer |

### 13.3 Rate transitions (cross-domain crossings)

| crossing | can drop? | can duplicate? | delay |
|---|---|---|---|
| host send (50 Hz) → `pending_mit` mailbox (F2→F3) | yes, if two commands land within one 5 ms poll window (older overwritten) — **not** at 50 Hz | no | ≤ one poll (≤5 ms) |
| `pending_mit` → SPI poll (F3→F4) | — | yes: HOLD/NOP re-sent when no fresh command | 0–5 ms (poll quantization) |
| slave SPI dispatch → control tick (F6→F7) | no | no | 0–5 ms (next 200 Hz tick) |
| CAN feedback (~200 Hz) → telemetry rebuild (R1→R2) | no (latest state rebuilt next feedback) | no | deferred to the rebuild if main loop busy |
| telemetry ping-pong → SPI (R2→R3) | — | yes: previous telemetry re-sent if no fresh frame staged | 0–5 ms |
| `latest_atom` → `MSG_MOTOR_STATE` (R4) | **yes**: if this tick's poll CRC failed, no emit (host silence = freshness signal) | — | ≤5 ms |
| master ring → host (R5) | yes on ring overflow (not expected at these rates) | no | ~0.3–1 ms |
| host RX → `latest_state` mailbox (R6→F1) | policy-visible: intermediate frames coalesced (newest wins); **all** frames kept in the log | no | ≤ one runner period (≤20 ms @ 50 Hz) |

### 13.4 Timing diagrams

**MIT command — host send → motor apply → applied echo back** (measured round trip ≈ **19 ms**
median, `logs/2026-10-01/10-35-16_man_1s_1m.bin`):

```
t=0   host send_mit (cmd_seq k)
  │  USB OUT + master CDC RX ISR            ~0.3–1 ms   → pending_mit (latest-wins)
  │  wait for next master SPI poll          0–5 ms      (200 Hz quantization)
  │  SPI transfer (blocking)                ~0.1 ms     → slave cmd_inbox
  │  wait for next slave control tick       0–5 ms      (200 Hz) → apply_mit stores target
  │  control tick: Type-1 MIT → CAN         ~0.1 ms     ►► [APPLIED at motor], reply window open
  ── return ──
  │  motor Type-2 feedback                  motor-internal
  │  slave CAN RX ISR → motors[], fb_count  ~0.1 ms     → cmd_seq_on_reply: last_applied=k
  │  telemetry rebuild → ping-pong          < next poll
  │  next master poll clocks telemetry back 0–5 ms
  │  master emit MSG_MOTOR_STATE (200 Hz)   0–5 ms      → USB ring
  │  host RX thread ingest                  0–1 ms
t≈19 ms  host sees last_applied_seq ≥ k      (median; min 17.7, p95 20.6, max 21.2)
```

**Telemetry sample — motor → policy** (and → log):

```
t=0   motor Type-2 on bus
  │  slave CAN RX ISR → motors[] + fb_count  ~0.1 ms
  │  telemetry rebuild (event) → tele_stage  < next poll
  │  master SPI poll clocks it back          0–5 ms
  │  master emit MSG_MOTOR_STATE (gated,200) 0–5 ms     → ring
  │  master USB TX pump → host               ~0.3–1 ms
  │  host RX thread ingest → latest_state     0–1 ms    ├─► log RX_FRAME (writer thread)
  │  policy.step reads snapshot              0–20 ms    (next 50 Hz runner tick)
```

Reference slices: **ping RTT ~1.6 ms** (host↔master only, no SPI/CAN —
`logs/2026-09-29/21-25-09_latency.bin`); **cmd_seq round trip ~19 ms** (above); **torque
onset ~29 ms** (`21-25-09_latency.bin`) — physical torque departs baseline *after* the command
is acknowledged, so it trails the ~19 ms echo.

### 13.5 Host rate vs the master's 200 Hz poll

| host rate | per command | coalescing in `pending_mit` | polls with no fresh cmd | expected cmd_seq effect |
|---|---|---|---|---|
| 50 Hz | 1 / 20 ms | none (two commands never share one 5 ms window) | ~3 of 4 → HOLD re-sent | 0 never-applied; spread ≈ one poll period |
| 200 Hz | 1 / 5 ms | host/master clocks are independent (no handshake) → beats: occasionally two commands land in one poll window (older cmd_seq overwritten, never applied), occasionally zero (HOLD) | occasional | small but **nonzero** never-applied; lower mean latency, similar spread |

**Measured:** the 50 Hz row — median 19.17 ms, **0 never-applied of 1500**
(`10-35-16_man_1s_1m.bin`). **Analysis (not measured):** the 200 Hz row — the beat between the
free-running host and master clocks is the mechanism that would drop the occasional cmd_seq.

### 13.6 Options for a 200 Hz policy (not recommendations)

| option | idea | stages it changes |
|---|---|---|
| observation-triggered runner | step on telemetry receipt instead of a fixed grid, aligning host cadence to the slave feedback and removing the host/poll beat | F1 (runner loop); MasterLink would signal new telemetry |
| event-triggered forwarding | master forwards a command to SPI on USB RX rather than at the next 200 Hz poll; slave applies on SPI RX rather than at the next control tick — removes both 0–5 ms quantization waits | F4 (master poll), F6/F7 (slave dispatch/tick) |
| faster loop rates | raise master poll + slave control tick above 200 Hz | `MASTER_POLL_PERIOD_MS`, `MOTOR_LOOP_PERIOD_MS`, SPI bandwidth — but the **CAN bus caps the slave loop**, see §13.7 |

### 13.7 CAN bus bandwidth ceiling (N motors)

The hard ceiling on the slave control-tick rate is the **CAN bus**, not CPU or SPI. RobStride
is classic CAN 2.0 only — 1 Mbps, 29-bit extended, 8 data bytes, no FD/XL (see
`robostride-motor-reference.md` §3). Each armed motor costs **2 frames per tick**: the slave's
Type-1 MIT out + the motor's Type-2 feedback back. This is **analysis** (only 1 motor @ 200 Hz
is bench-verified, bus pristine — `can-investigation.md`).

Classic extended 8-byte frame @ 1 Mbps (1 µs/bit), incl. 3-bit IFS:

| frame | bits | time |
|---|---|---|
| min (no bit-stuffing) | 131 | 131 µs |
| typical (some stuffing) | ~140 | ~140 µs |
| worst case (max stuffing) | ~160 | 160 µs |

**6 motors = 12 frames/tick:**

| | bus time/tick | ceiling @ 100 % bus |
|---|---|---|
| typical (~140 µs) | ~1.68 ms | ~595 Hz |
| worst case (160 µs) | ~1.92 ms | ~520 Hz |

So the bus-saturated ceiling for 6 motors is **~520–600 Hz**; 100 % utilization is never a
target, so budget ~65–70 %:

| slave loop rate (6 motors) | bus load | note |
|---|---|---|
| 200 Hz (today) | ~34 % | comfortable |
| 400 Hz | ~67 % | workable, tight |
| 500 Hz | ~84 % | too close to the edge |

**Practical sustained target ≈ 300–400 Hz for 6 motors.** Caveats: (a) `AutoRetransmission` is
**DISABLE** (one-shot CAN) — a frame lost to arbitration is not retried, so contention at high
load drops frames; keep real headroom. (b) bxCAN has **3 TX mailboxes**, so 6 Type-1 frames
take ~2 mailbox rounds (~660 µs of the tick at ~110 µs/frame) — the second-tightest resource
after raw bus time. (c) the RS02's own max MIT-acceptance rate is unverified (typically ≥1 kHz,
so unlikely to bind first). General rule: **ceiling ≈ 1 / (2·N·~140 µs)** at full bus; halve
for a safe target. Finally, raising the slave loop past the master's 200 Hz SPI poll only helps
end-to-end if `MASTER_POLL_PERIOD_MS` rises too (§13.6).
