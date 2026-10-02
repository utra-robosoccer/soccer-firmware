# Robosoccer motor-control system — architecture

**Snapshot anchor:** branch `akp/single_motor_rework`.
**Doc date:** 2026-10-01.

> **PROTO_VERSION 3 — robot/chain/motor hierarchy (current).** The wire is now a
> fixed-point robot/chain/motor hierarchy: the host sends one `MSG_ROBOT_CMD`
> (`cmd_robot_t`) per tick and the master returns one `MSG_ROBOT_TELE`
> (`tele_robot_t`) per telemetry tick. Per-motor **mode requests** (IDLE / HOLD /
> MIT / DAMPED / TO_ZERO) replace the old ARM/GOTO_ZERO control messages and the
> MIT-only command. Scalars are fixed-point (shared scales), not u16-over-bounds.
> The firmware **never writes mechanical zero**; a wound shaft is refused at arm
> (`CAUSE_WOUND`). Timing (200 Hz poll/tele, 200 Hz slave tick, 50 Hz host) is
> unchanged and is now **config-driven** (`configs/<setup>/system.yaml`). Where the
> code and older prose disagree, follow the **code**; discrepancies are flagged.

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
decodes those to `float`, then re-encodes to **little-endian fixed-point** wire fields
(`tele_motor_t`) for SPI. From the slave's SPI output all the way to the host it is
**little-endian, pass-through** — the master never reinterprets motor data. Fixed-point
scales (shared, in `protocol.h`): pos ×10000 (home-frame ±π, i16), vel ×100 (i16),
tau ×100 (i16), Kp ×10 (u16), Kd ×100 (u16) — universal across every RobStride model.

Wire contract source of truth: `firmware/common/include/protocol.h` (C, included by both
MCUs) mirrored by `host/master_link/protocol.py` (Python). A cross-language fixture test
(`firmware/common/test/gen_fixture.c` ↔ `host/tests/test_protocol.py`) byte-compares them.

Common integrity primitive: **CRC16-CCITT** (`poly 0x1021`, `init 0xFFFF`), `proto_crc16`.

---

## 1. Hop: policy → runner → MasterLink (host)

Pure-Python, on the Jetson/PC. Files: `host/policies/`, `host/apps/run_policy.py`,
`host/master_link/link.py`.

### Messages (in-process, not on a wire)
- **`Action`** (`policies/base.py`) — `motors: list[MotorCommand]` (one cmd_robot_t per tick).
  - `MotorCommand{slave, local, mode, pos, vel, kp, kd, tau_ff, use_config_gains, fault_reset}`
    — `mode ∈ {MODE_IDLE, MODE_HOLD, MODE_MIT, MODE_DAMPED, MODE_TO_ZERO}`; SI floats.
- **`LinkState`** (`link.py`) — snapshot passed to the policy: `motors {(slave,local)→MotorSnap}`,
  `master`, `slaves`, `robot` (tele_robot_t meta: cycle_id/last_cmd_seq_rx/missed_deadlines),
  `stamp_ns`.
  - `MotorSnap` — decoded SI fields (`pos/vel/tau/temp`), `state`, `cause`, `motor_mode`,
    `motor_fault`, `flags` (+ `request_rejected`/`to_zero_arrived`/`saturated` props),
    `fault_word`, `fb_age`, `last_applied_seq`, `master_ts_ms`, `recv_ns`.

### Built / parsed
- Policy produces `Action` in `Policy.step(self, state, t_ns)` (base: `policies/base.py`).
  `step`/`setup` take `t_ns` (runner's `time.monotonic_ns()`); policies derive all timing from
  it, never read a clock. `listen` requests IDLE for every motor each tick.
- `run_policy.main` (`apps/run_policy.py`) is the loop; it calls `link.latest_state()` →
  `policy.step(state, t_ns)` → `link.send_robot_cmd(action.motors)`.

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
| `ver_flags` | u16 | low byte = `PROTO_VERSION` (=3), high byte reserved 0 |
| `crc16` | u16 | CRC16-CCITT over header(crc=0)+payload |

### Hierarchy (shared `protocol.h`; fixed WIRE caps MAX_CHAINS=4, MAX_MOTORS_PER_CHAIN=5)
- `cmd_motor_t` (12 B): `mode_req`, pos/vel/kp/kd/tau_ff (fixed-point), `flags`
  (`VALID` / `USE_CONFIG_GAINS` / `FAULT_RESET`).
- `cmd_chain_t` (62 B): `chain_id`, `n_motors`, `cmd_motor_t[5]`.
- `cmd_robot_t` (254 B): `cycle_id`, `cmd_seq`, `n_chains`, `cmd_chain_t[4]`.
- `tele_motor_t` (21 B, §5): pos/vel/tau, temp, `state`, `cause`, `motor_mode`, `motor_fault`,
  `flags`, `fb_age_ms`, `fault_word`, `last_applied_seq`, reserved.
- `tele_chain_t` (117 B): `chain_id`, `n_motors`, `spi_seq_echo`, `slave_time_us`,
  `cmd_crc_errors`, `can_tx_errors`, `tele_motor_t[5]`.
- `tele_robot_t` (480 B): `cycle_id`, `master_time_us`, `last_cmd_seq_rx`, `missed_deadlines`,
  `n_chains`, `robot_state`, `tele_chain_t[6]`.

### Messages — host → master
| type | struct | fields / encoding | built | parsed | trigger/rate |
|---|---|---|---|---|---|
| `MSG_ROBOT_CMD` 0x08 | `cmd_robot_t` | per-motor **mode request** + fixed-point targets; `cmd_seq` = one per tick (≥1, 0 reserved), echoed as `last_applied_seq` (§5,§6). Grouped into per-slave chains by `chain_id`. Replaces the old MOTOR_CMD + CONTROL_REQ. | `link.send_robot_cmd` (`pack_robot_cmd`) | `CDC_Receive_FS` → `MotorMaster_HandleRobotCmd` | one per host tick (50 Hz) |
| `MSG_PING` 0x01 | (empty) | — | `link._write_frame(encode_frame(PING))` | `CDC_Receive_FS` → PONG | on request |

### Messages — master → host
| type | struct | fields / encoding | built | parsed | trigger/rate |
|---|---|---|---|---|---|
| `MSG_ROBOT_TELE` 0x09 | `tele_robot_t` | whole-robot telemetry; the master transmits only the **populated prefix** (header + `n_chains` chains ≈ 129 B for one chain, not the full 480 B) — emission-gated at chain granularity (a dead slave's chain is omitted → that chain goes silent on the host) | `emit_robot_tele` (`spi_master.c`) | `MasterLink._ingest`→`parse_robot_tele` | 200 Hz |
| `MSG_MASTER_STATUS` 0x02 | `MasterStatus{robot_state,slave_alive,uptime_ms,link_errors,rx_frames, master_poll_hz,telemetry_hz,slave_tick_hz,host_cmd_hz}` | now carries the configured rates (§10) | `emit_master_status` | `parse_master_status` | 20 Hz |
| `MSG_SLAVE_STATUS` 0x03 | `SlaveStatus{slave_id,motors_alive,uptime_ms,crc_errors,cmd_crc_errors,seq_gaps}` | counters | `emit_slave_status` | `parse_slave_status` | 20 Hz |
| `MSG_PING` 0x01 (PONG) | (empty) | **echoes request `seq`** | `usbd_cdc_if.c` → posted to TX ring | logged as RX_FRAME | on PING |

`MSG_MOTOR_STATE` / `MSG_CONTROL_REQ` / `MSG_CONTROL_RESP` / `MSG_MOTOR_CMD` are **removed** in v3.

### Encode / decode
- Encode: `proto_build` (C) / `encode_frame` (Py). Decode: `decode_frame` (Py, resync on
  bad CRC by dropping one byte; host CRC is `binascii.crc_hqx`, C-speed, byte-identical to
  `proto_crc16`). Master ingress reassembles multi-packet frames in `accum[512]` (a
  `cmd_robot_t` frame spans ~7 USB OUT packets).

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
- Master egress: single-producer TX **ring** (`usb_tx.c`, 8192 B); ISR-built PONGs are
  posted and drained by main into the ring. `usb_tx_pump` drains one 64 B packet per call
  (a larger per-transfer chunk raises IN throughput but starves the shared OTG core's
  multi-packet OUT reception — it's capped at one packet for that reason; the telemetry
  prefix-truncation keeps the stream ~26 KB/s, well under the ~64 KB/s single-packet ceiling).
- Master ingress: `accum[512]` reassembly (USB-ISR context) — must hold a full
  `MSG_HEADER + cmd_robot_t` (394 B) across ~5 USB OUT packets.
- **USB soft-disconnect at boot** (`main.c` SysInit): drives D+ (PA12) low ~10 ms so the host
  re-enumerates fresh after an ST-Link reflash — without it the OTG OUT endpoint could wedge
  (multi-packet OUT stops completing) until a power cycle.

### Latency (measured)
- `MSG_PING` host↔master RTT: **median 1.56 ms** (min 0.67, p95 2.64, max 3.34) —
  `logs/2026-09-29/21-25-09_latency.bin`. This is the pure link, no motor/CAN.
- **cmd_seq latency** (`host/analysis/latency.py`, primary): TX of a command's `cmd_seq`
  → the first telemetry whose `last_applied_seq ≥` it (wrap-aware). This **includes the full
  return path** — host→master→SPI→slave→CAN→motor to apply, then
  motor→CAN→SPI→master→USB→host for the echo to come back — so it is strictly larger than
  the one-way command delay. Reported per-tick (all commanded motors applied) and per-motor,
  each with a never-applied count.
  - **Measured, v3 (2026-10-01):** **median 24.4 ms** (min 20.1, p95 30.2, max 31.1),
    **0 never-applied of 1000** — `logs/2026-10-01/16-36-18_man_1s_1m.bin`, a 20 s in-range
    MIT sine at 50 Hz on `bench-1-motor`. Path: host `send_robot_cmd` → USB-CDC → master
    per-slave chain mailbox → SPI poll → slave `apply_cmd` → Type-1 MIT → RS02 **(applied)**;
    echo returns Type-2 → slave pairs `last_applied_seq` → SPI → master → USB-CDC → host.
    Comparable to the v2 ~19 ms (slightly higher: larger frames + the DAMPED/TO_ZERO-capable
    path). It lands below the ~29 ms physical torque-onset because the echo flips on the
    controller's acknowledgement, before a measurable torque departure.
  - **Throughput note (v3):** the full 480 B `tele_robot_t` at 200 Hz (~96 KB/s) swamped the
    pure-Python host and built a ~400 ms backlog. Resolved by (a) the master transmitting only
    the populated prefix (~129 B, ~26 KB/s), (b) `binascii.crc_hqx` on the host (decode
    ~37 k frames/s, ≫ the 200 Hz stream), and (c) dropping the blocking `flush()` in
    `_write_frame`. Host RX decode is no longer the bottleneck.

---

## 3. Master internals

STM32F446 (`firmware/master/`). No motor logic — it is a USB↔SPI bridge + command mailbox +
telemetry forwarder. The 200 Hz cycle is driven by a **hardware timer (TIM2)**, not
`HAL_GetTick` deadlines (`master_cycle.c`).

### Loops / rates
TIM2 (32-bit) runs free at 1 MHz as a monotonic µs clock; its CH1 output-compare fires at
`MASTER_POLL_HZ` on an absolute grid (`CCR1 += period`, no drift). The ISR does **no work** —
it sets a "cycle due" flag + the fire time; `MotorMaster_ProcessLoop` does the work:

| loop | trigger | does |
|---|---|---|
| cycle (poll **+** telemetry) | TIM2 CH1 @ `MASTER_POLL_HZ` (200 Hz) | `master_cycle_id++`; `poll_one_slave` for every slave in order; then `emit_robot_tele` immediately (telemetry is **tied to the poll**, no separate timer). `master_time_us` = the cycle's TIM2 µs timestamp. |
| status emit | cycle divider (every `MASTER_POLL_HZ/20` = 10 cycles ⇒ 20 Hz) | `emit_master_status` + `emit_slave_status` |
| USB TX drain | every main-loop iteration | `usb_tx_pump_responses` + `usb_tx_pump` |

(Rates are generated into `motor_config.h` / `system_config.h` from the config, §10; the TIM2
period = `1e6 / MASTER_POLL_HZ` µs.) A CRC-passed poll updates `latest_tele[s]` and marks the
slave alive; `emit_robot_tele` includes only currently-alive slaves' chains, so a failed/absent
poll drops that chain (silence = dead). **`missed_deadlines` now = cycle overruns** —
`master_cycle_overruns()`, incremented when a cycle's due-flag is still set as the next TIM2
tick fires (the main loop fell a full period behind); replaces the old `HAL_GetTick` heuristic.

**Measured (1 slave, TIM2, 2026-10-01):** cycle period 5000 µs, σ ≈ 9 µs idle / 26 µs under
load (min 4907, max 5093); poll+assembly ≈ 2.55 ms/cycle (max 2.60, headroom to 5 ms);
flag→service delay ≈ 2 µs (max 4); 0 overruns. cmd_seq latency **median 23.0 ms** (p95 23.8,
max 24.4, 0 never-applied/1000) — vs the pre-TIM2 24.4 ms the median is slightly lower and the
spread collapses from ~11 ms to ~2.5 ms (the hardware grid removes the 1 ms `HAL_GetTick`
cadence jitter; the ~5 ms command→cycle quantization is structural, unchanged).

### Command mailbox (USB-ISR writes, main reads)
Level-triggered, so there is no one-shot staging or priority ladder anymore.
`MotorMaster_HandleRobotCmd` (USB-ISR) splits the host's `cmd_robot_t` into per-slave
`host_chain[s]` mailboxes by `chain_id` (**latest wins**) and records `host_cmd_seq`.
`poll_one_slave` snapshots the slave's chain under a brief `__disable_irq()` and sends it as
`SPI_OP_ROBOT_CMD`; if no host command has arrived it sends `SPI_OP_NOP` (still refreshes the
slave watchdog and clocks telemetry). There is **no `CONTROL_RESP`** — telemetry (the motor's
`state` / `last_applied_seq`) is the acknowledgement.

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
the slave clocks out its telemetry frame simultaneously. **One chain per slave**, so the SPI
payload is a single fixed-size `cmd_chain_t` / `tele_chain_t` (no per-N arithmetic). Transfer
length = the larger of the two = `SPI_XFER_SIZE` = 119 B. Master SPI: `SPI_MODE_MASTER`,
CPOL=0/CPHA=0, MSB-first.

**SPI clock — configurable.** The master prescaler is a generated value,
`MASTER_SPI_PRESCALER_DIV` (`system_config.h`, from `gen_motor_config.py`), mapped to the
HAL enum in `MX_SPI1_Init`. APB2 is 72 MHz, so div `{64,32,16,8}` → `{1.125, 2.25, 4.5, 9}`
MHz. A 2-min streaming sweep found **all of 64/32/16/8 CRC-clean on the bench** (short wiring),
both directions, 0 overruns; the HAL blocking transfer keeps up at every setting (its fixed
~0.1–0.2 ms poll overhead just grows as a fraction of the shrinking bit-time). **Default: div
16 (4.5 MHz)** — two steps of margin below the fastest tested, and the slave SPI resync (below)
recovers isolated glitches. **Re-validate on the robot harness** (longer/noisier wiring) by
watching the CRC counters (`slave_status.crc_errors`/`cmd_crc_errors`) and `spi_resyncs`, and
drop the prescaler if they climb.

**Slave DMA resync.** The slave receives in SPI-slave mode via a fixed-length DMA; a single
bad exchange (a CS/clock glitch, over-speed bit error, or master reset mid-transfer) would
otherwise offset the byte counter and wedge **every** later exchange permanently (persistent
CRC failures both directions until the slave resets). On a command CRC failure the slave calls
`slave_spi_resync` (`slave_spi.c`): gated on **NSS (PA4) high** (between exchanges, via the
pure `spi_resync_poll`) it disables SPI, aborts both DMA streams, flushes the RX FIFO, and
re-arms so the next exchange re-aligns — recovering in ~1–2 exchanges instead of wedging. The
count rides in `tele_chain_t.spi_resyncs` (u8, **wraps**; the host takes deltas). A build-flag
injector (`-DSPI_INJECT_TEST`) clocks one deliberately short exchange for the recovery test.

### Command frame (master → slave), CRC-protected — 70 B
```
[ opcode u8 ][ spi_seq u8 ][ cycle_id u16 ][ cmd_seq u16 ][ cmd_chain_t (62) ][ crc16 u16 ]
```
- `opcode`: `SPI_OP_NOP` / `SPI_OP_ROBOT_CMD` (per-motor mode requests subsume the old
  ARM/DISARM/GOTO_ZERO opcodes).
- `spi_seq`: `spi_seq[s]++` each poll; echoed in telemetry (`spi_seq_echo`). **SPI link-health
  seq — distinct from the host `cmd_seq`** (which rides in the header above).
- `cycle_id`: master poll counter; `cmd_seq`: host command seq (§2), forwarded so the slave
  echoes it as `last_applied_seq` (§5, §6).
- `crc16`: `proto_crc16` over `[opcode … last cmd_chain_t byte]` at `SPI_CMD_CRC_OFF`; built
  for **every** frame (NOP included).
- Built: `poll_one_slave`/`spi_exchange` (`spi_master.c`). Parsed: `spi_proto_parse_cmd`
  (`spi_proto.c`), dispatched by slave `main.c` → `motor_runtime_apply_cmd` per valid motor.

### Telemetry frame (slave → master), CRC-protected — 119 B
```
[ tele_chain_t (117) ][ crc16 u16 ]
```
- `tele_chain_t` carries `spi_seq_echo` (gap detect), `spi_resyncs` (DMA realigns, wrapping),
  `cmd_crc_errors`, `can_tx_errors`, `slave_time_us`, and `tele_motor_t[5]` (valid =
  `n_motors`). Motor "alive" is inferred by
  the master from each motor's `state` (past BOOT/DISCOVERING).
- `crc16` over the `tele_chain_t`. Built: `spi_proto_build_tele` (`spi_proto.c`).
  Parsed/verified: master `spi_exchange`.

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

## 5. The `tele_motor_t` (21 B) — the per-motor telemetry unit

Built in `motor_runtime_sample` (slave) into a `tele_chain_t`, CRC'd by `spi_proto_build_tele`;
consumed by `protocol.py parse_robot_tele` → `TeleMotor` (host). Forwarded byte-identical by
the master. Fixed-point fields decode as raw ÷ scale (shared `PROTO_*_SCALE`).

| off | field | type | units | encoding / precision |
|---|---|---|---|---|
| 0 | `pos` | i16 | rad | **home-frame wrapped [−π,π]** ×10000 (res 0.0001 rad); saturates at ±π |
| 2 | `vel` | i16 | rad/s | ×100 (res 0.01); range ±327 |
| 4 | `tau` | i16 | N·m | ×100 (res 0.01); range ±327 |
| 6 | `temp_c` | u8 | °C | integer degrees |
| 7 | `state` | u8 | enum | `MotorLifecycle` (LIFE_*, full byte — no nibble packing) |
| 8 | `cause` | u8 | enum | `MotorFaultCause` (latched) |
| 9 | `motor_mode` | u8 | enum | RS Type-2 run mode (0 reset / 1 cal / 2 normal) |
| 10 | `motor_fault` | u8 | bits | b0 undervolt, b1 driver, b2 overheat, b3 encoder, b4 stall/overload, b5 uncalibrated |
| 11 | `flags` | u8 | bits | b0 `REQUEST_REJECTED`, b1 `TO_ZERO_ARRIVED`, b2 `SATURATED`, b3 `CLAMPED_POS`, b4 `CLAMPED_TAU`, b5 `CMD_STALE` |
| 12 | `fb_age_ms` | u8 | ms | ms since last Type-2, saturating 255 |
| 13 | `fault_word` | u32 | code | `0` clear · `0xFFFFFFFF` read pending/fail · else raw `0x3022` |
| 17 | `last_applied_seq` | u16 | — | host `cmd_seq` the motor last confirmed applied; `0` = none since arm |
| 19 | `reserved` | u16 | — | 0, growth slot |

**`last_applied_seq` semantics:** the slave pairs each MIT (Type-1) frame it sends with the
next fresh Type-2 feedback and promotes the pending `cmd_seq` to `last_applied_seq`
(reply-window pairing in `cmd_seq_track.c`). **Pairing is MIT-only** — any non-MIT frame
(enable/disable) closes the window *without* crediting it. It is **frozen** while holding/idle
and **reset to 0** on arm / disable. `latency.py` matches against it (§2, wrap-aware); pairing
mechanics in §6.

**Scales are universal across every RobStride model** (Kp ×10 covers 0–5000, Kd ×100 covers
0–100). `pos` carries only the **wrapped home-frame** angle — multi-turn winding is not on the
wire (a wound shaft is refused at arm, §6). Structs are **append-only** (grow at the end +
bump `PROTO_VERSION`). Guards: `_Static_assert` on every struct size + the cross-language
fixture test (`gen_fixture.c` ↔ `test_protocol.py`, both a `cmd_robot_t` and a `tele_robot_t`).

---

## 6. Slave internals

STM32F446 (`firmware/slave/slave_general/`). 200 Hz control loop
(`LOOP_POLL_PERIOD_MS = 5 ms`) driving `motor_runtime_update`.

### Pieces
- **CAN RX ISR** (`motor_chain.c HAL_CAN_RxFifo0MsgPendingCallback`): decodes each Type-2
  into a private `motors[]` slot, stamps `last_fb_ms`/`fb_count`, latches `0x3022` on fault.
- **Snapshot** (`motor_get_snapshot`): IRQ-masked whole-struct copy — the only read path into
  `motors[]`. One snapshot per motor per tick.
- **Per-tick control** (`motor_runtime_update`): fault checks, then a per-lifecycle action;
  sends Type-1 via `send_mit`→`can_mit_control_set`→`can_tx`.
- **SPI command handler** (`main.c`): CRC-verify (`spi_proto_parse_cmd`) → for each valid motor
  in the `cmd_chain_t`, `motor_runtime_apply_cmd(idx, &cmd_motor, cmd_seq)`.
- **Mode state machine** (`mode_sm.c`, pure/host-tested): decides the next `MotorLifecycle` +
  side-effect actions (arm / disable / capture-hold / enter-TO_ZERO / clear-fault / wound) from
  `(state, mode_req, flags, wound)`. `motor_runtime_apply_cmd` performs the CAN side effects.
- **Telemetry build** (`spi_proto_build_tele`) on each new CAN feedback → staged into the
  ping-pong TX buffer.

### Mode-request state machine (level-triggered, `mode_sm.c`)
Modes: `IDLE / HOLD / MIT / DAMPED / TO_ZERO`. Every cycle the slave applies the requested
mode for each motor. **HOLD is the only arm-from-IDLE transition and the only enable.** From an
armed state any armed mode or IDLE is free; from IDLE an armed request other than HOLD is
rejected (`REQUEST_REJECTED` flag). **HOLD** captures the current position on entry. **DAMPED**
sets Kp=0 + the commanded (or config) Kd. **TO_ZERO** creeps to home-frame 0 and holds, setting
`TO_ZERO_ARRIVED`. Faults **latch**: while FAULT, armed requests are rejected; an IDLE request
or the `FAULT_RESET` flag clears it (then the same request is re-evaluated from IDLE). See §9.

**No mechanical-zero writes.** The old GOTO_ZERO conditional set-zero and the ZEROING-arrival
disable→set-mech-zero→re-enable sequence are **removed**: firmware never writes mech zero.
A **wound shaft** (|pos_offset| > `MOTOR_WOUND_OFFSET_MAX`, ≈ pinned near ±4π) is refused at
HOLD-arm → `CAUSE_WOUND` (re-zero offline).

### Per-tick MIT/HOLD/DAMPED path is non-blocking
Armed steady-state only calls `send_mit` (queue-and-return) + a watchdog check. The only
bounded wait is inside `can_tx` (~1 ms busy-wait for a free TX mailbox, frees ~110 µs).
Blocking waits exist only in `discover()` (startup) and the HOLD-arm enable handshake
(`arm_enable`, ~40 ms, once per IDLE→HOLD) — **not** the steady-state path.

### Soft limits + clamp
On a MIT target, `apply_soft_clamp` clamps `pos` to `[soft_min, soft_max]` with a **one-sided
velocity clamp** (cancels only feed-forward driving further into the limit), setting
`CLAMPED_POS`/`CLAMPED_TAU`. Host sends the full command; the slave enforces the clamp.

### cmd_seq reply-window pairing
`cmd_seq_track.c` (pure, host-tested) turns each host `cmd_seq` into `last_applied_seq` (§5).
`apply_cmd` stores a MIT target's `cmd_seq`; the next `send_mit` stamps it onto the Type-1 frame
and **opens a reply window**; the next fresh Type-2 feedback **closes it**
(`cmd_seq_on_reply`, before the per-state action so it pairs with the previous tick's frame).
Non-MIT frames `cmd_seq_on_other_frame` (drop without crediting); arm/disable `cmd_seq_reset`.
Host comparison is **wrap-aware** (`protocol.seq_ge`, RFC-1982).

### Enable monitor
Per tick, `enable_monitor_step` (pure, `enable_monitor.c`) checks an armed motor reports
`RS_MODE_NORMAL` on fresh feedback; after **K=`MOTOR_ENABLE_MON_K`** (derived from config,
=3 at 200 Hz) not-running fresh frames it faults `CAUSE_NOT_ENABLED`. (Bench-verified: a direct
CH341 CAN-disable while armed trips `CAUSE_NOT_ENABLED` and latches FAULT.)

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
- Type-1 sent once per motor per 200 Hz tick (in HOLD/MIT/DAMPED/TO_ZERO); Type-2 arrives async
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

`MotorLifecycle` (full `state` byte), driven by `motor_runtime_update` + the pure `mode_sm`
(level-triggered per-motor mode requests). States: `BOOT, DISCOVERING, IDLE, HOLD, MIT,
DAMPED, TO_ZERO, FAULT`.

```
 BOOT ─▶ DISCOVERING ─▶ IDLE ──REQ_HOLD (not wound)──▶ HOLD ◀──────────────┐
                         ▲   ▲                          │  ├─REQ_MIT──▶ MIT │ (REQ_HOLD
                         │   │  REQ_IDLE / watchdog      │  ├─REQ_DAMPED─▶ DAMPED  from any
                         │   └──────────────────────────┤  └─REQ_TO_ZERO▶ TO_ZERO armed:
                         │                               │        (arrived flag)   capture)
  REQ_HOLD & wound ──▶ FAULT ◀── any fault trip         └── between armed modes: free ──┘
                         │  (latched: armed reqs rejected)
      REQ_IDLE / FAULT_RESET flag ──▶ IDLE (clears latch)
```

Transitions & triggers (the decision is pure — `mode_sm_step`; `apply_cmd` does the CAN work):
- **BOOT→DISCOVERING→IDLE:** startup probe (`discover()`); a dead motor stays BOOT (not alive).
- **IDLE→HOLD:** `REQ_HOLD` → `arm_enable` (mode-change + enable; the only enable). Refused if
  not alive, or **wound** (|offset| > `MOTOR_WOUND_OFFSET_MAX`) → FAULT/`CAUSE_WOUND`.
- **HOLD↔MIT↔DAMPED↔TO_ZERO:** any armed→armed request is free. HOLD captures position; MIT
  stores fixed-point targets; DAMPED is Kp=0+Kd; TO_ZERO creeps to 0 then holds (`arrived`).
- **MIT watchdog:** loss of fresh MIT for the watchdog window → fall back to HOLD (lock pos).
- **any armed→IDLE:** `REQ_IDLE` (`do_disable`) or the master-link watchdog (`CAUSE_WATCHDOG`).
- **any armed→FAULT:** a fault trip drops output and latches the cause.
- **FAULT→IDLE:** `REQ_IDLE`, or `FAULT_RESET` flag (clears, then re-evaluates the same request).

### Fault causes (`MotorFaultCause`; latched until IDLE request / FAULT_RESET)
| cause | value | raised by |
|---|---|---|
| `CAUSE_NONE` | 0 | — |
| `CAUSE_OVERTORQUE` | 1 | measured `|tau| > cfg->max_tau` while driving |
| `CAUSE_CAN_TIMEOUT` | 2 | Type-2 feedback stale ≥ `MOTOR_CAN_FB_TIMEOUT_MS` (100 ms) while driving |
| `CAUSE_WATCHDOG` | 3 | SPI-command watchdog expired ≥ `MOTOR_WATCHDOG_MS` (200 ms) — master link stopped |
| `CAUSE_MOTOR_FAULT` | 4 | RS motor's own Type-2 fault bits set; triggers a `0x3022` read |
| `CAUSE_ZERO_TIMEOUT` | 5 | TO_ZERO made no progress toward home for `MOTOR_ZERO_STALL_MS` (1500 ms) |
| `CAUSE_NOT_ENABLED` | 6 | armed motor reported not-running for K fresh feedback frames |
| `CAUSE_WOUND` | 7 | HOLD-arm refused: shaft wound beyond the safe single-turn range (re-zero offline) |

### Timeouts / watchdogs (values + effect)
| name | value | effect |
|---|---|---|
| `MOTOR_WATCHDOG_MS` | 200 ms | no valid SPI command in window → armed motor → IDLE (`CAUSE_WATCHDOG`); MIT first falls back to HOLD. Refreshed by the master keepalive → detects master-link, **not host**, death |
| `MOTOR_CAN_FB_TIMEOUT_MS` | 100 ms | stale feedback → `CAUSE_CAN_TIMEOUT` |
| `MOTOR_ZERO_STALL_MS` | 1500 ms | no homing progress → damp + `CAUSE_ZERO_TIMEOUT` |
| `can_tx` mailbox wait | ≤1 ms | bounded busy-wait for a free TX mailbox (not a reply wait) |

---

## 10. Config flow

```
 configs/<setup>/{slave*.yaml, system.yaml}  ──(scripts/gen_motor_config.py)──▶ generated files
```
`gen_motor_config.py` reads `configs/active` (a pointer file; `$SOCCER_SETUP` overrides the
*generator only*), the setup's `slave*.yaml`, and an optional **`system.yaml`** (rates +
timeouts; defaults in the generator if absent), then emits:

| generated file | flag | consumed by |
|---|---|---|
| `firmware/common/include/motor_config.h` | `--slave slaveN` | **slave** build: `N_MOTORS`, `MotorConfig[]` (can_id, model, soft_min/max, max_vel, **max_tau**, default_kp/kd), `MOTOR_ZERO_*`, `MOTOR_WOUND_OFFSET_MAX`, **rates + derived periods/tick-counts** (`MASTER_POLL_PERIOD_MS`, `MOTOR_LOOP_PERIOD_MS`, `MOTOR_WATCHDOG_MS`, `MOTOR_ZERO_SETTLE_TICKS`, `MOTOR_ENABLE_MON_K`, …) |
| `firmware/common/include/system_config.h` | `--system` | **master** build: `NUM_SLAVES`, `MAX_MOTORS_PER_SLAVE`, `TOTAL_MOTORS`, per-slave counts, the same rate/period block (`MASTER_POLL_HZ`, …) |
| `host/master_link/motor_config_gen.py` | `--system` | **host**: `MOTORS`, `N_MOTORS`, `MOTOR_DEFAULT_KP/KD`, `MOTOR_SOFT_MIN/MAX`, `RATES`/`HOST_CMD_HZ`, `CONFIG_NAME`, `CONFIG_HASH` (sha256 over the setup's YAMLs) |

**Rates as config (`system.yaml`):** `master_poll_hz`, `telemetry_hz`, `slave_tick_hz`,
`host_cmd_hz` (Hz) and timeouts/debounces in **ms**; the generator derives all tick-based
firmware constants from them, so changing a rate keeps the real-world durations fixed. The
rates are reported in `MasterStatus` and recorded in the `.bin` header.

The host `config_meta.check_config_fresh()` compares the generated `CONFIG_NAME/HASH` against
the live `configs/active` YAMLs and warns loudly on staleness or a `$SOCCER_SETUP` divergence.
The wire motor count `N` is baked into all three at build time; there is **no in-band N check**.

---

## 11. End-to-end traces

### A. MIT command → applied echo (cmd_seq round trip, ~24 ms median; `16-36-18_man_1s_1m.bin`)
1. `policy.step` → `MotorCommand(mode=MIT)` → `MasterLink.send_robot_cmd` → `pack_robot_cmd`
   (one `cmd_robot_t`, cmd_seq=k) → `encode_frame` → `serial.write`. **[USB ≈ 0.5–1 ms]**
2. master `CDC_Receive_FS` (USB-ISR, ~7 pkts) → CRC + version gate → `MotorMaster_HandleRobotCmd`
   → `host_chain[s]` + `host_cmd_seq=k`. **[waits for next poll ≤ 5 ms]**
3. master `poll_one_slave` (200 Hz) → `SPI_OP_ROBOT_CMD` + `cmd_chain_t` → `spi_exchange`.
4. slave SPI ISR → `cmd_inbox` (ping-pong); main → `spi_proto_parse_cmd` → per-motor
   `motor_runtime_apply_cmd` → mode SM → MIT targets (`target_cmd_seq=k`), clamp, `LIFE_MIT`.
5. slave `motor_runtime_update` (200 Hz) → `send_mit` (stamps k, opens reply window) →
   `can_mit_control_set` → **Type-1 CAN** → motor **[APPLIED]**. **[CAN ≈ 0.11 ms]**
6. motor **Type-2** reply → slave CAN RX ISR (`motors[]`, `fb_count++`) → next tick
   `cmd_seq_on_reply` promotes `last_applied_seq=k`. **[fb_age ≤ 5 ms]**
7. slave `motor_runtime_sample` → `tele_motor_t` → `spi_proto_build_tele`.
8. master next poll reads `tele_chain_t` → `latest_tele`; `emit_robot_tele` (populated prefix) →
   TX ring → USB. **[USB ≈ 0.5–1 ms]**
9. host RX thread → `decode_frame` → `parse_robot_tele` → `MotorSnap`; `latency.py` sees
   `last_applied_seq ≥ k`.
- **Where the ~24 ms goes:** ~1.6 ms USB RTT + **two 200 Hz poll quantizations** (command poll +
  telemetry poll, ~5 ms each) + CAN hop + `fb_age`. The echo flips on the controller's
  acknowledgement, ~5 ms before a measurable torque departure (the old ~29 ms torque-onset).

### B. HOLD arm (no response frame — telemetry is the ack)
1. `policy`/user → `MotorCommand(mode=HOLD)` in the per-tick `cmd_robot_t` → USB → master
   `host_chain[s]`.
2. master `poll_one_slave` → `SPI_OP_ROBOT_CMD` → slave `motor_runtime_apply_cmd`: mode SM says
   IDLE+HOLD (not wound) → `arm_enable` (fault-clear → Type-18 MIT-mode → Type-3 enable → settle,
   **~40 ms blocking**) → `LIFE_HOLD`. A **wound** shaft → refused → `CAUSE_WOUND`.
3. The host confirms arming by watching the motor's `state` reach `HOLD` in `MSG_ROBOT_TELE`
   (there is no CONTROL_RESP). Level-triggered: the host keeps requesting HOLD each tick; once
   armed, a re-HOLD re-captures position without re-enabling.

### C. Fault → convert_log output
1. slave `motor_runtime_update` fault check (e.g. `|tau| > max_tau`) → `fault_to` →
   `state = LIFE_FAULT`, `cause = CAUSE_OVERTORQUE` (latched).
2. `motor_runtime_sample` → `tele_motor_t{state,cause}` → `spi_proto_build_tele` → SPI.
3. master `spi_exchange` → `latest_tele` → `emit_robot_tele` → USB.
4. host RX thread → `parse_robot_tele` (`cause=1`); the raw frame is logged as `RX_FRAME`.
5. `convert_log.py` → `motor_state.csv` row with `cause_name=OVERTORQUE`;
   `plot_motor_state.py` draws a red fault-onset line. Clear: request IDLE then HOLD, or the
   `FAULT_RESET` flag.

---

## 12. Known limitations

- **Blocking HOLD-arm handshake.** `arm_enable` blocks the slave loop ~40 ms on the IDLE→HOLD
  transition (mode-change + enable with ACK waits); arming N motors serializes. Steady-state
  HOLD/MIT/DAMPED/TO_ZERO is non-blocking. (Async confirm-and-retry handshake is planned.)
- **No host-death timeout.** The 200 ms `MOTOR_WATCHDOG_MS` is refreshed by the master's
  keepalive (NOP/command every poll), so it detects **master-link** death, **not host** death:
  if the host stops but the master keeps polling, an armed motor holds indefinitely. `kill -9`
  of the runner does not safe the motor — only a clean Ctrl-C (or explicit IDLE) does. A
  host-liveness watchdog is a separate task (the master-clock redesign).
- **One-shot CAN TX** (`AutoRetransmission = DISABLE`): a lost frame is not retransmitted
  (rare enable-loss, ~0.2%). Fix planned.
- **`max_tau` trip sensitivity.** `max_tau = 0.8 N·m` on the RS02 is easily hit by a position
  step (`kp·Δpos`, default `kp=15`, trips at ~0.053 rad). Zero/ramp to the trajectory start first.
- **No in-band SPI `N` check.** Slave/master built from different configs → silent "dead slave"
  with climbing `crc_errors`. Both must build from the same active config.
- **Fixed-point ranges** (v3): `pos` carries only home-frame ±π (multi-turn not on the wire);
  a shaft wound beyond `MOTOR_WOUND_OFFSET_MAX` is refused at arm (`CAUSE_WOUND`) and must be
  re-zeroed offline (firmware never writes mech zero). Gains use universal scales (Kp ×10,
  Kd ×100) covering every model; values beyond those saturate (flagged `SATURATED`).
- **USB first-command settle.** After a fresh connect, the first host→master command(s) may not
  land until the CDC link settles (~1 s); the runner's retry loop and `wait_until_live` absorb
  this. The boot soft-disconnect (§2) prevents the harder multi-reflash OTG-OUT wedge.

---

## 13. Timing & dataflow model

**Snapshot:** PROTO_VERSION 3 (robot/chain/motor hierarchy). A scheduling/
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
| F1 | policy → Action | time, 50 Hz | host main | `latest_state()` snapshot (latest-value, lock-copied) | `Action{motors=[MotorCommand]}` | `run_policy.main`, `Policy.step` |
| F2 | send → USB | event (step) | host main | Action list → per-slave chains | one `MSG_ROBOT_CMD` (`cmd_robot_t`); **cmd_seq stamped once per `send_robot_cmd` call** | `link.send_robot_cmd`→`pack_robot_cmd`/`_write_frame` |
| F3 | master USB RX | event (USB OUT, ~7 pkts) | OTG_FS ISR (0) | `accum[512]` reassembly; CRC + version gate | `host_chain[s]` (latest-value mailbox) + `host_cmd_seq` | `CDC_Receive_FS`→`MotorMaster_HandleRobotCmd` |
| F4 | master SPI poll | time, 200 Hz | master main | `host_chain[s]` (latest-value), else NOP | one SPI cmd frame (**blocking** `HAL_SPI_TransmitReceive`) | `ProcessLoop`→`poll_one_slave`→`spi_exchange` |
| F5 | slave SPI RX | event (master clock) | DMA2 (0) + SPI1 TxRxCplt ISR (1) | ping-pong RX halves | `cmd_inbox_buf` + `data_receive_flag` | `slave_spi.HAL_SPI_TxRxCpltCallback` |
| F6 | slave dispatch | event (`data_receive_flag`) | slave main | `cmd_local` copy (IRQ-masked), CRC-checked | per-motor `apply_cmd` (mode SM): mode/targets/`target_cmd_seq` | `main.c`→`spi_proto_parse_cmd`→`motor_runtime_apply_cmd` |
| F7 | control tick → CAN | time, 200 Hz | slave main | `motors_rt[]` | Type-1 MIT frame; **opens cmd_seq reply window** | `motor_runtime_update`→`send_mit`→`can_mit_control_set` |
| F8 | motor apply | event (Type-1 on bus) | RS02 firmware | — | **[APPLIED]**, emits Type-2 feedback | motor |
| R1 | slave CAN RX | event (Type-2 on bus) | CAN1_RX0 ISR (1) | decode into `motors[]` slot | `motors[]`, `fb_count++`, `can_feedback_count++` | `motor_chain.HAL_CAN_RxFifo0MsgPendingCallback` |
| R2 | telemetry rebuild | event (`can_feedback_count` Δ) | slave main | `motor_get_snapshot` (IRQ-masked) + `cmd_track.last_applied` | `tele_chain_t` staged into `tele_stage_buf` (ping-pong) | `main.c`→`motor_runtime_sample`→`spi_proto_build_tele`→`spi_write_next_tx_buf` |
| R3 | slave→master SPI | event (next master poll) | DMA + ISR | ping-pong TX | `tele_chain_t` into master RX | `spi_exchange` (RX half) |
| R4 | master telemetry emit | time, 200 Hz, **gated on slave_alive** | master main | `latest_tele[s]` (latest-value) | `MSG_ROBOT_TELE` (populated prefix) into TX ring | `emit_robot_tele`→`usb_tx` |
| R5 | master USB TX | event (ring non-empty) | master main (`usb_tx_pump`) | ring buffer (8192 B), 64 B/transfer | bytes to host | `usb_tx.c` |
| R6 | host RX ingest | event (bytes), ~1 kHz poll | host RX thread | `bytearray` reassembly | `_motors[key]=MotorSnap` (latest-value, lock); log `RX_FRAME` | `link._rx_loop`→`_ingest` |
| R7 | host log write | event (per frame/record) | queued → writer thread | bounded queue (100 000) | `.bin` on disk | `datalog` writer |

### 13.3 Rate transitions (cross-domain crossings)

| crossing | can drop? | can duplicate? | delay |
|---|---|---|---|
| host send (50 Hz) → `host_chain` mailbox (F2→F3) | yes, if two commands land within one 5 ms poll window — **not** at 50 Hz | no | ≤ one poll (≤5 ms) |
| `host_chain` → SPI poll (F3→F4) | — | yes: last chain (or NOP) re-sent when no fresh command (level-triggered, intended) | 0–5 ms (poll quantization) |
| slave SPI dispatch → control tick (F6→F7) | no | no | 0–5 ms (next 200 Hz tick) |
| CAN feedback (~200 Hz) → telemetry rebuild (R1→R2) | no (latest state rebuilt next feedback) | no | deferred to the rebuild if main loop busy |
| telemetry ping-pong → SPI (R2→R3) | — | yes: previous telemetry re-sent if no fresh frame staged | 0–5 ms |
| `latest_tele` → `MSG_ROBOT_TELE` (R4) | **yes**: a dead slave's chain is omitted (host silence = freshness signal) | — | ≤5 ms |
| master ring → host (R5) | yes on ring overflow (not expected: ~26 KB/s ≪ ~64 KB/s ceiling) | no | ~0.3–1 ms |
| host RX → `latest_state` mailbox (R6→F1) | policy-visible: intermediate frames coalesced (newest wins); **all** frames kept in the log | no | ≤ one runner period (≤20 ms @ 50 Hz) |

### 13.4 Timing diagrams

**MIT command — host send → motor apply → applied echo back** (measured round trip ≈ **24 ms**
median, `logs/2026-10-01/16-36-18_man_1s_1m.bin`):

```
t=0   host send_robot_cmd (cmd_seq k)
  │  USB OUT (~5 pkts) + master CDC RX ISR  ~0.5–1 ms   → host_chain[s] (latest-wins)
  │  wait for next master SPI poll          0–5 ms      (200 Hz quantization)
  │  SPI transfer (blocking)                ~0.1 ms     → slave cmd_inbox
  │  wait for next slave control tick       0–5 ms      (200 Hz) → apply_cmd stores target
  │  control tick: Type-1 MIT → CAN         ~0.1 ms     ►► [APPLIED at motor], reply window open
  ── return ──
  │  motor Type-2 feedback                  motor-internal
  │  slave CAN RX ISR → motors[], fb_count  ~0.1 ms     → cmd_seq_on_reply: last_applied=k
  │  telemetry rebuild → ping-pong          < next poll
  │  next master poll clocks telemetry back 0–5 ms
  │  master emit MSG_ROBOT_TELE (200 Hz)    0–5 ms      → USB ring (populated prefix)
  │  host RX thread ingest                  0–1 ms
t≈24 ms  host sees last_applied_seq ≥ k      (median; min 20.1, p95 30.2, max 31.1)
```

**Telemetry sample — motor → policy** (and → log):

```
t=0   motor Type-2 on bus
  │  slave CAN RX ISR → motors[] + fb_count  ~0.1 ms
  │  telemetry rebuild (event) → tele_stage  < next poll
  │  master SPI poll clocks it back          0–5 ms
  │  master emit MSG_ROBOT_TELE (gated,200)  0–5 ms     → ring
  │  master USB TX pump → host               ~0.3–1 ms
  │  host RX thread ingest → latest_state     0–1 ms    ├─► log RX_FRAME (writer thread)
  │  policy.step reads snapshot              0–20 ms    (next 50 Hz runner tick)
```

Reference slices: **ping RTT ~1.6 ms** (host↔master only, no SPI/CAN —
`logs/2026-09-29/21-25-09_latency.bin`); **cmd_seq round trip ~24 ms** (above, v3); **torque
onset ~29 ms** (`21-25-09_latency.bin`) — physical torque departs baseline *after* the command
is acknowledged, so it trails the echo.

### 13.5 Host rate vs the master's 200 Hz poll

| host rate | per command | coalescing in `host_chain` | polls with no fresh cmd | expected cmd_seq effect |
|---|---|---|---|---|
| 50 Hz | 1 / 20 ms | none (two commands never share one 5 ms window) | ~3 of 4 → last chain re-sent | 0 never-applied; spread ≈ one poll period |
| 200 Hz | 1 / 5 ms | host/master clocks are independent (no handshake) → beats: occasionally two commands land in one poll window (older cmd_seq overwritten, never applied), occasionally zero | occasional | small but **nonzero** never-applied; lower mean latency, similar spread |

**Measured:** the 50 Hz row — median **24.4 ms**, **0 never-applied of 1000**
(`16-36-18_man_1s_1m.bin`). **Analysis (not measured):** the 200 Hz row — the beat between the
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
