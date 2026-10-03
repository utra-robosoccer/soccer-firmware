# Robosoccer motor-control system — architecture

**Commit:** `e30d756` (2026-10-02, branch `akp/single_motor_rework`).
**Verified against code at that commit** — where older prose and the code disagreed, the code
wins; the discrepancies are listed at the end of this document.

This is the single system-overview document. The manufacturer CAN reference is
`robostride-motor-reference.md`; the CAN enable-loss study is `can-investigation.md`. (Four
docs an older `architecture.md` claimed to replace — `protocol.md`, `telemetry-path.md`,
`slave.md`, `command.md` — do not exist in the tree.)

---

## 0. Snapshot + glossary

| | |
|---|---|
| Commit / date | `e30d756` / 2026-10-02 |
| `PROTO_VERSION` | **8** (`firmware/common/include/protocol.h`) |
| Active config | **`1s_5m`** (`configs/active`) — 1 slave, 5 motors: 2×RS02 (can 1,2) + 3×RS00 (can 3,4,5) |
| Master MCU | STM32F446, SYSCLK 72 MHz, `firmware/master/` |
| Slave MCU | STM32F446, SYSCLK 84 MHz, `firmware/slave/slave_general/` |
| Other configs on disk | `1s_1m` (1 RS02), `2s_10m` (2 chains ×5), `robot/` |

### Glossary
| term | meaning |
|---|---|
| **cycle** | one master control period (TIM2, 5 ms @ 200 Hz): poll every slave, then emit telemetry |
| **cycle_id** | `uint16` master cycle counter (`tele_robot_t.cycle_id`); the host echoes it in commands |
| **chain** | one slave's motors on its own CAN bus; the SPI payload is exactly one chain |
| **exchange** | one full-duplex SPI transfer between master and one slave (command out, telemetry in) |
| **service** | the slave running `motor_runtime_update` once → one Type-1 CAN frame per motor |
| **forward-on-command** | the slave services on receipt of each valid exchange, not on its own timer |
| **cmd_seq** | `uint16` host command sequence, one per host tick (≥1, 0 = `CMD_SEQ_NONE`); echoed back as `last_applied_seq` |
| **spi_seq** | `uint8` per-slave SPI link-health counter, echoed as `spi_seq_echo` — distinct from `cmd_seq` |
| **arm (motor)** | enable a motor: the blocking CAN handshake on IDLE→HOLD (`arm_enable`, ~23 ms) |
| **arm (SPI TX)** | load the slave's next telemetry frame into the TX DMA (`spi_arm_tx`) — unrelated to motor arming |
| **mailbox (master)** | the double-buffered per-slave command store (`pend_*` → swap → `host_chain[]`) |
| **late-arm** | the slave delays the SPI-TX arm to a TIM3 deadline so the fresh CAN reply rides the next exchange |
| **service / fallback** | forward-on-command vs the slave's backup tick when exchanges stop (`slave_service.h`) |

---

## 1. High-level overview

### Components and ownership
| component | where | owns |
|---|---|---|
| **Jetson host** | `host/` (Python) | policies (`Policy.step` → `Action`), `MasterLink` (serial, RX thread, binary log), analysis tools. No real-time control; reacts to telemetry. |
| **Master** | `firmware/master/` | USB-CDC ↔ SPI bridge; the TIM2 200 Hz clock; the double-buffered command mailbox; telemetry assembly/forwarding; the host-death watchdog. **No motor logic** — never interprets motor data. |
| **Slave(s)** | `firmware/slave/slave_general/` | all per-motor control: mode state machine, CAN Type-1/2 codec, soft limits, `cmd_seq` pairing, enable monitor, CAN-timeout + master-loss watchdogs, discovery/arming. One slave = one CAN bus = one chain. |
| **RobStride motors** | RS02 / RS00 | the actuators; classic CAN 2.0, 1 Mbps, operation-control (MIT) mode. |

### Timing model — one clock
The master's **TIM2 compare interrupt is the only clock in the system** (`master_cycle.c`, 200 Hz
on an absolute µs grid, no drift). Everything downstream **reacts on receipt**:
- the slave services its motors on each valid SPI exchange (**forward-on-command**), not on a timer;
- the host steps its policy when a telemetry frame arrives (`wait_robot`), not on a host timer.

The remaining independent timers are **safety/backup only**:
| timer | where | role |
|---|---|---|
| slave fallback tick | `main.c` `HAL_GetTick` / `LOOP_POLL_PERIOD_MS` | services motors **only if exchanges stop** (SPI link lost) so they keep holding until a watchdog trips |
| slave TIM3 late-arm deadline | `tx_arm_timer.c` | fires `TX_ARM_DEADLINE_US` after each exchange to arm the next TX frame; paced by the exchange, not free-running |
| master status divider | `spi_master.c` (off TIM2) | 20 Hz `MasterStatus`/`SlaveStatus`, counted in cycles |
| host RX thread | `link.py` `_rx_loop` | blocking serial read; decodes frames, not a control clock |
| host log writer | `datalog` | background flush ~1 Hz |

Master SYSCLK 72 MHz; slave SYSCLK 84 MHz (slave CAN kernel = APB1 42 MHz, see §3e).

### One cycle, start to end (200 Hz, 5 ms)
```
t=0.00 ms  TIM2 CC1 fires → g_due=1 (ISR does no work)
           master main loop: master_cycle_take() → master_cycle_id++
                             mailbox_swap()  (apply the command received last cycle: n→n+1)
                             host_watchdog_step()  (host-death check)
           poll_one_slave(s): build cmd frame (active chain or NOP or DAMPED/IDLE override),
                              blocking HAL_SPI_TransmitReceive (one exchange) ───────────────┐
t≈0.05 ms  slave SPI DMA completes → HAL_SPI_TxRxCpltCallback: hand RX to main, start TIM3   │
           slave main: copy+CRC command → apply_cmd (mode SM) → motor_runtime_update:        │
                       send one Type-1 MIT per motor on CAN  ►► [targets APPLIED]            │
t≈0.9–1.5  motor Type-2 replies arrive on CAN (serialized ~140 µs apart) → CAN RX ISR        │
           → motor_runtime_on_feedback: mirror state, pair cmd_seq → stage telemetry         │
t≈3.0 ms   TIM3 deadline → spi_arm_tx: swap the freshly-staged frame into the TX DMA         │
t=5.00 ms  next TIM2 cycle: master clocks that telemetry back in the next exchange ──────────┘
           master: latest_tele[s] ← telemetry; emit_robot_tele → USB TX ring → host
           host RX thread decodes → wait_robot wakes the runner → policy.step → send command
```
Net: a command computed from cycle *n*'s telemetry is applied at the slave in cycle *n+1* and its
confirmation (`last_applied_seq`) rides the telemetry of cycle *n+2* — **answered→confirmed = 2
master cycles** (see §4).

### Command path (master's view, one line each)
Host `send_robot_cmd` → USB → master RX ring → `proto_frame_scan` → `MotorMaster_HandleRobotCmd`
fills **pending** mailbox → `mailbox_swap` at next cycle start moves it to **active** → `poll_one_slave`
sends it as `SPI_OP_ROBOT_CMD` → slave applies at that exchange.

### Telemetry path (master's view, one line each)
Slave stages `tele_chain_t` on CAN feedback → clocked into the master's RX during the next exchange
→ CRC-checked, stored in `latest_tele[s]`, slave marked alive → `emit_robot_tele` assembles the
alive chains into a `tele_robot_t` (populated prefix only) → USB TX ring → host.

---

## 2. Data structures

All multi-byte wire fields are **little-endian, packed** (`PROTO_PACKED`). Fixed-point scales are
shared (`protocol.h`): pos ×10000 (home-frame ±π, i16), vel ×100 (i16), tau ×100 (i16), Kp ×10
(u16), Kd ×100 (u16). The C header and `host/master_link/protocol.py` are kept in lockstep by the
cross-language fixture test (`firmware/common/test/gen_fixture.c` ↔ `host/tests/test_protocol.py`).

### 2.1 Wire structs (`firmware/common/include/protocol.h`, `_Static_assert` on every size)

**`MsgHeader` — 16 B** (every USB frame; CRC covers header-with-crc=0 + payload)
| field | type | meaning | writer | reader |
|---|---|---|---|---|
| `type` | u16 | `MsgType` | `proto_build` | `proto_frame_scan` / `decode_frame` |
| `seq` | u16 | per-message counter (PING echoes it) | sender | diagnostics |
| `src`,`dst` | u8,u8 | `NodeId` (JETSON 1, MASTER 2, SLAVE_0 3, SLAVE_1 4) | sender | — |
| `ts_ms` | u32 | sender `HAL_GetTick()` (host live-detect reads the master's) | sender | host `_LiveDetector` |
| `len` | u16 | payload bytes | `proto_build` | scanner |
| `ver_flags` | u16 | low byte = `PROTO_VERSION` (8); high reserved 0 | `proto_build` | both ends drop+count on mismatch |
| `crc16` | u16 | CRC16-CCITT | `proto_build` | scanner |

**Command hierarchy (host→master→slave)**
| struct | size | fields |
|---|---|---|
| `cmd_motor_t` | 12 B | `mode_req` (`MotorModeReq`), `pos/vel/kp/kd/tau_ff` (fixed-point), `flags` (`CMD_FLAG_*`) |
| `cmd_chain_t` | 62 B | `chain_id`, `n_motors`, `cmd_motor_t[5]` |
| `cmd_robot_t` | 254 B | `cycle_id` (host echo of last telemetry cycle), `cmd_seq`, `n_chains`, `reserved` (test sentinel 0xDE/0xB0), `cmd_chain_t[4]` |

**Telemetry hierarchy (slave→master→host)**
| struct | size | fields |
|---|---|---|
| `tele_motor_t` | 21 B | see 2.2 |
| `tele_chain_t` | **118 B** | `chain_id`, `n_motors`, `spi_seq_echo`, `spi_resyncs` (u8, wraps), `slave_time_us` (u32), `cmd_crc_errors` (u16), `can_tx_errors` (u16), `spi_tx_arm_fails` (u8, wraps — added v8), `tele_motor_t[5]` |
| `tele_robot_t` | **492 B** | `cycle_id`, `master_time_us` (u32), `last_cmd_seq_rx`, `cmd_seq_active`, `cmd_on_time`, `cmd_late`, `cmd_missing`, `cmd_duplicate` (all u16), `n_chains`, `robot_state`, `tele_chain_t[4]` |

> The in-code size **comments** on `tele_chain_t` (“117”), `tele_robot_t` (“488”) and the SPI block
> (“119 B”) are stale; the `_Static_assert`s (118 / 492) and `SPI_XFER_SIZE` are correct.

**`tele_motor_t` — 21 B** (built by `motor_runtime_sample`, parsed into host `TeleMotor`)
| off | field | type | units / encoding | writer | purpose |
|---|---|---|---|---|---|
| 0 | `pos` | i16 | rad ×10000, home-frame wrapped [−π,π] | slave | measured position |
| 2 | `vel` | i16 | rad/s ×100 | slave | measured velocity |
| 4 | `tau` | i16 | N·m ×100 | slave | measured torque |
| 6 | `temp_c` | u8 | °C | slave | motor temperature |
| 7 | `state` | u8 | `MotorLifecycle` (full byte) | slave mode SM | lifecycle |
| 8 | `cause` | u8 | `MotorFaultCause` (latched) | slave | why faulted |
| 9 | `motor_mode` | u8 | RS Type-2 run mode (0 reset/1 cal/2 normal) | slave (from CAN) | enable monitor source |
| 10 | `motor_fault` | u8 | packed Type-2 fault bits | slave | motor self-faults |
| 11 | `flags` | u8 | `TELE_FLAG_*` | slave | rejected/arrived/saturated/clamped/stale |
| 12 | `fb_age_ms` | u8 | ms since last Type-2, sat 255 | slave | feedback freshness |
| 13 | `fault_word` | u32 | 0 clear / 0xFFFFFFFF read-fail / raw 0x3022 | slave | RS fault register |
| 17 | `last_applied_seq` | u16 | host `cmd_seq` last confirmed applied (0 = none) | slave (`cmd_seq_track`) | latency pairing |
| 19 | `reserved` | u16 | 0 | — | growth slot |

**`MasterStatus` — 30 B** (`emit_master_status`, 20 Hz)
`robot_state`, `slave_alive` (bitmask), `uptime_ms`, `link_errors`, `rx_frames`, `master_poll_hz`,
`telemetry_hz`, `slave_tick_hz`, `host_cmd_hz`, `rx_resyncs`, `rx_discarded_bytes`.
> **There is no `missed_deadlines` field.** TIM2 overruns are counted internally
> (`master_cycle_overruns()`) but are not currently on the wire.

**`SlaveStatus` — 18 B** (`emit_slave_status`, 20 Hz, master-generated)
`slave_id`, `motors_alive` (bitmask), `uptime_ms`, `crc_errors` (telemetry frames the master
dropped), `cmd_crc_errors` (relayed from the slave), `seq_gaps` (master-observed echo stalls).

**SPI frame (one exchange; `SPI_XFER_SIZE` = max of the two = 120 B)**
| dir | layout | size |
|---|---|---|
| master→slave | `[opcode u8][spi_seq u8][cycle_id u16][cmd_seq u16][cmd_chain_t (62)][crc16]` | 70 B |
| slave→master | `[tele_chain_t (118)][crc16]` | 120 B |

**Enums / flags**
| enum | values |
|---|---|
| `MsgType` | PING 0x01, MASTER_STATUS 0x02, SLAVE_STATUS 0x03, ROBOT_CMD 0x08, ROBOT_TELE 0x09 |
| `MotorModeReq` (cmd) | IDLE 0, HOLD 1, MIT 2, DAMPED 3, TO_ZERO 4 |
| `MotorLifecycle` (tele `state`) | BOOT 0, DISCOVERING 1, IDLE 2, HOLD 3, MIT 4, DAMPED 5, TO_ZERO 6, FAULT 7 |
| `MotorFaultCause` | NONE 0, OVERTORQUE 1, CAN_TIMEOUT 2, WATCHDOG 3*, MOTOR_FAULT 4, ZERO_TIMEOUT 5, NOT_ENABLED 6, WOUND 7, MASTER_LOST 8 |
| `RobotState` | INIT 0, READY 1, DEGRADED 2, HOST_LOST 3 |
| `CMD_FLAG_*` | VALID b0, USE_CONFIG_GAINS b1, FAULT_RESET b2 |
| `TELE_FLAG_*` | REQUEST_REJECTED b0, TO_ZERO_ARRIVED b1, SATURATED b2, CLAMPED_POS b3, CLAMPED_TAU b4, CMD_STALE b5 |

\* `CAUSE_WATCHDOG` (3) is defined but **no longer raised** — the master-loss ramp uses
`CAUSE_MASTER_LOST` (8) instead (see §7, §8).

### 2.2 Firmware internal state

**Master (`spi_master.c`)**
| structure | type | role |
|---|---|---|
| `pend_chain[NUM_SLAVES]`, `pend_chain_valid`, `pend_cmd_seq`, `pend_cycle_echo`, `pend_fresh`, `pend_fault_reset` | pending mailbox | filled by USB dispatch; `pend_fresh` set last |
| `host_chain[NUM_SLAVES]`, `host_chain_valid`, `g_cmd_seq_active` | active mailbox | swapped in at cycle start; re-sent each poll |
| `latest_tele[NUM_SLAVES]` | `tele_chain_t` | last CRC-valid telemetry per slave |
| `slave_alive[]`, `slave_motors_alive[]` | u8 | presence / per-motor alive mask |
| `cnt_on_time/late/missing/duplicate` | u16 | host-loop-vs-cycle counters |
| `g_host_wd`, `g_host_action`, `g_cycles_since_fresh` | `HostWatchdog` | host-death FSM |
| `usb_rx_ring[1024]` (head/tail) | SPSC ring | ISR produces, main consumes |
| TX slot ring `tx_slot[16][512]` + `resp_buf[8][40]` | `usb_tx.c` | whole-frame TX; ISR responses queued separately |

**Slave**
| structure | file | role |
|---|---|---|
| `motors[]` (`motor_t`) | `motor_chain.c` | CAN-ISR-written feedback; IRQ-masked snapshot is the only read path |
| `motors_rt[]` (`MotorRuntime`) | `motor_runtime.c` | mirrored state, mode, targets, `cmd_kp/kd`, `hold_pos`, `watchdog_ms`, `armed_ms`, cmd_seq track, enable-mon count |
| SPI RX/TX ping-pong halves | `slave_spi.c` | one DMA-owned, one main-owned, swapped at `spi_arm_tx` |
| `CmdSeqTrack` | `cmd_seq_track.c` | `cur_seq`/`awaiting`/`last_applied` reply-window pairing |
| enable-monitor count | `enable_monitor.c` | consecutive not-NORMAL fresh frames |

**Host**
| structure | file | role |
|---|---|---|
| `LinkState{motors,master,slaves,robot,stamp_ns}` | `link.py` | snapshot handed to `policy.step` |
| `MotorSnap` | `link.py` | decoded per-motor telemetry (SI units + flags props) |
| `Action{motors:[MotorCommand]}` | `base.py` | a policy's per-tick output |
| binary log header + records | `datalog/format.py` | `RLOG` header + `[kind u8][mono_ns u64][len u32][payload]` records |

---

## 3. Per-hop detail

### 3a. Host ↔ master (USB CDC)
- **Transport:** STM32 Virtual ComPort, VID:PID `0483:5740`, 115200 nominal (CDC ignores baud).
  `MasterLink` opens `exclusive=True` (TIOCEXCL) so ModemManager / a second process can't inject
  bytes; a udev rule maps it to `/dev/robosoccer-master`. `reset_input_buffer()` on open.
- **Boot soft-disconnect** (`main.c`): drives D+ (PA12) low ~10 ms at boot so the host re-enumerates
  after an ST-Link reflash — otherwise the shared OTG-FS OUT endpoint can wedge.
- **Host → master:** one write per frame (`_write_frame`, no `flush()`), `MSG_ROBOT_CMD`
  (270 B frame ≈ 5 USB OUT packets) or `MSG_PING`.
- **Master RX (out of ISR):** `CDC_Receive_FS` (OTG_FS ISR, prio 0) only copies bytes into the
  lock-free `usb_rx_ring` (`MotorMaster_UsbRxFromISR`). The main loop drains it with the
  resynchronizing `proto_frame_scan` (`MotorMaster_ProcessUsbRx`): header-plausibility (type /
  version / length) first, CRC only when plausible; junk drops one byte at a time
  (`master_rx_resyncs`/`master_rx_discarded`) so a stray byte can't desync permanently. Wrong
  version → resync (dropped+counted).
- **Master TX:** whole-frame slot ring (`usb_tx.c`, 16×512 B); `usb_tx_pump` hands one frame per
  `CDC_Transmit_FS`, adds a ZLP after any exact 64 B multiple, kicks on enqueue. ISR-built PONGs go
  through a separate `resp_buf` queue drained by main (`usb_tx_pump_responses`) → `usb_tx_write`
  stays single-producer. Ring full → whole frame dropped (`usb_tx_drops`).
- **Host RX:** `_rx_loop` blocks on `read()` (0.1 s timeout), decodes with `decode_frame`
  (one-byte resync on bad CRC; CRC via `binascii.crc_hqx`). `MSG_ROBOT_TELE` bumps a frame counter
  and notifies `wait_robot`; the runner steps on telemetry (see 3b, §7e).

### 3b. Master internals
- **TIM2 cycle** (`master_cycle.c`): 32-bit free-running µs counter (PSC 71 → 1 MHz), CH1 compare at
  `MASTER_CYCLE_US` on an absolute grid (`CCR1 += period`). The ISR sets `g_due` + the fire time and
  **does no work**; `MotorMaster_ProcessLoop` does everything. Prio 2 (below OTG_FS/SPI1 = 0).
- **Per cycle:** `master_cycle_id++` → `mailbox_swap()` → `host_watchdog_step()` → `poll_one_slave`
  for every slave in order → `emit_robot_tele(t0)` (`master_time_us` = the cycle's TIM2 µs stamp) →
  every 10th cycle (`MASTER_POLL_HZ/20`) `emit_master_status` + `emit_slave_status`.
- **Double-buffered mailbox / apply rule:** a command received during cycle *n* is applied at *n+1*.
  `MotorMaster_HandleRobotCmd` fills `pend_*` and sets `pend_fresh` last; `mailbox_swap` (cycle
  start) moves the newest complete set into `host_chain[]` and sets `g_cmd_seq_active`, or holds the
  active set and counts a missing cycle. It classifies the four counters (§6).
- **Poll:** `poll_one_slave` sends the active chain (`SPI_OP_ROBOT_CMD`), a `SPI_OP_NOP` keepalive if
  nothing was ever applied, or a master **override** (`REQ_DAMPED`/`REQ_IDLE`) when the host-death
  watchdog has tripped. CRC-valid reply → `latest_tele[s]`, slave/motor alive masks, echo-stall
  diagnostics. CRC-fail/absent → slave marked offline (its chain is then omitted from telemetry).
- **Telemetry assembly + prefix truncation:** `emit_robot_tele` copies only currently-alive slaves'
  chains and transmits only the **populated prefix** (`offsetof(chains)+nc·118`, 138 B payload for
  one chain) instead of the full 492 B — the host parses by `n_chains`, so the truncation decodes
  identically and keeps the 200 Hz stream small.
- **Host-death watchdog:** `host_watchdog_step` (see §7c).

### 3c. Master ↔ slave (SPI)
- **Exchange:** one blocking `HAL_SPI_TransmitReceive` per slave per cycle (master is SPI master,
  CPOL0/CPHA0, MSB-first; CS on GPIOC `SLAVE_CS_0..3`). 120 B each way. Frame CRC doubles as integrity
  and presence check (absent slave clocks back garbage → CRC fail).
- **Prescaler:** generated `MASTER_SPI_PRESCALER_DIV` (= **16** → 4.5 MHz; APB2 72 MHz). A bench
  sweep found 64/32/16/8 all CRC-clean on short wiring; **re-validate on the robot harness** and drop
  the prescaler if CRC/resync counters climb.
- **CRC:** CRC16-CCITT (`poly 0x1021`, init 0xFFFF), 256-entry table in flash; identical on both MCUs
  and to the host's `binascii.crc_hqx`.
- **Slave DMA ping-pong:** SPI-slave DMA fills one RX half while main parses the other; TX likewise.
- **Late-arm deadline (TIM3):** `TxRxCplt` does **not** arm the next exchange (the CAN reply isn't in
  yet); it starts TIM3 for `TX_ARM_DEADLINE_US` (3000 µs = 0.6 cycle). TIM3's ISR calls `spi_arm_tx`,
  which swaps the freshly-staged frame and arms the DMA — so the reply rides exchange A+1, not A+2.
  Guards: `spi_write_next_tx_buf` sets `data_tx_ready_flag` only after the whole frame incl. CRC is
  written, behind a `__DMB()`, so the ISR never arms a half-written buffer; `spi_arm_tx` runs once per
  cycle (`armed_this_cycle`) and only from the TIM3 ISR; every `HAL_SPI_TransmitReceive_DMA` return is
  checked → `spi_tx_arm_fails++` + retry-once; TIM3 priority = 0 (== DMA2 streams) so arm and
  exchange-complete can't preempt each other; `slave_spi_resync` masks `TIM3_IRQn` while it resets.
- **SPI resync:** on a command-CRC failure the slave calls `slave_spi_resync` — gated on NSS high
  (`spi_resync_poll`), it disables SPI, aborts both DMA streams, flushes the RX FIFO, re-arms (byte 0
  realigns next exchange). Count in `tele_chain_t.spi_resyncs` (u8, wraps). Test injector:
  `-DSPI_INJECT_TEST` clocks one short exchange (armed by a `0xDE` `cmd.reserved` sentinel).

### 3d. Slave internals
- **Main loop (`main.c`):** on `data_receive_flag`, copy the command under a brief IRQ mask
  (`cmd_local`), CRC-check (`spi_proto_parse_cmd`); on a fresh `SPI_OP_ROBOT_CMD` refresh every
  motor's watchdog, `motor_runtime_apply_cmd` per motor, then **re-capture `now`/DWT** (arming can
  block ~23 ms) and `motor_runtime_update` (forward service). The fallback tick services only when
  exchanges have stopped (`slave_service_due`, `slave_service.h`).
- **Feedback (`motor_runtime_on_feedback`):** on each CAN feedback (`can_feedback_count` change)
  mirror the Type-2 into `motors_rt`, step the enable monitor, and pair `cmd_seq` **now** (so the
  staged frame carries the fresh confirmation).
- **cmd_seq reply-window pairing (`cmd_seq_track.c`):** a MIT frame opens the window
  (`cmd_seq_on_mit_frame`); the next fresh Type-2 closes it crediting `last_applied`
  (`cmd_seq_on_reply`); any non-MIT frame closes it without crediting; arm/re-hold/to-zero/disable
  reset it (`reset_cmd_seq` from the mode SM). Host comparison is wrap-aware (`seq_ge`, RFC-1982).
- **Soft-limit one-sided clamp (`soft_limit.h`):** `soft_clamp_pos` extends `[lo,hi]` to include the
  shaft's actual position, so a joint parked outside its range can hold / return but not be driven
  further out (avoids a clamp-induced overtorque at arm). Sets `CLAMPED_POS`/`CLAMPED_TAU`.
- **Master-loss watchdog (`master_watchdog.h`):** see §7c.
- **Timebase:** DWT cycle counter (84 MHz); `ms_since(now,t)` is a signed subtraction so a
  `watchdog_ms`/`armed_ms` set during the ~23 ms arm can't underflow into a spurious trip.

### 3e. Slave ↔ motor (CAN, RobStride)
- **Bus:** classic CAN 2.0, 1 Mbps (`Prescaler 2, BS1 16TQ, BS2 4TQ, SJW 1TQ` → 21 TQ, ~81 % sample
  point; APB1 42 MHz). **`AutoRetransmission = DISABLE` (one-shot / NART)**, `AutoBusOff = DISABLE`.
- **Extended-ID codec (`robostride_id.h`):** 29-bit ID = `mode[28:24] | data[23:8] | node_id[7:0]`.
  Replaced an earlier C bitfield (endianness/packing-fragile) with explicit shift/mask helpers
  (`rs_extid_pack/mode/data/id`), pinned by `test_robostride_id.c`.
- **Types used in the hot loop (`robostride.c`):** Type-1 operation-control out (`can_mit_control_set`:
  ID data = torque_ff u16, payload BE u16×4 = pos/vel/Kp/Kd); Type-2 feedback in
  (`can_unpack_motor_feedback`: id, 6 fault bits, run mode, payload pos/vel/torque/temp). Enable
  (Type-3), stop/clear (Type-4), write run-mode (Type-18), set-zero (Type-6), id/param (Type-0/7/17)
  are used only in discovery/arming/diagnostics.
- **3 TX mailboxes + busy-wait:** `can_tx` busy-waits (≤~1 ms) for a free mailbox before
  `HAL_CAN_AddTxMessage`. With 5 motors, 2 sends/cycle find all 3 busy and spin — ~280 µs of wasted
  slave CPU per cycle. **The bus serializes frames regardless of mailbox count**, so this is a CPU
  cost, not a latency one (see §4, §11).
- **Arm handshake (`arm_enable`, ~23 ms, blocking, once per IDLE→HOLD):** optional fault-clear
  (`HAL_Delay(5)`) → write MIT run-mode + ≤10 ms ACK wait + `HAL_Delay(10)` → enable + ≤10 ms ACK
  wait + `HAL_Delay(10)` → first MIT. Never writes mechanical zero.
- **`wrap_pi` / `pos_offset` and the boot-angle finding:** `pos` on the wire is the home-frame angle
  wrapped to [−π,π]; `pos_offset` holds the whole-turn remainder so a power-cycle single-turn reading
  can't command a long-way rotation. A shaft wound beyond `MOTOR_WOUND_OFFSET_MAX` (9.0 rad) is
  **refused at arm** (`CAUSE_WOUND`) rather than re-zeroed in firmware.
- **`zero_sta`:** the RobStride "mechanical zero" write is not used on the MIT feedback path (it does
  not change the reported feedback frame); re-zeroing is an offline operation.
- **Measured send/reply timing (5 motors):** see §4.

---

## 4. Timing and performance

Measured numbers and their source log. Anything marked *(1-slave baseline)* predates the 5-motor
bench and has not been re-measured.

| quantity | value | source |
|---|---|---|
| master cycle period / jitter | 5000 µs, σ ≈ 9 µs idle / 26 µs under load; 0 overruns *(1-slave baseline)* | TIM2 bring-up run |
| poll + telemetry assembly | ≈ 2.55 ms/cycle (headroom to 5 ms) *(1-slave baseline)* | TIM2 bring-up run |
| SPI exchange | ~0.05 ms (120 B @ 4.5 MHz + HAL overhead) | derived |
| CAN per frame | ~130–140 µs (8-byte extended @ 1 Mbit) | §3e / can-investigation |
| CAN first-send → last-reply (5 motors) | span mean **1413 µs**, max 1431; last reply max **1490 µs** | `logs/2026-10-02/20-25-16_bench_sine.bin` |
| CAN per-motor reply time (from exchange) | m0 911 / m1 1051 / m2 1192 / m3 1333 / m4 1473 µs (~140 µs apart) | same |
| RobStride processing | ≈ 0.7 ms (first-reply latency ≈850 µs minus the request frame) | same |
| TX-arm deadline | steady 3006 µs; 200 Hz margin (arm − last reply) mean **1533 µs**, min 1516 | same |
| cmd_seq round trip (TX → applied, per tick) | median **9.25 ms**, p95 9.4, max 10.0, 0 superseded | `20-25-16_bench_sine.bin` |
| master-clock latency | **answered→confirmed 2 cycles** (applied→confirmed 1) | same |
| on-time / late / missing / duplicate (200 Hz, N=1) | 100 % / 0 / 0 / 0 | same |
| host loop budget (arrival→written) | ~1.6 ms ≈ 33 % of a 5 ms cycle | same |
| ping RTT (host↔master only) | median ~1.56 ms *(older `21-25-09_latency.bin`)* | latency log |
| torque onset | ~29 ms *(older)* — physical departure trails the echo | latency log |

### 200 Hz budget, 4 slaves
The master polls slaves sequentially; one exchange ≈ 0.05 ms transfer + ~0.1–0.2 ms HAL overhead.
Four slaves + telemetry assembly fit comfortably inside 5 ms (1-slave assembly is ~2.55 ms; the
per-slave increment is small). The slave side is independent per chain (one CAN bus each), so the
CAN floor is per-chain, not aggregate.

### 400 Hz analysis and levers
At 400 Hz the cycle is 2.5 ms and the 0.6-cycle deadline is 1.5 ms. The last reply (max 1490 µs)
lands only **~10 µs** before it (p95 +19, mean +27) → effectively no headroom. The last reply is
`m0 request + ~0.7 ms RobStride processing + 4 serialized replies (~140 µs each)`; the trailing
~560 µs is a pure CAN-bus floor. **Levers:** (1) a tuned TX-arm deadline fraction (not fixed 0.6);
(2) a fixed early “all-replied” arm (fix the tracking bug first, §11); (3) split the chain across the
F446's two CAN buses so replies parallelize (last reply ≈ request + 0.7 ms + 2 replies). The slave
loop's hard ceiling is the CAN bus: 2 frames/motor/tick, so ~1/(2·N·140 µs) at full bus — ~520–600 Hz
for 6 motors on one bus; budget ~65 %.

---

## 5. Optimizations in place

| optimization | why | measured effect |
|---|---|---|
| CRC-16 lookup table (`PROTO_CRC16_TABLE`, flash) | per-byte CRC in the hot path | byte-wise CRC at negligible cost; identical wire format |
| `-O2` + `-fno-strict-aliasing` | speed without aliasing UB on the packed-struct casts | stable builds; no aliasing miscompiles |
| configurable SPI prescaler | tune link speed vs margin per harness | div 16 (4.5 MHz) default, all of 64–8 CRC-clean on bench |
| whole-frame USB TX + ZLP (`usb_tx.c`) | frames never interleave; multi-packet frames terminate | supersedes the old 64 B-per-call drain |
| telemetry prefix truncation (`emit_robot_tele`) | the full 492 B @ 200 Hz swamped the Python host | ~138 B/frame for one chain; no host backlog |
| host C CRC (`binascii.crc_hqx`) | Python per-byte CRC too slow at 200 Hz | decode ≫ 200 Hz; RX no longer the bottleneck |
| USB RX resync out of the ISR | ISR only rings bytes; scan+CRC in main | a stray byte/lost packet can't desync permanently |
| double-buffered mailbox | deterministic apply-at-n+1, no torn command set | 0 superseded at 200 Hz telemetry-driven |
| forward-on-command slave service | apply on exchange, not on the slave's own tick | removes a 0–5 ms slave-tick quantization |
| feedback-driven pairing + staging | credit `cmd_seq` and stage telemetry when the reply lands | fresh confirmation rides the next exchange |
| late-arm TX (TIM3) | the reply rides A+1, not A+2 | answered→confirmed 3→**2** cycles; round trip ~14.5→**9.5 ms** |
| telemetry-triggered host loop | step on the master clock, not a host timer | on-time 100 %, deterministic, replay-identical |
| blocking host RX | wake the instant a frame lands, no busy-poll | each tele frame decoded ASAP |

---

## 6. Diagnostics, timing and debug fields

### Counters / timestamps / flags
| field | where | units / wrap | updated by | host use |
|---|---|---|---|---|
| `cycle_id` | tele_robot | u16, wraps | master per cycle | runner stepping (`%N`), dedup |
| `master_time_us` | tele_robot | u32 µs, wraps ~71 min | TIM2 stamp | unwrapped (`U32Unwrapper`) → `policy.step` time |
| `slave_time_us` | tele_chain | u32 µs | slave | diagnostics |
| `cmd_seq` | cmd_robot | u16 ≥1 | host per tick | latency pairing key |
| `last_applied_seq` | tele_motor | u16, 0=none | slave pairing | `latency.py` applied test (wrap-aware) |
| `cmd_seq_active` | tele_robot | u16 | master mailbox | the applied cycle for master-clock latency |
| `last_cmd_seq_rx` | tele_robot | u16 | master RX | command bunching analysis |
| `cmd_on_time/late/missing/duplicate` | tele_robot | u16, wraps | `mailbox_swap` | host-loop-vs-cycle health (deltas) |
| `fb_age_ms` | tele_motor | u8, sat 255 | slave | CAN reply freshness |
| `spi_seq_echo` | tele_chain | u8, wraps | slave echo | master echo-stall → `seq_gaps` |
| `cmd_crc_errors` | tele_chain | u16 | slave | SPI command frames the slave rejected |
| `can_tx_errors` | tele_chain | u16 | slave | CAN TX errors |
| `spi_tx_arm_fails` | tele_chain | u8, wraps (v8) | slave `spi_arm_tx` | TX-arm DMA failures (then retried) |
| `spi_resyncs` | tele_chain | u8, wraps | slave resync | SPI DMA realign events (deltas) |
| `rx_resyncs` / `rx_discarded_bytes` | MasterStatus | u32 | master RX scanner | USB RX resync health |
| `link_errors` / `rx_frames` | MasterStatus | u32 | master | bad-CRC/unknown vs good frames |
| `crc_errors` / `seq_gaps` | SlaveStatus | u32 | master | telemetry CRC fails / echo stalls |
| cycle overruns | `master_cycle_overruns()` | u32 | TIM2 ISR | **internal only** (not on the wire) |
| `TELE_FLAG_*` | tele_motor.flags | bits | slave | rejected / to-zero-arrived / saturated / clamped / stale |

### Tools (host)
| tool | reads | reports / how to run |
|---|---|---|
| `host/analysis/latency.py` | `.bin` | cmd_seq round trip (per-tick + per-motor, applied/**superseded**/never-applied), bunching, master-clock cycles (answered→applied→confirmed), on-time/late/missing/duplicate, missing-vs-expected holds (`--host-rate`). `python3 host/analysis/latency.py <log> --host-rate 200` |
| `host/analysis/convert_log.py` | `.bin` | CSVs (`motor_state` with `cause_name` + `last_applied_seq`, `motor_cmd` with `cmd_seq`, `status`, `events`, `loop_timing`); prints `log complete`/INCOMPLETE from `LOG_DROP` |
| `host/analysis/plot_motor_state.py` | `.bin`/CSV dir | per-motor pos/vel/tau (measured ×, commanded +), fault + mode-change markers, descriptive time axis |
| `host/policies/bench_sine.py` | — | canonical headless driver: minimum-jerk start then centered sine; `--amp/--freq/--motors/--dur`, `--drop-motor s.l --drop-at T` (fault isolation). `python3 host/apps/run_policy.py --policy bench_sine --rate 200 --amp 0.2 --motors all` |
| test-only build flags | firmware | `-DSPI_INJECT_TEST` (short exchange via `0xDE` sentinel), `-DUSB_TX_TEST` (packet-boundary frames via `0xB0`) |

---

## 7. State machines (current implementation)

> **The per-motor mode machine (§7a) is temporary** — a drive-state / control-mode redesign is under
> consideration (§12).

### 7a. Per-motor mode machine (slave, `mode_sm.c` `mode_sm_step`; CAN effects in `motor_runtime.c`)
Pure decision from `(state, mode_req, valid, fault_reset, wound)` → `ModeDecision`
(`do_arm/do_disable/capture_hold/enter_to_zero/clear_fault/set_cause_wound/rejected/reset_cmd_seq`).
States: `BOOT, DISCOVERING, IDLE, HOLD, MIT, DAMPED, TO_ZERO, FAULT`. **Level-triggered** — the
requested mode is re-applied every cycle.

| from \ request | IDLE | HOLD | MIT | DAMPED | TO_ZERO |
|---|---|---|---|---|---|
| IDLE | IDLE | **arm**→HOLD (wound→FAULT/WOUND) | reject | reject | reject |
| HOLD/MIT/DAMPED/TO_ZERO | disable→IDLE | re-capture→HOLD | MIT | DAMPED | enter→TO_ZERO |
| FAULT | clear→IDLE | reject (fault_reset→eval as IDLE) | reject | reject | reject |
| BOOT/DISCOVERING | IDLE | reject | reject | reject | reject |

Key rules:
- **HOLD is the only arm-from-IDLE and the only enable.** On entry: `arm_enable` (handshake), capture
  the home-frame position (`hold_pos`), load config gains.
- **MIT** stores fixed-point targets and `target_cmd_seq`; `apply_soft_clamp` applies the one-sided
  soft-limit clamp. **DAMPED** = Kp 0 + config Kd. **TO_ZERO** creeps to home 0, sets
  `TO_ZERO_ARRIVED`, stalls → `CAUSE_ZERO_TIMEOUT`.
- **Wound rejection:** IDLE→HOLD with `|offset| > MOTOR_WOUND_OFFSET_MAX` → FAULT/`CAUSE_WOUND`.
- **Faults latch:** armed requests rejected until an IDLE request or `FAULT_RESET` clears, then the
  same request is re-evaluated from IDLE.
- **`last_applied_seq` resets to 0** on any of arm / re-hold / to-zero / disable (`reset_cmd_seq`).
- The **arm handshake** (`arm_enable`) and the **enable monitor** interact: after enabling, the
  monitor (§8) watches for `RS_MODE_NORMAL`; K not-NORMAL fresh frames → `CAUSE_NOT_ENABLED`.
- Telemetry per state: `state` = the lifecycle; `cause` set on FAULT; `TELE_FLAG_REQUEST_REJECTED` on
  an illegal request; `CMD_STALE` while running a held/watchdog setpoint.

### 7b. Master/robot state (`compute_robot_state`, `emit_*`)
`ROBOT_INIT` (no slave alive) → `ROBOT_READY` (every alive slave's full motor mask alive) /
`ROBOT_DEGRADED` (some missing). `ROBOT_HOST_LOST` overrides whenever the host-death watchdog is not
`HOST_LINK_OK`.

### 7c. Watchdog state machines
- **Master host-death (`host_watchdog.h`, per cycle):** `HOST_LINK_OK` → (armed && cycles-since-fresh
  ≥ `HOST_LOST_CYCLES` 12 = 60 ms) → `HOST_LINK_DAMPED` (master sends DAMPED, config Kd) → (after
  `HOST_LOST_DAMP_CYCLES` 60 = 300 ms) → `HOST_LINK_IDLE` (master sends IDLE), **latched**; recovery
  only via a fresh command carrying `FAULT_RESET`. Gated on any motor armed.
- **Slave master-loss (`master_watchdog.h`, on a no-fresh-command cycle):** from ms since the last
  `ROBOT_CMD` (`watchdog_ms`; NOP keepalives do **not** refresh it) — `[0,grace)` HOLD with v/tau
  zeroed, `[grace,grace+damp)` DAMPED (config Kd, `CAUSE_MASTER_LOST`), beyond → IDLE. Thresholds
  `MASTER_LOST_GRACE_MS` 50, `MASTER_LOST_DAMP_MS` 300.

### 7d. Slave SPI-TX arm cycle (`slave_spi.c`, `tx_arm_timer.c`)
`TxRxCplt` (exchange complete) → hand RX half to main, start TIM3 → (main stages telemetry on CAN
feedback, sets ready flag) → TIM3 deadline → `spi_arm_tx` swaps + arms the TX DMA. Error path:
`HAL_SPI_ErrorCallback` claims the cycle and re-arms immediately. Resync path: `slave_spi_resync`
masks TIM3, resets DMA, re-arms (see §3c).

### 7e. Chain lifecycle + host side
- **Slave:** `BOOT` → `discover()` (3× ping, ≤300 ms each) → `IDLE` (alive) or `FAULT` (absent). Then
  the forward-on-command loop.
- **Host (`run_policy.py`):** connect (auto VID:PID) → `wait_until_live` (telemetry live-detect, 2 s)
  → `wait_master_status` + **rate verify-or-refuse** (`master_poll_hz` == config) → step on
  `cycle_id % N == 0` (N = `MASTER_POLL_HZ/rate`), late-step/dropped-step accounting → telemetry-stall
  exit (5 cycles) → shutdown disables any armed motor.

### Cross-layer interaction — a single motor faults
| layer | action |
|---|---|
| motor | sets Type-2 fault bits |
| slave | `fault_to` latches `cause`, drops that motor's output → IDLE/FAULT; other motors unaffected |
| master | forwards telemetry unchanged; `slave_motors_alive` reflects it; robot_state → DEGRADED if an expected motor drops |
| host | `MotorSnap.cause`/`state`; `latency.py`/`plot` mark it; policy decides (bench_sine keeps the rest running) |

Per-motor fault isolation is verified: CAN-disabling one motor mid-run leaves the other four in MIT
with unchanged latency (§4 bench).

---

## 8. Safety and fault handling

| watchdog / check | where | trigger | effect |
|---|---|---|---|
| host-death | master `host_watchdog.h` | no fresh host command ≥ `HOST_LOST_CYCLES` while armed | DAMPED → IDLE, latched; `ROBOT_HOST_LOST`; recover via `FAULT_RESET` |
| master-loss | slave `master_watchdog.h` | SPI `ROBOT_CMD` exchanges stopped | HOLD(grace) → DAMPED → IDLE; `CAUSE_MASTER_LOST` |
| CAN feedback timeout | slave `motor_runtime_update` | Type-2 stale ≥ `MOTOR_CAN_FB_TIMEOUT_MS` (100 ms) while driving | `CAUSE_CAN_TIMEOUT` |
| enable monitor | slave `enable_monitor.c` | armed motor not `RS_MODE_NORMAL` for K=3 fresh frames | `CAUSE_NOT_ENABLED` |
| overtorque | slave | `|tau| > cfg->max_tau` (0.8 N·m) while driving | `CAUSE_OVERTORQUE` |
| motor self-fault | slave | Type-2 fault bits set | `CAUSE_MOTOR_FAULT` + 0x3022 read |
| zero stall | slave (TO_ZERO) | no homing progress ≥ `MOTOR_ZERO_STALL_MS` (1500 ms) | damp + `CAUSE_ZERO_TIMEOUT` |
| wound arm | slave `mode_sm` | IDLE→HOLD with `|offset| > MOTOR_WOUND_OFFSET_MAX` | `CAUSE_WOUND` (refuse arm) |

- **All causes latch** until an IDLE request or a `FAULT_RESET` flag clears them.
- `CAUSE_WATCHDOG` (3) and `MOTOR_WATCHDOG_MS` (200 ms) are **defined but no longer used** — the
  master-loss ramp replaced the old hard SPI watchdog.
- **Failure responses:** host dies → master DAMPED→IDLE (host-death). Master resets/SPI dies → slave
  HOLD→DAMPED→IDLE (master-loss). Slave SPI DMA wedges → persistent telemetry CRC → master marks it
  offline, chain goes silent; recovery needs a slave reset. Motor disabled externally → enable monitor
  → `CAUSE_NOT_ENABLED`. Joint parked outside its soft range at arm → one-sided clamp holds it, no
  overtorque.
- **CAN enable loss** (rare, ≤~0.2 %) is a consequence of one-shot TX (`AutoRetransmission=DISABLE`);
  caught by the enable monitor. Full study: `can-investigation.md`.

---

## 9. Configuration and code generation

```
configs/<setup>/{slave0.yaml, slave1.yaml, …}  ──(scripts/gen_motor_config.py)──▶ generated files
configs/active   (one line: the active setup name; $SOCCER_SETUP overrides the generator)
```
Per-slave YAML holds the chain (idx, can_id, model, soft_min/max, max_vel, max_tau, default_kp/kd);
`shared_ranges`/`models` give CAN encode bounds. Rates/timeouts have generator defaults
(`TIMING_MS_DEFAULTS`) — optionally overridable per setup.

| generated file | flag | consumed by | key contents |
|---|---|---|---|
| `firmware/common/include/motor_config.h` | `--slave slaveN` | slave build | `N_MOTORS`, `motor_configs[]`, `MOTOR_WOUND_OFFSET_MAX`, rates + derived periods/counts |
| `firmware/common/include/system_config.h` | `--system` | master build | `NUM_SLAVES`, `slave_motor_counts[]`, `slave_motor_ids[][]`, global bounds, same rate block |
| `host/master_link/motor_config_gen.py` | `--system` | host | `SLAVES`/`MOTORS`, `MOTOR_SOFT_MIN/MAX`, `MASTER_POLL_HZ`, `CONFIG_NAME`, `CONFIG_HASH` |

**Rate-derived constants** (`timing_block`): `MASTER_CYCLE_US = 1e6/master_poll_hz`;
`MOTOR_ENABLE_MON_K = round(enable_mon_ms/tick_ms)` = 3; `HOST_LOST_CYCLES =
round(max(25, 3·1000/host_cmd_hz)·master_poll_hz/1000)` = 12; `HOST_LOST_DAMP_CYCLES =
round(master_lost_damp_ms·master_poll_hz/1000)` = 60; `TX_ARM_DEADLINE_US = round(0.6·1e6/master_poll_hz)`
= 3000. Changing a rate keeps the real-world durations fixed.

`config_meta.check_config_fresh()` compares the generated `CONFIG_NAME/HASH` against the live
`configs/active` YAMLs and warns on staleness or a `$SOCCER_SETUP` divergence. The wire motor count
is compile-time on all three sides — there is **no in-band N check**.

**Switch configs + reflash:** `echo <setup> > configs/active` → `python3 scripts/gen_motor_config.py`
→ `./scripts/build.sh --config slave0 firmware/slave/slave_general` (and `--config slave1` …) +
`./scripts/build.sh firmware/master` → flash each board by its ST-Link serial (connect-under-reset).

---

## 10. Startup and operating sequence

| phase | master | slave | host |
|---|---|---|---|
| power/boot | USB soft-disconnect (PA12), TIM2 start, `MotorMaster_Init` | clocks, SPI-slave DMA armed, TIM3 init **before** SPI, `motor_runtime_init` | — |
| discovery | polls; absent slave → offline | `discover()` each motor (≤~1 s if absent) → IDLE/FAULT (~1.2 s for 5 present) | — |
| connect | — | — | open port, `reset_input_buffer` |
| live-detect | emits telemetry | services on exchange | `wait_until_live` (2 s) on master-ts advance |
| rate check | reports `master_poll_hz` in status | — | refuse if `master_poll_hz` ≠ config |
| arming | forwards HOLD | IDLE→HOLD per motor, `arm_enable` ~23 ms each (**~116 ms for 5 at once**) | policy requests HOLD |
| streaming | poll → telemetry each cycle | forward-on-command MIT | step on telemetry, send commands |
| shutdown | — | — | Ctrl-C → disable armed motors, flush log, print summary |

---

## 11. Known issues

| issue | impact |
|---|---|
| **Blocking arm handshake** | `arm_enable` blocks the slave ~23 ms/motor; arming 5 at once stalls the loop **~116 ms**. It does not trip MASTER_LOST (the post-arm service is cmd-fresh), but it is a visible stall. |
| **Early all-replied TX arm — tracking bug** | the attempted early arm skipped `last_applied` values (~24 % superseded), most likely a per-cycle "replied this cycle" tracking bug (crediting the previous frame's reply), not timing. Reverted; deadline-only is used. Fix before enabling it for 400 Hz. |
| **CAN TX busy-wait** | `can_tx` spins ~280 µs of slave CPU per cycle waiting for a free mailbox (5 motors). CPU waste only (bus-serialized anyway); fix with an interrupt-driven TX queue. |
| **400 Hz deadline margin** | last reply max 1490 µs vs a 1.5 ms deadline → ~10 µs margin. Needs a tuned deadline / fixed early-arm / split CAN buses (§4). |
| **`max_tau` sensitivity** | `max_tau` 0.8 N·m trips on a ~0.05 rad position step at `kp=15`; ramp to the trajectory start first. |
| **Robot-harness SPI re-validation** | div 16 is bench-validated on short wiring only; re-check CRC/resync counters on the real harness. |
| **`configs/active` handling** | the generated headers + `configs/active` are build state, easy to leave mismatched; `check_config_fresh` warns but does not refuse. No in-band N check. |
| **One-shot CAN TX** | `AutoRetransmission=DISABLE` drops a frame lost to arbitration/error (rare enable loss). |

---

## 12. Improvements under consideration

*(Maintained by hand. Template per item: Problem / Idea / Trade-offs / Dependencies / Status.)*

**Async (non-blocking) arm handshake + staggered arming**
- Problem: `arm_enable` blocks ~23 ms/motor, ~116 ms for 5 at once.
- Idea: fire enable + confirm NORMAL over several cycles (confirm-and-retry); stagger to one motor
  per cycle.
- Trade-offs: more state; arming takes more cycles; must stay safe with absent motors.
- Dependencies: enable monitor (exists); the drive-state machine below.
- Status / notes:

**Drive state / control mode split (CiA-402-style)**
- Problem: the single mode machine conflates lifecycle and control mode.
- Idea: joint drive state (OFF, ENABLING, ENABLED, STOPPING, DISABLING, FAULT, ABSENT) × control
  mode (HOLD/MIT/DAMPED/TO_ZERO).
- Trade-offs: a protocol/telemetry change; larger state table; migration.
- Dependencies: async arm; wire-version bump.
- Status / notes:

**Robot-level supervisor on the Jetson**
- Problem: no orchestration (connect, gates, staggered enable, move to start, run, stop, safe).
- Idea: a supervisor above policies with explicit startup gates (link up, rate verified, all motors
  live, in-range, within limits, e-stop clear) and a safe-state path.
- Trade-offs: host complexity.
- Dependencies: live-detect + rate check (exist); start-pose routine.
- Status / notes:

**Whole-robot fault reaction in the master**
- Problem: faults are per-motor; no coordinated whole-robot safing.
- Idea: a master policy that damps/idles the whole robot on a configurable fault class.
- Trade-offs: policy in the master (kept logic-free today).
- Dependencies: robot_state; host-death watchdog.
- Status / notes:

**Chain discovery report + config hash across host/master/slaves**
- Problem: no in-band presence bitmask / firmware-version / config-hash agreement.
- Idea: slaves report present mask + fw version + config hash; master aggregates; host compares.
- Trade-offs: wire additions; version bump.
- Dependencies: `CONFIG_HASH` (host has it).
- Status / notes:

**Fix + re-enable the early all-replied TX arm; tuned deadline fraction for 400 Hz**
- Problem: fixed 0.6-cycle deadline has no 400 Hz margin; early-arm tracking bug.
- Idea: fix the "replied this cycle" tracking; arm on all-replied OR a tuned deadline.
- Trade-offs: determinism vs latency; needs multi-motor re-validation.
- Dependencies: §4 measurements (done).
- Status / notes:

**Interrupt-driven CAN TX queue**
- Problem: `can_tx` busy-waits ~280 µs CPU/cycle.
- Idea: queue frames, feed mailboxes from the CAN TX-empty interrupt.
- Trade-offs: more ISR state; no latency gain (bus-bound).
- Dependencies: none.
- Status / notes:

**Split chains across the F446's two CAN buses**
- Problem: 5 replies serialize on one bus (~560 µs floor).
- Idea: 3+2 motors across CAN1/CAN2 so replies parallelize.
- Trade-offs: wiring; per-bus codec paths; config model.
- Dependencies: config generator per-bus support.
- Status / notes:

**400 Hz master cycle**
- Problem: 200 Hz today.
- Idea: raise `master_poll_hz` once the deadline/CAN levers land.
- Trade-offs: SPI + CAN headroom; deadline margin.
- Dependencies: TX-arm levers; CAN-bus split.
- Status / notes:

**CAN auto-retransmit with abort-on-timeout**
- Problem: one-shot TX drops lost frames.
- Idea: `NART=0` + abort a TX that an absent motor never ACKs (else it wedges a mailbox).
- Trade-offs: must test with a motor unplugged.
- Dependencies: interrupt-driven TX queue helps.
- Status / notes:

**Slave UART debug log (FT232) + logic-analyzer timing points**
- Problem: limited on-slave visibility without a temp instrument.
- Idea: a permanent low-rate UART diag + GPIO timing pins.
- Trade-offs: pins/CPU; keep off the hot path.
- Dependencies: none.
- Status / notes:

**Start-pose routine, max_tau tuning, live viewer, hardware e-stop**
- Problem: operational polish + safety.
- Idea: a move-to-start routine; per-joint `max_tau` tuning; a live telemetry viewer; a hardware
  e-stop in the safe-state path.
- Trade-offs: scope.
- Dependencies: supervisor.
- Status / notes:

*(Add new items below.)*

---

## Discrepancies found (old docs vs code)

1. **Active config** was `1s_1m`; code/`configs/active` = **`1s_5m`**.
2. **`PROTO_VERSION`** doc said 3/6; code = **8**.
3. **`tele_chain_t`** doc said 117 B; code = **118** (`spi_tx_arm_fails` added). `tele_robot_t` 488 → **492**; SPI tele frame 119 → **120**. (The in-code size *comments* in `protocol.h` are also stale.)
4. **`MasterStatus.missed_deadlines`** referenced by the old doc **does not exist**; the struct has `rx_resyncs`/`rx_discarded_bytes`; overruns are internal-only.
5. **`CAUSE_WATCHDOG` / `MOTOR_WATCHDOG_MS`** described as active; both are now **unused** — replaced by the master-loss ramp (`CAUSE_MASTER_LOST`).
6. **Master USB RX** described as ISR `accum[512]` reassembly; it is now a **lock-free ring + main-loop `proto_frame_scan`**.
7. **Host loop** described as a 50 Hz deadline-scheduled grid; it is now **telemetry-triggered** (`wait_robot`, step on `cycle_id % N`), with rate verify-or-refuse.
8. **"No host-death timeout"** (old §12) is **fixed** — the master host-death watchdog is implemented.
9. **`arm_enable`** quoted ~40 ms; measured **~23 ms** (~116 ms for 5 motors).
10. **cmd_seq round trip** quoted ~24 ms (50 Hz grid); now **~9.25–9.5 ms** (telemetry-driven + late-arm), answered→confirmed **2** cycles.
11. The old §13 clock-domain table said the **master runs on `HAL_GetTick` deadlines**; it runs on **TIM2** (the doc contradicted itself).
12. **CAN "lever"** earlier claimed mailbox serialization delayed the last reply; corrected — the **bus** serializes regardless of mailboxes (the mailbox wait is CPU-only).
13. The named docs `protocol.md`/`telemetry-path.md`/`slave.md`/`command.md` the old header claimed to have replaced **do not exist**.
14. Minor in-code stale comments (not doc-fixable here): `link.py` `_LiveDetector` and `run_policy.py` docstrings still say "MOTOR_STATE frames" / "deadline-scheduled"; `MSG_MOTOR_STATE` etc. are gone.
