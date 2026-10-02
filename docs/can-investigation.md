# CAN enable-loss investigation

Investigation into the intermittent RobStride **enable** failure — a motor commanded
to arm/zero occasionally stayed in reset mode, so the slave's streamed setpoints did
nothing (originally surfaced as a misleading `ZERO_TIMEOUT` ~1.5 s later; now caught
promptly as `CAUSE_NOT_ENABLED` by the continuous enable monitor).

Bench: single unloaded RS02, `1s_1m`, slave `slave_general`, master over
USB-CDC. Investigation used temporary slave instrumentation (`TEMP DIAG`) streaming a
per-enable-attempt record to the host `.bin`; that scaffolding has since been removed,
leaving the permanent enable monitor.

## CAN bus configuration (`firmware/slave/.../main.c` `MX_CAN1_Init`)

- **`AutoRetransmission = DISABLE`** — one-shot transmit (NART=1): the peripheral does
  **not** retransmit a frame that loses arbitration or hits a bus error. This is the
  key finding: normal CAN would retry until ACKed; here a single lost enable is dropped
  with no recovery.
- `AutoBusOff = DISABLE` — no automatic bus-off recovery.
- Clocks: HSE 8 MHz → PLL (M4, N84, P2) → SYSCLK 84 MHz, APB1 = 42 MHz (CAN1 kernel clock).

### Bit timing
`Prescaler=2, BS1=16TQ, BS2=4TQ, SJW=1TQ` → bit time = 1+16+4 = 21 TQ.
- Bitrate = 42 MHz / 2 / 21 = **1.000 Mbps**
- Sample point = (1+16)/21 = **~81 %**
- Standard/correct for RobStride.

### Termination (measure with the bus powered OFF)
Resistance across CANH–CANL should read **~60 Ω** (two 120 Ω terminators in parallel).
~120 Ω = only one terminator; ≪60 Ω = an extra terminator; open/kΩ = none.

## Measured per-enable-attempt data (instrumented)

Every captured attempt (arm entry, goto-zero entry, goto-zero arrival re-enable) was
**clean and identical**:

| field | value (all attempts) |
|---|---|
| `HAL_CAN_AddTxMessage` | OK, free mailboxes = 3 |
| TX result (TSR) | **TXOK=1, ALST=0, TERR=0** (sent, no arbitration loss, no TX error) |
| bus errors (ESR) | **TEC=0, REC=0, LEC=0**; no error-passive / bus-off |
| RX FIFO overrun | none |
| mode-change TX → its reply | ~320 µs |
| mode-change reply → enable TX | **~10.5 ms** (the `HAL_Delay(10)` gap) |
| enable TX → its reply | ~2–4 ms |
| reported mode after enable | NORMAL |

The ~10.5 ms reply→enable gap (present on *every* attempt, never near 0) means the
enable is queued into a quiet bus long after the mode-change reply has drained — so in
the current blocking handshake the enable **cannot** collide with that reply. The
handshake also halts the 200 Hz poll loop, so the bus is otherwise idle at enable time.

## Attempt counts and failure rate

| run | condition | enable attempts | NOT_ENABLED |
|---|---|---|---|
| `22-10-31_can_exp1_baseline.bin` | isolated arm/goto | ~320 | 0 |
| `22-13-10_can_exp2_streamed.bin` | arm → 1.5 s sine → disable → goto | ~510 | 0 |
| `22-28-44_man_1s_1m.bin` | manual reproduction attempt | 24 captured | 0 |
| `21-35-43_val1_disable.bin` | forced disable over CH341 (positive control) | 1 | **1** (detected in K=3 frames ≈ 10 ms) |

- **~860 natural enable attempts across all conditions: 0 failures.**
- 0/320 isolated statistically rules out a 4 % rate (p ≈ e^(−0.04·320) ≈ 3×10⁻⁶).
- The original "~4 %" came from **1 failure in 23** zeroing episodes (`15-13-23_man_1s_1m.bin`)
  — a small-sample estimate. Combined with 0/860 the true rate is **≤ ~0.2 %** (≈ 1 in 500+).
- `val1` confirms the *detection* path works: a real disable is caught as `NOT_ENABLED`,
  and a disabled RS02 keeps replying to MIT (reports `reset`), so the loss is seen as
  fresh not-running frames, not silence (not `CAN_TIMEOUT`).

## Conclusion

- The enable loss is **rare** (≤ ~0.2 %), not 4 %. Under bench conditions the TX path,
  bus, and timing are pristine — it refutes bus-error / arbitration-loss / TX-not-sent /
  reply-lost as *routine* causes.
- When a loss does occur, **`AutoRetransmission = DISABLE` is why it isn't recovered**:
  a single arbitration/error event (or a rare motor-side non-ACK) is dropped with no retry.
- The gap-sweep (0/1/2/5/10 ms mode-change→enable) was **not run** — the ~10.5 ms gap
  already prevents the reply collision and no failures were available to characterize.

## Fix plan (not yet implemented)

- **B — confirm-and-retry** (chosen): fold into the async handshake. After enable,
  confirm the motor reports NORMAL (the enable monitor already detects the failure);
  retry a bounded number of times; a permanent enable-retry counter records occurrences.
  Safe with absent motors (bounded, no wedged mailbox).
- **A — auto-retransmit + abort-on-timeout** (`NART=0`): separate later change, tested
  with a motor unplugged (an absent motor never ACKs → must abort the TX after a timeout
  or it wedges a mailbox). Recovers bus-level losses at the hardware layer.
- C (schedule critical frames after the reply) is redundant given the measured 10.5 ms gap.

## Log files

- Baseline: `logs/2026-09-28/22-10-31_can_exp1_baseline.bin`
- Streamed: `logs/2026-09-28/22-13-10_can_exp2_streamed.bin`
- Manual reproduction: `logs/2026-09-28/22-28-44_man_1s_1m.bin`
- Positive control (forced disable): `logs/2026-09-28/21-35-43_val1_disable.bin`
- Historical natural failure (1/23): `logs/2026-09-28/15-13-23_man_1s_1m.bin`
