# Telemetry Path Audit — Motor → Jetson

A complete, field-level audit of the **upstream** (telemetry) path only. Command
direction is out of scope. Purpose: know every struct, every byte, every
produce/consume site, and every drift risk — so the telemetry contract can be
frozen as "v1" before command work begins.

> **Status: v1 frozen (implemented).** This audit's recommendations are now
> applied. Deltas from the "before" text below:
> - **Version byte added** — `MsgHeader.flags` → `ver_flags`; low byte = `PROTO_VERSION` (1). The host drops + counts frames whose version ≠ 1.
> - **Emission gating added** — the master emits `MOTOR_STATE` only for a slave whose most recent poll passed CRC. Silence now means "dead"; the host tracks `age_ms = (now − last_msg) + fb_age`, closing the freshness gap.
> - **Dead fields removed** — `MasterStatus.motors_alive` and `SlaveStatus.motor_state` are gone.
> - **Renames** — `_rsvd`→`reserved_v2`, frame `health_rsvd`→`slave_debug_rsvd`, `MotorStatePayload.motor`→`.atom`, master `latest_tele`→`latest_atom`, and the slave SPI ping-pong buffers. **Wire layout is byte-identical** (proven by the fixture test).
> - **Two hard rules** (now enforced by convention + guards): `MotorState` is append-only; slave & master must build from the same active config.
>
> The "before" analysis below is kept for the reasoning; field names in the tables
> reflect the frozen v1 names.

Line references are approximate (as of the audit; the v1 renames shifted some).

## Layered diagram

```
 MOTOR ── CAN Type-2 ──▶ SLAVE ── SPI frame ──▶ MASTER ── USB MsgHeader ──▶ JETSON
 (RobStride)             (STM32)                (STM32)                     (python)

  big-endian     robostride.c        pack_tele        pass-through        protocol.py
  16-bit fields  → motor_t (float)   → MotorState      MotorState atom     → SI units
                 → snapshot          (LE, 16 B)        verbatim (LE)       + joint names
                 → motors_rt         + CRC16 frame     + MsgHeader (LE)
```

Two re-encodings happen: **CAN big-endian 16-bit → float** (in `robostride.c`),
then **float → little-endian 16-bit `MotorState`** (in `pack_tele`). From the
slave's SPI output to the Jetson it is **little-endian, pass-through, no
re-interpretation**.

---

## Layer 1 — The atom: `MotorState`

**Definition:** `firmware/common/include/protocol.h:109` — 16 bytes, `PROTO_PACKED`.

| off | field | type | units | encoding / quantization | produced | consumed (Jetson) |
|---|---|---|---|---|---|---|
| 0 | `pos_raw` | u16 | rad | home-frame wrapped [−π,π], quantized over ±12.57 (±4π) → 0..65535; **res ≈ 0.00038 rad** | Type-2 decode (`robostride.c:253`) → `motor_t.pos` → snapshot → `motors_rt.pos` = `wrap_pi()` → `f_to_u16` (`motor_runtime.c:453`) | `MotorState.pos` (`protocol.py`, `_decode` over `MOTOR_P_MIN/MAX`) |
| 2 | `vel_raw` | u16 | rad/s | over ±MOTOR_V (widest model, e.g. ±44) → 0..65535; **res ≈ 0.0013 rad/s** | Type-2 (per-model range, `robostride.c:256`) → `motor_t.rpm` → snapshot → `f_to_u16` over **global** bound | `MotorState.vel` |
| 4 | `tau_raw` | u16 | N·m | over ±MOTOR_T (widest, e.g. ±17) → 0..65535; **res ≈ 0.0005 N·m** | Type-2 (per-model, `:259`) → `motor_t.torq` → snapshot → `f_to_u16` global | `MotorState.tau` |
| 6 | `temp_c` | u8 | °C | integer degrees (CAN sends ×10, ÷10 to float, truncated to u8) | Type-2 (`:262`) → `motor_t.temperature` → `(uint8_t)temp` (`:456`) | `MotorState.temp` |
| 7 | `state` | u8 | enum | `[3:0]` lifecycle · `[7:4]` fault cause; `SPI_STATE_PACK` | **runtime logic** — lifecycle from the slave state machine, cause latched on trip (`motor_runtime.c`) | `.lifecycle` / `.cause` (nibble split) |
| 8 | `motor_fault` | u8 | bits | 6 compact fault bits (bit0 undervolt … bit5 uncalibrated) | Type-2 fault byte → `motor_errors` (`robostride.c:237`) → `pack_faults()` u8 | `.motor_fault` |
| 9 | `cmd_flags` | u8 | bits | `CLAMPED_POS`(0) `CLAMPED_TAU`(1) `CMD_STALE`(2); recomputed each tick | **runtime logic** (`apply_mit` + `update` tail) | `.clamped_pos/.clamped_tau/.cmd_stale` |
| 10 | `fault_word` | u32 | code | `0`=clear, `0xFFFFFFFF`=read pending/fail, else raw 0x3022 register | **runtime** (sentinel) + **CAN ISR** (0x3022 latch, `motor_chain.c:99`), mirrored to `motors_rt.fault_word` | `.fault_word` (raw int) |
| 14 | `fb_age` | u8 | ms | ms since this motor's last Type-2, **saturating 255** | computed from cached `last_fb_ms` (`motor_runtime.c:463`) | `.fb_age` |
| 15 | `reserved_v2` | u8 | — | always 0 | `pack_tele` sets 0 | reserved growth byte (append-only), unread |

**Flagged fields:**
- `_rsvd` (byte 15): **packed but never meaningfully read** — pure reserved padding-to-alignment. Fine to keep as the v1 growth/version slot.
- Every other atom field is both set and consumed. No "read-but-never-set" fields.
- **Freshness caveat (important):** `fb_age` measures only the **motor→slave CAN hop**. It is *frozen into the atom* at pack time. If the **slave→master SPI link** dies, the master keeps re-emitting the last good atom (see Layer 3) with a **static** `fb_age` — it does **not** climb to 255. So `fb_age` is not an end-to-end freshness signal; slave-liveness must come from the status message.

---

## Layer 2 — The slave's SPI telemetry frame

**Layout** (`protocol.h:185`), size `SPI_TELE_FRAME_SIZE(N)` = `2 + 16·N + 8 + 2`. For N=3 → **60 bytes**.

| offset | field | bytes | set by | read by | notes |
|---|---|---|---|---|---|
| 0 | `alive_mask` | 1 | slave `main.c:254` (`motor_runtime_motors_alive`) | master `spi_exchange` → `slave_motors_alive[s]` (`spi_master.c`) | bit i = motor i alive; feeds status msgs |
| 1 | `echo_seq` | 1 | slave `main.c:255` (echoes last command seq) | master captures then **`(void)`s it** | command-handshake artifact; **not forwarded to Jetson** → dead weight on the telemetry path |
| 2 | `MotorState × N` | 16·N | `pack_tele` | master `memcpy` → `latest_tele[s][]` | the payload |
| 2+16N | `slave_debug_rsvd[8]` | 8 | slave `main.c` (`memset` 0) | **nobody** — covered by CRC, never parsed | reserved for future per-slave debug (8 B/frame) |
| −2 | `crc16` | 2 | slave `main.c:260` (`proto_crc16` over all preceding) | master verifies (`spi_master.c:102`) | integrity + presence check |

**`echo_seq` and `health_rsvd` are vestigial on the telemetry path.** `echo_seq`
belongs to the command handshake and is discarded at the master; `health_rsvd`
is zeroed, CRC-covered, and never read.

**How N is agreed:** **compile-time, on both sides, from the same generated
config** — there is no negotiation.
- Slave: `PAYLOAD_LENGTH = SPI_TELE_FRAME_SIZE(N_MOTORS)`, `N_MOTORS` from the
  generated `motor_config.h` (per-slave).
- Master: per-slave length `SPI_PKT_SIZE(slave_motor_counts[s])` from the
  generated `system_config.h`.
- Both are generated by `gen_motor_config.py` from the **active setup's YAML**.

**If slave and master disagree on N** (e.g. built from different configs): the
master clocks a different byte count than the slave's DMA expects → **CRC fails
persistently** (best case: slave looks permanently absent) or the slave's DMA
never completes a transfer (worse: telemetry stalls). It **fails safe but
silently** — there is no "N mismatch" diagnostic; it just looks like a dead
slave. Both images must be built from the same active config.

---

## Layer 3 — The master's USB envelope

**`MsgHeader`** (`protocol.h:92`) — 16 bytes, prepended to every USB message:

| field | type | meaning | consumed |
|---|---|---|---|
| `type` | u16 | `MsgType` (MOTOR_STATE=0x04, MASTER_STATUS=0x02, SLAVE_STATUS=0x03) | dispatch |
| `seq` | u16 | per-message counter (`tx_seq++`) | informational |
| `src`/`dst` | u8/u8 | NodeId (MASTER→JETSON) | informational |
| `ts_ms` | u32 | `HAL_GetTick()` at send | informational |
| `len` | u16 | payload bytes | frame boundary + bound check |
| `flags` | u16 | **reserved, must be 0** | unused |
| `crc16` | u16 | CRC16-CCITT over header(crc=0)+payload | integrity + stream resync |

There is no magic byte, but **`ver_flags` low byte now carries the version**
(`= 1`): the host drops + counts frames whose version ≠ 1 (a stale-host guard).
Frame boundaries in the byte stream are still found by the CRC (`decode_frame`
drops a byte and retries on mismatch).

**`MotorStatePayload`** (`protocol.h:145`), 18 B: `slave_id (u8) + motor_idx (u8)
+ MotorState (16 B)`. The atom is embedded **by value**.

**Pure pass-through?** Yes. `emit_motor_state` does
`pay.atom = latest_atom[s][idx]` — a whole-struct copy, **byte-identical**. It
prepends `slave_id`/`motor_idx` and does **not touch any atom byte**. No
endianness swap anywhere: STM32 is little-endian, the packed structs are LE, and
`protocol.py` uses `"<…"` formats — **little-endian is assumed end-to-end**.

**Status message (20 Hz):** two messages.
- `MasterStatus` (`protocol.h:125`, `emit_master_status:117`): `robot_state`
  (INIT/READY/DEGRADED), `slave_alive` bitmask, `motors_alive` (**legacy —
  `slave_motors_alive[0]` only, wrong for multi-slave**), `uptime_ms`,
  `link_errors` (USB), `rx_frames` (command-dir count). Consumed by
  `dashboard.py` for the link/robot panel.
- `SlaveStatus` (`protocol.h:134`, `:152`): `slave_id`, `motors_alive`,
  `motor_state` (**motor 0's lifecycle only — vestigial now that per-motor state
  is in the atoms**), `uptime_ms`, `crc_errors` (the SPI CRC counter). This is
  the **only place slave-liveness and SPI-CRC health reach the host**.

---

## Layer 4 — Contract ownership & drift points

**Shared definition:** `MotorState`, `MsgHeader`, and all payload structs live in
**one header, `firmware/common/include/protocol.h`, included by BOTH MCU builds**
(via `proto_common.h`). No duplicated C definitions. Good.

**Packing:** `PROTO_PACKED = __attribute__((packed))`. The atom is **`memcpy`'d
raw onto/off the wire** on both MCUs (slave builds it through a packed pointer
into the frame buffer; master `memcpy`s wire→struct→`latest_tele`→payload). This
relies entirely on the packed layout matching byte-for-byte — natural alignment
is *not* assumed.

**The Python decode is hand-maintained, not generated.** `protocol.py` carries
its own format strings. The same logical layout is therefore duplicated in **4
places**:

| # | location | form |
|---|---|---|
| 1 | `protocol.h:109` | C struct `MotorState` |
| 2 | `motor_runtime.c` `pack_tele` | field-by-field assignment (implicit order) |
| 3 | `protocol.py` `MOTORSTATE_FMT = "<HHHBBBBIBB"` + dataclass | Python struct format |
| 4 | `docs/protocol.md` / this file | byte table |

**Guards against drift:**
- `_Static_assert(sizeof(MotorState)==16)` (`protocol.h:209`) → catches **C size**
  drift at compile time (loud).
- The **cross-language fixture test** (`host/jetson/tests/test_protocol.py` +
  `firmware/common/test/gen_fixture.c`) packs known values in C and byte-compares
  against Python → catches **C↔Python byte/offset** drift (loud, in CI).
- **Not guarded:** `pack_tele` writing the *wrong field* into a correctly-sized
  slot (semantic, not size) — only the fixture's distinct per-field values catch
  this, and only if the field is exercised. Docs are guarded by nothing.

**Joint-name mapping is defined in two places** (both derived from the YAML, so
consistent, but duplicated logic): `tools/motor_config_gen.py` `MOTORS`
(generated; used by `test_client.py`/`dashboard.py` via `GLOBAL_OF`) **and**
`tools/telemetry.py` `load_motor_table()` (re-reads the YAML directly).

---

## Layer 5 — Versioning & evolution

**There is no protocol version field anywhere** — not in `MsgHeader` (its `flags`
is reserved but unused), not in the atom, not in the frame. Evolution is
therefore **unversioned and unnegotiated**.

**To add a field to `MotorState`** (e.g. a per-motor timestamp or debug word),
every one of these must change together:

| file | change | if missed → |
|---|---|---|
| `protocol.h` | struct field + `_Static_assert` size | **C build fails loudly** (size assert) |
| `motor_runtime.h` | `MotorRuntime` field (if derived) | field ships as garbage/stale |
| `motor_runtime.c` `pack_tele` | set the new field | ships as zero |
| `protocol.py` | `MOTORSTATE_FMT` + dataclass + `parse_motor_state` | **silent misparse** if not updated |
| `gen_fixture.c` + `test_protocol.py` | fixture values | **CI fails loudly** (byte compare) |
| `docs/protocol.md`, this file | byte table | stale docs (silent) |

Frame sizing (`SPI_TELE_FRAME_SIZE`, `BUFFER_SIZE`, `SPI_MAX_PKT_SIZE`) is derived
from `sizeof(MotorState)` → **auto-updates** on both MCUs. Good.

**What breaks silently if one side is missed:** a deployed **old Python against
new firmware**. If the new field is **appended at the end**, old Python reads the
16-byte prefix fine and ignores the extra (graceful). If the field is **inserted
mid-struct**, every following field misaligns → **silent garbage**, and only the
fixture test (if run) would have caught it. The C `_Static_assert` and the CI
fixture are the loud guards; a stale host binary is the silent hole. **A version
byte would turn that silent hole into a detectable mismatch.**

---

## Layer 6 — Rates & identity

**Per-motor emit rate — 200 Hz end-to-end:**
- Motor → slave: one Type-2 per command, ~200 Hz/motor.
- Slave → master: one frame (all N motors) per SPI poll, `MASTER_POLL_PERIOD_MS=5` → 200 Hz.
- Master → Jetson: `emit_motor_state` per motor at `MASTER_TELE_PERIOD_MS=5` → **200 Hz/motor** (emitted whether or not the motor is alive).
- Status: `MASTER_STATUS_PERIOD_MS=50` → **20 Hz** (`MasterStatus` + one `SlaveStatus` per slave).

**Identity at each layer:**
| layer | how a motor is named |
|---|---|
| slave | config array index `i` (0..N−1) → CAN id via `motor_configs[i].can_id` |
| SPI frame | **positional** — array position in `MotorState[N]` = motor index (implicit, no id in the atom) |
| USB | **explicit** — `MotorStatePayload.slave_id` + `motor_idx` |
| Jetson | `(slave_id, motor_idx)` → global flattened index → joint name/model via the YAML-derived table |

The `(slave, idx) → joint` mapping is **spread across `motor_config_gen.py` and
`telemetry.py`** (both from the YAML). There is no id inside the atom — identity
is positional on SPI and explicit only at the USB envelope.

---

## Cleanliness assessment (telemetry-specific)

### Dead / vestigial fields
- **`MotorState._rsvd`** (byte 15) — reserved, unused. *Keep* as the v1 version/growth slot (see below).
- **Frame `echo_seq`** — command-handshake artifact, discarded at the master, never reaches the host. Dead on the telemetry path (leave it; it's the command path's, not telemetry's, to own).
- **Frame `health_rsvd[8]`** — zeroed, CRC-covered, read by nobody. Either *use it* (per-slave bus voltage / temp / error counters — genuinely useful) or shrink it. Right now it's 8 wasted bytes/frame.
- **`MasterStatus.motors_alive`** — legacy, slave-0-only, wrong for multi-slave. **Remove or redefine**; `SlaveStatus.motors_alive` already carries the correct per-slave data.
- **`SlaveStatus.motor_state`** — motor-0-only lifecycle, superseded by per-motor `state` in the atoms. **Remove or repurpose.**

### Duplicated definitions to unify
- `MotorState` layout lives in 4 hand-kept places (C struct, `pack_tele`, Python format, docs). Size + byte drift are guarded (assert + fixture); **field-semantic drift and docs are not.** Optional hardening: generate `protocol.py`'s format string from the C header, or add a field-order fixture assertion.
- Joint mapping duplicated in `motor_config_gen.py` and `telemetry.py`. Pick one source (prefer importing `motor_config_gen`).

### Missing fields that block next steps
- **End-to-end per-motor freshness.** `fb_age` covers only the CAN hop and *freezes* if the SPI link dies. To trust a single motor's data end-to-end you currently must also watch `SlaveStatus.slave_alive`. A per-motor **monotonic sequence or timestamp that the host can watch for "stopped advancing"** would make freshness self-contained. Candidate: repurpose `_rsvd` as a per-motor rolling `tele_seq`, or widen `fb_age`.
- **Debug fields.** There is no spare per-motor debug space except `_rsvd` (1 byte). Per-slave debug can go in `health_rsvd` (8 B) without touching the atom. Per-motor debug means growing `MotorState` — a breaking change, which argues for adding a version first.

### Proposed "v1 frozen telemetry contract"
Freeze exactly this, document it in `protocol.md`, and hold it stable:

1. **Endianness:** little-endian, end-to-end. Assert it in the docs.
2. **`MsgHeader` (16 B):** as-is. **Claim `flags` low byte as `version` (=1)** so future changes are detectable instead of silent.
3. **`MotorState` (16 B):** the 10 fields as defined. **Rename `_rsvd` → `atom_ver` (=1)** or reserve it explicitly as "v2 growth". No mid-struct inserts ever — only append (which keeps old hosts graceful).
4. **Frame:** `[alive_mask][echo_seq][MotorState×N][health_rsvd[8]][crc16]`, CRC16-CCITT over all-but-CRC. N is compile-time from the active config; **document that both images must be built from the same config.**
5. **USB:** `MotorStatePayload = slave_id + motor_idx + atom`, pass-through; identity is explicit here. Status at 20 Hz carries slave-liveness + SPI CRC health (the only freshness signal for the SPI hop).
6. **Guards that keep it honest:** the `_Static_assert` (size) + the cross-language fixture (bytes) are the contract's enforcement — keep both green.

With that frozen, the two things worth doing *before* building on it: (a) add the
one-byte **version**, and (b) decide the **per-motor end-to-end freshness**
mechanism (seq or timestamp), because command/closed-loop logic will want to
reject stale per-motor data without cross-referencing a separate status message.
Everything else (dead-field cleanup, dedup) is non-blocking hygiene.
