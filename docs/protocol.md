# The wire protocol: single source of truth and how to change it

Everything on the wire — constants, enums, flag bits, fixed-point scales, every
packed struct, the derived frame sizes — is defined in ONE file:

    firmware/common/protocol.yaml

`scripts/gen_protocol.py` (stdlib + PyYAML) emits the per-language code from it.
The generated files are committed, because the firmware build does not run
Python:

| output | consumer |
|---|---|
| `firmware/common/include/protocol_gen.h` | master + slave firmware, host-run C tests |
| `host/master_link/_protocol_gen.py` | `master_link.protocol` (the host public API re-exports from it) |
| `generated/cpp/robot_wire_gen.hpp` | soccer-software (header-only C++ mirror) |
| `generated/cpp/fixtures/*.bin` | golden byte fixtures soccer-software pins |

Hand-written code stays hand-written: `protocol.h` keeps the fixed-point
codecs, CRC16 and the USB frame builder/scanner; `protocol.py` keeps the
pack/parse API, `TeleMotor`, CRC and frame codec. **Never edit a generated
file** — `gen_protocol.py --check` runs inside the normal test suite and fails
on any drift between the schema and the committed outputs.

## SCHEMA_HASH — how mismatched builds refuse each other

`PROTO_SCHEMA_HASH` is a CRC-32 of the schema's canonical *wire projection*:
the parsed YAML minus every `doc`/`unit`/`hex` key, as compact sorted-key JSON.
Doc-only edits don't change it; any change to names, field order, types,
counts, `truncate_by`, enum/flag values, scales, constants or derived
expressions does.

The hash is enforced at every seam, so two builds generated from different
`protocol.yaml` revisions refuse to talk instead of misparsing each other:

- **host ↔ master**: `MasterStatus.schema_hash`; the host refuses to run on a
  mismatch (same style as the cycle-rate check).
- **master ↔ slave**: both SPI directions end with `spi_trailer_t`
  (`proto_version` + `schema_hash`) before the CRC. A mismatched peer's payload
  is refused; polling continues, so a reflashed board is re-accepted with no
  reset on either side. A pre-trailer build fails CRC at the new offsets and
  simply never becomes alive.
- `PROTO_VERSION` (in every USB `MsgHeader`) remains the coarse fence for
  builds that predate the hash.

## How to change the protocol

1. **Edit `firmware/common/protocol.yaml`** — and only it. No renames or
   semantic changes hidden in generated files or consumers.
2. **Regenerate**: `python3 scripts/gen_protocol.py`. Generation fails loudly
   on rule violations (below) and recomputes SCHEMA_HASH.
3. **Build**: `./scripts/build.sh firmware/slave/slave_general` and
   `./scripts/build.sh firmware/master`. The generated `_Static_assert`s on
   every sizeof and field offset show exactly what moved. Beware: the headless
   builder prints a `Build Finished` line for the *clean* step too — check for
   the real compile pass (or the ELF timestamp) before trusting it.
4. **Producer/consumer code**: firmware fills the new fields; the hand-written
   halves of `protocol.py` (pack/parse) follow; `gen_fixture.c`, the
   `FIXTURES` table in `gen_protocol.py` and `test_cpp_wire.cpp` pin the new
   bytes in all three languages.
5. **Tests**: `PYTHONPATH=host python3 -m unittest discover -s host/tests`
   (includes the C tests, the C++ fixture round-trip and the `--check` drift
   gate), both firmware builds with 0 warnings.
6. **Bench**: flash master + all slaves together (the version gates make a
   partial flash safe but useless), run `arm_timing`, `bench_sine` at 200 Hz
   and `host/analysis/latency.py`; counters that must stay clean:
   `link_errors`, `rx_resyncs`, per-slave `crc_errors`/`seq_gaps`.
7. **Tag the firmware repo**; soccer-software bumps its pin to that tag,
   re-vendors `generated/cpp/robot_wire_gen.hpp` + `generated/cpp/fixtures/`
   and rebuilds. Its fixture tests catch any divergence.
8. Bump `meta.proto_version` whenever a released (flashed-anywhere) build could
   otherwise misparse the new layout — the schema hash protects builds that
   know about it; the version byte protects the ones that don't.

## Schema rules the generator enforces

- **`truncate_by` arrays end the struct.** `tele_robot_t.chains` is transmitted
  as a populated prefix sized by `n_chains`; any field after such an array
  would be silently cut off on the wire, so generation fails.
- **Asymmetric truncation is deliberate**: `cmd_robot_t` also carries
  `n_chains` but is ALWAYS transmitted full-size (254 B) — the host packs it,
  bandwidth up is cheap, and a fixed size keeps the master's RX path trivial.
  Only telemetry truncates.
- **Chain identity is positional**: the master emits every configured chain
  every cycle at its fixed slot (`chains[s]` = slave s) with a per-chain
  `link` byte (`LOST/ALIVE/MISMATCH`). Never remap chains by packing order.
- Unknown or forward-referenced types, duplicate names, enum values that
  overflow their wire type, duplicate enum values, overlapping flag bits,
  non-integer values (PyYAML 1.1 coerces unquoted `no`/`on` to bools — the
  generator rejects them) all fail generation, each with a negative test in
  `host/tests/test_gen_protocol.py`.
- Derived expressions support only integer literals, constant names,
  `sizeof(Struct)`, `+` and `max(a, b)` — a tiny evaluator, no `eval`.

## Transport budgets

- **USB TX slot**: every full frame (header + payload) must fit
  `USB_TX_SLOT_MAX` (schema constant, currently 512). Enforced twice:
  generation fails, and `protocol_gen.h` carries the static assert.
  **Known constraint: the largest tele frame (16 + 496 = 512 B) uses the slot
  exactly — margin 0.** Any telemetry growth (e.g. a ~17 B IMU block) requires
  raising `USB_TX_SLOT_MAX`; RAM cost on the master is `NUM_SLOTS` (16) × Δ.
- **SPI**: `SPI_XFER_SIZE` = max(cmd 75 B, tele 126 B) = 126 B per slave per
  cycle (~224 µs at 4.5 MHz, ~4.5 % of the 5 ms cycle per slave) — derived in
  the schema, asserted in code.

## Bandwidth sanity (200 Hz)

One telemetry frame is `16 + 20 + n_chains × 119` B: 2 chains → 274 B ≈
55 KB/s. `cmd_robot_t` up: 270 B framed ≈ 54 KB/s. Well inside USB FS CDC.
