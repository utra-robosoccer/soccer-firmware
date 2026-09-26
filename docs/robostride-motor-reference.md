# RobStride Motor Reference

Team reference for the RobStride RS-series integrated joint actuators. This
distills the **maximum capabilities and full communication protocol** from the
vendor documentation (most of which is only in Chinese) into one place.

The focus here is what the motors *can do* per the manufacturer — model specs,
the complete CAN command set, control modes, the full parameter register map,
and all fault/state feedback. It is not a description of any one firmware build.

**Sources** (in `robstride/Product_Information/`):

- `灵足时代RS系列产品规格介绍(260713).pdf` — product specification (2026-07-13) and `README.md`
- `Product Literature/<model>/<model>User Manual260713.pdf` — per-model English manuals (RS00–RS06)
- All-model MIT ranges and parameter tables were extracted directly from each model's manual.

Models covered: **RS00, RS01, RS02 (+IP67), RS03, RS04, RS05, RS06.**

---

## 1. Model lineup & physical capabilities

All models: FOC drive, 3-phase, integrated quasi-direct-drive (QDD) actuator,
CW/CCW, two magnetic encoders (RS01 has one). 48 V nominal except RS01 (36 V).

| Model | Rated / Peak torque | No-load speed | Rated-load speed | Rated / Peak current | Torque const | Ratio | Poles | Rated voltage | Voltage range | Weight |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| RS00 | 5 / 14 N·m | 315 rpm (33.0 rad/s) | 100 rpm (10.5 rad/s) | 4.7 / 15.5 A | 1.48 N·m/Arms | 10:1 | 28 | 48 V | 24–60 V | 310 g |
| RS01 | 7 / 17 N·m | 315 rpm | 100 rpm | — | — | 7.75:1 | — | 36 V | 24–48 V | 380 g |
| RS02 | 6 / 17 N·m | 410 rpm (42.9 rad/s) | 100 rpm (10.5 rad/s) | 7 / 23 A | 1.22 N·m/Arms | 7.75:1 | 28 | 48 V | 24–60 V | 380 g |
| RS03 | 20 / 60 N·m | 195 rpm (20.4 rad/s) | 100 rpm | 13 / 43 A | 2.36 N·m/Arms | 9:1 | 42 | 48 V | 15–60 V | 900 g |
| RS04 | 40 / 120 N·m | 200 rpm | 50 rpm | — | — | 9:1 | — | 48 V | 15–60 V | 1420 g |
| RS05 | 1.6 / 5.5 N·m | 480 rpm (50.3 rad/s) | 100 rpm | 2.4 / 11 A | 0.94 N·m/Arms | 7.75:1 | 20 | 48 V | 15–60 V | 191 g |
| RS06 | 11 / 36 N·m | 480 rpm (50.3 rad/s) | 100 rpm | 14.3 / 57 A | 1.1 N·m/Arms | 9:1 | 28 | 48 V | 15–60 V | 621 g |

- **RS02-IP67**: RS02 electrically, IP67-sealed, 490 g. RS02-IP67 is the only
  model with a stated IP rating.
- **RS04** rated torque is 40 N·m with a 345×345 mm aluminum heat sink, or
  35 N·m with a 220×200 mm sink.
- Rated torque figures assume the aluminum heat-sink plate specified in the spec
  sheet. Peak torque is a short-duration limit (see overload timing).

### Winding / power constants

| Model | Rated power | No-load current | Line resistance | Inductance | Back-EMF |
| --- | ---: | ---: | ---: | ---: | ---: |
| RS00 | 50 W | 0.5 Arms | 1.5 Ω | 750 µH | 0.095 Vrms/rpm |
| RS02 | 60 W | 0.5 Arms | 0.55 Ω | 486 µH | 0.096 Vrms/rpm |
| RS03 | 210 W | 0.6 Arms | 0.39 Ω | 0.275 mH | 17 Vrms/krpm |
| RS05 | 17 W | 0.14 Arms | 2.72 Ω | 0.813 mH | 7.4 Vrms/krpm |
| RS06 | 115 W | 0.98 Arms | 0.23 Ω | 0.165 mH | 7.6 Vrms/krpm |

### Overload timing (continuous rating → temporary overload points)

| Model | Continuous | Temporary overload |
| --- | --- | --- |
| RS00 | 5 N·m | 7 N·m/120 s · 10/18 s · 12/10 s · 14/5 s |
| RS02 | 6 N·m | 6.5/3000 s · 11/100 s · 15/18 s · 17/10 s |
| RS03 | 20 N·m | 30/189 s · 40/126 s · 50/12 s · 60/7 s |
| RS05 | 1.6 N·m | 2/300 s · 3/41 s · 4/12 s · 5/5.5 s · 5.5/4 s |
| RS06 | 11 N·m | 17/200 s · 20/36 s · 25/18 s · 30/8 s · 36/4 s |

### Common environmental / electrical ratings

- Drive method: FOC · Phases: 3 · Running direction: CW/CCW
- Operating temperature: −20…50 °C · Humidity: 5–85%
- Storage temperature: −30…70 °C · Insulation: Class B · Gears: machined steel
- Stator winding insulation: DC 500 VAC, 100 MΩ · Hi-pot: 600 VAC, 1 s, 2 mA

---

## 2. MIT control envelope (per model)

The MIT "operation control" command carries five signals, each packed into a
16-bit field (`0…65535`) mapped linearly over a `[min, max]` range. **Position,
Kp and Kd share a scale within a size class; velocity and torque are per-model.**
These are the full commandable ranges the motor's protocol accepts:

| Model | Position | Velocity | Kp | Kd | Torque |
| --- | ---: | ---: | ---: | ---: | ---: |
| RS00 | ±12.57 rad | ±33 rad/s | 0–500 | 0–5 | ±14 N·m |
| RS01 | ±12.57 rad | ±44 rad/s | 0–500 | 0–5 | ±17 N·m |
| RS02 | ±12.57 rad | ±44 rad/s | 0–500 | 0–5 | ±17 N·m |
| RS03 | ±12.57 rad | ±20 rad/s | 0–5000 | 0–100 | ±60 N·m |
| RS04 | ±12.57 rad | ±15 rad/s | 0–5000 | 0–100 | ±120 N·m |
| RS05 | ±12.57 rad | ±50 rad/s | 0–500 | 0–5 | ±5.5 N·m |
| RS06 | ±12.57 rad | ±50 rad/s | 0–5000 | 0–100 | ±36 N·m |

- Position is always **±12.57 rad = ±4π** (± two turns).
- The larger actuators (**RS03/04/06**) use **wider gain ranges — Kp 0–5000,
  Kd 0–100** — where the small/medium ones use Kp 0–500, Kd 0–5. Gains are not
  interchangeable across size classes.
- The MIT torque field range equals the model's peak torque for RS00–RS02 but is
  the raw protocol envelope for the rest — always clamp to the physical rating.
- Temperature in feedback is transmitted as `raw / 10` °C.

> Note: RS00's manual prints `V_MIN 33.0f` (missing minus sign); the range is
> symmetric ±33 rad/s.

---

## 3. CAN communication protocol

Interface: **CAN 2.0, 1 Mbps, 29-bit extended frames**, 8 data bytes per frame.
The extended ID is split into fields:

```
bits[28:24] = communication type (mode)
bits[23:8]  = data area 2  (usually host CAN_ID; command-specific otherwise)
bits[7:0]   = target motor CAN_ID
```

Default host/master CAN_ID = `0xFD`. MIT payload and feedback are **big-endian**
16-bit fields (high byte first). Each motor has its own CAN_ID (settable).

### 3.1 Complete communication-type table

| Type (hex) | Name | Direction | Purpose |
| ---: | --- | --- | --- |
| 0 (0x00) | Get device ID | Tx→Rx | Read the motor's CAN_ID and 64-bit MCU UID |
| 1 (0x01) | Operation (MIT) control | Tx | Full 5-parameter MIT command; motor replies with a Type-2 frame |
| 2 (0x02) | Motor feedback | Rx | Status/telemetry frame (reply to 1/3/4/6 and to reads) |
| 3 (0x03) | Enable / run | Tx | Energize and start closed-loop |
| 4 (0x04) | Stop | Tx | Disable output; `Byte0=1` also clears faults |
| 6 (0x06) | Set mechanical zero | Tx | `Byte0=1`; sets current shaft angle as zero |
| 7 (0x07) | Set CAN_ID | Tx | Change motor CAN_ID (effective immediately); bits[23:16]=new id |
| 17 (0x11) | Read single parameter | Tx→Rx | Read a register by index (see §4) |
| 18 (0x12) | Write single parameter | Tx | Write a register / switch run-mode. **Volatile** unless saved (Type 22) |
| 21 (0x15) | Fault feedback frame | Rx | Full 32-bit fault word + warning word |
| 22 (0x16) | Save motor data | Tx | Persist `0x20xx` parameters to flash |
| 23 (0x17) | Baud-rate change | Tx | `F_CMD`: 01=1M, 02=500K, 03=250K, 04=125K (**re-power to apply**) |
| 24 (0x18) | Active reporting | Tx | `F_CMD`: 00=off (default), 01=on (auto-report, default 10 ms) |
| 25 (0x19) | Protocol switch | Tx | `F_CMD`: 0=private (default), 1=CANopen, 2=MIT (**re-power to apply**) |

There is also a **version-read** variant of Type 4 (`Byte0=0x00, Byte1=0xC4`)
whose reply carries the motor firmware version number.

### 3.2 Type 1 — MIT operation control (the primary command)

- ID `data area 2` (bits 23:8) = **torque** setpoint (16-bit, per-model range).
- Payload:

| Bytes | Field | Range |
| --- | --- | --- |
| 0–1 | Target angle | ±4π rad (±12.57) |
| 2–3 | Target angular velocity | per-model (e.g. ±44 rad/s RS02) |
| 4–5 | Kp | per-class (0–500 or 0–5000) |
| 6–7 | Kd | per-class (0–5 or 0–100) |

Setting Kp=Kd=0 with zero torque/vel makes it a pure state-poll: the motor
answers with a Type-2 feedback frame without moving.

### 3.3 Control modes (run_mode register `0x7005`, written via Type 18)

| Value | Mode | Description |
| ---: | --- | --- |
| 0 | Operation / MIT | Simultaneous pos+vel+torque+Kp+Kd control (impedance) |
| 1 | Position (PP) | Point-to-point trapezoidal move to a position |
| 2 | Velocity | Closed-loop speed |
| 3 | Current | Direct Iq current command |
| 5 | CSP | Cyclic Synchronous Position (streamed position) |

Each mode is driven by its own reference/limit registers (§4): e.g. velocity
mode uses `spd_ref`/`limit_cur`/`acc_rad`; position/CSP uses `loc_ref`/
`limit_spd`/`limit_cur`; current mode uses `iq_ref`; MIT uses the Type-1 frame.

---

## 4. Parameter register map (Type 17 read / Type 18 write)

Two banks: **`0x70xx`** are live control/observation parameters (volatile);
**`0x20xx`** are the flash-persistent copies (saved via Type 22). Defaults below
are the RS02 manual values — other models scale similarly but confirm per model.

### Control & tuning (`0x70xx`, read/write unless noted)

| Index | Name | Meaning | Range / default |
| --- | --- | --- | --- |
| 0x7005 | run_mode | Control mode select | 0/1/2/3/5 |
| 0x7006 | iq_ref | Current-mode Iq command | −16…16 A |
| 0x700A | spd_ref | Velocity-mode speed command | −33…33 rad/s |
| 0x700B | limit_torque | Torque limit | 0…14 N·m |
| 0x7010 | cur_kp | Current-loop Kp | default 0.17 |
| 0x7011 | cur_ki | Current-loop Ki | default 0.012 |
| 0x7014 | cur_filt_gain | Current filter gain | 0…1.0, default 0.1 |
| 0x7016 | loc_ref | Position-mode angle command | rad |
| 0x7017 | limit_spd | Position/CSP speed limit | 0…33 rad/s |
| 0x7018 | limit_cur | Velocity/position-mode current limit | 0…16 A |
| 0x701E | loc_kp | Position-loop Kp | default 40 |
| 0x701F | spd_kp | Velocity-loop Kp | default 6 |
| 0x7020 | spd_ki | Velocity-loop Ki | default 0.02 |
| 0x7021 | spd_filt_gain | Velocity filter gain | default 0.1 |
| 0x7022 | acc_rad | Velocity-mode acceleration | default 20 rad/s² |
| 0x7024 | vel_max | Position-mode (PP) speed | default 10 rad/s |
| 0x7025 | acc_set | Position-mode (PP) acceleration | default 10 rad/s² |
| 0x7026 | EPScan_time | Active-report interval (1=10 ms, +1=+5 ms) | default 1 |
| 0x7028 | canTimeout | CAN watchdog threshold (20000 = 1 s → reset) | default 0 (off) |
| 0x7029 | zero_sta | Zero convention: 0 = 0–2π, 1 = −π…π | default 0 |
| 0x702A | damper | Post-power-off anti-backdrive damping switch | default 0 |
| 0x702B | add_offset | Zero (position) offset | default 0 |
| 0x702C | alveolous_open | Cogging compensation switch | default 0 |

### Observation only (`0x70xx`/`0x30xx`, read-only)

| Index | Name | Meaning |
| --- | --- | --- |
| 0x7019 | mechPos | Mechanical angle of the load (rad) |
| 0x701A | iqf | Filtered Iq current (−16…16 A) |
| 0x701B | mechVel | Load speed (−33…33 rad/s) |
| 0x701C | VBUS | Bus voltage (V) |
| 0x3007 | vBus | Bus voltage (mV) |
| 0x3016 | mechPos | Mechanical position |
| 0x3017 | mechVel | Mechanical velocity |
| 0x301E | iqf | Filtered Iq current |
| 0x3022 | fault | Full fault code word (see §6) |

### Flash-persistent (`0x20xx`, saved via Type 22)

Key entries: `0x2007` limit_torque · `0x2008` I_FW_MAX (field-weakening current)
· `0x2009` motor_baud · `0x200a` CAN_ID · `0x200b` CAN_MASTER · `0x200c`
CAN_TIMEOUT · `0x2012/0x2013` cur_kp/cur_ki · `0x2014/0x2015` spd_kp/spd_ki ·
`0x2016` loc_kp · `0x2018` limit_spd · `0x2019` limit_cur · `0x2023` damper ·
`0x2024` add_offset · `0x2005` MechOffset (encoder mechanical offset).

---

## 5. Type 2 — feedback frame (readable state)

Reply to Type 1/3/4/6 and to state polls; also streamed if active-reporting
(Type 24) is enabled. State is split across the CAN-ID data field and the 8-byte
payload:

**CAN-ID `data` field (bits 23:8):**

| Bits | Field |
| --- | --- |
| 15:8 | Motor CAN_ID |
| 21:16 | Fault bits (see below) |
| 23:22 | Mode status: 0 = Reset · 1 = Cali (calibration) · 2 = Motor (Run) |

**8-byte payload (big-endian 16-bit each):**

| Bytes | Field | Scaling |
| --- | --- | --- |
| 0–1 | Position | ±4π rad (±12.57) |
| 2–3 | Velocity | per-model (e.g. ±44 rad/s RS02) |
| 4–5 | Torque | per-model (e.g. ±17 N·m RS02) |
| 6–7 | Temperature | value / 10 = °C |

**Compact fault bits (feedback field, bits 21:16):**

| Bit | Meaning |
| ---: | --- |
| 16 | Undervoltage |
| 17 | Three-phase overcurrent |
| 18 | Overtemperature |
| 19 | Magnetic-encoder fault |
| 20 | Stall / overload |
| 21 | Uncalibrated |

---

## 6. Full fault code (register `0x3022` / Type 21 frame)

The Type-21 frame carries a 32-bit **fault** word (Byte0–3) and a **warning**
word (Byte4–7). Any non-zero value is a fault. This is the complete set (the
compact §5 field is a subset):

| Bit | Meaning |
| ---: | --- |
| 0 | Motor overtemperature (thermistor > 135 °C) |
| 1 | Driver-chip fault |
| 2 | Undervoltage (bus below ~12 V protection) |
| 3 | Overvoltage (bus above protection, ~60 V margin) |
| 4 | B-phase current-sampling overcurrent |
| 5 | C-phase current-sampling overcurrent |
| 7 | Encoder uncalibrated |
| 8 | Hardware-identification fault |
| 9 | Position-initialization fault |
| 14 | Stall / overload algorithm protection |
| 16 | A-phase current-sampling overcurrent |

Warning word (Byte4–7): bit 0 = overtemperature warning (default 135 °C).

**Clearing faults:** send a Type-4 (stop) frame with `Byte0=1`. Under CANopen,
clear via Controlword `0x6040`.

---

## 7. Zeroing, backdrive & version notes

- **Mechanical zero** (Type 6) sets the current shaft angle as 0. Prefer zeroing
  in **CSP or MIT/motion-control** mode; older firmware's zero-calibration could
  induce a large jump, fixed in newer firmware when zeroing in CSP.
- **Zero convention** (`zero_sta`, `0x7029`): 0 → reports 0…2π, 1 → reports −π…π.
- **Position offset**: `add_offset` (`0x702B` / persistent `0x2024`) shifts the
  reported zero without re-homing.
- **Anti-backdrive damping** (`damper`, `0x702A` / `0x2023`): configurable
  damping when unpowered so a de-energized joint doesn't freewheel.
- **CAN watchdog** (`canTimeout`, `0x7028`): if no CAN command arrives within the
  threshold (20000 ≈ 1 s) the motor drops to reset mode. Off by default.
- **CANopen / DS402**: selectable via Type 25. Uses Controlword `0x6040`
  (15 = enable, 1 = stop, 11 = e-stop, plus fault-clear) and a CAN watchdog via
  object `0x6099`. EDS files are on the vendor site.
- **CAN termination**: later hardware batches of RS02/RS03 omit the onboard
  240 Ω resistor. Read `can_status` (`0x3048` on RS02, `0x3041` on RS03):
  0 = has the 240 Ω resistor, 1 = no resistor — add external termination.
- **Firmware maturity** (per vendor README): RS00 through `RS00_0.0.3.6`,
  RS02 through `RS02_0.2.3.9`, RS03 through `RS03_APP_V0311_V1001_20250507` all
  support CANopen/MIT switching, corrected Kp/Kd coefficients, readable/writable
  parameters, CSP/MIT zeroing, position offset, zero dead-zone, and CANopen node
  id matching the private-protocol CAN_ID. Some later commands (save/report/
  read-params) require firmware ≥ `0.2.3.32` on RS02.

---

## 8. Quick model-selection summary

| Need | Pick |
| --- | --- |
| Smallest / lightest, low torque | RS05 (1.6/5.5 N·m, 191 g) |
| General small joint | RS00 (5/14 N·m) |
| General medium joint, higher speed | RS02 / RS01 (6–7/17 N·m) |
| Sealed (IP67) medium joint | RS02-IP67 |
| High torque | RS06 (11/36 N·m) or RS03 (20/60 N·m) |
| Maximum torque | RS04 (40/120 N·m) |

For dimensions, mounting, performance curves, and full overload curves, see the
original per-model PDF manuals in `robstride/Product_Information/`.
