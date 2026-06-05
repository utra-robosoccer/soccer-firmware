# RobStride Motor Reference

This note distills firmware-relevant RobStride communication and motor data for RS00, RS02, RS03, RS05, and RS06.

Sources:

- `RobStride Product Specification Document 20250626.pdf` in `/home/anthonykpinson/robosoccer/robstride/Product_Information/`
- `/home/anthonykpinson/robosoccer/robstride/Product_Information/README.md`
- `firmware/slave/slave_general/Core/Inc/robostride.h`
- `firmware/slave/slave_general/Core/Src/robostride.c`
- `tools/robostride_usb_can/rs02_can.py`

## Shared Communication Characteristics

- STM32 CAN1 is configured for 1 Mbps in the slave and motor-test projects: prescaler 2, BS1 16TQ, BS2 4TQ, normal mode, auto-retransmission disabled.
- The firmware uses the RobStride private 29-bit extended CAN protocol:

```text
mode[28:24] | data[23:8] | node_id[7:0]
```

- Current master/host ID is `0xFD`.
- Current implemented communication types:

| Type | Direction | Meaning | Firmware entry point |
| ---: | --- | --- | --- |
| 0 | Tx/Rx | Get device ID / MCU ID | `can_get_motor_id`, `can_unpack_get_id` |
| 1 | Tx | MIT operation control | `can_mit_control_set` |
| 2 | Rx | Motor feedback | `can_unpack_motor_feedback` |
| 3 | Tx | Enable motor | `can_enable_motor` |
| 4 | Tx | Disable motor | `can_disable_motor` |
| 6 | Tx | Set mechanical zero | `can_set_mech_zero` |
| 7 | Tx | Set motor CAN ID | `can_set_motor_can_id` |
| 17 | Tx/Rx | Read single parameter | `can_read_single_param`, `can_unpack_single_param` |
| 18 | Tx | Write parameter / change mode | `can_change_motor_mode` |

- Current run modes use register `0x7005`: `0` MIT, `1` position PP, `2` velocity, `3` current, `4` CSP.
- Current MIT scaling in firmware is common across all motors:

| Signal | Range | Bits | Payload location |
| --- | ---: | ---: | --- |
| Position | -12.57 to +12.57 rad | 16 | Type 1 bytes 0-1, Type 2 bytes 0-1 |
| Velocity | -20 to +20 rad/s | 16 | Type 1 bytes 2-3, Type 2 bytes 2-3 |
| Kp | 0 to 5000 | 16 | Type 1 bytes 4-5 |
| Kd | 0 to 100 | 16 | Type 1 bytes 6-7 |
| Torque | -60 to +60 Nm | 16 | Type 1 CAN ID `data`, Type 2 bytes 4-5 |
| Temperature | raw / 10 C | 16 | Type 2 bytes 6-7 |

- Type 1 MIT payload and Type 2 feedback payload are big-endian 16-bit fields.
- Type 2 feedback CAN ID `data` field layout:

```text
bits 7:0   motor_id
bits 13:8  faults
bits 15:14 motor status
```

- Feedback fault bits are `undervoltage`, `driver_fault`, `overheat`, `encoder_fault`, `stall_overload`, and `uncalibrated`.
- The host-side CH341 test path wraps extended CAN frames as:

```text
b"AT" + u32_be((can_id << 3) | 0x04) + u8(data_len) + data + b"\r\n"
```

## Product Motor Characteristics

Physical peak torque is not the same thing as the current firmware's common `T_MIN/T_MAX` protocol scaling. Clamp controller commands to the model's physical limits even if the wire-format range remains broader.

| Model | Rated voltage | Voltage range | Rated / peak torque | No-load speed | Rated-load speed | Rated / peak phase current | Torque constant | Ratio | Poles | Weight |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| RS00 | 48 V | 24-60 V | 5 / 14 Nm | 315 rpm, 33.0 rad/s | 100 rpm, 10.5 rad/s | 4.7 / 15.5 Apk | 1.48 Nm/Arms | 10:1 | 28 | 310 g |
| RS02 | 48 V | 24-60 V | 6 / 17 Nm | 410 rpm, 42.9 rad/s | 100 rpm, 10.5 rad/s | 7 / 23 Apk | 1.22 Nm/Arms | 7.75:1 | 28 | 380 g |
| RS03 | 48 V | 15-60 V | 20 / 60 Nm | 195 rpm, 20.4 rad/s | 100 rpm, 10.5 rad/s | 13 / 43 Apk | 2.36 Nm/Arms | 9:1 | 42 | 900 g |
| RS05 | 48 V | 15-60 V | 1.6 / 5.5 Nm | 480 rpm, 50.3 rad/s | 100 rpm, 10.5 rad/s | 2.4 / 11 Apk | 0.94 Nm/Arms | 7.75:1 | 20 | 191 g |
| RS06 | 48 V | 15-60 V | 11 / 36 Nm | 480 rpm, 50.3 rad/s | 100 rpm, 10.5 rad/s | 14.3 / 57 Apk | 1.1 Nm/Arms | 9:1 | 28 | 621 g |

| Model | Encoder count | Rated output power | No-load current | Back-EMF | Line resistance | Inductance |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| RS00 | 2 pcs | 50 W +/-10% | 0.5 Arms | 0.095 Vrms/rpm +/-10% | 1.5 ohm +/-10% | 750 uH +/-20% |
| RS02 | 2 pcs | 60 W +/-10% | 0.5 Arms | 0.096 Vrms/rpm +/-10% | 0.55 ohm | 486 uH +/-20% |
| RS03 | 2 pcs | 210 W +/-10% | 0.6 Arms +/-10% | 17 Vrms/krpm +/-10% | 0.39 ohm +/-10% | 0.275 mH +/-10% |
| RS05 | 2 pcs | 17 W +/-10% | 0.14 Arms +/-10% | 7.4 Vrms/krpm +/-10% | 2.72 ohm +/-10% | 0.813 mH +/-10% |
| RS06 | 2 pcs | 115 W +/-10% | 0.98 Arms +/-10% | 7.6 Vrms/krpm +/-10% | 0.23 ohm +/-10% | 0.165 mH +/-10% |

Shared environmental/electrical ratings from the product spec:

- Running direction: CW/CCW.
- Drive method: FOC.
- Operating temperature: -20 to 50 C.
- Humidity: 5% to 85%.
- Storage temperature: -30 to 70 C.
- Insulation class: Class B.
- Stator winding insulation resistance: DC 500 VAC, 100 Mohm.
- Stator/casing high-voltage resistance: 600 VAC, 1 s, 2 mA.

## Overload Timing

| Model | Continuous rating | Temporary overload points |
| --- | --- | --- |
| RS00 | 5 Nm rated | 7 Nm for 120 s; 10 Nm for 18 s; 12 Nm for 10 s; 14 Nm for 5 s |
| RS02 | 6 Nm rated | 6.5 Nm for 3000 s; 11 Nm for 100 s; 15 Nm for 18 s; 17 Nm for 10 s |
| RS03 | 20 Nm rated | 30 Nm for 189 s; 40 Nm for 126 s; 50 Nm for 12 s; 60 Nm for 7 s |
| RS05 | 1.6 Nm rated | 2 Nm for 300 s; 3 Nm for 41 s; 4 Nm for 12 s; 5 Nm for 5.5 s; 5.5 Nm for 4 s |
| RS06 | 11 Nm rated | 17 Nm for 200 s; 20 Nm for 36 s; 25 Nm for 18 s; 30 Nm for 8 s; 36 Nm for 4 s |

## Firmware Notes From Product README

- RS00 firmware through `RS00_0.0.3.6` supports CANopen and MIT switching, restored factory reset, configurable unpowered backdrive damping via `0x2023 damper`, zeroing in CSP and motion-control modes, position offset via `0x2024 add_offset`, a zero dead zone, and CANopen node ID matching the private-protocol CAN ID.
- RS02 firmware through `RS02_0.2.3.9` has the RS00/RS01 fixes relevant to MIT operation: corrected `kp`/`kd` coefficients, CANopen standard-frame switching, readable/writable parameters, CANopen ID matching CAN ID, CSP/MIT zeroing behavior, position offset, and zero dead zone. Later hardware may omit the CAN resistor; read `0x3048 can_status`, where `0` means the initial 240 ohm CAN resistor version and `1` means no CAN resistor.
- RS03 firmware through `RS03_APP_V0311_V1001_20250507` includes CANopen/MIT switching, improved position accuracy after repower that requires mechanical-zero recalibration after upgrade, backdrive damping configuration, CANopen ID matching CAN ID, zero dead zone, CSP/MIT zeroing behavior, position offset, and PLC error variables. Later hardware may omit the CAN resistor; read `0x3041 can_status`, where `0` means the initial 240 ohm CAN resistor version and `1` means no CAN resistor.
- RS05 and RS06 are present in the product spec PDF, but no RS05/RS06 firmware changelog entries were present in the local product README.

## Firmware Implications

- `T_MAX = 60 Nm` matches RS03 physical peak torque but exceeds RS00, RS02, RS05, and RS06 peak torque. Add model-aware command limiting before using the same control code across these motors.
- `V_MAX = 20 rad/s` is below the product no-load speed for RS00, RS02, RS05, and RS06. That may be an intentional MIT control envelope, but it is not the full mechanical speed capability from the product spec.
- If a bus mixes newer RS02/RS03 hardware, verify CAN termination because some later batches removed the onboard CAN resistor.
- Mechanical zero behavior is firmware-version sensitive. Prefer CSP or MIT/motion-control zeroing; PP-mode zeroing may be blocked on newer firmware.
