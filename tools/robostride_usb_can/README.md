# Robstride USB-CAN Debug CLI

This directory talks directly to RS02 motors from the host. It does not use
Python-CAN and does not require flashing the STM32 firmware.

The CH341 adapter is opened as a serial port and uses the Robstride raw frame:

```text
b"AT" + u32_be((can_id << 3) | 0x04) + u8(data_len) + data + b"\r\n"
```

Default connection settings:

```sh
--port /dev/ttyCH341USB0
--baud 921600
```

## Priority commands

Read one feedback frame with a zero-gain MIT ping:

```sh
python3 tools/robostride_usb_can/cli.py read 1
```

Read by sending Enable:

```sh
python3 tools/robostride_usb_can/cli.py read 1 --enable
```

Discover an unknown motor ID with Robstride type 0:

```sh
python3 tools/robostride_usb_can/cli.py scan-ids
```

For one known target ID:

```sh
python3 tools/robostride_usb_can/cli.py get-id 1
```

Set MIT mode, enable, then read:

```sh
python3 tools/robostride_usb_can/cli.py init 1
```

Send one MIT frame:

```sh
python3 tools/robostride_usb_can/cli.py mit 1 0.0 0.0 0.0 0.0 0.0
```

Continuously command MIT setpoints:

```sh
python3 tools/robostride_usb_can/cli.py command 1 0.0 0.0 15.0 1.0 0.0 --duration 2 --hz 100
```

Track a sine wave using position plus analytic velocity feed-forward:

```sh
python3 tools/robostride_usb_can/cli.py sine 1 --amp 0.20 --freq 0.25 --duration 8 --hz 100
```

Track the same sine wave on a chain of motors:

```sh
python3 tools/robostride_usb_can/cli.py sine-chain 1 2 --amp 0.20 --freq 0.25 --duration 8 --hz 100
```

`sine` centers on the current position by default, fades the amplitude in and
out with `--ramp-time`, and disables the motor afterward unless `--hold` is set.
The MIT velocity field is set to the derivative of the commanded position.
`sine-chain` does the same thing for each listed motor ID over one serial
connection.

Slow host-side ramp to a position:

```sh
python3 tools/robostride_usb_can/cli.py goto 1 1.57 0.30 --accel 0.30 --hz 200
```

Slow zero-position ramp:

```sh
python3 tools/robostride_usb_can/cli.py zero 1 --rate 0.30 --accel 0.30 --hz 200
```

`zero` disables the motor after the ramp by default. Add `--hold` if you want it
to keep holding zero.

Velocity-style zero using MIT damping only:

```sh
python3 tools/robostride_usb_can/cli.py zero-vel 1 --rate 0.50 --kd 1.5
```

This follows the RS02 manual's MIT velocity example: `kp=0`, `t_ff=0`, and
`kd` supplies damping/velocity tracking. It disables the motor after the move by
default. `--min-rate` keeps the final creep from dropping below the motor's
stick-slip threshold.

Disable motor output so the shaft can be manually rotated:

```sh
python3 tools/robostride_usb_can/cli.py off 1
```

Set the current mechanical position as zero:

```sh
python3 tools/robostride_usb_can/cli.py set-zero 1
```

Change a motor CAN ID, verifying before and after:

```sh
python3 tools/robostride_usb_can/cli.py set-id 1 2
```

Only keep the target motor on the bus when changing IDs. The command first reads
the current ID, disables motor output, sends the Robstride type 7 ID-change
frame, then reads feedback from the new ID.

`zero` first reads the current motor state. If no feedback is received, it
refuses to ramp blindly. By default it only moves to position `0.0`; add
`--set-mech-zero` only when you intentionally want to send the motor's
mechanical-zero command after the slow ramp.
