# RobStride USB-CAN Debug CLI

Talks directly to RobStride RS02 motors from the host over a CH341 USB-CAN
adapter. It does not use Python-CAN and does not require flashing the STM32
firmware — use it for one-time motor setup (change CAN ID, set zero position)
with the motor connected through the CH341 debugger.

The adapter is opened as a serial port and uses the RobStride raw frame:

```text
b"AT" + u32_be((can_id << 3) | 0x04) + u8(data_len) + data + b"\r\n"
```

Default connection (override with `--port` / `--baud`, or the `RS02_PORT` /
`RS02_BAUD` env vars):

```sh
--port /dev/ttyCH341USB0
--baud 921600
```

> Keep only the target motor on the bus for the setup operations below.

## 1. Change the motor CAN ID

```sh
python3 tools/robstride_usb_can/cli.py set-id <current_id> <new_id>
```

Example — change ID 1 to ID 2:

```sh
python3 tools/robstride_usb_can/cli.py set-id 1 2
```

Reads the current ID, disables motor output, sends the RobStride type-7
ID-change frame, then reads feedback from the new ID to verify. It fails loudly
if the motor does not respond on the new ID.

If you don't know the current ID, discover it first:

```sh
python3 tools/robstride_usb_can/cli.py scan-ids
```

## 2. Set the zero position

Stores the motor's *current* mechanical position as zero, so first move the
shaft to where you want zero to be.

```sh
python3 tools/robstride_usb_can/cli.py set-zero <id>
```

Example:

```sh
python3 tools/robstride_usb_can/cli.py set-zero 1
```

Disables the motor (so it isn't holding torque), then sends the raw comm-type-6
SetZero command. Pass `--no-stop` to skip the disable step.

Verify the new zero — read feedback and confirm `pos` is ~0:

```sh
python3 tools/robstride_usb_can/cli.py read 1
```

## Other commands

`cli.py` also exposes lower-level helpers used mainly for bring-up and
debugging. Run `cli.py --help` for the full list; the common ones:

```sh
python3 tools/robstride_usb_can/cli.py read <id>        # one feedback frame (zero-gain MIT ping)
python3 tools/robstride_usb_can/cli.py get-id <id>      # read one motor's type-0 device ID / MCU UID
python3 tools/robstride_usb_can/cli.py off <id>         # disable output so the shaft turns freely
```
