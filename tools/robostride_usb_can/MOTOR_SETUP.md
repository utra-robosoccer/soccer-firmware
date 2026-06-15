# Motor Setup: Change ID & Set Zero Position

Quick reference for the two one-time-per-motor setup tasks, using the RS02
USB-CAN debug CLI. Run these from the repo root with the motor connected through
the CH341 USB-CAN debugger.

Default connection (override with `--port` / `--baud` or the `RS02_PORT` /
`RS02_BAUD` env vars):

```sh
--port /dev/ttyCH341USB0
--baud 921600
```

> Keep only the target motor on the bus for these operations.

## 1. Change the motor CAN ID

```sh
python3 tools/robostride_usb_can/cli.py set-id <current_id> <new_id>
```

Example, change ID 1 to ID 2:

```sh
python3 tools/robostride_usb_can/cli.py set-id 1 2
```

What it does: reads the current ID, disables motor output, sends the Robstride
type-7 ID-change frame, then reads feedback from the new ID to verify. It fails
loudly if the motor does not respond on the new ID.

If you don't know the current ID, discover it first:

```sh
python3 tools/robostride_usb_can/cli.py scan-ids
```

## 2. Set the zero position

This stores the motor's *current* mechanical position as zero, so first move the
shaft to where you want zero to be.

```sh
python3 tools/robostride_usb_can/cli.py set-zero <id>
```

Example:

```sh
python3 tools/robostride_usb_can/cli.py set-zero 1
```

What it does: disables the motor (so it isn't holding torque), then sends the
raw comm-type-6 SetZero command. Pass `--no-stop` to skip the disable step.

To verify the new zero, read feedback and confirm `pos` is ~0:

```sh
python3 tools/robostride_usb_can/cli.py read 1
```

> `set-zero` captures wherever the shaft currently sits. To park the shaft at the
> existing zero before/after, use `cli.py zero <id>` (a slow ramp to position 0),
> which is a different operation from defining a new mechanical zero.

See `README.md` in this directory for the full command list.
