# Host

All host-side Python — runs on the Linux PC now, the Jetson later. It talks to the
master MCU over USB-CDC (the "STM32 Virtual ComPort", usually `/dev/ttyACM2`).

## Layout

```
host/
├── master_link/        # importable library (pip install -e host/)
│   ├── protocol.py            wire protocol: framing, CRC, message codecs
│   ├── motor_config_gen.py    GENERATED motor table (scripts/gen_motor_config.py)
│   └── session_logger.py      per-session CSV logging
├── apps/              # runnable programs (no library code here)
│   ├── dashboard.py           interactive TUI: control + live telemetry + logging
│   └── test_client.py         keyboard test client
├── analysis/          # offline log tools
│   └── plot_motor_state.py            plot a session log
└── tests/             # host unit tests
```

Device utilities that are not part of this stack live in `tools/` (e.g.
`tools/robostride_usb_can/`).

## System prerequisites

Install with the system package manager (outside the venv):

```sh
sudo apt install python3-venv python3-tk
```

- `python3-venv` — to create the virtual environment below.
- `python3-tk` — Tk backend for matplotlib, so `analysis/plot_motor_state.py` can open
  interactive windows. Without it, matplotlib falls back to a non-interactive
  backend and `plot_motor_state.py` auto-saves PNGs instead of showing them. (Alternatively,
  `pip install PyQt5` in the venv provides the Qt backend.)

## Install

From the repo root, create a virtual environment and install `master_link` editable:

```sh
python3 -m venv .venv && source .venv/bin/activate
pip install --upgrade pip setuptools   # setuptools >= 64 needed for PEP 660 editable installs
pip install -e host/                    # makes `master_link` importable
pip install -r host/requirements.txt
```

The `.venv/` is gitignored. Re-activate it in new shells with
`source .venv/bin/activate`. (Editable install means `git pull` / regenerating
`motor_config_gen.py` is picked up without reinstalling.)

`motor_config_gen.py` is generated — regenerate after editing a config:

```sh
python3 scripts/gen_motor_config.py --system
```

## Run

```sh
python3 host/apps/dashboard.py /dev/ttyACM2     # control + live view (logs to logs/<date>/)
python3 host/apps/test_client.py /dev/ttyACM2   # keyboard test client
```

(For plotting a run_policy session, see below — `plot_motor_state` takes a session
folder or `.bin`, not the dashboard's live CSV.)

### Headless policy runner (`run_policy`)

Runs a policy in a deadline-scheduled loop and writes a **binary** session log
(`logs/<date>/<time>_<policy>.bin`) — raw TX/RX frames, discards, events, and loop
timing. The master port is auto-detected by USB VID:PID (override with `--port`).

```sh
python3 host/apps/run_policy.py --policy listen --rate 50     # auto-detect port
python3 host/apps/run_policy.py --policy listen --port /dev/ttyACM2
```

`listen` sends nothing — use it to passively capture (e.g. moving a joint by hand).
Stop with **Ctrl-C**: it disables any armed motors, writes a stop event, flushes the
log, and prints a summary (frames RX/TX, discards, loop-timing mean/p99/max).

> Note: nothing safes the motors if the runner is `kill -9`'d (the firmware has no
> host-death timeout — see docs). Always stop with Ctrl-C.

Plot a session directly — give the plotter a **session folder** or the **`.bin`**
(it converts first if needed); you never pick a CSV:

```sh
python3 host/analysis/plot_motor_state.py logs/<date>/<time>_listen        # session folder
python3 host/analysis/plot_motor_state.py logs/<date>/<time>_listen.bin    # or the .bin
```

`plot_motor_state` opens interactive windows when a GUI backend is available (see
`python3-tk` under System prerequisites); otherwise it auto-saves `motor_<N>.png`
into the session folder.

To convert without plotting (a folder of CSVs beside the `.bin`):

```sh
python3 host/analysis/convert_log.py logs/<date>/<time>_listen.bin
# → logs/<date>/<time>_listen/{motor_state,motor_cmd,status,control_resp,events,loop_timing}.csv
```

## Tests

```sh
python3 -m unittest discover -s host/tests
```

Session logs are written to `logs/YYYY-MM-DD/HH-MM-SS.csv` at the repo root
(gitignored).
