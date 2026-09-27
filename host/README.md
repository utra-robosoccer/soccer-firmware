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
│   └── plot_log.py            plot a session log
└── tests/             # host unit tests
```

Device utilities that are not part of this stack live in `tools/` (e.g.
`tools/robostride_usb_can/`).

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
python3 host/analysis/plot_log.py logs/<date>/<time>.csv
```

## Tests

```sh
python3 -m unittest discover -s host/tests
```

Session logs are written to `logs/YYYY-MM-DD/HH-MM-SS.csv` at the repo root
(gitignored).
