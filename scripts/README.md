# Build & Flash Scripts

## Dependencies

### build.sh — STM32CubeIDE

`build.sh` requires STM32CubeIDE installed. The script searches these locations
in order:

1. `$STM32CUBEIDE_PATH` env var (explicit override)
2. `stm32cubeide` on `$PATH`
3. `/opt/st/stm32cubeide*/stm32cubeide`
4. `~/st/stm32cubeide*/stm32cubeide`

Download from ST's site: <https://www.st.com/en/development-tools/stm32cubeide.html>

If the auto-detection misses your install:

```sh
export STM32CUBEIDE_PATH="/path/to/stm32cubeide"
./scripts/build.sh master
```

---

### flash.sh — flashing tool

You need one of the following tools installed:

### Option A — STM32CubeProgrammer (preferred)

Provides `STM32_Programmer_CLI`. Download from ST's site and install it, then
ensure the install's `bin/` directory is on your `PATH`:

```sh
export PATH="/opt/st/stm32cubeprog/bin:$PATH"
```

### Option B — stlink-tools

```sh
sudo apt install stlink-tools    # Debian/Ubuntu
sudo dnf install stlink          # Fedora
```

Provides `st-flash`. The script falls back to this automatically when
`STM32_Programmer_CLI` is not found.

---

## Linux udev rules (ST-Link permissions)

Without udev rules you'll need `sudo` to access the ST-Link. Add the rules once:

```sh
# stlink-tools ships rules at /lib/udev/rules.d/49-stlinkv* after install.
# If missing, or if using CubeProgrammer only, add them manually:

sudo tee /etc/udev/rules.d/49-stlink.rules > /dev/null << 'EOF'
# ST-Link V2
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="3748", MODE="0666", GROUP="plugdev"
# ST-Link V2-1
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374b", MODE="0666", GROUP="plugdev"
# ST-Link V3
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374e", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374f", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="3753", MODE="0666", GROUP="plugdev"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger
# Log out and back in, or run: sudo usermod -aG plugdev $USER
```

---

## Usage

### build.sh

```sh
# List all projects
./scripts/build.sh --list
./scripts/build.sh           # (no args also lists)

# Build a project (Debug by default)
./scripts/build.sh master
./scripts/build.sh rs-motor-test
./scripts/build.sh slave/slave_general

# Build a specific configuration
./scripts/build.sh master Release

# Clean then build
./scripts/build.sh --clean master
./scripts/build.sh --clean master Release
```

### flash.sh

```sh
# List all flashable projects in the repo
./scripts/flash.sh --list
./scripts/flash.sh           # (no args also lists)

# Flash a project by its directory path (relative to repo root or CWD)
./scripts/flash.sh master
./scripts/flash.sh rs-motor-test
./scripts/flash.sh slave/slave_general
./scripts/flash.sh spi_master_test/SPI_MULTI_SLAVE
./scripts/flash.sh spi_master_test/SPI_Slave_Send
./scripts/flash.sh SPI_MULTI_SLAVE
./scripts/flash.sh SPI_Slave_Send
```

The script reads the `<name>` field from the project's `.project` file to find
the correct `.elf` — so `rs-motor-test/` flashes `motor_test.elf`, not
`rs-motor-test.elf`.

If the `.elf` doesn't exist you'll see:

```
error: no .elf found under 'rs-motor-test/Debug/'
Build the project in STM32CubeIDE first (Project → Build All).
```

### Build and flash in one step

```sh
./scripts/build.sh master && ./scripts/flash.sh master
./scripts/build.sh slave/slave_general && ./scripts/flash.sh slave/slave_general
./scripts/build.sh --clean rs-motor-test && ./scripts/flash.sh rs-motor-test
```

---

## Project → binary name map

| Directory                       | Binary name       |
|---------------------------------|-------------------|
| `master`                        | `master`          |
| `rs-motor-test`                 | `motor_test`      |
| `slave/slave_general`           | `slave_general`   |
| `spi_master_test/SPI_MULTI_SLAVE` | `SPI_MULTI_SLAVE` |
| `spi_master_test/SPI_Slave_Send`  | `SPI_Slave_Send`  |
| `SPI_MULTI_SLAVE`               | `SPI_MULTI_SLAVE` |
| `SPI_Slave_Send`                | `SPI_Slave_Send`  |
