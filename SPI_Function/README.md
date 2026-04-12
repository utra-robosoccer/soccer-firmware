# SPI_Function

STM32F446ZETx firmware project for multi-slave SPI motor command exchange.

This project is generated with STM32CubeIDE/CubeMX and extended in user code to:
- Pack motor position/speed commands into 4-byte frames
- Send frames to up to 4 SPI slaves (selected by GPIO chip-select)
- Receive SPI feedback and decode it back to float values
- Print debug logs over USART3

## Target MCU and Tools

- MCU: STM32F446ZETx
- IDE: STM32CubeIDE
- HAL package: STM32Cube FW_F4 V1.28.3
- Toolchain: GCC (from STM32CubeIDE)

## Project Structure

- `Core/Src/main.c`: user SPI protocol logic and main loop
- `Core/Inc/main.h`: board-level pin and peripheral declarations
- `SPI_Function.ioc`: CubeMX hardware/peripheral configuration
- `Debug/`: generated build artifacts (can be regenerated)

## Peripheral Configuration (Current)

- SPI1 (master, full duplex)
  - SCK: PA5
  - MISO: PA6
  - MOSI: PA7
- DMA for SPI1
  - RX: DMA2_Stream0
  - TX: DMA2_Stream3
- Slave-select GPIO outputs
  - DEV1: PE2
  - DEV2: PE4
  - DEV3: PE5
  - DEV4: PE6
- Debug UART
  - USART3 TX: PD8
  - USART3 RX: PD9

## Motor Data Format

Each motor command is 4 bytes:
- Byte0-1: position (uint16, little-endian)
- Byte2-3: speed (uint16, little-endian)

Float values are clamped/scaled in `main.c` with:
- Position range: `P_MIN=-12.57`, `P_MAX=12.57`
- Speed range: `V_MIN=-20.0`, `V_MAX=20.0`

## Multi-Slave Mapping

Motor-to-slave assignment is controlled by:

- `motors_per_slave[4]` in `Core/Src/main.c`

Example:
- `{1,1,1,1}`: one motor per slave
- `{2,1,0,1}`: DEV1 has two motors, DEV3 has zero

Initial demo commands are set in:
- `g_motor_cmd[]` in `Core/Src/main.c`

## Build and Flash

1. Open this folder in STM32CubeIDE.
2. Import as existing STM32CubeIDE project if needed.
3. Select build configuration (`Debug` or `Release`).
4. Build project.
5. Connect ST-LINK and flash to target.

Command-line build (from project root) can use generated makefiles:

```bash
make -C Debug all
```

## Runtime Debug

USART3 prints:
- Encoded TX motor values
- Raw RX bytes
- Decoded RX float values

Use a serial terminal on the board virtual COM port to observe logs.

## Quick Reminders (How To Use)

Main control entry is in `Core/Src/main.c`.

1. Mode switch
- `g_master_mode = MASTER_MODE_DEBUG`: runs built-in test cases.
- `g_master_mode = MASTER_MODE_REAL`: runs real loop and sends command cache.

2. Where to set motor targets
- Write desired targets into `g_motor_cmd[]` (position + speed).
- Real mode sends the cached safe copy, not raw values directly.

3. Safety structure (single place to tune)
- `g_master_safety.enabled`: `1` = safety checks on, `0` = checks off.
- `g_master_safety.command_cache_timeout_ms`: command cache timeout threshold.
- `g_master_safety.max_consecutive_spi_errors`: SPI error threshold before ESTOP.
- `g_master_safety.control_loop_period_ms`: loop delay in real mode.
- Runtime stats in same struct:
  - `tx_total_count`, `tx_success_count`, `tx_fail_count`
  - `last_success_tick`, `last_error_tick`, `last_error_reason`
  - `estop_latch_count`

4. Command source management (avoid overwrite)
- `CMD_SOURCE_TEST`: debug test generator writes commands into `g_test_cmd[]`.
- `CMD_SOURCE_HOST`: host-side logic should write commands into `g_host_cmd[]`.
- Active source is selected through `g_master_safety.active_command_source`.
- Real mode sends the active source cache only.

5. Safety behavior reminder
- If ESTOP is latched, master sends safe-stop command cache (all zero targets).
- Manual clear entry is implemented: press USER button to clear ESTOP latch.

6. Key helper functions
- `publish_command_cache(...)`: bounds and publishes outgoing cache.
- `run_debug_test_cycle(...)`: debug test patterns.
- `run_real_master_cycle(...)`: real control path with safety checks.
- `clear_emergency_stop_manual(...)`: explicit ESTOP clear function.

6. Function map (quick lookup)
- `float_to_uint(...)` / `uint_to_float(...)`: value encode/decode helpers.
- `pack_one_motor(...)`: pack one motor command into 4 bytes.
- `build_slave_buf(...)`: build one slave packet from global commands.
- `spi_send_to_one_slave(...)`: one slave transaction + debug print.
- `spi_update_all_slaves_param(...)`: send one cycle to all slaves.
- `build_safe_stop_commands(...)`: build all-zero safe command table.
- `latch_emergency_stop(...)`: latch safety stop state.
- `CS_SELECT(...)` / `CS_ALL_HIGH(...)`: chip-select control helpers.

## Notes

- Keep user custom code inside `USER CODE BEGIN/END` blocks to survive CubeMX re-generation.
- If `SPI_Function.ioc` is changed and code is re-generated, verify custom SPI logic in `Core/Src/main.c` is still intact.

## Git

Remote repository currently configured as:

`git@github.com:utra-robosoccer/soccer-firmware.git`

Current working branch created for development:

`feature/spi-function-20260404`
