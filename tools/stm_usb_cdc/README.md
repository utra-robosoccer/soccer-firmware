# STM USB CDC Command Tools

Host-side scripts for sending packed `motor_cmd_t` arrays from a Linux laptop
to the STM32F446 SPI master board over USB CDC.

- `send.py`: send sine position commands.
- `sendnread.py`: send sine position commands and read returned feedback.
- `plot.py`: send commands and plot command vs feedback positions live.
