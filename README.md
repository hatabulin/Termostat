# Termostat
18b20 termostat project

This project using hw usart2 interface for working with onewire 18b20, 18s20 temperature sensors.

With configurable hysteresis values (via USB VirtualCOmport/UART RS232) with terminal app.
Implement commands:
- hystX_min, hystX_max  - change hysteresis for profile number X
- hystX_power           - change power (number of pulses on GPIO for emulate button pressed on induction cooker)
- get_cfg               - output config data
- save_cfg              - save config
- profile:X             - set current used profile.
