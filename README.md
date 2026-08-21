# Khageswara Flight Controller

A Teensy 4.1-based flight controller that provides an autopilot platform for fixed-wing vehicles in manual, stabilize, and auto modes.

## Spesifikasi
### Processor
- ARM Cortex-M7 at 600 MHz
- Float point math unit, 64 & 32 bits
- 7936K Flash, 1024K RAM (512K tightly coupled), 4K EEPROM (emulated)
- QSPI memory expansion, locations for 2 extra RAM or Flash chips
- USB device 480 Mbit/sec & USB host 480 Mbit/sec
### Hardware
- Teensy 4.1
- Inertial Measurement Sensor **BNO055**
- Barometric Sensor **BMP580 / BMP581** (previously MS5611)
- Global Positioning System **Radiolink SE100**
- Airspeed Sensor **MS4525DO**
- Voltage Sensor


<details>
<summary>Fly Mode</summary>

- **Manual**: Full pilot control of the aircraft
- **Stabilize**: The autopilot helps maintain the stability of the aircraft, making flight easier.
- **Auto**: The autopilot takes over control, allowing autonomous flight with waypoints and missions.
</details>