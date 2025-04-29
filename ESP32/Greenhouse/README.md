| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-C61 | ESP32-H2 | ESP32-P4 | ESP32-S2 | ESP32-S3 | Linux |
| ----------------- | ----- | -------- | -------- | -------- | -------- | --------- | -------- | -------- | -------- | -------- | ----- |

# Greenhouse Control System

An ESP32-based control system for automation of greenhouse environmental conditions.

## Overview

This project provides a comprehensive control solution for greenhouse environments, supporting:

* Temperature and humidity monitoring via SHT20 sensors
* Wind speed monitoring
* Automatic ventilation control based on temperature thresholds
* Cooling system control with multiple ventilators
* MQTT connectivity for remote monitoring and control
* Persistent settings via non-volatile storage
* Modbus communication with relay modules and sensors

## Hardware Requirements

* ESP32 development board (ESP-WROOM-32)
* RS485 modules for Modbus communication
* SHT20 temperature/humidity sensors
* Relay modules for controlling actuators
* Ventilation motors and cooling system components

## Pin Configuration

The system uses the following pins:
* UART2 (RX: GPIO16, TX: GPIO17) - For sensors communication via RS485
* UART1 (RX: GPIO26, TX: GPIO25) - For relay control via RS485

## Getting Started

### Setup

1. Install ESP-IDF (Espressif IoT Development Framework)
2. Clone this repository
3. Configure WiFi and MQTT settings in `greenhouse_config.h`
4. Build and flash to your ESP32 board

```bash
idf.py build
idf.py -p [PORT] flash monitor
```

### Board Selection

When configuring the project, select "ESP32 Dev Module" as the board type.

## Project Structure

The project is organized into the following modules:

* `main.c` - Main application entry point and main loop
* `wifi.c` - WiFi connection management
* `mqtt.c` - MQTT client and message handling
* `sensors.c` - Temperature, humidity and wind sensors
* `relays.c` - Relay control via Modbus
* `ventilation.c` - Ventilation system control
* `cooling.c` - Cooling system control
* `greenhouse_config.h` - Central configuration

## MQTT Topics

The system communicates using the following MQTT topics:

* `sensors/sht20` - Temperature and humidity data
* `relay/status` - Relay status information
* `ventilation/status` - Ventilation status
* `ventilation/command` - Commands for ventilation control
* `cooling/status` - Cooling system status
* `cooling/command` - Commands for cooling system

## License

MIT

