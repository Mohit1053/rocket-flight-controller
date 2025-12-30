# Changelog

All notable changes to Predicones will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Planned
- PlatformIO support
- Kalman filter for sensor fusion
- Auto-tuning LQR gains
- Web-based configuration interface

## [1.0.0] - 2025-12-30

### Added
- Initial release of Predicones Flight Controller
- Integrated flight controller combining all modules:
  - `Fins_Deployment_SD_Integrated.ino` - Main integrated firmware
- Individual component modules:
  - `Fins.ino` - LQR-based fin stabilization
  - `deployment_GPS_integrated.ino` - CanSat deployment system
  - `SD_card_working.ino` - SD card data logging
  - `telemetry.ino` - XBee wireless telemetry
  - `BMP_final_final.ino` - BMP388 altimeter
  - `NEO_M8N_working.ino` - GPS module
  - `LORA_working.ino` - LoRa communication

### Hardware Support
- STM32H7A3ZIQ microcontroller
- MPU9250 9-DOF IMU
- BMP388 barometric altimeter
- NEO-M8N GPS module
- XBee Pro telemetry
- Feetech SCServo actuators
- SD card logging (SPI3)

### Features
- Real-time LQR attitude control
- Dual deployment system (CanSat + Parachute)
- 50Hz data logging to SD card
- Wireless telemetry via XBee
- GPS position tracking
- Graceful degradation on component failure

### Documentation
- Comprehensive README
- Hardware pinout documentation
- Installation instructions
- Sample flight data files
