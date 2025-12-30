# 🚀 Predicones - Rocket Flight Controller

> Advanced flight controller firmware for model rockets and CanSat missions

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Platform](https://img.shields.io/badge/Platform-STM32H7-blue.svg)](https://www.st.com/en/microcontrollers-microprocessors/stm32h7-series.html)
[![Arduino](https://img.shields.io/badge/Arduino-Compatible-green.svg)](https://www.arduino.cc/)

---

## 📋 Table of Contents

- [Overview](#-overview)
- [Features](#-features)
- [Hardware Requirements](#-hardware-requirements)
- [System Architecture](#-system-architecture)
- [Installation](#-installation)
- [Configuration](#-configuration)
- [Flight Data](#-flight-data)
- [Contributing](#-contributing)
- [License](#-license)
- [Safety Notice](#-safety-notice)

---

## 🎯 Overview

**Predicones** is a comprehensive flight controller firmware designed for model rockets and CanSat missions. Built on the STM32H7 platform, it provides real-time attitude stabilization, GPS tracking, telemetry, and data logging capabilities.

### Key Capabilities

| Feature | Description |
|---------|-------------|
| **LQR Stabilization** | Linear Quadratic Regulator-based fin control |
| **GPS Tracking** | Real-time position monitoring with NEO-M8N |
| **Telemetry** | Wireless data transmission via XBee/LoRa |
| **Data Logging** | High-speed SD card logging at 50Hz |
| **Deployment System** | Automated parachute and CanSat deployment |
| **Sensor Fusion** | MPU9250 IMU + BMP388 altimeter |

---

## ✨ Features

### Flight Control
- 🎯 **4-Fin Active Stabilization** - LQR-based attitude control
- 📊 **Real-time Sensor Fusion** - 9-DOF IMU data processing
- 🔄 **Dual Deployment System** - CanSat + Parachute release

### Sensors
- 📡 **MPU9250** - 9-axis IMU (accelerometer, gyroscope, magnetometer)
- 🌡️ **BMP388** - High-precision barometric altimeter
- 🛰️ **NEO-M8N** - GPS module for position tracking

### Communication
- 📻 **XBee Telemetry** - Real-time wireless data transmission
- 📡 **LoRa Support** - Long-range communication option
- 💾 **SD Card Logging** - High-speed data recording

### Safety
- 🪂 **Automated Deployment** - Altitude-based parachute deployment
- ⚡ **Graceful Degradation** - Continues operation if components fail
- 📝 **Comprehensive Logging** - Flight data for post-flight analysis

---

## 🔧 Hardware Requirements

### Microcontroller
- **STM32H7A3ZIQ** or compatible STM32H7 series

### Sensors
| Component | Model | Interface |
|-----------|-------|-----------|
| IMU | MPU9250 | I2C (0x69) |
| Altimeter | BMP388 | SPI1 |
| GPS | NEO-M8N | USART2 (38400 baud) |

### Communication
| Component | Model | Interface |
|-----------|-------|-----------|
| Telemetry | XBee Pro | USART6 (115200 baud) |
| LoRa | SX1278 | SPI (optional) |

### Actuators
| Component | Model | Interface |
|-----------|-------|-----------|
| Fin Servos (4x) | Feetech SCServo | Half-duplex UART |
| Deployment Servos (2x) | Feetech SCServo | Half-duplex UART |

### Storage
- MicroSD Card (FAT32, Class 10 recommended)

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    STM32H7 Flight Controller                 │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌──────────┐    ┌──────────┐    ┌──────────┐              │
│  │ MPU9250  │    │  BMP388  │    │ NEO-M8N  │              │
│  │  (IMU)   │    │(Altitude)│    │  (GPS)   │              │
│  └────┬─────┘    └────┬─────┘    └────┬─────┘              │
│       │I2C            │SPI1           │USART2              │
│       └───────┬───────┴───────────────┘                    │
│               ▼                                             │
│       ┌───────────────┐                                    │
│       │  LQR Control  │                                    │
│       │    Engine     │                                    │
│       └───────┬───────┘                                    │
│               │                                             │
│    ┌──────────┼──────────┐                                 │
│    ▼          ▼          ▼                                 │
│ ┌──────┐  ┌──────┐  ┌──────┐  ┌──────┐                    │
│ │Servo1│  │Servo2│  │Servo3│  │Servo4│  (Fin Control)     │
│ └──────┘  └──────┘  └──────┘  └──────┘                    │
│                                                             │
│ ┌──────┐  ┌──────┐                                         │
│ │Servo5│  │Servo6│  (Deployment)                          │
│ └──────┘  └──────┘                                         │
│                                                             │
│       ┌──────────┐    ┌──────────┐                         │
│       │ SD Card  │    │   XBee   │                         │
│       │ (SPI3)   │    │ (USART6) │                         │
│       └──────────┘    └──────────┘                         │
└─────────────────────────────────────────────────────────────┘
```

---

## 🚀 Installation

### Prerequisites

- [Arduino IDE](https://www.arduino.cc/en/software) 2.0+ or [PlatformIO](https://platformio.org/)
- STM32 board support package
- Required libraries (see below)

### Required Libraries

Install via Arduino Library Manager:

```
- Wire (built-in)
- SPI (built-in)
- Adafruit_Sensor
- Adafruit_BMP3XX
- MPU9250 (hideakitai/MPU9250)
- SCServo
- SdFat
- TinyGPS++ (optional, for GPS parsing)
```

### Setup Steps

1. **Clone the repository**
   ```bash
   git clone https://github.com/Mohit1053/predicones.git
   cd predicones
   ```

2. **Install STM32 board support**
   - In Arduino IDE: File → Preferences → Additional Board URLs
   - Add: `https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json`
   - Tools → Board → Boards Manager → Search "STM32" → Install

3. **Install required libraries**
   - Sketch → Include Library → Manage Libraries
   - Search and install each required library

4. **Select board and port**
   - Tools → Board → STM32 Boards → Select your STM32H7 variant
   - Tools → Port → Select appropriate COM port

5. **Upload firmware**
   - Open `Fins_Deployment_SD_Integrated.ino`
   - Click Upload

---

## ⚙️ Configuration

### Pin Assignments

Edit the following defines in the main sketch if your hardware differs:

```cpp
// IMU
#define MPU_ADDRESS 0x69

// BMP388 (SPI1)
#define BMP_CS   7
#define BMP_SCK  13
#define BMP_MISO 12
#define BMP_MOSI 11

// SD Card (SPI3)
#define SD_CS_PIN PD14

// Servo IDs
const int SERVO_FIN_IDS[4] = {1, 2, 3, 4};
```

### Flight Parameters

Adjust these values based on your rocket configuration:

```cpp
// Control gains (LQR)
const float K[4] = {...};  // Tune for your rocket

// Deployment altitudes
const float CANSAT_DEPLOY_ALT = 500.0;  // meters
const float CHUTE_DEPLOY_ALT = 300.0;   // meters
```

---

## 📊 Flight Data

The system logs comprehensive flight data to the SD card:

### Log Format (CSV)

```csv
timestamp,altitude,velocity,ax,ay,az,gx,gy,gz,lat,lon,servo1,servo2,servo3,servo4,state
```

### Sample Data Files

- `FLIGHT_DATA.CSV` - Example flight data
- `FLIGHT_DATA_1.CSV` - Additional flight recording

---

## 📁 Project Structure

```
predicones/
├── .github/
│   ├── workflows/
│   │   └── ci.yml
│   ├── ISSUE_TEMPLATE.md
│   └── PULL_REQUEST_TEMPLATE.md
├── src/
│   ├── Fins_Deployment_SD_Integrated.ino  # Main integrated firmware
│   ├── Fins.ino                           # Fin control module
│   ├── deployment_GPS_integrated.ino      # Deployment system
│   ├── telemetry.ino                      # XBee telemetry
│   ├── BMP_final_final.ino               # Altimeter module
│   ├── NEO_M8N_working.ino               # GPS module
│   ├── SD_card_working.ino               # SD logging module
│   └── LORA_working.ino                  # LoRa communication
├── data/
│   ├── FLIGHT_DATA.CSV
│   └── FLIGHT_DATA_1.CSV
├── docs/
│   └── wiring-diagram.md
├── .gitignore
├── CHANGELOG.md
├── CONTRIBUTING.md
├── LICENSE
└── README.md
```

---

## 🤝 Contributing

Contributions are welcome! Please read our [Contributing Guidelines](CONTRIBUTING.md) first.

### Quick Start

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/improvement`)
3. Commit changes (`git commit -m 'feat: add new feature'`)
4. Push to branch (`git push origin feature/improvement`)
5. Open a Pull Request

---

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## ⚠️ Safety Notice

**IMPORTANT: This is experimental rocket flight controller firmware.**

- Always follow local regulations for model rocketry
- Test thoroughly on the ground before flight
- Use appropriate safety measures during launches
- Never fly near people, buildings, or airports
- The authors are not responsible for any damage or injury

---

## 👤 Author

**Predicones Flight Controller**

- GitHub: [@Mohit1053](https://github.com/Mohit1053)

---

## 🙏 Acknowledgments

- STM32duino community
- Adafruit for sensor libraries
- Feetech for servo documentation
- Model rocketry community

---

<p align="center">
  Made with ❤️ for the rocketry community
</p>


