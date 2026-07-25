# 🛰️ Satellite Telemetry Monitor - LoRa Edition

**A high-performance real-time GUI application for monitoring and visualizing telemetry data from CubeSat/small satellites via LoRa communication on Raspberry Pi 5.**

![Version](https://img.shields.io/badge/Version-2.4-blue) ![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%205-red) ![Python](https://img.shields.io/badge/Python-3.7+-green) ![License](https://img.shields.io/badge/License-Educational-yellow) ![Protocol](https://img.shields.io/badge/Protocol-17%20fields-orange) ![Platform](https://img.shields.io/badge/MCU-STM32G431CBU6-blue) ![IDE](https://img.shields.io/badge/IDE-STM32CubeIDE-yellow)

---

## 📋 Table of Contents
1. [System Overview](#system-overview)
2. [Features](#features)
3. [Hardware Components](#hardware-components)
4. [Telemetry Protocol](#telemetry-protocol)
5. [Project Structure](#project-structure)
6. [Installation](#installation)
7. [Usage](#usage)
8. [Transmitter (STM32)](#transmitter-stm32)
9. [Receiver GUI (Raspberry Pi)](#receiver-gui-raspberry-pi)
10. [Data Analysis Tools](#data-analysis-tools)
11. [Signal Processing Tutorials](#signal-processing-tutorials)
12. [LIDAR Point Cloud Processing](#lidar-point-cloud-processing)
13. [Troubleshooting](#troubleshooting)
14. [Known Issues & Bug Fixes](#known-issues--bug-fixes)

---

## 🎯 System Overview

Complete satellite telemetry system with:
- **Real-time LoRa telemetry** at 869.53 MHz
- **High-speed LIDAR logging** (50 Hz) for 3D point cloud generation
- **Full sensor fusion** (IMU, GPS, Barometer, LIDAR)
- **Real-time GUI visualization** with 3D satellite model and map
- **Comprehensive post-flight analysis** suite
- **Signal processing tutorials** for educational purposes

### System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    SATELLITE (STM32G4)                          │
├─────────────────────────────────────────────────────────────────┤
│  Sensors:                                                       │
│    • ICM-20948 IMU (accel, gyro, mag)                           │
│    • BMP581 Barometer (altitude, temp)                          │
│    • GNSS (GPS position)                                        │
│    • LightWare SF20 LIDAR (distance)                            │
│    • Battery voltage (ADC)                                      │
│                                                                 │
│  Communication:                                                 │
│    • LoRa SX1276 @ 869.53 MHz → Ground Station                  │
│                                                                 │
│  Storage:                                                       │
│    • SD Card (NOT IMPLEMENTED YET)                              │
│      - LIDAR.CSV (high-speed, 50 Hz)                            │
│      - TELEM.CSV (full telemetry, 2 Hz)                         │
└─────────────────────────────────────────────────────────────────┘
                              ↓ LoRa
┌─────────────────────────────────────────────────────────────────┐
│              GROUND STATION (Raspberry Pi 5)                    │
├─────────────────────────────────────────────────────────────────┤
│  Hardware:                                                      │
│    • LoRa SX1276 receiver @ 869.53 MHz                          │
│                                                                 │
│  Real-Time GUI Software:                                        │
│    • main.py - Application launcher                             │
│    • gui_main_window.py - Main telemetry window                 │
│    • data_plotter.py - Real-time plots (3 graphs)               │
│    • satellite_3d.py - 3D STL model visualization               │
│    • map_view.py - Interactive OSM tile map                     │
│    • lora_data_handler.py - LoRa reception                      │
│    • Saves flight.csv with all data                             │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│                POST-FLIGHT ANALYSIS SUITE                       │
├─────────────────────────────────────────────────────────────────┤
│  Basic Analysis:                                                │
│    • analyze_telemetry.py - Comprehensive flight report         │
│                                                                 │
│  Advanced Analysis:                                             │
│    • advanced_analysis.py - 6-panel automated analysis          │
│    • advanced_features.py - Physics-based deep analysis         │
│                                                                 │
│  Signal Processing:                                             │
│    • signal_processing_tutorial.py - Educational package        │
│    • Generates 4 Octave tutorials + Python examples             │
│                                                                 │
│  Point Cloud:                                                   │
│    • merge_point_cloud.py - LIDAR + GPS fusion                  │
│    • generate_synthetic_data.py - Test data generator           │
│    • Generates 3D terrain map                                   │
└─────────────────────────────────────────────────────────────────┘
```

---

## ✨ Features

### Real-Time Ground Station GUI:
- **Real-time Data Plotting**: Visualizes vertical speed, temperature, and altitude data in real-time
- **3D Satellite Visualization**: Shows your custom STL satellite model that rotates based on IMU data
- **2D Map Tracking**: Fast OSM tile map — tiles fetched once, position updates are instant
- **LoRa Communication**: Direct radio reception using SX127x LoRa module on Raspberry Pi 5
- **Resizable Layout**: Adjustable splitters to customize view sizes
- **Flight Phase Indicators**: Visual flags for calibration, drop, parachute deployment, and landing
- **Battery Monitoring**: Real-time battery percentage with 12-sample moving average smoothing
- **GPS Satellite Count**: Color-coded display of GPS fix quality

### Post-Flight Analysis:
- **Comprehensive Flight Reports**: 18-panel plots with statistics
- **Advanced Physics Analysis**: Drag coefficient, energy analysis, rotation detection
- **Signal Processing Education**: Low-pass filtering, FFT analysis, Kalman filtering
- **3D Point Cloud Generation**: LIDAR + GPS fusion for terrain mapping
- **Google Earth Export**: KML files for 3D flight visualization

---

## 🔧 Hardware Components

### Transmitter (Satellite)
| Component | Model | Interface | Purpose |
|-----------|-------|-----------|---------|
| MCU | STM32G431 | - | Main controller |
| IMU | ICM-20948 | I2C3 | Orientation, acceleration |
| Barometer | BMP581 | I2C1 | Altitude, temperature |
| GPS | Generic GNSS | UART1 | Position, time |
| LIDAR | LightWare SF20/LW20 | UART3 | Distance to ground |
| LoRa | SX1276 | SPI2 | Telemetry transmission |
| Battery Monitor | - | ADC1 | Voltage monitoring |
| SD Card | - | SPI (NOT IMPL.) | Data logging |

### Receiver (Ground Station)
| Component | Model | Interface | Purpose |
|-----------|-------|-----------|---------|
| Computer | Raspberry Pi 5 | - | Data reception & visualization |
| LoRa | SX1276 | SPI | Telemetry reception |
| Display | HDMI Monitor | - | GUI display |

**LoRa Hardware Connections (Raspberry Pi 5):**
- VCC → 3.3V / GND → GND
- MISO → GPIO 9 / MOSI → GPIO 10 / SCK → GPIO 11
- NSS/CS → GPIO 8 / RESET → GPIO 21
- DIO0 → GPIO 22 / DIO1 → GPIO 23 / DIO2 → GPIO 24

---

## 📡 Telemetry Protocol

### Protocol Version: **v2.4** (17-field)

### Packet Types

#### 1. Calibration Packet (sent once at startup)
```
CAL,<calibration_duration_ms>,<timestamp_ms>\r\n
```

**Example:**
```
CAL,3245,3245\r\n
```

**Fields:**
- `CAL` - Packet identifier
- `calibration_duration_ms` - Time taken to calibrate sensors (ms)
- `timestamp_ms` - STM32 timestamp when calibration completed (ms)

---

#### 2. Telemetry Packet (sent continuously during flight)
```
accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,roll,pitch,yaw,
temperature,altitude,latitude,longitude,satellites,flags,timestamp_ms,battery_voltage\r\n
```

**Example:**
```
0.12,0.15,9.81,1.2,0.5,0.3,5.2,2.1,45.3,22.5,385.2,48.8566,2.3522,8,3,4500,7.54\r\n
```

**17 Fields:**

| # | Field | Unit | Description |
|---|-------|------|-------------|
| 1 | accel_x | m/s² | X-axis acceleration |
| 2 | accel_y | m/s² | Y-axis acceleration |
| 3 | accel_z | m/s² | Z-axis acceleration |
| 4 | gyro_x | °/s | X-axis rotation rate |
| 5 | gyro_y | °/s | Y-axis rotation rate |
| 6 | gyro_z | °/s | Z-axis rotation rate |
| 7 | roll | ° | Roll angle |
| 8 | pitch | ° | Pitch angle |
| 9 | yaw | ° | Yaw angle (heading) |
| 10 | temperature | °C | Barometer temperature |
| 11 | altitude | m | Altitude above sea level |
| 12 | latitude | ° | GPS latitude |
| 13 | longitude | ° | GPS longitude |
| 14 | satellites | count | GPS satellites locked |
| 15 | flags | bitmask | Flight phase flags |
| 16 | timestamp_ms | ms | STM32 timestamp |
| 17 | battery_voltage | V | Battery voltage |

**Note:** `vertical_speed` is **NOT transmitted**. The receiver calculates it from consecutive altitude readings:
```python
vertical_speed = (altitude_now - altitude_prev) / (time_now - time_prev)
```

---

### Flight Phase Flags (Bitmask)

| Bit | Value | Flag | Phase |
|-----|-------|------|-------|
| 0 | 0x01 | GO_FOR_LAUNCH | Ready for launch |
| 1 | 0x02 | ASCENSION | Ascending |
| 2 | 0x04 | DROP | Dropping/falling |
| 3 | 0x08 | RECOVERY | Recovery/landed |

**Examples:**
- `flags = 0` (0b0000) → Calibrating
- `flags = 1` (0b0001) → Ready for launch
- `flags = 3` (0b0011) → Ready + Ascending
- `flags = 7` (0b0111) → Ready + Ascending + Dropping
- `flags = 15` (0b1111) → All phases (recovered)

---

### LoRa Configuration

| Parameter | Value | Notes |
|-----------|-------|-------|
| Frequency | 869.53 MHz | EU ISM band |
| Spreading Factor | SF7 | Good range/data rate balance |
| Bandwidth | 125 kHz | Standard BW |
| Coding Rate | CR4/8 | Maximum error correction |
| Sync Word | 0x12 | Private network |
| TX Power | +17 dBm | Maximum allowed |
| Preamble | 8 symbols | Standard |

**Link Budget:**
- **Range:** ~5-10 km (line of sight)
- **Data Rate:** ~980 bps
- **Packet Size:** ~140 bytes
- **Air Time:** ~1.2 seconds per packet

---

## 📁 Project Structure

```
satellite_telemetry/
│
├── 📄 main.py                    # Application entry point
│   └── Launches GUI, handles venv path prioritization
│
├── 🖼️ gui_main_window.py         # Main telemetry display window (v2.4)
│   └── Integrates 3D view, map, plots; manages LoRa receiver lifecycle
│   └── Flag mapping: GO_FOR_LAUNCH, ASCENSION, DROP, RECOVERY
│
├── 📡 lora_data_handler.py       # LoRa receiver integration (v2.4)
│   └── SX127x hardware interface, CSV parsing (17 fields)
│   └── Battery voltage + GPS satellite count parsing
│   └── Labeled console output with emojis
│
├── 📊 data_plotter.py            # Real-time sensor data plotting (v2.4)
│   └── Optimized PyQtGraph plots with numpy circular buffers
│   └── Altitude-based vertical speed (no accelerometer drift!)
│   └── Battery percentage with 12-sample moving average smoothing
│   └── GPS satellite count display with color coding
│
├── 🛰️ satellite_3d.py            # 3D satellite model visualization
│   └── STL loader with optimized OpenGL rendering and rotation
│
├── 🗺️ map_view.py                # Interactive map widget
│   └── OSM tile-based map with async loading, zoom 18 detail
│
├── 📦 requirements.txt           # Python dependencies
│   └── PyQt5, PyQtGraph, NumPy, PyOpenGL, numpy-stl, etc.
│
├── 🎨 Corps.stl                  # Satellite 3D model (user-provided)
│   └── Custom STL file for your specific satellite design
│
├── 📖 README_GUI.md                  # This file
│   └── Complete documentation and usage guide
│
├── 📂 SX127x/                    # LoRa library (external dependency)
│   ├── __init__.py
│   ├── LoRa.py                   # Base LoRa class
│   ├── constants.py              # LoRa register constants
│   ├── board_config.py           # Raspberry Pi GPIO configuration
│   └── ...
│
├── 📂 Cansat_V1/               # STM32 firmware
│   └── Core/
│       ├── Src/
│       │   ├── main.c           # Main application
│       │   ├── bmp581.c         # Barometer driver
│       │   ├── imu.c            # IMU driver
│       │   ├── sx1276.c         # LoRa driver
│       │   ├── gnss_reader.c    # GPS parser
│       │   ├── lidar.c          # LIDAR driver
│       │   └── cansat_core.c    # Core definitions
│       └── Inc/
│           ├── main.h
│           ├── bmp581.h          # Barometer driver
│           ├── imu.h             # IMU driver
│           ├── sx1276.h          # LoRa driver
│           ├── gnss_reader.h     # GPS parser
│           ├── lidar.h           # LIDAR driver
│           └── cansat_core.h     # Core definitions
│   
│
└── 📂 Analysis_Tools/            # Post-flight analysis
    ├── Basic/
    │   └── analyze_telemetry.py     # Comprehensive report
    ├── Advanced/
    │   ├── advanced_analysis.py     # Automated suite
    │   └── advanced_features.py     # Physics-based analysis
    ├── Signal_Processing/
    │   └── signal_processing_tutorial.py  # Educational package
    └── Point_Cloud/
        ├── generate_synthetic_data.py     # Test data generator
        └── merge_point_cloud.py           # LIDAR + GPS fusion
```

### File Descriptions

| File | Lines | Purpose | Key Features |
|------|-------|---------|--------------|
| `main.py` | ~50 | Entry point | venv path fix, error handling |
| `gui_main_window.py` | ~220 | Main window | 4-panel layout, resource cleanup |
| `lora_data_handler.py` | ~180 | LoRa interface | SX127x config, CSV parsing, signals |
| `data_plotter.py` | ~280 | Data plots | Numpy buffers, 3 plots, flight flags |
| `satellite_3d.py` | ~240 | 3D rendering | STL loading, rotation, OpenGL |
| `map_view.py` | ~320 | Map display | Tile fetching, zoom 18, Y-axis inversion |

---

## 🚀 Installation

### Prerequisites

- **Raspberry Pi 5** with Raspberry Pi OS
- **SX127x LoRa module** connected to Raspberry Pi (SPI interface)
- Python 3.7 or higher
- Internet connection (for OSM map tiles on first load)

### Hardware Setup

1. **Connect LoRa Module to Raspberry Pi 5**:
   - VCC → 3.3V / GND → GND
   - MISO → GPIO 9 / MOSI → GPIO 10 / SCK → GPIO 11
   - NSS/CS → GPIO 8 / RESET → GPIO 21
   - DIO0 → GPIO 22 / DIO1 → GPIO 23 / DIO2 → GPIO 24

2. **Enable SPI**:
   ```bash
   sudo raspi-config  # Interface Options → SPI → Enable
   sudo reboot
   ```

### Software Setup

```bash
sudo apt-get update
sudo apt-get install python3-pyqt5 python3-spidev
pip install -r requirements.txt
```

> **Note**: `python3-pyqt5.qtwebengine` is NOT needed — QtWebEngine has been removed entirely.

---

## 🎮 Usage

### Running the Real-Time GUI

```bash
python main.py
# or with sudo for GPIO access:
sudo python main.py
```

### CSV Data Format (Receiver Output)

```csv
accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,roll,pitch,yaw,temperature,altitude,latitude,longitude,satellites,flags_raw,timestamp_ms,battery_voltage,rssi,snr
```

**Exported to:** `flight.csv` (automatically saved during reception)

### Interface Overview

**Top Left**: 3D STL model rotating with roll/pitch/yaw.

**Bottom Left**: OSM tile map (zoom 15, 5×5 tile grid). Tiles are fetched once at startup using 8 parallel threads and cached in memory. Position updates are instant — they only move PyQtGraph scatter/line overlay items with no tile work involved. The grid reloads only if the satellite leaves the current tile boundary. Blue polyline path + red dots per point.

**Right**: Vertical speed, temperature, altitude plots + flight phase flags.

**Bottom**: Stop/Clear buttons, LoRa status indicator.

---

## 🖥️ Transmitter (STM32)

### Current Implementation Status

| Feature | Status | Notes |
|---------|--------|-------|
| IMU Reading | ✅ Implemented | ICM-20948 via I2C3 |
| Barometer Reading | ✅ Implemented | BMP581 via I2C1 |
| GPS Reading | ✅ Implemented | UART1 parsing |
| LIDAR Reading | ✅ Implemented | LightWare SF20 via UART3 |
| Battery Monitoring | ✅ Implemented | ADC1 with voltage divider |
| LoRa Transmission | ✅ Implemented | SX1276 at 869.53 MHz |
| Flight State Machine | ✅ Implemented | 5 states + transitions |
| SD Card Logging | ❌ **NOT IMPLEMENTED** | Ready for integration |

### Main Application Features

**main.c** includes:

#### RTOS Tasks (FreeRTOS)

1. **TaskFSM** - Flight State Machine
   - Priority: Normal
   - Stack: 1KB
   - Controls: State transitions, thresholds
   
2. **TaskSensors** - Sensor Data Acquisition
   - Priority: High (sensor reading critical)
   - Stack: 2KB
   - Handles: IMU, Baro, GPS, LIDAR events
   
3. **TaskSDCard** - Data Logging
   - Priority: Low
   - Stack: 1KB
   - Status: **EMPTY (NOT IMPLEMENTED)**
   - Planned: Dual-file logging (LIDAR.CSV + TELEM.CSV)
   
4. **TaskLoRa** - Radio Transmission
   - Priority: Low
   - Stack: 2KB
   - Handles: LoRa packet transmission

#### Queues

```c
QueueHandle_t qLoRa;          // Telemetry packets to transmit
QueueHandle_t qSensorEvents;  // Sensor event notifications
QueueHandle_t qSDCard;        // NOT USED (SD card not implemented)
```

#### Flight State Machine

```
CALIBRATING (0)
    ↓ (sensors stable)
READY (1) - GO_FOR_LAUNCH
    ↓ (altitude > 400m OR LIDAR < 2m)
ASCENSION (2)
    ↓ (LIDAR > 25m)
DROP (3)
    ↓ (stable on ground)
RECOVERY (4)
```

**Thresholds:**
```c
#define THRESH_ALTITUDE_LAUNCH_M     400.0f  // Launch altitude
#define THRESH_LIDAR_IN_BOX_M        2.0f    // LIDAR in box threshold
#define THRESH_LIDAR_DEPLOYED_M      25.0f   // Deployment threshold
```

### Battery Monitoring

**Circuit:**
```
Battery (7.4V) → R1 (20kΩ) → ADC1 ← R2 (10kΩ) → GND
```

**Voltage Divider Ratio:** 3.0× (accounts for 10kΩ/(20kΩ+10kΩ))

**ADC Reading:**
```c
HAL_ADC_Start(&hadc1);
if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
    uint32_t raw_adc = HAL_ADC_GetValue(&hadc1);
    vbat = ((float)raw_adc / 4095.0f) * 3.3f * 3.0f * 1.025f;
    // 1.025 is calibration factor
}
HAL_ADC_Stop(&hadc1);
```

### SD Card Logging (Future Implementation)

**Planned Dual-File System:**

#### File 1: LIDAR.CSV (High-Speed, ~50 Hz)
```csv
timestamp_ms,distance_m,roll,pitch,yaw,latitude,longitude,altitude,flags
3501,12.34,5.2,2.1,45.3,48.8566,2.3522,385.2,3
3502,12.35,5.2,2.1,45.3,48.8566,2.3522,385.2,3
...
```

#### File 2: TELEM.CSV (Full Telemetry, ~2 Hz)
```csv
timestamp_ms,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,roll,pitch,yaw,temperature,altitude,latitude,longitude,satellites,flags_raw,battery_voltage
3500,0.12,0.15,9.81,1.2,0.5,0.3,5.2,2.1,45.3,22.5,385.2,48.8566,2.3522,8,3,7.54
...
```

**Benefits:**
- No wasted space (no zeros)
- LIDAR at full rate
- Easy post-processing
- Small file sizes (~225 KB per 60s flight)

**See:** `main_with_lidar_logging.c` for implementation template

---

## 📻 Receiver GUI (Raspberry Pi)

### Software Components

#### 1. main.py
**Application entry point**

**Features:**
- venv path prioritization (fixes PyQtGraph version conflicts)
- Error handling and graceful shutdown
- Resource cleanup on exit

**Run:**
```bash
python main.py
```

---

#### 2. gui_main_window.py
**Main telemetry display window (v2.4)**

**Features:**
- 4-panel resizable layout
- Integrates 3D satellite view, map, and plots
- LoRa receiver lifecycle management
- Flight phase flag mapping
- CSV data export
- Resource cleanup on close

**Layout:**
```
┌─────────────────────────────────────┐
│  3D Satellite  │  Plots + Flags     │
│  (STL model)   │  (v_speed, temp,   │
│                │   altitude)        │
├─────────────────────────────────────┤
│  Map View      │  Controls          │
│  (OSM tiles)   │  (Stop/Clear/      │
│                │   Status)          │
└─────────────────────────────────────┘
```

---

#### 3. lora_data_handler.py
**LoRa receiver integration (v2.4)**

**Features:**
- SX127x hardware interface
- Packet type detection (CAL vs telemetry)
- 17-field CSV parsing
- Battery voltage monitoring
- GPS satellite count tracking
- RSSI/SNR monitoring
- Qt signal emission for GUI
- Labeled console output with emojis

**Configuration:**
- Frequency: 869.53 MHz
- Spreading Factor: SF7
- Bandwidth: 125 kHz
- Coding Rate: CR4/8
- Sync Word: 0x12

**Usage:**
```python
from lora_data_handler import LoRaDataReceiver

receiver = LoRaDataReceiver(verbose=True)
receiver.data_received.connect(on_data_callback)
receiver.configure_and_start()
```

---

#### 4. data_plotter.py
**Real-time sensor data plotting (v2.4)**

**Features:**
- Optimized PyQtGraph plots with numpy circular buffers (10,000 points)
- 3 real-time plots:
  1. **Vertical Speed** - Calculated from altitude deltas (m/s)
  2. **Temperature** - Barometer temperature (°C)
  3. **Altitude** - Above sea level (m)
- Flight phase color-coded backgrounds:
  - Gray: CALIBRATION
  - Blue: GO_FOR_LAUNCH
  - Green: ASCENSION
  - Red: DROP
  - Orange: RECOVERY
- Battery percentage with 12-sample moving average smoothing
- GPS satellite count display with color coding
- Auto-scaling axes
- Grid lines for readability

**Performance:**
- Update rate: Real-time (as packets arrive)
- Buffer size: 10,000 points
- Circular buffer prevents memory overflow

---

#### 5. satellite_3d.py
**3D satellite model visualization**

**Features:**
- STL file loader (reads `Corps.stl`)
- Optimized OpenGL rendering
- Real-time rotation based on IMU data:
  - Roll (X-axis)
  - Pitch (Y-axis)
  - Yaw (Z-axis)
- Smooth rotation interpolation
- Lighting and shading
- Perspective camera

**Supported formats:**
- Binary STL
- ASCII STL

---

#### 6. map_view.py
**Interactive map widget with OSM tiles**

**Features:**
- OpenStreetMap tile-based map
- Async tile loading with ThreadPoolExecutor (8 workers)
- Zoom level 18 detail
- 7×7 tile grid (auto-expands as satellite moves)
- In-memory tile caching
- Real-time GPS tracking:
  - Blue polyline path
  - Red dots per position
  - Red marker at current position
- Instant position updates (no tile re-fetching)
- Y-axis inversion fix for correct tile rendering
- Lazy tile loading (only fetches when needed)

**Performance:**
- First load: ~2-3 seconds (25 tiles fetched)
- Position updates: <1ms (zero network work)
- Tile cache: Permanent for session

**Map Controls:**
- Auto-centering on satellite position
- Pan viewport on GPS update
- Grid reloads only when satellite leaves boundary

---

## 📊 Data Analysis Tools

### Quick Reference

| Tool | Purpose | Input | Output | Runtime |
|------|---------|-------|--------|---------|
| analyze_telemetry.py | Comprehensive flight report | flight.csv | 18 plots + stats | ~10s |
| advanced_analysis.py | Automated 6-panel analysis | flight.csv | 6 analyses + folder | ~15s |
| advanced_features.py | Physics-based deep dive | flight.csv | 5 advanced analyses | ~20s |
| signal_processing_tutorial.py | Educational package | flight.csv | Python + 4 Octave scripts | ~5s |
| merge_point_cloud.py | 3D point cloud generator | LIDAR.CSV + TELEM.CSV | 3D terrain map | ~30s |
| generate_synthetic_data.py | Test data generator | - | LIDAR.CSV + TELEM.CSV | ~2s |

---

### 1. analyze_telemetry.py

**Basic flight analysis with comprehensive reporting**

#### Features:
- ✅ 18 time-series plots (one per field)
- ✅ Flight phase color-coded backgrounds
- ✅ Automatic statistics calculation
- ✅ Text report generation
- ✅ Windows-compatible (UTF-8 encoding)

#### Usage:
```bash
python analyze_telemetry.py flight.csv output_folder/
```

#### Output:
```
output_folder/
├── all_fields_plot.png          # 18-panel comprehensive plot
└── flight_analysis_report.txt   # Statistics summary
```

---

### 2. advanced_analysis.py

**Automated comprehensive analysis suite**

#### Features:
- ✅ 6 different analyses run automatically
- ✅ All outputs to single folder
- ✅ Publication-ready plots
- ✅ KML export for Google Earth

#### Usage:
```bash
python advanced_analysis.py flight.csv
```

#### Output:
```
advanced_analysis_folder/
├── 1_rotation_analysis.png      # Gyroscope magnitude + tumbling detection
├── 2_gps_trajectory.png         # 2D map + displacement calculation
├── 3_signal_quality.png         # RSSI/SNR vs altitude correlation
├── 4_anomaly_detection.txt      # Gap detection, GPS jumps, sensor issues
├── 5_energy_analysis.png        # Kinetic/potential energy over time
└── 6_google_earth.kml           # Import to Google Earth
```

---

### 3. advanced_features.py

**Physics-based deep analysis**

#### Features:
- ✅ 5 advanced analyses
- ✅ Animated flight replay (GIF)
- ✅ 3D orientation visualization
- ✅ Drag coefficient calculation
- ✅ Parachute deployment detection
- ✅ Power consumption analysis

#### Usage:
```bash
python advanced_features.py flight.csv
```

---

## 🎓 Signal Processing Tutorials

### signal_processing_tutorial.py

**Educational package teaching signal processing with real flight data**

#### What it Creates:

```
signal_processing_output/
├── README.txt                          # Getting started guide
├── telemetry_data.mat                  # MATLAB/Octave format (21 fields)
├── python_filtering_demo.png           # Low-pass filter example
├── python_fft_demo.png                 # FFT analysis example
├── octave_tutorial_1_basics.m          # Loading & plotting
├── octave_tutorial_2_filtering.m       # Filter design
├── octave_tutorial_3_fft.m             # Frequency analysis
└── octave_tutorial_4_kalman.m          # Kalman filter
```

#### Usage:
```bash
python signal_processing_tutorial.py flight.csv
```

#### Key Concepts Taught:

**Low-Pass Filtering:**
- Why? Remove high-frequency noise (vibrations, ADC jitter)
- How? Keep signals below cutoff, attenuate above
- When? Temperature smoothing, GPS cleaning

**FFT Analysis:**
- Why? Reveal hidden patterns in data
- How? Decompose signal into frequency components
- When? Finding rotation rates, vibration frequencies

**Kalman Filtering:**
- Why? Optimal fusion of noisy measurements with physics model
- How? Predict using model, correct using measurements
- When? GPS smoothing, sensor fusion

---

## 🗺️ LIDAR Point Cloud Processing

### System Overview

Fuses GPS position with LIDAR distance measurements to create 3D terrain map.

**Physics:**
```
Ground_Point = Satellite_Position + (LIDAR_Distance × Direction_Vector)

Where:
  Satellite_Position = (latitude, longitude, altitude) from GPS
  Direction_Vector = calculated from (roll, pitch, yaw) using rotation matrices
  LIDAR_Distance = distance measurement from LIDAR sensor
```

---

### 1. generate_synthetic_data.py

**Creates realistic test data for development**

#### Usage:
```bash
python generate_synthetic_data.py
```

#### Output:
```
LIDAR.CSV       231 KB   3000 readings @ 50 Hz
TELEM.CSV       18 KB    120 packets @ 2 Hz
```

#### Scenario Simulated:
```
0-5s:    Calibration (ground, 400m altitude)
5-15s:   Go for launch (waiting)
15-35s:  Ascension to 500m (~5 m/s climb)
35-60s:  Drop with tumbling (terminal velocity ~15 m/s)
60s+:    Recovery/landing
```

---

### 2. merge_point_cloud.py

**Merges LIDAR + Telemetry into 3D point cloud**

#### Usage:
```bash
python merge_point_cloud.py LIDAR.CSV TELEM.CSV
```

#### Output:
```
point_cloud.csv                     # Full merged dataset
point_cloud_XYZ.txt                 # For CloudCompare import
point_cloud_visualization.png       # 6-panel plot
```

#### CloudCompare Workflow:

**1. Import Point Cloud:**
```
File → Open → point_cloud_XYZ.txt
Format: ASCII (space-separated)
Columns: X=longitude, Y=latitude, Z=altitude
```

**2. Colorize by Altitude:**
```
Edit → Colors → Height Ramp
Choose color scheme (e.g., terrain)
```

**3. Export:**
```
File → Save As
Formats: .las, .laz (LiDAR standard), .ply, .obj
```

---

## 🔧 Troubleshooting

### LoRa Module Not Detected
- Verify SPI: `lsmod | grep spi`
- Check GPIO connections
- Try: `sudo python main.py`

### Map tiles slow to appear on first load
- This is normal — 25 tiles are fetched on startup
- After that, updates are instant
- Subsequent runs reuse the in-memory cache for the session

### Invalid GPS coordinates
- Values outside lat [-90,90] / lon [-180,180] are rejected
- This is a transmitter calibration issue, not a software bug
- Check GPS antenna and sky view

### GPS: No lock
- Clear sky view required
- Wait 2-3 minutes for cold start
- Check antenna connected
- Verify UART baud rate: 9600 or 115200

### LIDAR: No data
- Check UART3 baud: 115200
- Stream mode enabled: `Lidar_RequestStream()`
- Use `Lidar_DirectDebug()` to verify

### Analysis: Script crashes
- Check Python dependencies installed
- Verify CSV file not corrupted
- Ensure correct column names
- Check for empty data

---

## 🐛 Known Issues & Bug Fixes

### Fix 1 — PyQtGraph `drawLines` TypeError (`main.py`)
**Symptom**: `TypeError: arguments did not match any overloaded call: drawLines(...)`

**Cause**: System pyqtgraph 0.13.1 being loaded instead of venv's 0.14.0.

**Fix**: `main.py` inserts the venv's site-packages at the front of `sys.path` before any imports.

---

### Fix 2 — QtWebEngine Chromium crash → removed entirely (`map_view.py`)
**Symptom**: `FATAL: page_allocator_internals_posix.h(169)] Check failed` / `Trace/breakpoint trap`

**Cause**: Chromium renderer incompatible with Raspberry Pi 5 ARM kernel memory allocator. No flag combination (`--no-sandbox`, `--disable-gpu`, `--single-process`) resolved it.

**Fix**: QtWebEngine removed entirely. Map reimplemented in PyQtGraph with urllib tile fetching.

---

### Fix 3 — Map too slow to update (`map_view.py`)
**Symptom**: Map position updates were visibly laggy.

**Cause**: Tiles were being fetched and composited on every `update_position()` call.

**Fix**: Tile loading and overlay updates are now fully decoupled:
- `_load_tiles()` runs once at startup in a `ThreadPoolExecutor` with 8 workers
- `update_position()` only calls `_redraw_overlays()` which moves PyQtGraph items — zero network or compositing work
- Tiles are cached in memory; the grid only reloads when the satellite leaves its boundary

---

### Fix 4 — Y-axis tile inversion (`map_view.py`)
**Symptom**: Map tiles displayed upside-down.

**Cause**: OSM uses TMS tile coordinates (Y increases southward), but PyQtGraph ImageItem expects Y increasing upward.

**Fix**: Apply Y-axis inversion when positioning ImageItems:
```python
tile_y_inverted = (2**zoom - 1) - tile_y
```

---

### Fix 5 — Map slow to rebuild on GPS update (`map_view.py`)
**Symptom**: Map was slow to respond to incoming GPS positions.

**Cause**: Previous implementation rebuilt a numpy canvas on every GPS update — re-blitting all visible tiles into a single array each time.

**Fix**: Tiles are now permanent `pg.ImageItem` objects placed once at fixed world coordinates and never moved or rebuilt. Each GPS update does only three O(1) `setData` calls (polyline, dots, marker) and a viewport pan — zero tile work. A 7×7 tile grid is pre-fetched asynchronously at startup and lazily expanded as the satellite moves.

---

## 📊 Performance Metrics

### System Performance

| Metric | Value | Notes |
|--------|-------|-------|
| Telemetry Rate | ~2 Hz | Limited by LoRa air time |
| LIDAR Rate | ~50 Hz | (Future, on SD card) |
| LoRa Range | 5-10 km | Line of sight |
| Packet Loss | <5% | Typical in good conditions |
| Battery Life | ~2 hours | Depends on usage |
| SD Card Storage | ~4400 flights | 1 GB card, 60s flights |
| GUI Update Rate | Real-time | <50ms latency |
| Map Tile Load | ~2-3 seconds | First load only |
| Map Update | <1ms | After tiles loaded |

### File Sizes (60 second flight)

| File | Size | Samples | Rate |
|------|------|---------|------|
| flight.csv (receiver) | ~15 KB | ~120 | 2 Hz |
| LIDAR.CSV (SD card) | ~231 KB | ~3000 | 50 Hz |
| TELEM.CSV (SD card) | ~18 KB | ~120 | 2 Hz |
| **Total per flight** | **~250 KB** | - | - |

---

## 📚 Additional Resources

### Python Dependencies
```bash
# Core dependencies
pip install numpy pandas matplotlib scipy

# For GUI (receiver)
pip install pyqt5 pyqtgraph pyopengl numpy-stl

# For LoRa (Raspberry Pi only)
pip install RPi.GPIO spidev
```

### Octave/MATLAB
```bash
# Install Octave (free, open-source)
sudo apt install octave
```

### CloudCompare
```
Download: https://www.danielgm.net/cc/
Free, open-source 3D point cloud processing
```

---

## 📝 Version History

### v2.4 (Current)
- ✅ 17-field telemetry protocol
- ✅ Battery voltage monitoring
- ✅ GPS satellite count display
- ✅ Optimized PyQtGraph plots
- ✅ Fast OSM tile map
- ✅ Complete analysis suite
- ✅ Signal processing tutorials
- ✅ Point cloud processing pipeline
- ❌ SD card logging (not yet implemented)

### v2.3
- ✅ LIDAR integration
- ✅ Flight phase flags

### v2.2
- ✅ GPS satellite count added
- ✅ Calibration packet

### v2.1
- ✅ Initial LoRa telemetry
- ✅ Basic sensor fusion
- ✅ FSM implementation

---

## ⚠️ Important Notes

**Current Limitations:**
- ❌ SD card logging **NOT IMPLEMENTED** in main.c
- ❌ TaskSDCard is empty (placeholder)
- ✅ All other features working

**Before Flight:**
- ✅ Test LoRa communication
- ✅ Verify GPS lock
- ✅ Check battery voltage
- ✅ Calibrate sensors
- ✅ Test real-time GUI
- ❌ Cannot log to SD card yet (future feature)

**Data Analysis:**
- ✅ All tools tested with synthetic data
- ✅ Point cloud pipeline validated
- ✅ CloudCompare export working
- ✅ Octave tutorials functional
- ✅ GUI tested on Raspberry Pi 5

---

## 🎯 Future Enhancements

**High Priority:**
1. ⬜ Implement SD card logging (dual-file system)
2. ⬜ Add real-time LIDAR visualization in GUI
3. ⬜ Implement data compression

**Medium Priority:**
4. ⬜ Add second GPS for redundancy
5. ⬜ Implement parachute deployment detection
6. ⬜ Add temperature compensation for sensors
7. ⬜ Export to Google Earth from GUI

**Low Priority:**
8. ⬜ Machine learning for anomaly detection
9. ⬜ Predictive landing zone calculation
10. ⬜ Real-time 3D point cloud streaming

---

## 📜 License

This project is provided as-is for educational and research purposes.

---

## 🎓 Educational Use

This system is designed for educational purposes and includes:
- Complete signal processing tutorials
- Physics-based analysis examples
- Octave/MATLAB learning materials
- Real-world sensor fusion examples
- Real-time visualization techniques

Perfect for:
- Engineering students
- CanSat competitions
- Rocketry projects
- Drone development
- Signal processing education
- GUI development learning

---

*Built with ❤️ for the Cansat*

---

**README.md Version:** 2.4.0  
**Last Updated:** 2025-04-22  
**System Status:** Operational (SD card pending)

---

**END OF DOCUMENTATION**