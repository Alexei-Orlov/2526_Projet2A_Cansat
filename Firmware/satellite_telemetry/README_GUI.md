# 🛰️ CanSat Vortex — Ground Segment & Data Pipeline

**Real-time ground station GUI, LoRa telemetry protocol, onboard data logging and post-flight analysis suite for the ENSEA CanSat Vortex (2025–2026).**

![Version](https://img.shields.io/badge/Version-2.4-blue) ![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%205-red) ![Python](https://img.shields.io/badge/Python-3.7+-green) ![License](https://img.shields.io/badge/License-Educational-yellow) ![Protocol](https://img.shields.io/badge/Protocol-v2.4%20(17%20fields)-orange) ![MCU](https://img.shields.io/badge/MCU-STM32G431CBU6-blue) ![IDE](https://img.shields.io/badge/IDE-STM32CubeIDE-yellow)

---

## 📋 Table of Contents
1. [System Overview](#-system-overview)
2. [Features](#-features)
3. [Hardware Components](#-hardware-components)
4. [Telemetry Protocol](#-telemetry-protocol)
5. [Project Structure](#-project-structure)
6. [Installation](#-installation)
7. [Usage](#-usage)
8. [Transmitter (STM32 Flight Software)](#%EF%B8%8F-transmitter-stm32-flight-software)
9. [Receiver GUI (Raspberry Pi)](#-receiver-gui-raspberry-pi)
10. [Post-Flight Telemetry Analysis](#-post-flight-telemetry-analysis)
11. [Signal Processing Tutorials](#-signal-processing-tutorials)
12. [3D LIDAR Point Cloud Processing](#%EF%B8%8F-3d-lidar-point-cloud-processing)
13. [Troubleshooting](#-troubleshooting)
14. [Known Issues & Bug Fixes](#-known-issues--bug-fixes)
15. [Performance Metrics](#-performance-metrics)
16. [Version History](#-version-history)

---

## 🎯 System Overview

The Vortex data chain covers the full mission lifecycle:
- **Real-time LoRa telemetry downlink** at 869.53 MHz (custom v2.4 protocol)
- **High-rate onboard SD logging** — dual-file scheme: full telemetry (`DATA_xxx.CSV`, 10 Hz) and LIDAR georeferencing stream (`LIDA_xxx.CSV`, up to 50 Hz)
- **Full sensor fusion** (BNO055 IMU, SAM-M10Q GNSS, BMP581 barometer, LW20/C LIDAR)
- **Real-time GUI visualization** with a 3D satellite model, live map and telemetry plots
- **Comprehensive post-flight analysis suite** (flight reports, physics analysis, anomaly detection)
- **3D point cloud reconstruction** of the overflown terrain from the LIDAR + attitude + position data
- **Signal processing tutorials** for educational purposes

### System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    SATELLITE (STM32G431CBU6)                    │
├─────────────────────────────────────────────────────────────────┤
│  Sensors:                                                       │
│    • BNO055 9-axis IMU (accel, gyro, fused Euler/quaternion)    │
│    • BMP581 barometer (altitude, temperature)                   │
│    • SAM-M10Q GNSS (position, MSL altitude, 10 Hz)              │
│    • LightWare LW20/C LIDAR (distance, up to 50 Hz)             │
│    • Battery voltage (ADC1 + divider)                           │
│                                                                 │
│  Communication:                                                 │
│    • LoRa SX1276 @ 869.53 MHz → Ground Station                  │
│    • HMI PCB (I2C): SSD1306 OLED status screen + button         │
│                                                                 │
│  Storage (FATFS over SPI, implemented):                         │
│    • LIDA_xxx.CSV — LIDAR + attitude + position (up to 50 Hz)   │
│    • DATA_xxx.CSV — full telemetry snapshot (10 Hz)             │
└─────────────────────────────────────────────────────────────────┘
                              ↓ LoRa
┌─────────────────────────────────────────────────────────────────┐
│              GROUND STATION (Raspberry Pi 5)                    │
├─────────────────────────────────────────────────────────────────┤
│  Hardware:                                                      │
│    • LoRa SX1276 receiver @ 869.53 MHz (SPI)                    │
│                                                                 │
│  Real-Time GUI (Firmware/satellite_telemetry/GUI/):             │
│    • main.py               — application launcher               │
│    • gui_main_window.py    — main telemetry window              │
│    • data_plotter.py       — real-time plots (3 graphs)         │
│    • satellite_3d.py       — 3D STL model visualization         │
│    • map_view.py           — OSM tile map                       │
│    • lora_data_handler.py  — LoRa reception & packet parsing    │
│    • Manual CSV export of the received telemetry                │
└─────────────────────────────────────────────────────────────────┘
                              ↓ post-flight (SD card + ground log)
┌─────────────────────────────────────────────────────────────────┐
│              POST-FLIGHT ANALYSIS SUITE (3D/)                   │
├─────────────────────────────────────────────────────────────────┤
│  Point cloud pipeline (root of 3D/):                            │
│    • lidar_common.py          — shared loading/cleaning/georef  │
│    • cloudPoints.py           — 3D terrain point cloud          │
│    • distanceDistribution.py  — LIDAR range histogram           │
│    • generate_sample_data.py  — synthetic LIDA generator        │
│                                                                 │
│  Telemetry analysis (3D/Analysis_Tools/):                       │
│    • Basic/analyze_telemetry.py      — 10-panel dashboard       │
│    • Advanced/advanced_analysis.py   — rotation/GPS/energy/KML  │
│    • Advanced/advanced_features.py   — Cd, parachute, battery   │
│    • Signal_Processing/…             — educational package      │
└─────────────────────────────────────────────────────────────────┘
```

---

## ✨ Features

### Real-Time Ground Station GUI
- **Real-time data plotting**: vertical speed, temperature and altitude
- **3D satellite visualization**: custom STL model rotating with the incoming IMU Euler angles
- **2D map tracking**: fast OSM tile map — tiles are fetched once, position updates are instant
- **LoRa reception**: direct radio link using an SX127x module on the Raspberry Pi 5 SPI bus
- **Resizable layout**: adjustable splitters to customize panel sizes
- **Flight phase indicators**: visual flags for GO FOR LAUNCH, ASCENSION, DROP and RECOVERY
- **Battery monitoring**: real-time battery percentage with 12-sample moving-average smoothing
- **GNSS satellite count**: color-coded fix-quality display
- **Manual CSV export**: one-click export of the whole received dataset with a timestamped filename

### Post-Flight Analysis
- **Comprehensive flight reports**: multi-panel dashboards with per-phase statistics
- **Advanced physics analysis**: drag coefficient, energy budget, rotation/tumbling detection
- **Automatic anomaly detection**: data gaps, GPS jumps, accelerometer saturation, temperature spikes
- **Signal processing education**: low-pass filtering, FFT analysis, Kalman filtering (Python + Octave)
- **3D point cloud generation**: LIDAR + attitude + position fusion for terrain mapping
- **Google Earth export**: KML flight trajectory

---

## 🔧 Hardware Components

### Transmitter (Satellite)
| Component | Model | Interface | Purpose |
|-----------|-------|-----------|---------|
| MCU | STM32G431CBU6 | — | Main flight computer (FreeRTOS) |
| IMU | BNO055 | I2C (DMA reads) | Orientation (Euler/quaternion), acceleration, gyro |
| Barometer | BMP581 | I2C | Altitude, temperature |
| GNSS | u-blox SAM-M10Q | UART | Position, MSL altitude, satellite count (10 Hz GGA) |
| LIDAR | LightWare LW20/C | UART | Distance to ground (up to 50 Hz) |
| LoRa | SX1276 | SPI | Telemetry downlink |
| SD Card | MicroSD + Molex 473092651 | SPI (FATFS) | Dual-file flight logging |
| HMI | Custom PCB + SSD1306 OLED | I2C | Pre-flight status screen, config button, addressable LEDs |
| Battery Monitor | Voltage divider | ADC1 | 2S LiPo voltage monitoring |

### Receiver (Ground Station)
| Component | Model | Interface | Purpose |
|-----------|-------|-----------|---------|
| Computer | Raspberry Pi 5 | — | Data reception & visualization |
| LoRa | SX1276 | SPI | Telemetry reception |
| Display | HDMI monitor | — | GUI display |

**LoRa wiring (Raspberry Pi 5):**
- VCC → 3.3V / GND → GND
- MISO → GPIO 9 / MOSI → GPIO 10 / SCK → GPIO 11
- NSS/CS → GPIO 8 / RESET → GPIO 21
- DIO0 → GPIO 22 / DIO1 → GPIO 23 / DIO2 → GPIO 24

---

## 📡 Telemetry Protocol

### Protocol Version: **v2.4** (17 fields)

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
- `CAL` — packet identifier
- `calibration_duration_ms` — time taken to acquire the reference pressure and calibrate the sensors (ms)
- `timestamp_ms` — STM32 tick when calibration completed (ms)

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
| 11 | altitude | m | Barometric height above the calibration reference |
| 12 | latitude | ° | GNSS latitude |
| 13 | longitude | ° | GNSS longitude |
| 14 | satellites | count | GNSS satellites in the fix |
| 15 | flags | bitmask | Flight phase flags (see below) |
| 16 | timestamp_ms | ms | STM32 tick at packet build time |
| 17 | battery_voltage | V | Battery voltage |

**Notes:**
- `vertical_speed` is **not transmitted**. The receiver derives it from consecutive altitude readings:
  ```python
  vertical_speed = (altitude_now - altitude_prev) / (time_now - time_prev)
  ```
- The GNSS altitude (`gnss_alt`) is **not part of the LoRa packet** — it is only appended to the onboard SD logs, where it provides a redundant altitude source for post-flight processing.

---

### Flight Phase Flags (Cumulative Bitmask)

Each state ORs its own bit on top of the bits of every state already passed through, so the **highest set bit identifies the current phase**:

| Bit | Value | Flag | Cumulative value | Phase |
|-----|-------|------|------------------|-------|
| 0 | 0x01 | GO_FOR_LAUNCH | `1` (0b0001) | Ready on the pad |
| 1 | 0x02 | ASCENSION | `3` (0b0011) | Ascending under the drone/rocket |
| 2 | 0x04 | DROP | `7` (0b0111) | Released, descending |
| 3 | 0x08 | RECOVERY | `15` (0b1111) | Landed, awaiting recovery |

`flags = 0` means the CanSat is still calibrating (or in STANDBY/CONFIG).

---

### LoRa Configuration

| Parameter | Value | Notes |
|-----------|-------|-------|
| Frequency | 869.53 MHz | EU ISM band |
| Spreading Factor | SF7 | Good range/data-rate trade-off |
| Bandwidth | 125 kHz | Standard BW |
| Coding Rate | CR4/8 | Maximum forward error correction |
| Sync Word | 0x12 | Private network |
| TX Power | +17 dBm | Maximum allowed |
| Preamble | 8 symbols | Standard |
| CRC | Enabled | Corrupted frames discarded at the receiver |

**Link Budget & Rate:**
- **Range:** ~5–10 km line of sight (validated by outdoor range tests)
- **Dispatch rate:** the FSM builds a telemetry snapshot every 100 ms (10 Hz) and queues it to both the LoRa task and the SD task
- **Effective downlink rate:** bounded by packet air time at SF7/CR4/8 (~300 ms for a ~120-byte frame), i.e. a few packets per second over the air — the SD card, however, logs the full 10 Hz stream

---

## 📁 Project Structure

Paths are given from the repository root.

```
2526_Projet2A_Cansat/
│
├── 📖 README.md                       # Project overview (missions, hardware, mechanics)
├── 📖 README_GUI.md                   # This file — ground segment & data pipeline
│
├── 💻 Firmware/satellite_telemetry/
│   │
│   ├── 📂 Cansat_V1/                  # STM32CubeIDE project — CURRENT flight firmware
│   │   ├── Core/
│   │   │   ├── Src/
│   │   │   │   ├── main.c             # FreeRTOS tasks, FSM, SD logging, telemetry
│   │   │   │   ├── bmp581.c           # Barometer driver
│   │   │   │   ├── imu.c              # BNO055 driver (DMA reads)
│   │   │   │   ├── sx1276.c           # LoRa driver
│   │   │   │   ├── gnss_reader.c      # NMEA parser (SAM-M10Q)
│   │   │   │   ├── lidar.c            # LW20/C driver
│   │   │   │   ├── hmi.c              # HMI PCB / OLED status screen
│   │   │   │   ├── ssd1306.c          # OLED display driver
│   │   │   │   ├── fatfs_sd.c         # Low-level SPI SD driver
│   │   │   │   └── File_Handling_RTOS.c  # FATFS file helpers
│   │   │   └── Inc/                   # Matching headers + cansat_core.h (FSM, thresholds)
│   │   └── FATFS/, Middlewares/       # FATFS + FreeRTOS (CubeMX-generated)
│   │
│   ├── 📂 Cansat_V3/                  # Next firmware iteration (in progress)
│   │
│   ├── 📂 GUI/                        # Ground station application (Raspberry Pi 5)
│   │   ├── main.py                    # Entry point (venv path prioritization)
│   │   ├── gui_main_window.py         # Main window: 3D view + map + plots + controls
│   │   ├── lora_data_handler.py       # SX127x interface, v2.4 packet parsing
│   │   ├── data_plotter.py            # PyQtGraph plots, numpy circular buffers
│   │   ├── satellite_3d.py            # OpenGL STL renderer
│   │   ├── map_view.py                # OSM tile map (PyQtGraph)
│   │   ├── Corps.stl                  # 3D model of the CanSat used by the viewer
│   │   ├── requirements.txt           # Python dependencies
│   │   └── SX127x/                    # LoRa library (external dependency)
│   │
│   └── 📂 Analysis_Tools/             # ⚠️ Legacy analysis scripts (superseded by 3D/)
│
├── 📂 3D/                             # Post-flight analysis & point cloud pipeline
│   ├── lidar_common.py                # Shared loading / cleaning / georeferencing
│   ├── cloudPoints.py                 # 3D terrain point cloud (main tool)
│   ├── distanceDistribution.py        # LIDAR range histogram
│   ├── generate_sample_data.py        # Synthetic LIDA_xxx.CSV generator
│   ├── COMMANDES.txt                  # Full CLI reference (French)
│   ├── requirements.txt               # pandas, numpy, scipy, plotly
│   └── Analysis_Tools/
│       ├── Basic/analyze_telemetry.py       # Telemetry dashboard + report
│       ├── Advanced/advanced_analysis.py    # Rotation, GPS, energy, anomalies, KML
│       ├── Advanced/advanced_features.py    # Cd, parachute detection, battery
│       └── Signal_Processing/signal_processing_tutorial.py
│
├── 📂 Hardware/                       # KiCad projects (Mainboard V1/V2/V3, HMI) + 3D modeling
├── 📂 IMG/                            # Pictures used by the documentation
├── 📂 backup data cansat/             # Real SD-card datasets from field tests
├── 📂 Reports/                        # Project reports (PDF)
└── 📄 logs.md                         # Session-by-session work log
```

---

## 🚀 Installation

### Prerequisites

- **Raspberry Pi 5** with Raspberry Pi OS
- **SX127x LoRa module** connected to the Raspberry Pi (SPI interface)
- Python 3.7 or higher
- Internet connection (for OSM map tiles on first load)

### Hardware Setup

1. **Connect the LoRa module to the Raspberry Pi 5** (see wiring table above).

2. **Enable SPI**:
   ```bash
   sudo raspi-config  # Interface Options → SPI → Enable
   sudo reboot
   ```

### Software Setup

```bash
sudo apt-get update
sudo apt-get install python3-pyqt5 python3-spidev
pip install -r Firmware/satellite_telemetry/GUI/requirements.txt
```

> **Note**: `python3-pyqt5.qtwebengine` is NOT needed — QtWebEngine has been removed entirely (see [Known Issues](#-known-issues--bug-fixes)).

---

## 🎮 Usage

### Running the Real-Time GUI

```bash
cd Firmware/satellite_telemetry/GUI
python main.py
# or with sudo for GPIO access:
sudo python main.py
```

### Interface Overview

**Top left**: 3D STL model rotating with roll/pitch/yaw.

**Bottom left**: OSM tile map. Tiles are fetched once at startup using 8 parallel threads and cached in memory; position updates only move PyQtGraph overlay items, with no tile work involved. The grid reloads only if the satellite leaves the current tile boundary. Blue polyline path + red dots per fix.

**Right**: vertical speed, temperature and altitude plots, plus the flight phase flag panel.

**Bottom**: Stop / Clear / Export-to-CSV buttons and the LoRa status indicator.

### CSV Export

The received telemetry can be exported at any time with the **Export to CSV** button, which proposes a timestamped filename (`telemetry_data_YYYYMMDD_HHMMSS.csv`) and dumps every buffered field.

### Early Development Version

The screenshots below show an early development version of the GUI, when the ground station was still fed over a USB serial link (COM port) and used a Leaflet web map. The current version replaces the serial link with direct LoRa SPI reception and the web map with the PyQtGraph OSM tile engine.

<div align="center">
  <img src="./IMG/groundstation_gui.png" alt="Early GUI (serial link version)" width="800"><br><br>
  <img src="./IMG/groundstation_gui_config.png" alt="Early serial port configuration dialog" width="350">
</div>

---

## 🖥️ Transmitter (STM32 Flight Software)

> The reference flight firmware is **`Firmware/satellite_telemetry/Cansat_V1/`**. `Cansat_V3/` is the next iteration, still in progress.

### Implementation Status

| Feature | Status | Notes |
|---------|--------|-------|
| IMU reading | ✅ Implemented | BNO055 via I2C, 100 Hz DMA reads (Euler + quaternion + accel + gyro) |
| Barometer reading | ✅ Implemented | BMP581 via I2C, polled at 20 Hz, thermal feed-forward compensation |
| GNSS reading | ✅ Implemented | SAM-M10Q configured at boot for 10 Hz GGA + airborne dynamic model |
| LIDAR reading | ✅ Implemented | LW20/C via UART, streaming up to 50 Hz |
| Battery monitoring | ✅ Implemented | ADC1 with voltage divider |
| LoRa transmission | ✅ Implemented | SX1276 at 869.53 MHz, v2.4 protocol |
| Flight state machine | ✅ Implemented | 7 states with debounced transitions |
| SD card logging | ✅ Implemented | FATFS over SPI, dual-file scheme with `f_sync()` batching |
| HMI status screen | ✅ Implemented | SSD1306 OLED + button, SD format/erase from the CONFIG state |

### RTOS Tasks (FreeRTOS)

The firmware follows a **unified snapshot architecture**: sensor data is gathered continuously, then packaged every 100 ms into a single `TelemetryPacket_t` dispatched simultaneously to the LoRa and SD queues (`qLoRa`, `qSDCard`, plus `qSensorEvents` for acquisition tickets).

1. **TaskFSM** — the master metronome. Runs the flight state machine, schedules sensor acquisition tickets in 100 Hz / 20 Hz / 10 Hz blocks, builds the unified telemetry snapshot at 10 Hz and drives the status LED.
2. **TaskSensors** (high priority) — the data gatherer. Consumes acquisition tickets and hardware events to read the BNO055 (DMA), BMP581, LW20/C stream and GNSS NMEA feed.
3. **TaskLoRa** (low priority) — the communicator. Serializes the snapshot into the v2.4 comma-separated frame and transmits it through the SX1276. Also broadcasts the startup calibration packet.
4. **TaskSDCard** (low priority) — the data logger. Mounts the card through FATFS, opens one `DATA_xxx.CSV` and one `LIDA_xxx.CSV` per session, and batches `f_sync()` calls so the high-rate LIDAR stream never stalls the RTOS.
5. **TaskHMI** — drives the OLED status screen (LoRa link, battery, SD state) and handles the config button.

### Flight State Machine

```
OFF ──power on──▶ STANDBY ──config plugged──▶ CONFIG ──unplugged──▶ READY
READY ──altitude > 10 m held 2 s──▶ ASCENSION
ASCENSION ──LIDAR > 1 m & altitude > 30 m held 1 s──▶ DROP
DROP ──altitude < 5 m & |gyro| < 30 °/s held 3 s──▶ RECOVERY (buzzer on)
```

**Thresholds** (`cansat_core.h`):
```c
#define THRESH_LIDAR_DEPLOYED_M         1.0f   // Min LIDAR distance to confirm deployment out of the box
#define THRESH_ALTITUDE_DEPLOY_MIN_M    30.0f  // Min altitude for DROP detection (release nominally at 120 m)
#define THRESH_ALTITUDE_ASCENSION_M     10.0f  // Min altitude to trigger ASCENSION
#define THRESH_ASCENSION_HOLD_MS        2000   // Condition must hold before READY → ASCENSION
#define THRESH_DROP_HOLD_MS             1000   // Condition must hold before ASCENSION → DROP
#define THRESH_ALTITUDE_LANDING_M       5.0f   // Max altitude to trigger RECOVERY
#define THRESH_GYRO_STILL_DPS           30.0f  // Max |gyro| per axis to consider the CanSat motionless
#define THRESH_LANDING_HOLD_MS          3000   // Condition must hold before DROP → RECOVERY
```

Transitions are **debounced**: each condition must hold for its full window at 20 Hz, with a bounded number of outlier samples tolerated (e.g. 4 outliers out of 40 samples for the ascension window), which makes the FSM robust to single noisy readings.

### Battery Monitoring

**Circuit:**
```
Battery (7.4V) → R1 (20kΩ) → ADC1 ← R2 (10kΩ) → GND
```

**Voltage divider ratio:** 3.0× (10 kΩ / (20 kΩ + 10 kΩ))

**ADC reading:**
```c
HAL_ADC_Start(&hadc1);
if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
    uint32_t raw_adc = HAL_ADC_GetValue(&hadc1);
    vbat = ((float)raw_adc / 4095.0f) * 3.3f * 3.0f * 1.025f;
    // 1.025 is an empirical calibration factor
}
HAL_ADC_Stop(&hadc1);
```

### SD Card Logging (Dual-File Scheme)

At boot, the firmware scans the card for existing sessions and opens the next free pair of files (`DATA_%03d.CSV` / `LIDA_%03d.CSV`). Both files stay open for the whole flight; `f_sync()` is called after each write burst so the data is committed to the flash even in case of a hard landing. A full format/erase can be triggered from the CONFIG state through the HMI button.

#### File 1: LIDA_xxx.CSV (high rate, up to 50 Hz — feeds the point cloud pipeline)
```csv
tx_timestamp_ms,distance,roll,pitch,yaw,quat_w,quat_x,quat_y,quat_z,accel_x,accel_y,accel_z,latitude,longitude,altitude,flags_raw,gnss_alt
```

#### File 2: DATA_xxx.CSV (full telemetry, 10 Hz — feeds the analysis dashboard)
```csv
tx_timestamp_ms,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,roll,pitch,yaw,temperature,altitude,latitude,longitude,satellites,flags_raw,battery_voltage,gnss_alt
```

**Notes:**
- `gnss_alt` (GNSS MSL altitude) was appended last so that files from older firmware remain readable — the analysis tools detect it dynamically.
- Quaternions in `LIDA_xxx.CSV` give the point cloud pipeline a gimbal-lock-free attitude source; rows logged before the BNO055 fusion converges contain a null quaternion and are discarded during cleaning.
- Real flight/test datasets recorded with this scheme are archived in `backup data cansat/`.

---

## 📻 Receiver GUI (Raspberry Pi)

### Software Components

#### 1. main.py
**Application entry point**

- venv path prioritization (fixes PyQtGraph version conflicts, see [Known Issues](#-known-issues--bug-fixes))
- Error handling and graceful shutdown
- Resource cleanup on exit

---

#### 2. gui_main_window.py
**Main telemetry display window**

- 4-panel resizable layout (3D view, map, plots, controls)
- LoRa receiver lifecycle management
- Flight phase flag mapping to the plot panel
- Manual CSV export with timestamped filename
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
│                │   Export/Status)   │
└─────────────────────────────────────┘
```

---

#### 3. lora_data_handler.py
**LoRa receiver integration**

- SX127x hardware interface (frequency, SF, BW, CR, sync word matching the transmitter)
- Packet type detection (CAL vs telemetry)
- 17-field v2.4 packet parsing, with backwards compatibility down to the 14-field v2.1 format
- GNSS coordinate sanity check (out-of-range lat/lon rejected)
- Battery voltage and satellite count extraction
- RSSI / SNR / frequency-error monitoring
- Qt signal emission towards the GUI thread

**Usage:**
```python
from lora_data_handler import LoRaDataReceiver

receiver = LoRaDataReceiver(verbose=True)
receiver.data_received.connect(on_data_callback)
receiver.configure_and_start()
```

---

#### 4. data_plotter.py
**Real-time sensor data plotting**

- Optimized PyQtGraph plots backed by numpy circular buffers (10,000 points)
- 3 real-time plots:
  1. **Vertical speed** — derived from altitude deltas (no accelerometer drift)
  2. **Temperature** — barometer temperature (°C)
  3. **Altitude** — barometric height (m)
- Flight phase panel driven by the telemetry bitmask (GO FOR LAUNCH / ASCENSION / DROP / RECOVERY)
- Battery percentage with 12-sample moving-average smoothing
- GNSS satellite count with color coding
- Auto-scaling axes and grid lines
- CSV export of the full buffered dataset

---

#### 5. satellite_3d.py
**3D satellite model visualization**

- STL loader (binary and ASCII) reading `Corps.stl`
- Optimized OpenGL rendering with lighting and perspective camera
- Real-time rotation from the incoming roll/pitch/yaw angles

---

#### 6. map_view.py
**Interactive map widget with OSM tiles**

- OpenStreetMap tile map rendered with PyQtGraph `ImageItem`s
- Async tile loading with a `ThreadPoolExecutor` (8 workers), in-memory cache
- Tile grid pre-fetched at startup and lazily expanded when the satellite leaves the boundary
- Real-time GNSS tracking: blue polyline path, red dots per fix, marker at the current position
- Position updates are O(1) `setData` calls — zero network or compositing work

**Performance:**
- First load: ~2–3 seconds (tile grid fetch)
- Position updates: <1 ms
- Tile cache: kept for the whole session

---

## 📊 Post-Flight Telemetry Analysis

The analysis suite lives in **`3D/Analysis_Tools/`** and operates on the onboard `DATA_xxx.CSV` files (it shares the schema definitions of `3D/lidar_common.py`, so it automatically follows the firmware CSV header). A full CLI reference is kept in [`3D/COMMANDES.txt`](./3D/COMMANDES.txt).

### Quick Reference

| Tool | Purpose | Input | Output |
|------|---------|-------|--------|
| `Basic/analyze_telemetry.py` | Flight dashboard + report | DATA_xxx.CSV | 10-panel PNG + text report |
| `Advanced/advanced_analysis.py` | Rotation, GPS, energy, anomalies | DATA_xxx.CSV | 3 PNG + KML + report |
| `Advanced/advanced_features.py` | Physics: Cd, parachute, battery | DATA_xxx.CSV | 3 PNG analyses |
| `Signal_Processing/signal_processing_tutorial.py` | Educational package | flight CSV (legacy schema) | Python plots + 4 Octave scripts |

All commands below are run from the `3D/` directory.

---

### 1. analyze_telemetry.py — Flight Dashboard

Produces a 10-subplot dashboard (accelerometer, gyroscope, orientation, temperature, altitude, vertical speed, latitude, longitude, satellites, battery), each with colored bands per flight phase, plus a text report (overview, altitude, speed, GPS, battery, temperature and per-phase durations).

```bash
python Analysis_Tools\Basic\analyze_telemetry.py DATA_001.CSV                       # interactive display
python Analysis_Tools\Basic\analyze_telemetry.py DATA_001.CSV --output ./report     # save PNG + report
python Analysis_Tools\Basic\analyze_telemetry.py DATA_001.CSV --plots altitude,battery
python Analysis_Tools\Basic\analyze_telemetry.py DATA_001.CSV --list-plots          # list available plot keys
```

**Output:** `DATA_001_analysis.png` + `DATA_001_report.txt`

---

### 2. advanced_analysis.py — Rotation, GPS, Energy, Anomalies, KML

```bash
python Analysis_Tools\Advanced\advanced_analysis.py DATA_001.CSV [--output ./out]
```

**Output:**
```
rotation_analysis.png    # Gyro X/Y/Z + rotation magnitude (tumbling detection)
gps_trajectory.png       # Top view + altitude vs horizontal distance (skipped if no fix)
energy_analysis.png      # Potential + kinetic + total energy over time
flight_trajectory.kml    # Google Earth export (skipped if no fix)
analysis_report.txt      # Statistics + detected anomalies
```

**Automatic anomaly detection:**
- Data gaps > 3× the median sample interval
- GPS jumps > ~110 m between consecutive rows
- Accelerometer saturation > 15 m/s²
- Temperature spikes > 5 °C between consecutive rows

---

### 3. advanced_features.py — Physics-Based Analysis

```bash
python Analysis_Tools\Advanced\advanced_features.py DATA_001.CSV --mass 0.35 --area 0.008 [--output ./out]
```

**Output:**
```
drag_coefficient.png      # DROP-phase altitude, vertical speed, speed vs altitude, Cd figures
parachute_deployment.png  # Deployment instant detection (altitude, v_speed, vertical accel)
power_consumption.png     # Battery voltage + estimated remaining capacity + per-phase summary
```

**Method notes:**
- Drag coefficient: `Cd = 2mg / (ρ · v_terminal² · A)` — pass the real mass (`--mass`, kg) and cross-section (`--area`, m²)
- Parachute deployment is detected as the maximum upward jerk in vertical acceleration

---

## 🎓 Signal Processing Tutorials

### signal_processing_tutorial.py

**Educational package teaching signal processing with real flight data.**

> ⚠️ This script still expects the legacy ground-station schema with a `timestamp` column in seconds; it is not directly compatible with the onboard `DATA_xxx.CSV` files (`tx_timestamp_ms`). It is kept as a pedagogical reference.

```bash
python Analysis_Tools\Signal_Processing\signal_processing_tutorial.py flight.csv [output_folder]
```

**Output:**
```
signal_processing_output/
├── 1_lowpass_temperature.png       # Low-pass filter on temperature
├── 1_lowpass_gps.png               # Low-pass filter on GPS
├── 2_fft_gyroscope.png             # Gyroscope frequency analysis
├── 2_fft_accelerometer.png         # Accelerometer frequency analysis
├── telemetry_data.mat              # MATLAB/Octave export
├── octave_tutorial_1_basics.m      # Loading & plotting
├── octave_tutorial_2_filtering.m   # Filter design
├── octave_tutorial_3_fft.m         # Frequency analysis
├── octave_tutorial_4_kalman.m      # Kalman filter
└── README_OCTAVE.txt               # Octave instructions
```

**Key concepts covered:**

- **Low-pass filtering** — removing high-frequency noise (vibrations, ADC jitter) from temperature or GPS tracks
- **FFT analysis** — extracting spin rates and vibration frequencies from the gyro/accelerometer
- **Kalman filtering** — optimal fusion of noisy measurements with a physics model (GPS smoothing, sensor fusion)

---

## 🗺️ 3D LIDAR Point Cloud Processing

This is the heart of **Mission 3 (ground study)**: as the CanSat descends, the helicoidal body induces a spin, so the LW20/C laser sweeps the terrain in a spiral pattern. Post-flight, each range sample is georeferenced by fusing it with the attitude and position logged in `LIDA_xxx.CSV`, producing a 3D point cloud of the overflown terrain.

The pipeline lives at the root of **`3D/`**. A full CLI reference (in French) is kept in [`3D/COMMANDES.txt`](./3D/COMMANDES.txt).

> The earlier prototype (`Firmware/satellite_telemetry/Analysis_Tools/Point_Cloud/`: `merge_point_cloud.py`, `generate_synthetic_data.py`) is **legacy** and superseded by this pipeline.

### Georeferencing Model

```
Ground_Point = CanSat_Position + LIDAR_Distance × R(attitude) · boresight

Where:
  CanSat_Position = local ENU coordinates from (latitude, longitude, altitude)
  R(attitude)     = body→world rotation from the logged quaternion
                    (Euler-angle fallback for older, quat-less logs)
  boresight       = LIDAR beam direction in the body frame (default: lidar_mount)
```

### Processing Steps (`lidar_common.py`)

1. **Loading** — reads `LIDA_xxx.CSV`, detecting the schema dynamically (quaternion columns and `gnss_alt` are optional, so older logs stay compatible).
2. **Cleaning** — drops NaNs, samples outside the sensor's rated range gate (0.2–100 m by default) and rows with a null quaternion (logged before the BNO055 fusion converged).
3. **Phase filtering** — the cumulative `flags_raw` bitmask is decoded and, by default, only **DROP**-phase samples are kept.
4. **SLERP interpolation** — IMU attitude arrives in DMA bursts at a lower rate than the 50 Hz LIDAR stream; orientations are spherically interpolated (SLERP) between bursts so every range sample gets a smooth attitude estimate.
5. **Georeferencing** — lat/lon are projected to local metric ENU coordinates around the first fix; the vertical translation uses the barometric altitude by default, or the GNSS altitude with `--alt-source gnss`.

### 1. cloudPoints.py — 3D Point Cloud (main tool)

```bash
python cloudPoints.py -i LIDA_001.CSV                          # default: DROP phase, colored by distance
python cloudPoints.py -i LIDA_001.CSV --color-by time          # color by elapsed time
python cloudPoints.py -i LIDA_001.CSV --alt-source gnss        # GNSS altitude for the vertical axis
python cloudPoints.py -i LIDA_001.CSV --ignore-gps             # noisy fix: keep only the altitude translation
python cloudPoints.py -i LIDA_001.CSV --ignore-gps --ignore-baro  # rotation-only cloud
python cloudPoints.py -i LIDA_001.CSV --output-html cloud.html --export-xyz points.xyz --no-show
```

**Main options:**

| Option | Default | Description |
|--------|---------|-------------|
| `--phases` | `drop` | Flight phase(s) to keep; pass nothing to keep all |
| `--min-range` / `--max-range` | 0.2 / 100 m | LW20/C rated range gate |
| `--boresight` | `lidar_mount` | Beam direction in the body frame (`forward`, `down`, …) |
| `--color-by` | `distance` | `distance`, `time`, or any CSV column (e.g. `altitude`) |
| `--alt-source` | `baro` | Vertical translation source: `baro` or `gnss` (newer logs) |
| `--ignore-gps` / `--ignore-baro` | off | Disable the horizontal / vertical translation |
| `--no-slerp` | off | Use raw IMU snapshots instead of SLERP interpolation |
| `--no-trajectory` | off | Hide the CanSat descent trajectory overlay |
| `--output-html` / `--export-xyz` | — | Save the interactive Plotly figure / export a plain `x y z` file |

The interactive Plotly view shows the georeferenced point cloud together with the CanSat trajectory, and prints the reference point and per-quartile distance statistics.

### 2. distanceDistribution.py — Range Histogram

Quick sanity check of a flight log before building the cloud: histogram + box plot of the measured distances, filterable by phase.

```bash
python distanceDistribution.py -i LIDA_001.CSV --phases drop [--nbins 100] [--output-html dist.html]
```

### 3. generate_sample_data.py — Synthetic Test Data

Generates a realistic synthetic `LIDA_xxx.CSV` (pad → ascension → tumbling drop → recovery) for developing and testing the pipeline without a flight.

```bash
python generate_sample_data.py --output LIDA_test.csv --seed 42
python cloudPoints.py -i LIDA_test.csv --phases drop
```

### CloudCompare Workflow

The `.xyz` export can be post-processed in [CloudCompare](https://www.danielgm.net/cc/) (free, open source):

1. **Import**: `File → Open → points.xyz` (ASCII, space-separated, X/Y/Z columns)
2. **Colorize by height**: `Edit → Colors → Height Ramp`
3. **Export**: `.las`/`.laz` (LiDAR standard), `.ply`, `.obj`

### Typical Post-Flight Workflow

```bash
# 1. Sanity-check the LIDAR ranges
python distanceDistribution.py -i LIDA_001.CSV --phases drop

# 2. Build the point cloud (default: DROP phase, colored by distance)
python cloudPoints.py -i LIDA_001.CSV

# 3. Inspect the temporal structure of the spiral scan
python cloudPoints.py -i LIDA_001.CSV --color-by time

# 4. If the GNSS fix is noisy, fall back to altitude-only translation
python cloudPoints.py -i LIDA_001.CSV --ignore-gps

# 5. Telemetry dashboard and advanced analyses on the matching DATA file
python Analysis_Tools\Basic\analyze_telemetry.py DATA_001.CSV --output ./out
python Analysis_Tools\Advanced\advanced_analysis.py DATA_001.CSV --output ./out
python Analysis_Tools\Advanced\advanced_features.py DATA_001.CSV --mass 0.35 --area 0.008
```

---

## 🔧 Troubleshooting

### LoRa module not detected
- Verify SPI is enabled: `lsmod | grep spi`
- Check the GPIO wiring
- Try running with elevated privileges: `sudo python main.py`

### Map tiles slow to appear on first load
- This is normal — the tile grid is fetched at startup
- After that, position updates are instant
- The in-memory cache is reused for the whole session

### Invalid GPS coordinates
- Values outside lat [−90, 90] / lon [−180, 180] are rejected by the parser
- This indicates a transmitter-side GNSS issue, not a ground station bug
- Check the antenna and sky view

### GNSS: no fix
- A clear sky view is required
- Allow 2–3 minutes for a cold start (outdoor performance is significantly better than indoor)
- Check the antenna connection and the UART baud rate

### LIDAR: no data
- Check the UART baud rate (115200)
- Make sure streaming is active — the firmware re-arms it (`Lidar_ForceStream()`) when entering READY if no data arrived recently
- ⚠️ Do **not** send stray characters to a streaming LW20: they are interpreted as menu navigation and silently switch the streamed variable, which kills the `LIDA_xxx.CSV` log

### Analysis: script crashes
- Check that the Python dependencies are installed (`3D/requirements.txt`)
- Verify the CSV file is not corrupted and the header matches the expected schema
- Check for empty data after phase filtering (e.g. no DROP samples in a bench test)

---

## 🐛 Known Issues & Bug Fixes

### Fix 1 — PyQtGraph `drawLines` TypeError (`main.py`)
**Symptom**: `TypeError: arguments did not match any overloaded call: drawLines(...)`

**Cause**: system pyqtgraph 0.13.1 being loaded instead of the venv's 0.14.0.

**Fix**: `main.py` inserts the venv's site-packages at the front of `sys.path` before any imports.

---

### Fix 2 — QtWebEngine Chromium crash → removed entirely (`map_view.py`)
**Symptom**: `FATAL: page_allocator_internals_posix.h(169)] Check failed` / `Trace/breakpoint trap`

**Cause**: the Chromium renderer is incompatible with the Raspberry Pi 5 ARM kernel memory allocator. No flag combination (`--no-sandbox`, `--disable-gpu`, `--single-process`) resolved it.

**Fix**: QtWebEngine removed entirely. The map was reimplemented in PyQtGraph with urllib tile fetching.

---

### Fix 3 — Map too slow to update (`map_view.py`)
**Symptom**: map position updates were visibly laggy.

**Cause**: tiles were being fetched and composited on every `update_position()` call.

**Fix**: tile loading and overlay updates are now fully decoupled:
- `_load_tiles()` runs once at startup in a `ThreadPoolExecutor` with 8 workers
- `update_position()` only calls `_redraw_overlays()`, which moves PyQtGraph items — zero network or compositing work
- Tiles are cached in memory; the grid only reloads when the satellite leaves its boundary

---

### Fix 4 — Y-axis tile inversion (`map_view.py`)
**Symptom**: map tiles displayed upside-down.

**Cause**: OSM uses tile coordinates with Y increasing southward, but the PyQtGraph ImageItem expects Y increasing upward.

**Fix**: apply a Y-axis inversion when positioning the ImageItems:
```python
tile_y_inverted = (2**zoom - 1) - tile_y
```

---

### Fix 5 — Map slow to rebuild on GPS update (`map_view.py`)
**Symptom**: the map was slow to respond to incoming GPS positions.

**Cause**: the previous implementation rebuilt a numpy canvas on every GPS update — re-blitting all visible tiles into a single array each time.

**Fix**: tiles are now permanent `pg.ImageItem` objects placed once at fixed world coordinates and never moved or rebuilt. Each GPS update performs only three O(1) `setData` calls (polyline, dots, marker) and a viewport pan — zero tile work. The tile grid is pre-fetched asynchronously at startup and lazily expanded as the satellite moves.

---

## 📊 Performance Metrics

| Metric | Value | Notes |
|--------|-------|-------|
| Telemetry snapshot rate | 10 Hz | Unified packet dispatched to LoRa + SD queues |
| Effective LoRa downlink | a few packets/s | Bounded by air time at SF7 / CR4/8 (~300 ms per frame) |
| LIDAR logging rate | up to 50 Hz | `LIDA_xxx.CSV` on the SD card |
| IMU acquisition | 100 Hz | BNO055 via DMA |
| Barometer polling | 20 Hz | Sufficient for altitude; avoids starving the I2C bus |
| GNSS fix rate | 10 Hz | SAM-M10Q configured at boot (GGA) |
| LoRa range | 5–10 km | Line of sight |
| GUI update latency | <50 ms | As packets arrive |
| Map tile load | ~2–3 s | First load only |
| Map position update | <1 ms | After tiles are loaded |

---

## 📚 Additional Resources

### Python Dependencies
```bash
# Ground station GUI (Raspberry Pi)
pip install pyqt5 pyqtgraph pyopengl numpy-stl RPi.GPIO spidev

# Analysis & point cloud pipeline (any machine)
pip install -r 3D/requirements.txt    # pandas, numpy, scipy, plotly
pip install matplotlib                # for the Analysis_Tools dashboards
```

### Octave/MATLAB
```bash
sudo apt install octave   # free, open source — runs the tutorial scripts
```

### CloudCompare
```
https://www.danielgm.net/cc/ — free, open-source 3D point cloud processing
```

---

## 📝 Version History

### v2.4 (current)
- ✅ 17-field telemetry protocol with battery voltage and GNSS satellite count
- ✅ SD card logging implemented (dual-file scheme, `f_sync()` batching, session numbering)
- ✅ GNSS altitude (`gnss_alt`) appended to both SD logs, 10 Hz GNSS fix rate
- ✅ Quaternion attitude logged in `LIDA_xxx.CSV` for the point cloud pipeline
- ✅ HMI OLED status screen + SD format from the CONFIG state
- ✅ New point cloud pipeline (`3D/`): SLERP interpolation, phase filtering, selectable altitude source
- ✅ Telemetry analysis suite adapted to the onboard `DATA_xxx.CSV` schema

### v2.3
- ✅ LIDAR integration
- ✅ Flight phase flags

### v2.2
- ✅ GNSS satellite count added
- ✅ Calibration packet

### v2.1
- ✅ Initial LoRa telemetry
- ✅ Basic sensor fusion
- ✅ FSM implementation

---

## ⚠️ Pre-Flight Checklist

- ✅ Test the LoRa link (calibration packet received, RSSI/SNR nominal)
- ✅ Verify the GNSS fix (satellite count on the GUI and on the HMI screen)
- ✅ Check the battery voltage
- ✅ Let the sensor calibration complete (CAL packet broadcast)
- ✅ Confirm the SD session files were created (HMI screen)
- ✅ Verify the LIDAR stream is active

---

## 📜 License

This project is provided as-is for educational and research purposes.

---

## 🎓 Educational Use

This system is designed for educational purposes and includes:
- Complete signal processing tutorials
- Physics-based analysis examples
- Octave/MATLAB learning material
- Real-world sensor fusion and georeferencing examples
- Real-time visualization techniques

Well suited for:
- Engineering students
- CanSat competitions
- Rocketry projects
- Drone development
- Signal processing education
- GUI development learning

---

*Built with ❤️ for the CanSat Vortex*

---

**README_GUI.md version:** 2.4.1
**Last updated:** 2026-07-11
**System status:** Operational
