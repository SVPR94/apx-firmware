# APX Autopilot & Flight Control System

[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)
[![Standard](https://img.shields.io/badge/C%2B%2B-17-purple.svg)]()
[![Build System](https://img.shields.io/badge/Build-CMake_%26_Ninja-orange.svg)]()
[![CI/CD](https://github.com/SVPR94/apx-firmware/actions/workflows/firmware_ci_cd.yml/badge.svg)](https://github.com/SVPR94/apx-firmware/actions/workflows/firmware_ci_cd.yml)
[![Simulation](https://img.shields.io/badge/SITL-Gazebo_Harmonic_%26_RK4-green.svg)]()
[![Protocol](https://img.shields.io/badge/GCS-MAVLink_v2-brightgreen.svg)]()

**APX** is a modular, aerospace-grade flight controller and simulation ecosystem designed from first principles. Built for mission-critical reliability, APX delivers modern **C++17** software engineering, a **lock-free pub/sub IPC (`uBus`)**, **4-step redundant sensor voting**, an **aerospace 24-state EKF3 sensor fusion engine**, and specialized capabilities for **commercial agricultural multirotors** (Quad X/+, Hex X/Flat, Octo X/Coaxial).

APX is free of copyleft restrictions (licensed under **Apache 2.0**) and runs across:
- **Software-In-The-Loop (SITL)** with Gazebo Harmonic (3D physics & visual indicators) or built-in 6DOF RK4 physics.
- **Embedded Linux** companion platforms (Raspberry Pi, Jetson, Radxa).
- **STM32H7 Series Microcontrollers** (ARM Cortex-M7) using the Zephyr RTOS or bare-metal HAL.

---

## Table of Contents
1. [Key Architecture Features](#key-architecture-features)
2. [Prerequisites & Environment Setup](#prerequisites--environment-setup)
3. [Building APX for Different Platforms](#building-apx-for-different-platforms)
   - [Platform 1: Linux Native & SITL](#platform-1-linux-native--sitl)
   - [Platform 2: Embedded Linux Companion Computers](#platform-2-embedded-linux-companion-computers)
   - [Platform 3: STM32H7 Cross-Compilation](#platform-3-stm32h7-cross-compilation)
   - [Firmware build utility (GUI + CLI)](#firmware-build-utility-gui--cli)
   - [Platform 4: Zephyr RTOS Integration](#platform-4-zephyr-rtos-integration)
4. [Command-Line Reference & Execution Guide](#command-line-reference--execution-guide)
   - [Unified SITL Vehicle Launcher (`sim_vehicle.py`)](#1-unified-sitl-vehicle-launcher-sim_vehiclepy)
   - [Radar Simulation & Terrain-Following (Gazebo & SITL)](#radar-simulation--terrain-following-gazebo--sitl)
   - [Flight Controller Daemon (`apx_flight_controller`)](#2-flight-controller-daemon-apx_flight_controller)
   - [Built-in 6DOF RK4 Physics Engine (`apx_sim_physics_runner`)](#3-built-in-6dof-rk4-physics-engine-apx_sim_physics_runner)
   - [Developer Utilities: Parameter CLI & MAVLink Console](#4-developer-utilities-parameter-cli--mavlink-console)
   - [Unified Test Suite Runner (`run_test_suite.py`)](#5-unified-test-suite-runner-run_test_suitepy)
5. [Connecting Ground Control Stations (GCS)](#connecting-ground-control-stations-gcs)
   - [Communication Ports & Telemetry](#communication-ports--telemetry)
   - [APX Flight Manager Setup](#apx-flight-manager-setup)
   - [Ground Control Station Setup](#mission-planner-setup)
6. [Supported Flight Modes & Navigation Logic](#supported-flight-modes--navigation-logic)
   - [Flight Mode Summary](#flight-mode-summary)
   - [Autonomous Navigation, Yaw & Cornering Logic](#autonomous-navigation-yaw--cornering-logic)
   - [Precision Land Mode & Manual Repositioning](#precision-land-mode--manual-repositioning)
   - [Agricultural Spraying & ZigZag Modes](#agricultural-spraying--zigzag-modes)
7. [Supported MAVLink Mission & Condition Commands](#supported-mavlink-mission--condition-commands)
8. [Autopilot Parameters Reference](#autopilot-parameters-reference)
   - [Navigation & Waypoint Parameters](#navigation--waypoint-parameters)
   - [Attitude, Rate & Heading (Yaw) Control](#attitude-rate--heading-yaw-control)
   - [Landing & Return to Launch (RTL)](#landing--return-to-launch-rtl)
   - [Throttle & Hover Position Control](#throttle--hover-position-control)
   - [Agricultural Crop Sprayer](#agricultural-crop-sprayer)
   - [Safety, Battery & Failsafe](#safety-battery--failsafe)
   - [RC Input & Channel Mapping](#rc-input--channel-mapping)
   - [Sensor Redundancy & Calibration](#sensor-redundancy--calibration)
9. [Pre-Flight Checks & Sensor Calibration](#pre-flight-checks--sensor-calibration)
10. [Repository Structure](#repository-structure)
11. [License](#license)

---

## Key Architecture Features

```
               ┌────────────────────────────────────────────────────────┐
               │              GCS (APX Flight Manager / GCS)                 │
               └───────────────────────────▲────────────────────────────┘
                                           │ MAVLink v2 (UDP / TCP / UART)
               ┌───────────────────────────▼────────────────────────────┐
               │                  apx::gcs::MavlinkHandler              │
               └───────────────────────────▲────────────────────────────┘
                                           │
  ┌────────────────────────────────────────┴────────────────────────────────────────┐
  │                                    APX uBus                                     │
  │                   Lock-Free Single-Producer Multi-Consumer IPC                  │
  └───────▲────────────────────────▲─────────────────────────▲──────────────────────┘
          │ Sensor Data            │ State Estimates         │ Actuator Outputs
┌─────────┴─────────────┐  ┌───────┴───────────────┐ ┌───────┴──────────────────────┐
│  apx::sensor::Manager │  │  apx::estimator::EKF3 │ │ apx::control::RateController │
│  - 4-way Voter        │  │  - 24-State Aerospace │ │ apx::control::AttitudeCtrl   │
│  - Pairwise Concord   │  │  - Bias & Wind Estim  │ │ apx::control::PositionCtrl   │
│  - Health Decay       │  │  - Dynamic Chi-Square │ │ apx::control::MotorMixer     │
└─────────▲─────────────┘  └───────▲───────────────┘ └───────▲──────────────────────┘
          │                        │                         │
┌─────────┴─────────────┐  ┌───────┴───────────────┐ ┌───────┴──────────────────────┐
│ Device Drivers (HAL)  │  │ Dual Filtering Engine │ │ Output Backends (DShot/PWM)  │
│ - ICM-42688/20689     │  │ - 2nd Order B-worth   │ │ - DShot 150/300/600/1200     │
│ - BMI088 / BMI270     │  │ - Dynamic RPM Notch   │ │ - 16-channel Servo / ESC     │
│ - F9P RTK / M9 / M10  │  │ - Alpha-Beta & Median │ │ - Spray Pump & Relay Drivers │
└───────────────────────┘  └───────────────────────┘ └──────────────────────────────┘
```

- **Lock-Free uBus IPC**: Zero-allocation, single-producer multi-consumer (SPMC) ring buffer topics for microsecond message passing without lock contention.
- **4-Level Sensor Redundancy**: Simultaneous support for up to 4 IMUs, 4 barometers, 4 magnetometers, and 4 GPS receivers with pairwise statistical concord voting and automatic on-the-fly failover.
- **Aerospace 24-State EKF3**: Full state vector tracking attitude quaternion, NED velocity, NED position, gyro/accel biases, horizontal/vertical wind components, barometer height bias, and dynamic magnetometer field estimation with chi-squared innovation gating.
- **Dual Filtering Pipeline**: Pre-EKF Butterworth 2nd-order low-pass filters paired with motor RPM-tracking harmonic notch filters to eradicate propeller vibrations before sensor fusion.
- **DJI-Style Guided Calibration**: Eliminates cumbersome 360-degree tumble calibration. Fast 1-click level calibration for accelerometers, stationary drift sampling for gyros, and guided 2-axis slow turns for magnetometers.
- **Agricultural Field Workflows**: Dedicated spraying mode (`AGRI_SPRAY`) with automatic flow adjustment based on ground speed and tank level, and autonomous field coverage (`AGRI_ZIGZAG`) with manual altitude and line-position overrides on the fly.
- **Live Wind Telemetry**: MAVLink `WIND (#168)` telemetry broadcast at 4 Hz with visual representation in Gazebo (animated airfield windsock and wind-swaying trees).

---

## Prerequisites & Environment Setup

### 1. Ubuntu / Debian Systems (Recommended)

Install the required core compilers, build tools, and Python dependencies:

```bash
# Core build essentials
sudo apt update
sudo apt install -y build-essential cmake ninja-build git pkg-config python3 python3-pip libeigen3-dev

# Python libraries for MAVLink telemetry, testing, and GCS scripting
pip3 install --user pymavlink pyserial numpy
```

### 2. Cross-Compilation Toolchain (for STM32H7 / Cortex-M7)

```bash
sudo apt install -y gcc-arm-none-eabi g++-arm-none-eabi libnewlib-arm-none-eabi gdb-multiarch openocd
```

### 3. Gazebo Simulation Suite (Optional, for Gazebo 3D Visual SITL)

APX supports **Gazebo Harmonic** (gz-sim8) and **Gazebo Garden**:

```bash
# Add Open Robotics repository for Gazebo Harmonic
sudo apt install -y lsb-release gnupg curl
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
sudo apt update
sudo apt install -y gz-harmonic
```

> **Note**: If you do not have Gazebo installed or are running on a headless machine / cloud instance without a GPU, APX includes a **built-in 6DOF Runge-Kutta 4 (RK4) physics engine** that requires no graphical environment.

---

## Building APX for Different Platforms

APX uses modern **CMake (>= 3.22)** with standardized **CMakePresets.json** targets.

### Platform 1: Linux Native & SITL

The SITL build runs the entire autopilot on your Linux PC. It includes the simulated sensor suite, the 24-state EKF3, flight mode logic, and bidirectional MAVLink interfaces.

```bash
# Option A: Using CMake Presets (Recommended)
cmake --preset sitl
cmake --build build/sitl -j$(nproc)

# Option B: Standard CMake commands
cmake -B build/sitl -GNinja -DAPX_BOARD=sitl -DAPX_BUILD_SITL=ON -DCMAKE_BUILD_TYPE=Debug
cmake --build build/sitl -j$(nproc)
```

The resulting binaries are located at:
- `build/sitl/main/apx_flight_controller` — Autopilot flight controller daemon
- `build/sitl/sitl/apx_sim_physics_runner` — Standalone 6DOF RK4 physics engine
- `build/sitl/sitl/gazebo_plugin/apx_gazebo_plugin.so` — Gazebo Harmonic multirotor plugin

---

### Platform 2: Embedded Linux Companion Computers

For single-board computers (Raspberry Pi 4/5, NVIDIA Jetson Orin/Nano, Radxa Rock) connected to flight peripherals over `/dev/spidev*`, `/dev/i2c-*`, or `/dev/ttyUSB*`:

```bash
cmake --preset linux-release
cmake --build build/linux-release -j$(nproc)
```

Run directly on hardware:
```bash
./build/linux-release/main/apx_flight_controller
```

---

### Firmware build utility (GUI + CLI)

`python3 tools/apx_build.py` opens a GUI (CLI: `python3 tools/apx_build.py --help`) that builds every target below on
Windows, Linux or macOS. You pick a **Board** and an **OS** (Zephyr RTOS, bare-metal, or host). The GUI covers
debug/release options, probe flashing and GDB server, build statistics with memory usage, and opening the binary
folder. See [tools/build_tools/README.md](tools/build_tools/README.md).

**Board diagram** (*Board diagram* tab). It shows the selected flight controller's architecture and redraws when you change boards. The main MCU is drawn as a chip in the middle, with every pin it uses on its edge, wired to a block per peripheral group. What's connected to each block is drawn next to it with an icon:
- USB and serial ports (TELEM1/2, GPS1/2, ESC telemetry, spare; TX/RX/RTS/CTS), with a ground station, telemetry radio, GPS or RC receiver card
- SPI and I2C buses with each sensor: IMU (gyro + accelerometer), barometer, compass, power monitor, FRAM, OSD, with its chip-select pin or I2C address
- CAN buses, and the microSD card on the SD bus
- Power and analog inputs: battery voltage and current, 5 V rail, RSSI, analog airspeed
- LEDs drawn in their real colour, the buzzer, the IMU heater and the power-rail switches
- Motor outputs, one motor (or servo) per pin, with its timer channel

A header line summarises the board (IMUs, barometers, compasses, serial ports, outputs, CAN, microSD) and says whether it has an IO MCU. On boards with an IO MCU, a second chip on the right shows the UART link and the IO MCU's own pins: MAIN 1–8 motors, RC in (PPM/SBUS), DSM in, SBUS out, the safety switch and LED, servo-rail and RSSI sensing. The APX H743 Pro has an IO MCU; the APX H743 v1 is the revision without one. The tab also has a sortable pin table, zoom and fit, and SVG export. From the command line: `python3 tools/apx_build.py diagram pixhawk6x --os zephyr --svg pixhawk6x.svg`. The pin data comes from each board's hardware description: `board_pins.json`, written by `tools/import_board.py`, or APX's own `hwdef.dat` (plus `iomcu.dat` for the IO MCU) for the APX boards.

**Bootloader options** (Zephyr boards, *Bootloader* group in the GUI). These settings go to the APX bootloader image
only; the firmware is unaffected.

| Option | CLI key | Default | Purpose |
|---|---|---|---|
| Build bootloader + combined image | `bootloader` | on | Builds the bootloader with the firmware and writes `firmware/<stem>_combined.hex` |
| Wait for GCS at power-up (ms) | `bl_boot_delay_ms` | 2000 | Listening window before the firmware starts |
| Stay after reboot-to-bootloader (s) | `bl_stay_timeout_s` | 30 | Wait for an upload after a MAVLink reboot-to-bootloader |
| Start firmware after idle (s) | `bl_idle_timeout_s` | 30 | Leave the bootloader once the ground station goes quiet |
| UART link baud rate | `bl_uart_baud` | 115200 | Boards whose bootloader link is a UART (APX H743 v1, Custom H7 FC) |
| USB vendor / product ID, product string | `bl_usb_vid`, `bl_usb_pid`, `bl_usb_product` | Zephyr defaults | USB boards; APX Flight Manager detects bootloaders by USB ID |
| Optimization | `bl_optimization` | size | `debug` (-Og) for stepping through the bootloader with a probe |
| Extra bootloader Kconfig | `bl_extra_kconfig` | - | Any other `CONFIG_...` line for the bootloader |
| Image to flash (*Debug probe* group) | `flash_image` | firmware | `firmware`, `combined` (first install) or `bootloader` over SWD or USB DFU |
| Debug probe backend (*Debug probe* group) | `probe` | OpenOCD + ST-Link | add `dfu` to flash over the STM32's built-in USB DFU (no probe; hold BOOT while plugging in) |

```bash
python3 tools/apx_build.py options apx_h743_v1 --os zephyr          # list every option and default
python3 tools/apx_build.py build apx_h743_v1 --os zephyr -s bl_boot_delay_ms=500 -s bl_uart_baud=57600
python3 tools/apx_build.py flash apx_h743_v1 --os zephyr -s flash_image=combined
```


### Platform 3: STM32H7 Cross-Compilation (Bare-Metal)

To compile firmware for high-performance flight management units powered by the **STM32H743** or **STM32H757** (Cortex-M7, hardware double-precision FPU):

```bash
# 1. Generic STM32H743 board (apx_h743_v1):
cmake --preset stm32h7
cmake --build build/stm32h7 --target apx_firmware_apx_h743_v1

# 2. CubePilot Cube Orange+ (STM32H757, 3 sensor lanes):
cmake --preset stm32h7-cube-orange-plus
cmake --build build/stm32h7_cube_orange_plus --target apx_firmware_cube_orange_plus
```

The build produces:
- `apx_firmware_<board>.elf` (Executable and debug symbols)
- `apx_firmware_<board>.bin` (Raw flash binary)
- `apx_firmware_<board>.hex` (Intel HEX format)
- `apx_cube_orange_plus.apj` (Bootloader-packaged JSON with board ID 1063)
- `apx_firmware_<board>.memory.txt` / `.memory.json` (memory analysis: region usage, image fit, top components and symbols — also printed as the last build step; the build fails if the image overflows flash or RAM)

#### Flashing STM32H7 Target via OpenOCD / ST-Link:
```bash
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg -c "program build/stm32h7/platforms/stm32h7/boards/apx_h743_v1/apx_firmware_apx_h743_v1.elf verify reset exit"
```

---

### Platform 4: Zephyr RTOS Integration

APX flight firmware for real flight controllers runs on **Zephyr RTOS v4.4.2**. It supports 21 boards:
- **Pixhawk-class:** CubePilot Cube Orange / Orange+, Holybro Pixhawk 6X / 6C and Durandal, SIYI N7, CUAV Nora / X7 / 7-Nano, ARK V6X.
- **FPV and wing STM32H743:** Matek H743, Holybro Kakute H7 / H7 Mini, MicoAir H743.
- **STM32F405 (1 MB flash):** Matek F405-Wing / F405-TE, SpeedyBee F405 V4 / F405 Wing.
- **APX's own designs:** APX H743 Pro / v1, Custom H7 FC.

The full table is in [platforms/zephyr/README.md](platforms/zephyr/README.md#boards). Most of these board files are generated from each board's hwdef hardware description by `tools/import_board.py`.

**IO coprocessor:** on boards with an IO MCU (Cube Orange / Orange+, Pixhawk 6X / 6C, Durandal, SIYI N7), the main outputs, RC input and safety switch are handled by the IO MCU over a 1.5 Mbaud UART. APX talks to it with its IO coprocessor driver ([drivers/iomcu](drivers/iomcu/README.md)): it detects the IO MCU, reports RC input and the safety switch to the ground station, and leaves the outputs at safe values. Boards without an IO MCU are unaffected. The flight code is OS-independent. It uses the APX OS abstraction (`kernel/ports/<os>/`) and a board interface implemented from each board's devicetree, so boards from other manufacturers need only board files, and Zephyr can later be replaced by FreeRTOS, ChibiOS or NuttX. See [docs/porting.md](docs/porting.md) and [platforms/zephyr/README.md](platforms/zephyr/README.md).

```bash
# 1. One-time setup: installs Zephyr workspace & pinned Arm toolchain
platforms/zephyr/setup_zephyr.sh

# 2. Build firmware + APX bootloader in one step (any board in the table, e.g. pixhawk6x, matek_h743, matek_f405_wing)
source platforms/zephyr/workspace/env.sh
west build --sysbuild -b cube_orange_plus/stm32h757xx/m7 -d build/zephyr_cube_orange_plus platforms/zephyr/app
```

The build outputs, all in `build/zephyr_cube_orange_plus/firmware/`:
- `apx_cube_orange_plus.apj`: firmware for an upload through the bootloader (board ID 1063)
- `apx_cube_orange_plus_combined.hex` / `.bin`: APX bootloader + firmware in one image (first install over SWD)
- `apx_cube_orange_plus_bootloader.{hex,bin,elf}` and `apx_cube_orange_plus.{hex,bin,elf}`: each image on its own
- `*_memory.txt`: memory analysis for both images, printed at the end of every build. The build fails if an image overflows its partition or RAM. To run it by hand: `python3 tools/fw_memory_report.py --elf <elf> --map <map> --bin <bin>`.

Whether a board needs the APX bootloader (Cube Orange+ already ships with a compatible one), and how to install it: [bootloader/README.md](bootloader/README.md).

#### Flashing through the board's bootloader:
1. Open **APX Flight Manager** or **Ground Control Station**.
2. Go to **Vehicle Setup -> Firmware** (or Initial Setup -> Install Firmware).
3. Connect the Cube's USB, select **Advanced settings -> Custom firmware file...**.
4. Choose `apx_cube_orange_plus.apj` to flash. The bootloader validates board ID `1063` and writes the image to `0x08020000`.

---

## Command-Line Reference & Execution Guide

APX provides a suite of command-line tools for running simulations, testing firmware, managing parameters, and monitoring telemetry.

### 1. Unified SITL Vehicle Launcher (`sim_vehicle.py`)

The primary simulation launcher script [`sitl/sim_vehicle.py`](sitl/sim_vehicle.py) orchestrates the autopilot binary, physics backend (Built-in RK4 or Gazebo), port verification, and telemetry bridge.

#### Example Usage

```bash
# 1. Quick start with built-in 6DOF RK4 physics (no Gazebo required)
python3 sitl/sim_vehicle.py --vehicle quad_x --sim builtin

# 2. Hexacopter in Gazebo 3D simulation with 7.5 m/s wind from West (270°) and gusts
python3 sitl/sim_vehicle.py --vehicle hex_x --sim gazebo --wind 7.5,270,1.5

# 3. Headless simulation for CI/CD or remote servers
python3 sitl/sim_vehicle.py --vehicle quad_x --sim builtin --headless

# 4. Multi-sensor stress test with 4 IMU lanes, 2 GPS receivers, harsh noise, and CSV logging
python3 sitl/sim_vehicle.py --vehicle quad_x --sim builtin \
    --sensor-lanes 4 \
    --gps-count 2 \
    --sensor-profile harsh \
    --sensor-log logs/sensor_benchmark.csv

# 5. Custom home location and simulation speedup
python3 sitl/sim_vehicle.py --vehicle quad_x --sim builtin \
    --lat 18.5204 --lon 73.8567 --alt 540.0 --speedup 2.0

# 6. Multi-radar simulation and terrain-following in Gazebo
python3 sitl/sim_vehicle.py --vehicle hex_x --sim gazebo --radar --wind 1.2,270,1.5
```

#### Parameter Reference for `sim_vehicle.py`

| Option | Type | Default | Choices / Format | Description & Meaning |
|---|---|---|---|---|
| `--vehicle` | String | `quad_x` | `quad_x`, `hex_x`, `octo_x` | Airframe layout and motor geometry matrix. |
| `--sim` | String | `builtin` | `builtin`, `gazebo` | Physics backend engine. `builtin` runs the native 1000 Hz RK4 solver; `gazebo` loads Gazebo Harmonic 3D. |
| `--radar` | Flag | `False` | Presence flag | Enables 10-channel multi-radar simulation, real-time 3D visual beams, ground target footprint projection, and terrain-following in APX firmware. |
| `--wind` | String | `None` | `SPEED,FROM_DEG[,GUST]` | Injects wind disturbance. `SPEED` in m/s, `FROM_DEG` bearing wind blows **from** (0°=N, 90°=E, 270°=W), optional `GUST` standard deviation in m/s (e.g. `8.0,270,1.5`). |
| `--sensor-lanes` | Integer | `4` | `1`, `2`, `3`, `4` | Number of concurrent IMU, Barometer, and Magnetometer lanes processed by the sensor voter. |
| `--gps-count` | Integer | `1` | `0`, `1`, `2`, `3`, `4` | Number of simulated GNSS receivers connected. |
| `--sensor-profile` | String | `realistic` | `ideal`, `realistic`, `harsh` | Sensor imperfection model: `ideal` (zero noise/drift), `realistic` (datasheet noise & thermal walk), `harsh` (high vibration & GNSS outages). |
| `--seed` | Integer | `1` | Any positive integer | Random seed for reproducible sensor noise, IMU bias random walk, and wind gusts. |
| `--sensor-log` | Path | `None` | File path (`.csv`) | Writes ground-truth and every individual sensor lane measurement to a CSV file for analysis. |
| `--lat` | Float | `18.5204` | Latitude (-90 to +90) | Home latitude origin in degrees. |
| `--lon` | Float | `73.8567` | Longitude (-180 to +180) | Home longitude origin in degrees. |
| `--alt` | Float | `540.0` | Meters | Home altitude Above Sea Level (MSL) in meters. |
| `--speedup` | Float | `1.0` | > 0.0 | Physics simulation speedup factor relative to real-time. |
| `--headless` | Flag | `False` | Presence flag | Runs simulation without graphical UI (server-only for Gazebo). |
| `--world` | Path | `None` | `.sdf` file path | Path to custom Gazebo world SDF file. |
| `--port` | Integer | `9003` | 1024 - 65535 | Internal UDP physics socket port connecting the simulator to the flight controller. |
| `--gcs-port` | Integer | `14550` | 1024 - 65535 | MAVLink UDP broadcast port for Ground Control Stations (APX Flight Manager / GCS). |
| `--tcp-ports` | String | `5760,5761,5762,5763` | Comma-separated | Dedicated TCP telemetry server ports for reliable GCS and developer connections. |

---

### Radar Simulation & Terrain-Following (Gazebo & SITL)

APX includes a complete multi-sensor radar fusion and terrain-following suite that runs identically across Gazebo simulation, the built-in 6DOF physics runner, and physical STM32H7 hardware (CAN and UART radars).

#### 1. Enabling Radar in Gazebo SITL

Launch your desired vehicle with the `--radar` flag:

```bash
# Launch hexacopter in Gazebo with multi-radar and terrain-following enabled
python3 sitl/sim_vehicle.py --vehicle hex_x --sim gazebo --radar

# Or quadcopter with wind disturbance and radar
python3 sitl/sim_vehicle.py --vehicle quad_x --sim gazebo --radar --wind 1.2,270,1.5
```

When `--radar` is passed:
1. **Gazebo Visual Feedback**:
   - **Downward Radar (Channel 1 / Down)**: Renders a **vibrant cyan 3D cylindrical beam** and a **70cm circular ground footprint target disk** projected on the terrain surface directly underneath the vehicle, plus a ground contact sphere.
   - **Perimeter Radars (Channels 2-9 / 360° Horizontal)**: Renders bright green laser rays and obstacle hit markers that dynamically change color (Green $\rightarrow$ Orange $\rightarrow$ Red) based on obstacle proximity.
   - **Ceiling Radar (Channel 10 / Up)**: Renders a purple upward beam and collision warning sphere.
2. **APX Firmware Activation**:
   - Automatically sets `RADAR_ENABLE=1` and `TERRAIN_ENABLE=1` on startup.
   - Streams simulated radar distances to the firmware via `feed_sitl_radar()`.
   - Broadcasts standard MAVLink `DISTANCE_SENSOR (#132)` at 10 Hz and `OBSTACLE_DISTANCE (#330)` at 5 Hz to GCS applications.

#### 2. Radar Channels & Orientations

| Channel | Identifier | Orientation | Direction Vector (FLU) | Primary Role |
|---|---|---|---|---|
| **1** | `RADAR1` | `25` (MAV_SENSOR_ROT_PITCH_270) | $[0, 0, -1]$ | **Downward Terrain-Following (Altimeter)** |
| **2** | `RADAR2` | `0` (MAV_SENSOR_ROT_NONE) | $[1, 0, 0]$ | Front Forward Obstacle Detection |
| **3** | `RADAR3` | `1` (MAV_SENSOR_ROT_YAW_45) | $[\frac{\sqrt{2}}{2}, -\frac{\sqrt{2}}{2}, 0]$ | Front-Right Corner Detection |
| **4** | `RADAR4` | `2` (MAV_SENSOR_ROT_YAW_90) | $[0, -1, 0]$ | Right Side Obstacle Detection |
| **5** | `RADAR5` | `3` (MAV_SENSOR_ROT_YAW_135) | $[-\frac{\sqrt{2}}{2}, -\frac{\sqrt{2}}{2}, 0]$ | Rear-Right Corner Detection |
| **6** | `RADAR6` | `4` (MAV_SENSOR_ROT_YAW_180) | $[-1, 0, 0]$ | Rear Backward Obstacle Detection |
| **7** | `RADAR7` | `5` (MAV_SENSOR_ROT_YAW_225) | $[-\frac{\sqrt{2}}{2}, \frac{\sqrt{2}}{2}, 0]$ | Rear-Left Corner Detection |
| **8** | `RADAR8` | `6` (MAV_SENSOR_ROT_YAW_270) | $[0, 1, 0]$ | Left Side Obstacle Detection |
| **9** | `RADAR9` | `7` (MAV_SENSOR_ROT_YAW_315) | $[\frac{\sqrt{2}}{2}, \frac{\sqrt{2}}{2}, 0]$ | Front-Left Corner Detection |
| **10** | `RADAR10` | `24` (MAV_SENSOR_ROT_PITCH_90) | $[0, 0, 1]$ | Top Ceiling Obstacle Detection |

#### 3. Autopilot Parameter Reference

| Parameter | Type | Default | Range | Description |
|---|---|---|---|---|
| `RADAR_ENABLE` | Int | `1` | `0` - `1` | Master switch for multi-radar driver and processing pipeline. |
| `RADAR_RATE` | Int | `20` | `1` - `100` | Radar update and publishing loop rate in Hz. |
| `RADAR_TIMEOUT_MS`| Int | `500` | `50` - `5000` | Sensor loss timeout before marking reading as invalid/failsafe. |
| `TERRAIN_ENABLE` | Int | `1` | `0` - `1` | Enables altitude setpoint tracking relative to ground level (AGL). |
| `TERRAIN_ALT_TGT`| Float | `2.0` | `0.5` - `50.0` | Desired terrain clearance in meters (m). |
| `TERRAIN_MIN_M` | Float | `2.0` | `0.2` - `15.0` | Minimum valid altimeter reading range in meters for terrain effect to take effect. |
| `TERRAIN_MAX_M` | Float | `30.0` | `5.0` - `100.0` | Maximum valid altimeter reading range in meters for terrain effect to take effect. |
| `TERRAIN_SRC` | Int | `0` | `0` - `9` | Radar channel index used for ground distance (0 = Downward Radar 1). |
| `TERRAIN_MAX_TILT`| Float | `35.0`| `10.0` - `60.0`| Maximum vehicle tilt angle in degrees before gating radar measurements. |
| `RADAR<N>_ENABLE`| Int | `1` | `0` - `1` | Individual enable switch for radar channel $N$ ($1 \le N \le 10$). |
| `RADAR<N>_MIN_CM`| Float | `200.0` / `20.0` | `2.0` - `500.0` | Minimum valid measurement distance in cm ($200$ cm = $2$ m for altimeter). |
| `RADAR<N>_MAX_CM`| Float | `3000.0` / `4000.0` | `100.0` - `10000.0` | Maximum valid measurement distance in cm ($3000$ cm = $30$ m for altimeter). |
| `RADAR<N>_POS_X` | Float | `0.0` | `-5.0` - `5.0` | Sensor mounting offset forward/back in meters (body frame). |
| `RADAR<N>_POS_Y` | Float | `0.0` | `-5.0` - `5.0` | Sensor mounting offset right/left in meters (body frame). |
| `RADAR<N>_POS_Z` | Float | `0.0` | `-5.0` - `5.0` | Sensor mounting offset down/up in meters (body frame). |

#### 4. Gazebo GPU Lidar Altimeter Range Visualizer

Both `apx_hex` and `apx_quad` models incorporate a downward-facing `<sensor name='gpu_lidar' type='gpu_lidar'>` attached to `lidar_frame` (`pitch = 90°` downward):
- **Horizontal Viewing Angle**: $50^\circ$ ($\pm 25^\circ = \pm 0.436332$ rad) with $640$ scan samples.
- **Vertical Viewing Angle**: $40^\circ$ ($\pm 20^\circ = \pm 0.349066$ rad) with $16$ scan samples.
- **Range Bounds**: $2.0\text{ m}$ to $30.0\text{ m}$.
- **Effect Gating**: The autopilot strictly gates terrain-following setpoints to only apply when measured ground clearance is within `[TERRAIN_MIN_M, TERRAIN_MAX_M]` ($2\text{ m}$ to $30\text{ m}$). Below $2\text{ m}$ (such as on the ground or during initial takeoff) or above $30\text{ m}$, `range_gated` activates, preventing unwanted setpoint adjustments outside the sensor's certified reading envelope.

#### 5. Monitoring in APX Flight Manager

1. Open **APX Flight Manager**.
2. Navigate to **Vehicle $\rightarrow$ Radar & Terrain**.
3. **Live 360° Radar Ring**: Displays a top-down polar representation of the vehicle with 8-direction perimeter obstacles, color-coded based on proximity.
4. **Terrain Following Panel**: View live ground clearance ($h_{\text{AGL}}$), configure target terrain altitude (`TERRAIN_ALT_TGT`), and toggle terrain lock with one click.
5. **Sensor Matrix (1-10)**: View live distance, signal health, and enable/disable individual radar channels.
6. **Cockpit HUD / PFD**: Real-time `AGL` altitude pill displayed next to MSL altitude.

---

### 2. Flight Controller Daemon (`apx_flight_controller`)

The compiled autopilot executable can be launched standalone for debugging, bench testing, or hardware-in-the-loop:

```bash
./build/sitl/main/apx_flight_controller [options]
```

#### Parameter Options for `apx_flight_controller`

| Option | Format / Default | Description & Meaning |
|---|---|---|
| `--vehicle=<type>` | String (`quad_x`) | Vehicle frame configuration (`quad_x`, `hex_x`, `octo_x`). |
| `--sitl-port=<port>` | Integer (`9003`) | UDP port listening for simulator physics state packets. |
| `--gcs-udp=<port>` | Integer (`14550`) | MAVLink telemetry broadcast UDP port. |
| `--gcs-tcp=<port>` | Integer (`5760`) | Primary MAVLink TCP listener port. |
| `--tcp-ports=<p1,p2...>` | Port list (`5760,5761,5762,5763`) | List of concurrent MAVLink TCP server ports. |
| `--home-lat=<deg>` | Float (`18.5204`) | Home origin latitude. |
| `--home-lon=<deg>` | Float (`73.8567`) | Home origin longitude. |
| `--home-alt=<m>` | Float (`540.0`) | Home origin altitude MSL. |
| `--sensor-lanes=<1-4>` | Integer (`4`) | Number of active sensor voting lanes. |
| `--gps-count=<0-4>` | Integer (`1`) | Number of active GPS receivers. |
| `--sensor-profile=<prof>` | String (`realistic`) | Noise profile (`ideal`, `realistic`, `harsh`). |
| `--sim-seed=<int>` | Integer (`1`) | Deterministic PRNG seed. |
| `--sensor-log=<path>` | String | CSV telemetry logging destination. |

---

### 3. Built-in 6DOF RK4 Physics Engine (`apx_sim_physics_runner`)

The standalone physics binary runs the 1000 Hz multirotor rigid-body simulation directly:

```bash
./build/sitl/sitl/apx_sim_physics_runner [options]
```

#### Parameter Options for `apx_sim_physics_runner`

| Option | Syntax | Default | Description |
|---|---|---|---|
| Target Port | `--port=<port>` or `--port <port>` | `9003` | Destination UDP port of the flight controller daemon. |
| Local Port | `--listen-port=<port>` | `9004` | Local socket port for receiving motor throttle commands. |
| Vehicle Frame | `--vehicle=<type>` | `quad_x` | Airframe type (`quad_x`, `hex_x`, `octo_x`). |
| Wind Disturbance | `--wind=<S,D[,G]>` or `--wind <S,D[,G]>` | `0,0,0` | Wind vector: speed (m/s), direction from (deg), gust sigma (m/s). |
| Home Location | `--lat=<deg>`, `--lon=<deg>`, `--alt=<m>` | `18.5204`, `73.8567`, `540.0` | Geodetic origin coordinates. |
| Random Seed | `--seed=<int>` | `1` | PRNG seed for wind gusts and disturbance models. |

---

### 4. Developer Utilities: Parameter CLI & MAVLink Console

APX includes lightweight, zero-dependency Python tools in `tools/`:

#### A. Parameter Utility (`tools/apx_param.py`)

Export, import, and inspect vehicle parameter configurations:

```bash
# Export all active parameters to a file
python3 tools/apx_param.py --udp 127.0.0.1:14550 export agricultural_quad.params

# Import and flash parameters to the vehicle
python3 tools/apx_param.py --udp 127.0.0.1:14550 import agricultural_quad.params

# List connection status
python3 tools/apx_param.py --udp 127.0.0.1:14550 list
```

#### B. MAVLink Live Console (`tools/apx_console.py`)

Headless command-line tool for reading/writing individual parameters and streaming telemetry:

```bash
# Inspect a specific parameter value
python3 tools/apx_console.py --udp 127.0.0.1:14550 param get WPNAV_SPEED

# Tune a controller gain on the fly
python3 tools/apx_console.py --udp 127.0.0.1:14550 param set WPNAV_SPEED 650.0

# Connect via TCP and stream live telemetry
python3 tools/apx_console.py --tcp 127.0.0.1:5760
```

---

### 5. Unified Test Suite Runner (`run_test_suite.py`)

The automated test runner executes both GoogleTest C++ unit tests and SITL flight scenario tests:

```bash
# Run all unit tests and SITL integration tests
python3 tests/run_test_suite.py --all --build-dir build/sitl

# Run only C++ unit tests (fast verification)
python3 tests/run_test_suite.py --unit --build-dir build/sitl

# Run a specific SITL scenario test by name
python3 tests/run_test_suite.py --sitl --sitl-test test_auto_mission.py

# Generate Markdown and JUnit XML reports for CI/CD
python3 tests/run_test_suite.py --all --build-dir build/sitl \
    --report-md build/test_report.md \
    --report-junit build/junit.xml
```

---

## Connecting Ground Control Stations (GCS)

APX implements standard **MAVLink v2** communication compatible with all major GCS suites.

### Communication Ports & Telemetry

| Protocol | Address / Interface | Port | Purpose |
|---|---|---|---|
| **UDP** | `127.0.0.1` | **`14550`** | Standard GCS telemetry broadcast (GCS default discovery) |
| **TCP** | `127.0.0.1` | **`5760`** | Primary reliable telemetry link / GCS connection |
| **TCP** | `127.0.0.1` | **`5761` - `5763`** | Additional concurrent telemetry taps (companion computer / logging) |
| **UART** | `/dev/ttyUSB0` (or `ttyAMA0`) | Baud `57600` / `115200` | Real hardware telemetry radios (SiK, Holybro, RFD900) |

---

### APX Flight Manager Setup

1. Start **APX Flight Manager**.
2. APX Flight Manager **automatically listens on UDP port 14550**. Once you launch `sim_vehicle.py`, GCS connects automatically within 2 seconds.
3. If auto-connection is disabled:
   - Navigate to **Application Settings** $\rightarrow$ **Comm Links**.
   - Click **Add**: **Type**: `UDP`, **Port**: `14550`.
   - Click **Connect**.
4. The Primary Flight Display (PFD) will immediately indicate live artificial horizon, compass heading, GPS satellites, battery status, and flight mode.

---

### Ground Control Station Setup

1. Start Ground Control Station.
2. In the top-right connection dropdown:
   - Select **UDP**.
   - Click **Connect**.
   - When prompted for the local port, enter: `14550`.
3. Ground Control Station will download the parameter table from APX.
4. Open the **Flight Data** tab to view live telemetry or **Config/Tuning** $\rightarrow$ **Full Parameter List** to adjust gains.

---

## Supported Flight Modes & Navigation Logic

### Flight Mode Summary

| Mode Name | Custom ID | GPS Required | Position Lock | Description & Pilot Experience |
|---|---|---|---|---|
| **`STABILIZE`** | `0` | No | No | Manual pilot control with automatic self-leveling. Throttle directly commands collective thrust. |
| **`ALT_HOLD`** | `2` | No | Altitude Only | Automatic altitude hold. Roll/pitch control tilt; throttle stick centered maintains current height. |
| **`AUTO`** | `3` | **Yes** | 3D Trajectory | Autonomous execution of pre-programmed MAVLink missions and condition commands. |
| **`GUIDED`** | `4` | **Yes** | 3D Position | "Point-and-click" interactive flight from GCS (Takeoff, Go-To-Location, Reposition). |
| **`LOITER`** | `5` | **Yes** | 3D Position | Automatic 3D position and altitude hold. Deflecting sticks nudges position with smooth deceleration. |
| **`RTL`** | `6` | **Yes** | 3D Position | Return-to-Launch. Climbs to `RTL_ALT`, flies straight home, loiters, and auto-lands. |
| **`LAND`** | `9` | Optional | Descent | Stabilized vertical descent with forward braking and optional joystick repositioning. |
| **`POS_HOLD`** | `16` | **Yes** | 3D Position | Hybrid mode: behaves like Stabilize when sticks are moved, locks 3D position when sticks center. |
| **`BRAKE`** | `17` | **Yes** | 3D Position | Immediate emergency stop. Applies `BRAKE_ACCEL` to bring the vehicle to an instant hover. |
| **`AGRI_SPRAY`**| `100` | **Yes** | 3D Position | Precision spraying mode with speed-proportional chemical pump dosing. |
| **`AGRI_ZIGZAG`**| `101` | **Yes** | 3D Position | Autonomous back-and-forth agricultural field coverage between points A and B. |

---

### Autonomous Navigation, Yaw & Cornering Logic

APX incorporates an advanced **S-curve and Jerk-Limited Path Tracker** (`modules/modes/path_tracker.cpp`) that eliminates aggressive waypoint overshoot:

1. **Curvature-Continuous Corner Blending (`WPNAV_BLEND = 1`)**:
   - Instead of sharp circular cuts or abrupt stops, APX blends fly-through waypoints using spiral-arc-spiral curves.
   - Lateral acceleration smoothly ramps in and out at no more than `WPNAV_JERK` (default $5.0\text{ m/s}^3$).
2. **Turn-Speed Deceleration Limiting**:
   - The path generator evaluates the turn angle between consecutive waypoints.
   - For sharp turns (e.g. $90^\circ$ or $180^\circ$ switchbacks), the cruise velocity is automatically capped before the corner so the vehicle stays safely within `WPNAV_RADIUS` without swinging wide.
3. **Flexible Heading Behaviors (`WP_YAW_BEHAVIOR`)**:
   - `0`: **Constant Heading** — Drone holds a constant heading angle (set by `AUTO_YAW_HEAD`). If the next waypoint is to the side, the drone slides sideways while maintaining its orientation.
   - `1`: **Face Next Waypoint** — Nose automatically turns to face the approaching waypoint.
   - `2`: **Face Next Waypoint Except RTL** (Default) — Nose faces next waypoint during mission legs, but holds heading during RTL return.
   - `3`: **Along Course** — Nose continuously tracks the tangent of the flight path.
   - `4`: **Fixed / Condition Yaw** — Heading is commanded exclusively by `MAV_CMD_CONDITION_YAW`.

---

### Precision Land Mode & Manual Repositioning

Traditional autopilots often start vertical descent at the exact moment a `LAND` command is received, causing the drone to drift down diagonally if it had forward momentum. APX introduces two safety enhancements:

1. **Active Forward Velocity Braking**:
   - Upon entering `LAND`, APX applies horizontal deceleration based on `BRAKE_ACCEL` to bring horizontal speed to zero.
   - The landing point is locked **where the vehicle comes to a complete stop**, ensuring a perfectly vertical touchdown.
2. **Interactive Manual Repositioning**:
   - During descent in `LAND`, the pilot can deflect the roll and pitch sticks to nudge the vehicle horizontally up to `LAND_REPOS_SPEED` (default $1.5\text{ m/s}$) to steer clear of obstacles, puddles, or uneven ground.
   - Yaw stick enables rotating the aircraft heading to face into the wind for a safer landing.
3. **Two-Stage Descent Speed Profile**:
   - Above `LAND_ALT_LOW` (default $10\text{ m}$), the drone descends at `LAND_SPEED_HIGH` (or `WPNAV_SPEED_DN`, $1.5\text{ m/s}$).
   - Below `LAND_ALT_LOW`, descent smoothly slows down to `LAND_SPEED` (default $0.5\text{ m/s}$) for gentle ground contact.

---

### Agricultural Spraying & ZigZag Modes

Tailored for commercial spraying multirotors:
- **Automatic Pump Dosing (`SPRAY_PUMP_RATE`)**: Regulates spray pump PWM output proportional to ground speed ($10\%$ pump duty per $1\text{ m/s}$ ground speed).
- **Auto Boundary Cutoff**: Stops the spray pump whenever the drone slows down below `SPRAY_SPEED_MIN` ($1.0\text{ m/s}$) or enters a turn leg.
- **Pilot Altitude Override in `AGRI_ZIGZAG`**: While the drone flies automated parallel swaths, the pilot can adjust flight altitude via the throttle stick in real time without disengaging autonomy.

---

## Supported MAVLink Mission & Condition Commands

APX supports standard MAVLink mission protocols for autonomous flight plans:

| MAVLink Command | ID | Parameters & Mappings | Behavior in APX Firmware |
|---|---|---|---|
| **`MAV_CMD_NAV_WAYPOINT`** | `16` | **p1**: Hold time (s)<br>**p2**: Acceptance radius (m)<br>**p3**: Pass radius (m)<br>**p4**: Yaw angle (deg)<br>**p5..7**: Lat, Lon, Alt (m) | Flies to targeted 3D coordinate. Uses S-curve velocity profiling. If `p2 = 0`, falls back to `WPNAV_RADIUS`. |
| **`MAV_CMD_NAV_TAKEOFF`** | `22` | **p7**: Target altitude (m) | Commands vertical climb to target altitude above home at `WPNAV_SPEED_UP` rate. |
| **`MAV_CMD_NAV_LAND`** | `21` | **p5..7**: Lat, Lon, Alt (optional) | Safely brings horizontal velocity to a stop, locks touchdown position, and executes stabilized descent. |
| **`MAV_CMD_NAV_RETURN_TO_LAUNCH`** | `20` | None | Climbs to `RTL_ALT` (default $15\text{ m}$), flies straight to home coordinates, and auto-lands. |
| **`MAV_CMD_CONDITION_YAW`** | `115` | **p1**: Target angle (deg)<br>**p2**: Turn rate (deg/s)<br>**p3**: Direction (-1=CCW, 1=CW)<br>**p4**: 0=Absolute, 1=Relative | Rotates the drone to the target heading at the specified turn rate **before advancing to the next mission item**. Features smooth deceleration to prevent overshoot, and locks heading for subsequent legs. |
| **`MAV_CMD_CONDITION_CHANGE_ALT`** | `113` | **p1**: Climb/descent rate (m/s)<br>**p7**: Finish altitude (m) | Suspends waypoint navigation until the vehicle climbs or descends to the target altitude. |
| **`MAV_CMD_DO_CHANGE_SPEED`** | `178` | **p1**: Speed type (0=Airspeed, 1=Groundspeed)<br>**p2**: Target speed (m/s, or `-2` to reset) | Adjusts the horizontal cruise speed on the fly during mission execution. |
| **`MAV_CMD_DO_SET_SERVO`** | `183` | **p1**: Channel (1 - 16)<br>**p2**: PWM value (800 - 2200 µs) | Directly sets auxiliary PWM output (e.g. Channel 9 controls the chemical spray pump). |
| **`MAV_CMD_DO_CHANGE_ALTITUDE`** | `186` | **p7**: Target altitude (m) | Adjusts mission altitude while maintaining current flight progression. |
| **`MAV_CMD_DO_SPRAYER`** | `216` | **p1**: 0=Disable, 1=Enable | Toggles the automated crop sprayer system. |
| **`MAV_CMD_DO_SET_MODE`** | `176` | **p1**: Base mode<br>**p2**: Custom mode number | Changes the active flight mode via MAVLink. |
| **`MAV_CMD_DO_SET_HOME`** | `179` | **p1**: 1=Current position, 0=Use p5..7<br>**p5..7**: Lat, Lon, Alt MSL (m; NaN keeps the current home altitude) | Moves home, the point RTL and mission RTL items return to; takes effect at once, also in flight. Accepted as COMMAND_LONG or COMMAND_INT (exact lat/lon). Kept across arming and landing until reboot. Refuses invalid coordinates (`DENIED`) and "current position" without a fix (`FAILED`). Confirmed by `HOME_POSITION` (1 Hz). |
| **`MAV_CMD_COMPONENT_ARM_DISARM`** | `400` | **p1**: 0=Disarm, 1=Arm<br>**p2**: 21196 (Force flag) | Arms or disarms the multirotor motors following safety checks. |
| **`MAV_CMD_PREFLIGHT_CALIBRATION`** | `241` | **p1**: Accel cal<br>**p2**: Compass cal<br>**p5**: Gyro cal | Triggers DJI-style guided sensor calibration routines. |

---

## Autopilot Parameters Reference

APX parameters are stored in non-volatile flash and can be queried or updated via APX Flight Manager, Ground Control Station, or `tools/apx_param.py`.

### Navigation & Waypoint Parameters

| Parameter | Type | Default | Range | Units | Description & Meaning |
|---|---|---|---|---|---|
| **`WPNAV_SPEED`** | FLOAT | `500.0` | 20 - 2000 | cm/s | Maximum horizontal cruise velocity during autonomous missions ($5.0\text{ m/s}$). |
| **`WPNAV_RADIUS`** | FLOAT | `200.0` | 10 - 1000 | cm | Waypoint arrival acceptance radius and maximum allowable corner cut distance ($2.0\text{ m}$). |
| **`WPNAV_ACCEL`** | FLOAT | `250.0` | 50 - 1000 | $\text{cm/s}^2$ | Maximum allowable along-track and cornering acceleration ($2.5\text{ m/s}^2$). |
| **`WPNAV_JERK`** | FLOAT | `5.0` | 1.0 - 20.0 | $\text{m/s}^3$ | Rate of change of acceleration. Controls how smoothly velocity ramps in and out. |
| **`WPNAV_BLEND`** | INT32 | `1` | 0 - 1 | Bool | `1`: Curvature-continuous spiral blending (smooth turns). `0`: Traditional circular arc. |
| **`WPNAV_SPEED_UP`**| FLOAT | `250.0` | 10 - 1000 | cm/s | Maximum vertical climb rate during waypoint missions ($2.5\text{ m/s}$). |
| **`WPNAV_SPEED_DN`**| FLOAT | `150.0` | 10 - 500 | cm/s | Maximum vertical descent rate during waypoint missions ($1.5\text{ m/s}$). |
| **`BRAKE_ACCEL`** | FLOAT | `250.0` | 50 - 1000 | $\text{cm/s}^2$ | Horizontal deceleration applied when entering `BRAKE` or `LAND` modes. |
| **`LOIT_SPEED`** | FLOAT | `180.0` | 20 - 2000 | cm/s | Maximum horizontal velocity commanded by pilot stick deflection in `LOITER` mode. |

---

### Attitude, Rate & Heading (Yaw) Control

| Parameter | Type | Default | Range | Units | Description & Meaning |
|---|---|---|---|---|---|
| **`WP_YAW_BEHAVIOR`**| INT32 | `2` | 0 - 4 | Enum | Auto yaw mode: `0`=Constant heading, `1`=Face next WP, `2`=Face next WP except RTL, `3`=Along course, `4`=Condition yaw. |
| **`AUTO_YAW_HEAD`** | FLOAT | `-1.0` | -1 - 360 | deg | Fixed heading for `WP_YAW_BEHAVIOR=0`. `-1` maintains the heading angle at mode entry. |
| **`ATC_ANG_YAW_P`** | FLOAT | `4.5` | 1.0 - 12.0 | Gain | Yaw angle P gain (converts heading error to desired yaw rate). |
| **`ATC_RAT_YAW_P`** | FLOAT | `0.600` | 0.01 - 2.0 | Gain | Yaw angular rate PID proportional gain. |
| **`ATC_RAT_YAW_I`** | FLOAT | `0.060` | 0.001 - 0.5 | Gain | Yaw angular rate PID integral gain. |
| **`ATC_RATE_Y_MAX`** | FLOAT | `45.0` | 5.0 - 180.0 | deg/s | Maximum commandable yaw rotation rate ceiling. |
| **`ATC_ANG_RLL_P`** | FLOAT | `4.5` | 1.0 - 12.0 | Gain | Roll attitude angle P gain. |
| **`ATC_ANG_PIT_P`** | FLOAT | `4.5` | 1.0 - 12.0 | Gain | Pitch attitude angle P gain. |
| **`ATC_RAT_RLL_P`** | FLOAT | `0.135` | 0.01 - 1.0 | Gain | Roll angular rate PID P gain. |
| **`ATC_RAT_RLL_I`** | FLOAT | `0.135` | 0.01 - 1.0 | Gain | Roll angular rate PID I gain. |
| **`ATC_RAT_RLL_D`** | FLOAT | `0.0036`| 0.0001 - 0.1 | Gain | Roll angular rate PID D gain. |
| **`ATC_RAT_PIT_P`** | FLOAT | `0.135` | 0.01 - 1.0 | Gain | Pitch angular rate PID P gain. |
| **`ATC_RAT_PIT_I`** | FLOAT | `0.135` | 0.01 - 1.0 | Gain | Pitch angular rate PID I gain. |
| **`ATC_RAT_PIT_D`** | FLOAT | `0.0036`| 0.0001 - 0.1 | Gain | Pitch angular rate PID D gain. |
| **`ANGLE_MAX`** | INT32 | `2500` | 1000 - 4500 | cdeg | Maximum allowable vehicle lean angle ($25.0^\circ$). |

---

### Landing & Return to Launch (RTL)

| Parameter | Type | Default | Range | Units | Description & Meaning |
|---|---|---|---|---|---|
| **`RTL_ALT`** | FLOAT | `1500.0` | 0 - 10000 | cm | Return-to-Launch climb altitude ($15.0\text{ m}$). |
| **`LAND_SPEED`** | FLOAT | `50.0` | 30 - 200 | cm/s | Final touchdown descent speed ($0.5\text{ m/s}$). |
| **`LAND_SPEED_HIGH`**| FLOAT | `0.0` | 0 - 500 | cm/s | High-altitude descent speed above `LAND_ALT_LOW` (`0` uses `WPNAV_SPEED_DN`). |
| **`LAND_ALT_LOW`** | FLOAT | `1000.0` | 100 - 10000 | cm | Altitude threshold where descent rate transitions from high to `LAND_SPEED` ($10.0\text{ m}$). |
| **`LAND_REPOS_SPEED`**|FLOAT | `150.0` | 20 - 500 | cm/s | Maximum horizontal velocity allowed during pilot stick repositioning while landing. |
| **`DISARM_DELAY`** | FLOAT | `30.0` | 0 - 127 | s | Automatic motor disarm timeout after landing is confirmed (`0` = disabled). |

---

### Throttle & Hover Position Control

| Parameter | Type | Default | Range | Units | Description & Meaning |
|---|---|---|---|---|---|
| **`MOT_SPIN_ARM`** | FLOAT | `0.10` | 0.0 - 0.30 | Fraction | Motor idle spin throttle percentage when armed ($10\%$) for pilot awareness. |
| **`MOT_THST_HOVER`** | FLOAT | `0.50` | 0.1 - 0.9 | Fraction | Estimated hover thrust baseline. Automatically learned and refined in-flight. |
| **`PSC_POSXY_P`** | FLOAT | `1.0` | 0.1 - 3.0 | Gain | Horizontal position controller P gain (converts position error to velocity). |
| **`PSC_VELXY_P`** | FLOAT | `2.0` | 0.1 - 6.0 | Gain | Horizontal velocity controller P gain. |
| **`PSC_POSZ_P`** | FLOAT | `1.0` | 0.1 - 3.0 | Gain | Vertical altitude position controller P gain. |
| **`PSC_VELZ_P`** | FLOAT | `1.5` | 0.1 - 6.0 | Gain | Vertical climb-rate controller P gain (drives collective thrust correction). |
| **`PILOT_SPEED_UP`** | FLOAT | `250.0` | 50 - 500 | cm/s | Maximum climb rate commanded by pilot stick in `ALT_HOLD` / `LOITER`. |
| **`PILOT_SPEED_DN`** | FLOAT | `150.0` | 50 - 500 | cm/s | Maximum descent rate commanded by pilot stick in `ALT_HOLD` / `LOITER`. |

---

### Agricultural Crop Sprayer

| Parameter | Type | Default | Range | Units | Description & Meaning |
|---|---|---|---|---|---|
| **`SPRAY_ENABLE`** | INT32 | `1` | 0 - 1 | Bool | Master enable for agricultural spraying subsystem. |
| **`SPRAY_PUMP_RATE`**| FLOAT | `10.0` | 0 - 100 | % / (m/s) | Pump output percentage per $1\text{ m/s}$ of horizontal ground speed. |
| **`SPRAY_PUMP_MIN`** | FLOAT | `0.0` | 0 - 100 | % | Minimum pump output fraction while spraying is active. |
| **`SPRAY_SPEED_MIN`**| FLOAT | `100.0` | 0 - 1000 | cm/s | Minimum ground speed required to spray ($1.0\text{ m/s}$). Auto-shuts off below this speed. |
| **`SPRAY_SPINNER`** | INT32 | `1300` | 1000 - 2000 | µs | Centrifugal atomizer / spinner servo PWM duty while spraying. |
| **`SPRAY_PUMP_CH`** | INT32 | `9` | 1 - 16 | Index | Output servo channel mapped to the liquid chemical pump (Default: Aux Channel 9). |
| **`SPRAY_SPIN_CH`** | INT32 | `10` | 1 - 16 | Index | Output servo channel mapped to the droplet atomizer (Default: Aux Channel 10). |
| **`SPRAY_PWM_MIN`** | INT32 | `1100` | 800 - 2200 | µs | Pump motor full-off PWM signal. |
| **`SPRAY_PWM_MAX`** | INT32 | `1900` | 800 - 2200 | µs | Pump motor full-on ($100\%$) PWM signal. |

---

### Safety, Battery & Failsafe

| Parameter | Type | Default | Range | Units | Description & Meaning |
|---|---|---|---|---|---|
| **`ARMING_CHECK`** | INT32 | `1` | 0 - 1 | Bool | `1` enables comprehensive pre-flight sensor checks before allowing arming. |
| **`FS_THR_ENABLE`** | INT32 | `1` | 0 - 3 | Enum | Action on RC signal loss: `0`=Disabled, `1`=RTL, `2`=Land, `3`=Brake. |
| **`FS_GCS_ENABLE`** | INT32 | `1` | 0 - 3 | Enum | Action on GCS telemetry timeout: `0`=Disabled, `1`=RTL, `2`=Land. |
| **`BATT_CAPACITY`** | FLOAT | `5000.0` | 100 - 100000 | mAh | Total battery capacity for state-of-charge tracking. |
| **`BATT_LOW_VOLT`** | FLOAT | `10.5` | 5.0 - 50.0 | Volts | Low battery voltage threshold triggering warnings or RTL. |
| **`BATT_CRT_VOLT`** | FLOAT | `10.0` | 5.0 - 50.0 | Volts | Critical battery voltage threshold triggering emergency landing. |

---

### RC Input & Channel Mapping

| Parameter | Type | Default | Range | Units | Description & Meaning |
|---|---|---|---|---|---|
| **`RCMAP_ROLL`** | INT32 | `1` | 1 - 8 | Channel | Physical RC channel assigned to Roll control. |
| **`RCMAP_PITCH`** | INT32 | `2` | 1 - 8 | Channel | Physical RC channel assigned to Pitch control. |
| **`RCMAP_THROTTLE`**| INT32 | `3` | 1 - 8 | Channel | Physical RC channel assigned to Throttle control. |
| **`RCMAP_YAW`** | INT32 | `4` | 1 - 8 | Channel | Physical RC channel assigned to Yaw control. |
| **`RCn_MIN`** | INT32 | `1100` | 800 - 2200 | µs | Minimum calibrated pulse width for RC Channel $n$ ($n = 1..8$). |
| **`RCn_MAX`** | INT32 | `1900` | 800 - 2200 | µs | Maximum calibrated pulse width for RC Channel $n$. |
| **`RCn_TRIM`** | INT32 | `1500` | 800 - 2200 | µs | Neutral center stick position for RC Channel $n$. |
| **`RCn_DZ`** | INT32 | `30` | 0 - 200 | µs | Center deadzone threshold for RC Channel $n$. |
| **`FLTMODE1..6`** | INT32 | `0` | 0 - 101 | Enum | Flight mode assigned to each position of the 6-position flight mode switch. |

---

### Sensor Redundancy & Calibration

| Parameter | Type | Default | Range | Units | Description & Meaning |
|---|---|---|---|---|---|
| **`EK3_ENABLE`** | INT32 | `1` | 0 - 1 | Bool | Master enable for 24-state EKF3 navigation engine. |
| **`SIM_TRUTH_ALT`** | INT32 | `0` | 0 - 1 | Bool | `0`: Estimator uses baro+IMU (like real hardware). `1`: Uses simulator truth. |
| **`COMPASS_OFS_{X,Y,Z}`**| FLOAT| `0.0` | -2000 - 2000| mGauss | Primary magnetometer hard-iron bias offsets. |
| **`COMPASS_DEC`** | FLOAT | `0.0` | -3.14 - 3.14| rad | Local magnetic declination angle in radians. |
| **`INS_ACCSCAL_{X,Y,Z}`**|FLOAT| `1.0` | 0.8 - 1.2 | Scale | Primary accelerometer orthogonal scaling factor. |
| **`INS_ACCOFFS_{X,Y,Z}`**|FLOAT| `0.0` | -3.5 - 3.5 | $\text{m/s}^2$| Primary accelerometer static zero-gravity bias offsets. |

For detailed low-level documentation on all 150+ internal parameters, see the **[Parameter Reference Guide](docs/parameters/PARAMETER_REFERENCE.md)**.

---

## Pre-Flight Checks & Sensor Calibration

### DJI-Style Simple Sensor Calibration

Traditional autopilot calibrations requiring tumbling the aircraft 360 degrees on all axes are impractical for large agricultural drones. APX provides guided calibrations:

1. **Level Accelerometer Calibration**:
   - Place drone on a flat, level surface.
   - In APX Flight Manager / GCS, send MAVLink command `MAV_CMD_PREFLIGHT_CALIBRATION` with param1 = 1 (or click **Calibrate Accelerometer**).
   - APX samples gravity vector for 3 seconds, determines static offsets, and locks reference. Done.
2. **Stationary Gyroscope Calibration**:
   - Drone must remain completely still for 2 seconds.
   - Gyro biases are averaged and nullified. If movement ($>0.05\text{ rad/s}$) is detected, the test safely aborts.
3. **Compass Calibration**:
   - Slowly rotate the vehicle in a single horizontal circle ($360^\circ$ yaw) over 15 seconds.
   - APX performs an automated ellipsoid fit to calculate hard-iron offsets without needing vertical tumbling.

---

## Repository Structure

```text
apx/
├── CMakeLists.txt                # Root CMake build definition
├── CMakePresets.json             # Pre-configured build profiles (SITL, Linux, STM32H7)
├── VERSION                       # Software version (0.1.0)
│
├── kernel/                       # Core OS & IPC abstraction
│   └── include/apx/kernel/       # uBus lock-free ring buffer pub/sub, Task, Mutex, Timer
│
├── hal/                          # Hardware Abstraction Layer
│   ├── include/apx/hal/          # Pure virtual interfaces (UART, SPI, I2C, CAN, GPIO, PWM)
│   ├── linux/                    # POSIX implementation
│   └── stm32h7/                  # Embedded STM32H7 / Zephyr HAL
│
├── drivers/                      # Device Drivers
│   ├── imu/                      # ICM-42688-P, ICM-42605, ICM-20689, BMI088, BMI270
│   ├── baro/                     # MS5611, BMP388
│   ├── mag/                      # RM3100, IST8310, LIS3MDL
│   ├── gps/                      # u-blox M9, M10, ZED-F9P RTK
│   ├── battery/                  # INA226 I2C power monitor, Analog divider
│   ├── esc/                      # DShot 150/300/600/1200 & PWM backends
│   ├── rc_input/                 # S.BUS and CRSF / ExpressLRS decoders
│   ├── servo/                    # 16-channel servo controller
│   └── pump/                     # Agricultural spray pump & flow monitor
│
├── modules/                      # Autopilot Subsystems
│   ├── sensor_manager/           # 4-way redundancy voting & health scoring
│   ├── calibration/              # Guided Level Accel, Still Gyro, 2-axis Mag
│   ├── estimator/
│   │   ├── ekf3/                 # Aerospace 24-state Extended Kalman Filter
│   │   └── filters/              # 2nd-order Butterworth & Dynamic RPM Notch
│   ├── flight_control/           # PID rate, quaternion attitude, position controllers
│   ├── modes/                    # Stabilize, AltHold, PosHold, Auto, RTL, Spray, ZigZag
│   ├── rc_manager/               # Channel mapping, expo, deadbands, mode select
│   ├── safety/                   # Pre-flight check, multi-tier failsafe, watchdog
│   ├── params/                   # Parameter management engine & persistence
│   └── gcs/                      # MAVLink v2 handler & network router
│
├── sitl/                         # Simulation Infrastructure
│   ├── physics/                  # 6DOF RK4 rigid body dynamic model & wind simulation
│   ├── sensor_models/            # Realistic noisy IMU, Baro, GPS, Mag models
│   ├── gazebo_plugin/            # Gazebo Harmonic system plugin with visual indicators
│   ├── models/                   # SDF vehicle models (Quad X, Hex X), windsock & trees
│   └── sim_vehicle.py            # Unified Python simulation launcher
│
├── tools/                        # Utility & Tuning Scripts
│   ├── apx_console.py            # Headless MAVLink telemetry monitor
│   └── apx_param.py              # Parameter export, import, and diff utility
│
├── tests/                        # Automated Verification Suite
│   ├── unit/                     # GoogleTest C++ unit tests
│   └── run_test_suite.py         # Unified test executor & Markdown/JUnit reporter
│
└── docs/                         # Comprehensive Documentation
    ├── parameters/               # Complete Parameter Reference (150+ params)
    ├── simulated_sensors.md      # Sensor noise & environmental modeling guide
    └── TESTING_AND_CI_RELEASE_GUIDE.md # Local testing & CI/CD deployment guide
```

---

## License

APX is released under the **Apache License 2.0**. Commercial use, modification, closed-source distribution of derivatives, and patent use are permitted without copyleft obligations. See [LICENSE](LICENSE) for details.
