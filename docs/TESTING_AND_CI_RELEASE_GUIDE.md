# APX Firmware Testing & CI/CD Release Guide

This guide provides end-to-end documentation for the APX flight controller test suite, automated multi-platform compilation matrix, continuous integration (CI) pipeline, and automated GitHub release process—modeled after aerospace standards and industry release conventions.

---

## Table of Contents

- [1. Architecture Overview](#1-architecture-overview)
- [2. Multi-Platform Targets](#2-multi-platform-targets)
- [3. Local Build Instructions](#3-local-build-instructions)
  - [3.1 Prerequisites](#31-prerequisites)
  - [3.2 STM32H7 Bare-Metal Target](#32-stm32h7-bare-metal-target)
  - [3.3 SITL Simulation & Tests Target](#33-sitl-simulation--tests-target)
  - [3.4 Native Linux Target](#34-native-linux-target)
- [4. Test Suite Execution](#4-test-suite-execution)
  - [4.1 Unified Test Runner (`run_test_suite.py`)](#41-unified-test-runner-run_test_suitepy)
  - [4.2 C++ Unit Tests (GoogleTest)](#42-c-unit-tests-googletest)
  - [4.3 SITL Flight Integration Tests](#43-sitl-flight-integration-tests)
- [5. GitHub Actions CI/CD Pipeline](#5-github-actions-cicd-pipeline)
  - [5.1 Trigger Conditions](#51-trigger-conditions)
  - [5.2 Matrix Compilation Stage](#52-matrix-compilation-stage)
  - [5.3 Automated Testing & Step Summary Stage](#53-automated-testing--step-summary-stage)
  - [5.4 Automated Release Packaging](#54-automated-release-packaging)
- [6. What to Look For & Where to Look](#6-what-to-look-for--where-to-look)
  - [6.1 GitHub Actions Web UI](#61-github-actions-web-ui)
  - [6.2 Downloadable Artifacts & Checksums](#62-downloadable-artifacts--checksums)
  - [6.3 Diagnosing Failures & Flight Logs](#63-diagnosing-failures--flight-logs)

---

## 1. Architecture Overview

APX employs a two-tier testing and release architecture:

1. **Compilation Matrix**: Ensures every commit cleanly compiles across three distinct platform classes:
   - **Embedded Microcontroller**: ARM Cortex-M7 (STM32H743) bare metal.
   - **Simulation**: Linux x86_64 Software-In-The-Loop (SITL).
   - **Embedded Linux**: Native POSIX runtime.
2. **Dual-Tier Test Suite**:
   - **Tier 1 (Unit Tests)**: Sub-second GoogleTest verification of low-level algorithms (PID controllers, sensor voting, motor mixing, complementary filters, S-curve trajectory generation).
   - **Tier 2 (SITL Integration Tests)**: End-to-end mission execution scripts communicating over MAVLink (TCP/UDP) with simulated physics, verifying waypoints, dynamic speeds, loiter, failsafes, and landing detection.

---

## 2. Multi-Platform Targets

| Target | Architecture | Board Definition | Key Output Artifacts |
| :--- | :--- | :--- | :--- |
| **`stm32h7`** | ARM Cortex-M7 (`arm-none-eabi`) | `platforms/stm32h7/boards/apx_h743_v1/` | `apx_firmware_apx_h743_v1.bin`<br>`apx_firmware_apx_h743_v1.hex`<br>`apx_firmware_apx_h743_v1.elf`<br>`apx_firmware_apx_h743_v1.map` |
| **`sitl`** | x86_64 Linux | `sitl/` | `apx_sitl`<br>`apx_sim_physics_runner`<br>`run_unit_tests` |
| **`linux`** | x86_64 Linux (POSIX) | `main/` | `apx_flight_controller` |

---

## 3. Local Build Instructions

### 3.1 Prerequisites

Install common build tools and compilers:

```bash
# Common tools
sudo apt-get update
sudo apt-get install -y cmake ninja-build build-essential python3-pip

# ARM Embedded Toolchain for STM32H7
sudo apt-get install -y gcc-arm-none-eabi binutils-arm-none-eabi libnewlib-arm-none-eabi

# Simulation dependencies
sudo apt-get install -y libprotobuf-dev protobuf-compiler
pip3 install pymavlink
```

---

### 3.2 STM32H7 Bare-Metal Target

Configure and build the STM32H743 firmware:

```bash
# Configure
cmake -B build/stm32h7 \
      -DAPX_BOARD=stm32h7 \
      -DAPX_BUILD_SITL=OFF \
      -DAPX_BUILD_TESTS=OFF \
      -GNinja

# Compile
ninja -C build/stm32h7
```

**Output location:**
```text
build/stm32h7/platforms/stm32h7/boards/apx_h743_v1/
├── apx_firmware_apx_h743_v1.bin  # Raw binary for ST-Link / OpenOCD / DFU flashing
├── apx_firmware_apx_h743_v1.hex  # Intel HEX format
├── apx_firmware_apx_h743_v1.elf  # Debug symbols and ELF headers
└── apx_firmware_apx_h743_v1.map  # Full linker symbol and memory map
```

---

### 3.3 SITL Simulation & Tests Target

Configure and build the simulation binaries and test suite:

```bash
# Configure with SITL and Tests enabled
cmake -B build/sitl \
      -DAPX_BOARD=sitl \
      -DAPX_BUILD_SITL=ON \
      -DAPX_BUILD_TESTS=ON \
      -GNinja

# Compile
ninja -C build/sitl
```

GoogleTest is automatically fetched and configured via `FetchContent` (version `1.14.0`) if not pre-installed on the host system.

---

### 3.4 Native Linux Target

Configure and build the native POSIX flight controller:

```bash
# Configure
cmake -B build/linux \
      -DAPX_BOARD=linux \
      -DAPX_BUILD_SITL=OFF \
      -DAPX_BUILD_TESTS=OFF \
      -GNinja

# Compile
ninja -C build/linux
```

**Output location:** `build/linux/main/apx_flight_controller`

---

## 4. Test Suite Execution

### 4.1 Unified Test Runner (`run_test_suite.py`)

APX provides a centralized test orchestrator located at `tests/run_test_suite.py`. It runs both C++ unit tests and SITL flight scenario tests, formats console outputs, and generates Markdown and JUnit XML reports.

#### Running All Tests:
```bash
python3 tests/run_test_suite.py --all --build-dir build/sitl
```

#### Running Unit Tests Only:
```bash
python3 tests/run_test_suite.py --unit --build-dir build/sitl
```

#### Running SITL Tests Only:
```bash
python3 tests/run_test_suite.py --sitl
```

#### Running a Specific SITL Test:
```bash
python3 tests/run_test_suite.py --sitl-test test_mission_item_speed
```

#### Generating Reports:
```bash
python3 tests/run_test_suite.py \
  --all \
  --build-dir build/sitl \
  --report-md test_summary.md \
  --report-junit test_results.xml
```

---

### 4.2 C++ Unit Tests (GoogleTest)

Located in `tests/unit/`:
- **`test_pid_controller.cpp`**: Step response, anti-windup clamping, derivative filter cutoff, output saturation limits.
- **`test_sensor_voter.cpp`**: Multi-lane redundant sensor voting, outlier rejection, degraded sensor exclusion.
- **`test_motor_mixer.cpp`**: Quad-X and Hex-X thrust distributions, roll/pitch/yaw differential torque allocation.
- **`test_complementary_filter.cpp`**: Mahony attitude observer, level convergence, 45-degree pitch estimation.
- **`test_path_tracker.cpp`**: S-curve sqrt acceleration controller, stopping distances, cornering arc transitions, collinear waypoint cruise speed continuity.

You can also run them directly:
```bash
./build/sitl/tests/unit/run_unit_tests
# or with CTest:
ctest --test-dir build/sitl --output-on-failure
```

---

### 4.3 SITL Flight Integration Tests

Located in `sitl/`:
1. **`test_mission_item_speed.py`**:
   - Uploads multi-leg mission containing per-mission-item speed and `MAV_CMD_DO_CHANGE_SPEED` commands.
   - Verifies drone respects 4.0 m/s on Leg 1 and accelerates to 8.0 m/s on Leg 2.
2. **`test_waypoint_approach_and_braking.py`**:
   - Tests trajectory tracking accuracy across sharp corners and waypoint braking.
   - Verifies zero-drop vertical lift compensation during high-speed pitch tilts.
3. **`test_gcs_mission_and_joystick.py`**:
   - Tests APX Flight Manager MAVLink mission transfer protocol.
   - Tests virtual joystick RC override in LOITER and ALT_HOLD modes.
4. **`test_auto_path_tracking.py`**:
   - Autonomous S-curve path following and corner deviation verification (achieving < 0.28m path error).
5. **`test_loiter_landing.py`**:
   - Tests position hold loiter, stick arming/disarming, and automatic 10-second ground touchdown disarm.
6. **`test_sprayer.py`**:
   - Tests agricultural spray payload triggers and flow rate scaling.

---

## 5. GitHub Actions CI/CD Pipeline

The pipeline is defined in `.github/workflows/firmware_ci_cd.yml`.

### 5.1 Trigger Conditions

- **Pushes to `master` or `main`**: Automatically triggers compilation matrix and test suite.
- **Pull Requests targeting `master` or `main`**: Executes matrix build and test suite before review.
- **Tags matching `v*` (e.g. `v0.1.0`)**: Automatically builds, tests, packages, and publishes a new GitHub Release.
- **Manual Trigger (`workflow_dispatch`)**: Can be triggered manually from the GitHub Actions tab, with an option to create a release.

---

### 5.2 Matrix Compilation Stage (`build-matrix`)

Runs three parallel jobs on `ubuntu-22.04`:
1. `Build (stm32h7)`: Cross-compiles bare-metal image with `arm-none-eabi-gcc`.
2. `Build (sitl)`: Compiles simulation binaries and unit tests.
3. `Build (linux)`: Compiles native Linux flight controller binary.

Each job generates `SHA256SUMS.txt` and uploads zipped/tarballed artifacts to the workflow run.

---

### 5.3 Automated Testing & Step Summary Stage (`test-suite`)

- Installs MAVLink dependencies.
- Compiles the SITL binary and test executables.
- Invokes `tests/run_test_suite.py` with `--report-md "$GITHUB_STEP_SUMMARY"`.
- Formats a clean, high-visibility table directly on the GitHub Actions workflow run summary page.

---

### 5.4 Automated Release Packaging (`release`)

When triggered by a version tag (`v*`):
- Downloads artifacts from all matrix build jobs.
- Packages:
  * `apx-firmware-stm32h7-v*.zip` (contains `.bin`, `.hex`, `.elf`, `.map`)
  * `apx-sitl-linux-x86_64-v*.tar.gz` (contains `apx_sitl` and `apx_sim_physics_runner`)
  * `apx-flight-controller-linux-x86_64-v*.tar.gz` (contains `apx_flight_controller`)
- Generates master `SHA256SUMS.txt`.
- Creates a formal GitHub Release via `softprops/action-gh-release@v2` with attached binaries and release notes.

---

## 6. What to Look For & Where to Look

### 6.1 GitHub Actions Web UI

1. Navigate to: **`https://github.com/SVPR94/apx-firmware/actions`**
2. Click on the latest workflow run for your commit or tag.
3. **Job Matrix Overview**:
   - `Build (stm32h7)`: Check for green checkmark (successful Cortex-M7 cross-compilation).
   - `Build (sitl)`: Check for simulation binary compilation.
   - `Build (linux)`: Check for native Linux controller compilation.
   - `Automated Test Suite (Unit & SITL)`: Check for test execution status.
4. **Step Summary Page (`$GITHUB_STEP_SUMMARY`)**:
   - Scroll down to the bottom of the workflow run overview.
   - Look for the **"APX Firmware Test Suite Execution Report"** table:
     * Check that all 20+ unit tests show `✅ Pass`.
     * Check that flight scenarios show `✅ Pass` along with recorded flight metrics (speed, max path error, touchdown status).

---

### 6.2 Downloadable Artifacts & Checksums

On the workflow summary page, scroll to the **Artifacts** section:
- **`apx-stm32h7-artifacts`**: Download to flash onto hardware.
- **`apx-sitl-artifacts`**: Download to run simulation on a Linux workstation.
- **`apx-linux-artifacts`**: Download for embedded Linux companion boards.
- **`apx-test-reports`**: Download raw XML reports for CI test viewers.

Verify package integrity using:
```bash
sha256sum -c SHA256SUMS.txt
```

---

### 6.3 Diagnosing Failures & Flight Logs

If any test fails:
1. In the GitHub Actions Step Summary, expand the **"⚠️ Failure Diagnostics & Logs"** collapsible drawer for that test.
2. In the workflow run log, look at the step **"Execute Unified Test Suite"** for full console traces.
3. For SITL flight tests, look for:
   - **`sim_output.log`**: Standard output and error from the APX autopilot process.
   - **`track.csv`**: Time-series log containing `t, n, e, alt, speed, seq, yaw` which can be plotted in Python/Matplotlib to inspect flight trajectory and cross-track deviations.
