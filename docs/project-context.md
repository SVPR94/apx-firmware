# APX Project Context & AI Coding Agent Primer

This document is a machine-readable, high-signal reference designed for AI coding assistants and developers seeking immediate architectural context without browsing the full repository.

---

## 1. Project Purpose
APX is an open-source, aerospace-grade modular multirotor autopilot firmware and SITL simulation engine written in modern C++17. It supports autonomous agriculture (spraying/zigzag), standard GCS telemetry (APX Flight Manager / GCS), multi-estimator state fusion, and bare-metal STM32H7 hardware execution.

---

## 2. Fast Build Cheat-Sheet

```bash
# 1. Native SITL Simulation + Unit Tests (Debug)
cmake --preset sitl && cmake --build build/sitl -j$(nproc)
./build/sitl/tests/unit/run_unit_tests

# 2. Linux Release Build (Optimized for Companion Computers)
cmake --preset linux-release && cmake --build build/linux-release -j$(nproc)

# 3. STM32H743 Bare-Metal Firmware Build (ARM Cortex-M7)
cmake --preset stm32h7 && cmake --build build/stm32h7 -j$(nproc)

# 4. Generate New Hardware Board from hwdef.dat
python3 tools/generate_board.py --hwdef <path/to/hwdef.dat> --output-dir platforms/stm32h7/boards/<board>

# 5. Local Documentation Generation & Validation
./scripts/generate-docs.sh
```

---

## 3. Directory Map & Component Roles

| Directory | Core Role | Key Files |
|---|---|---|
| `main/` | Executive entry point & 400 Hz control loop | `apx_main.cpp` |
| `modules/flight_control/` | Cascade controllers & multirotor mixer | `rate_controller/`, `attitude_controller/`, `position_controller/`, `mixer/` |
| `modules/estimator/` | State estimation (EKF3, Mahony, UKF) | `ekf3/`, `attitude_estimator/`, `filters/` |
| `modules/modes/` | Mode management & guidance algorithms | `mode_manager.cpp`, `mode_brake.cpp`, `path_tracker.hpp`, `sprayer.hpp` |
| `modules/gcs/` | MAVLink telemetry & command execution | `mavlink_handler.cpp`, `command_handler.cpp`, `mission_executor.cpp` |
| `modules/params/` | Non-volatile parameter storage | `param_manager.cpp` |
| `hal/include/apx/hal/` | Pure virtual hardware interfaces | `hal_uart.hpp`, `hal_spi.hpp`, `hal_i2c.hpp`, `hal_pwm.hpp`, `hal_gpio.hpp` |
| `kernel/include/apx/kernel/` | Zero-allocation pub/sub message bus | `ubus.hpp`, `timer.hpp`, `mutex.hpp` |
| `drivers/` | Hardware drivers (IMU, Baro, Mag, GPS, etc.) | `imu/`, `baro/`, `mag/`, `gps/`, `esc/`, `pump/` |
| `platforms/stm32h7/` | STM32H7 board support, linker, startup | `boards/apx_h743_v1/`, `boards/custom_h7_fc/`, `toolchain-arm-none-eabi.cmake` |
| `sitl/` | Gazebo plugin & standalone physics | `gazebo_plugin/apx_gazebo_plugin.cpp`, `sim_vehicle.py` |
| `tests/unit/` | Unit tests (28 tests across 8 suites) | `test_pid_controller.cpp`, `test_motor_mixer.cpp`, `test_mode_brake_sprayer.cpp` |
| `tools/` | Code generators & tooling | `generate_board.py` |
| `scripts/` | Documentation & validation scripts | `generate-docs.sh`, `extract_cpp_api.py`, `validate-docs.py` |
| `docs/` | Comprehensive technical documentation | `architecture.md`, `modules.md`, `interfaces.md`, `build.md`, `api/` |

---

## 4. Key Architectural Invariants

1. **Inner Loop Rate:**
   - Inner flight control loop runs strictly at **400 Hz** (`apx_main.cpp`).
   - Rate PID gains assume 400 Hz sampling period; running at lower rates degrades attitude phase margin.
2. **Attitude Control Swing-Twist Decoupling:**
   - Thrust-vector tilt (swing) is decoupled from heading error (twist) to eliminate coupled divergence during sharp high-speed cornering.
3. **Multirotor Motor Headroom Priority:**
   - Roll and Pitch stabilization have strict priority over yaw; yaw is allocated remaining headroom (`yaw_headroom = max(0.15, 1.0 - rp_spread)`).
4. **Zero Heap Allocation:**
   - No `malloc` or heap reallocations during active flight. All buffers and topics are statically preallocated.
5. **Standard GCS Dialect:**
   - MAVLink heartbeat uses standard multicopter dialect identification so APX Flight Manager and Ground Control Station load standard multicopter flight modes (`0=STABILIZE`, `3=AUTO`, `4=GUIDED`, `17=BRAKE`).
6. **Auxiliary Payload Actuation:**
   - Agricultural spray pump is connected to servo channel 9 (`1100=OFF`, `1900=MAX`). Modulated in AUTO missions via `MAV_CMD_DO_SET_SERVO`.
