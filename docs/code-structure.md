# APX Codebase Structure & Conventions

This document outlines the directory structure, file naming patterns, namespace hierarchy, and coding standards of the APX project.

---

## 1. Directory Tree Layout

```text
apx/
├── CMakeLists.txt              # Root CMake build configuration (C++17, multi-platform)
├── CMakePresets.json           # Presets for sitl, linux-release, and stm32h7
├── VERSION                     # Release version string (0.1.0)
│
├── main/                       # Main executable entry points
│   ├── apx_main.cpp            # Primary flight controller executive & control loop (400 Hz)
│   └── CMakeLists.txt
│
├── hal/                        # Hardware Abstraction Layer interfaces
│   ├── include/apx/hal/        # Pure virtual interfaces (IHalUart, IHalSpi, IHalPwm, etc.)
│   └── CMakeLists.txt
│
├── kernel/                     # Microkernel services
│   ├── include/apx/kernel/     # Lock-free pub/sub bus (ubus.hpp), tasks, timers
│   ├── src/                    # uBus ringbuffer implementation
│   └── CMakeLists.txt
│
├── drivers/                    # Hardware device drivers
│   ├── include/apx/drivers/    # DriverBase, DriverRegistry
│   ├── imu/                    # ICM-42688, ICM-20689, BMI088, BMI270, BMI323
│   ├── baro/                   # MS5611, BMP388
│   ├── mag/                    # IST8310, RM3100, LIS3MDL
│   ├── gps/                    # u-blox M9, M10, ZED-F9P RTK
│   ├── battery/                # INA226, analog voltage/current
│   ├── esc/                    # DShot 150/300/600/1200
│   ├── servo/                  # PWM servo outputs
│   ├── rc_input/               # SBUS, CRSF / ExpressLRS
│   ├── relay/                  # GPIO relay channels
│   ├── pump/                   # Agricultural spray pump driver
│   └── telemetry/              # Serial MAVLink transport
│
├── modules/                    # Core Autopilot Modules
│   ├── flight_control/         # Rate, Attitude, Position controllers, Multirotor mixer
│   ├── estimator/              # EKF3, Mahony complementary filter, Notch/LPF prefilters
│   ├── modes/                  # Flight mode implementations (Stabilize, PosHold, Auto, Brake, etc.)
│   ├── gcs/                    # MAVLink handler, command dispatcher, mission executor
│   ├── params/                 # Persistent ParamManager
│   ├── safety/                 # Failsafe engine, pre-flight checks, watchdogs
│   ├── sensor_manager/         # Sensor voter, health monitor, hot-switching
│   ├── calibration/            # Accel 6-point, Gyro static, Mag ellipsoid, Radio cal
│   ├── rc_manager/             # RC channel mapping, deadbands, expo
│   ├── logging/                # High-rate binary ULog flight data logger
│   └── simulation/             # Physics simulation models and sensor noise injectors
│
├── platforms/                  # Target hardware platforms
│   ├── stm32h7/                # STM32H743 Cortex-M7 bare-metal target
│   │   ├── toolchain-arm-none-eabi.cmake
│   │   ├── boards/             # Concrete board support folders
│   │   │   ├── apx_h743_v1/    # Reference board support
│   │   │   └── custom_h7_fc/   # Example custom board
│   │   └── default_params/     # Board-specific default parameter overrides
│
├── sitl/                       # Software-In-The-Loop simulation
│   ├── gazebo_plugin/          # Gazebo Harmonic multirotor physics system plugin
│   ├── sitl_bridge/            # UDP/TCP simulator telemetry bridge
│   ├── models/                 # SDF models (apx_quad, apx_hex) and agricultural world
│   └── sim_vehicle.py          # SITL simulation launcher script
│
├── tests/                      # Automated unit tests
│   └── unit/                   # GoogleTest suites (PID, Mixer, Voter, Filters, Modes, Sprayer)
│
├── tools/                      # Development & generation tools
│   └── generate_board.py       # Hardware definition parser & board generator
│
├── scripts/                    # Documentation and CI validation scripts
│   ├── extract_cpp_api.py      # C++ API Markdown extractor
│   ├── validate-docs.py        # Documentation link & syntax validator
│   └── generate-docs.sh        # Local documentation build script
│
└── docs/                       # Comprehensive documentation
```

---

## 2. Namespace Organization

All code is placed in the root `apx` namespace, partitioned by subsystem:

| Subsystem | Namespace | Description |
|---|---|---|
| Hardware Abstraction | `apx::hal` | Pure virtual hardware interfaces |
| Kernel & Bus | `apx::kernel` | Message bus, task management, synchronization |
| Drivers | `apx::drivers` | Device driver implementations |
| Flight Control | `apx::control` | Rate, Attitude, Position PIDs, Motor Mixer |
| State Estimation | `apx::estimator` | EKF3, Mahony filter, sensor prefilters |
| Flight Modes | `apx::modes` | ModeManager, flight modes, path tracker |
| Telemetry & GCS | `apx::gcs` | MAVLink encoding/decoding, parameter storage |
| Sensor Management | `apx::sensor` | Redundancy voting, sensor health evaluation |
| Calibration | `apx::calibration` | Sensor calibration algorithms |
| Platform Support | `apx::platform::stm32h7::<board>` | Pin and hardware mapping tables |

---

## 3. Coding Guidelines & Memory Policy

1. **Zero Dynamic Allocation in Real-Time Loops:**
   - No `new`, `malloc`, or dynamic `std::vector` resizing in the 400 Hz control loop or sensor interrupt paths.
   - Fixed-size arrays (`std::array<T, N>`) and pre-allocated static ring buffers are used everywhere.
2. **Deterministic Mathematical Functions:**
   - Float comparisons use `std::fabs(v) < 1e-6f` or explicit deadbands, never direct floating-point equality.
   - Euler and quaternion transformations avoid trigonometric singularities via swing-twist decoupling.
3. **Hardware Independence:**
   - Modules never make direct register writes; all hardware interaction is routed through `apx::hal` interfaces.

---

## 4. C++ Documentation & Comment Standards

APX enforces standard Doxygen / Clang-compatible documentation comments across all public APIs and headers.

### 4.1 Class and Struct Comments
Use triple-slash `///` or multi-line `/** ... */` directly preceding class, struct, or enum declarations:

```cpp
/// @brief Quaternion-based attitude controller converting target attitude to angular rate setpoints.
/// @details Implements swing-twist error decomposition with tilt-angle and angular acceleration limiting.
class AttitudeController {
    ...
};
```

### 4.2 Member Function Comments
Document parameters (`@param`), return values (`@return`), and units:

```cpp
/// @brief Update the rate controller PID loop for all three rotational axes.
/// @param desired_rate Commanded body rates in radians/second [Roll, Pitch, Yaw].
/// @param actual_rate Measured angular velocity from primary IMU in radians/second.
/// @param dt_s Sampling period in seconds (e.g. 0.0025f for 400 Hz).
/// @param torque_out Output array populated with normalized torque setpoints [-1.0, 1.0].
void update(const float desired_rate[3], const float actual_rate[3], float dt_s, float torque_out[3]);
```

### 4.3 Enumeration Comments
Document each enumerator value:

```cpp
/// @brief Supported multirotor airframe geometric configurations.
enum class FrameType {
    QUAD_X = 0,     ///< 4-motor X layout with alternating CW/CCW rotation
    HEX_X = 1,      ///< 6-motor X layout
    OCTO_X = 2      ///< 8-motor X layout
};
```

### 4.4 Internal Implementation Comments
- Keep implementation comments inside `.cpp` files concise and focused on mathematical rationale or physical constraints.
- Avoid obvious restatements of code logic.

