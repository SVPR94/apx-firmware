# APX Git History & Architectural Milestones

This document records the major evolutionary milestones, architectural shifts, and critical bug fixes in the APX codebase.

---

## 1. Initial Architecture & Foundation
- **Microkernel & HAL Design (`afc025b`):**
  - Designed pure virtual HAL interfaces (`IHalUart`, `IHalSpi`, `IHalI2c`, `IHalPwm`, `IHalGpio`).
  - Created zero-allocation publish/subscribe message bus (`apx::kernel::Topic`).
  - Implemented multi-sensor device driver suite (ICM-42688, MS5611, IST8310, u-blox M9/F9P, DShot).

---

## 2. Telemetry & GCS Integration
- **Multi-Port Telemetry & GCS Bridge (`81ca592`, `84fcc87`):**
  - Implemented multi-port telemetry multiplexing across TCP (5760-5763) and UDP (14550).
  - Resolved MAVLink mission transfer issues and fixed Null Island (0,0) coordinate initialization when uploading waypoints from APX Flight Manager.
  - Added support for GCS virtual joystick and MAVLink manual control messages.

---

## 3. Aerospace State Estimation
- **Multi-Estimator Engine (`43322dd`):**
  - Formulated parallel multi-estimator architecture combining 24-state EKF3, Mahony complementary filter, and 15-state UKF.
  - Implemented dynamic sensor crossbar for hot-switching between redundant IMU and Barometer lanes during sensor degradation or vibration spikes.
  - Added Loiter and automated station-keeping algorithms.

---

## 4. Flight Dynamics & Trajectory Improvements
- **S-Curve Navigation & Yaw Headroom Priority (`a4f3b48`, `3471aff`):**
  - Replaced linear waypoint transitions with S-curve leash navigation, limiting lateral acceleration on sharp turns.
  - Decoupled roll/pitch lean stabilization from yaw authority in the motor mixer (`yaw_headroom = max(0.15, 1.0 - rp_spread)`), preventing multirotor wobbles during aggressive turns.
  - Introduced swing-twist quaternion error decoupling in `AttitudeController` to prevent coupled roll/pitch diverge during large simultaneous angle and heading changes.

---

## 5. Industrial & Agricultural Payload Controls
- **Brake Mode & Auxiliary Sprayer Actuation (`8a5cbdd`, `06762f6`):**
  - Implemented dedicated `BRAKE` flight mode (`FlightModeId::BRAKE`), instantly halting horizontal travel and holding position.
  - Routed auxiliary channels 9-12 for sprayer pumps and atomizers.
  - Added support for `MAV_CMD_DO_SET_SERVO` embedded inside autonomous mission items to modulate spray rates (1100-1900 us) over target crop rows.

---

## 6. Multi-Platform CI/CD & Automated Release
- **Continuous Integration Pipeline (`cabe0c8`, `b78fd8c`, `55b1216`, `9a23e2a`):**
  - Implemented GitHub Actions CI matrix building `linux-release`, `sitl`, and bare-metal `stm32h7`.
  - Added automated unit test suite execution with artifact publishing and SHA256 firmware checksums.

---

## 7. Codebase Generalization & Custom Mode Standardization
- **Legacy Flight Controller Decoupling (`d8bc02d`):**
  - Standardized mode mapping methods across `ModeManager` to `to_custom_mode_number()` and `from_custom_mode_number()`.
  - Rewrote internal comments and parameter documentation into clean, modern C++ autopilot conventions.
  - Replaced external simulation plugins with native `apx_gazebo_plugin.so`.
