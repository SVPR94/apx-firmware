# APX Testing & Quality Assurance Guide

This document describes the automated testing strategy, unit test suites, integration simulation tests, and guidelines for adding tests in the APX project.

---

## 1. Testing Philosophy

APX employs multi-tier verification before any code is approved for real flight:
1. **Deterministic Unit Tests:** GoogleTest suites verifying controllers, estimators, filters, and safety state machines.
2. **SITL Integration Tests:** End-to-end flight simulation testing takeoff, navigation, sprayer actuation, braking, and landing.
3. **Continuous Integration (CI):** Automated GitHub Actions running on every commit.

---

## 2. Unit Test Suite (`tests/unit/`)

All unit tests are built under the `sitl` preset and execute via the unified test binary:

```bash
# Build test executable
cmake --preset sitl
cmake --build build/sitl --target run_unit_tests

# Run all 28 unit tests
./build/sitl/tests/unit/run_unit_tests
```

### 2.1 Test Suite Coverage Summary

| Test Suite | File | Verified Features |
|---|---|---|
| `PidControllerTest` | `test_pid_controller.cpp` | Step response, anti-windup clamping, dynamic D-term LPF, output saturation |
| `MotorMixerTest` | `test_motor_mixer.cpp` | Quad-X & Hex-X mixing, yaw headroom allocation, full-vector desaturation |
| `SensorVoterTest` | `test_sensor_voter.cpp` | Tri-redundant consensus, outlier rejection, degraded sensor exclusion |
| `ComplementaryFilterTest` | `test_complementary_filter.cpp` | Level and pitch angle convergence, gyro bias rejection |
| `PathTrackerTest` | `test_path_tracker.cpp` | S-curve trajectory deceleration, corner speed capping, U-turn braking |
| `ModeBrakeTest` | `test_mode_brake_sprayer.cpp` | Immediate station keeping upon command, stick override refusal |
| `SprayerTest` | `test_mode_brake_sprayer.cpp` | PWM modulation (1100-1900us), disarm cutoff safety, spinner control |
| `ModeAutoTest` | `test_mode_brake_sprayer.cpp` | Autonomous mission execution with embedded `DO_SET_SERVO` pump triggers |

---

## 3. Running Specific Unit Tests

You can filter GoogleTest suites with `--gtest_filter`:

```bash
# Run only PathTracker and Braking tests
./build/sitl/tests/unit/run_unit_tests --gtest_filter="PathTrackerTest.*:ModeBrakeTest.*"

# Run only Sprayer PWM tests
./build/sitl/tests/unit/run_unit_tests --gtest_filter="SprayerTest.*"
```

---

## 4. Writing New Unit Tests

New test cases should be added under `tests/unit/` using standard GoogleTest syntax:

```cpp
#include <gtest/gtest.h>
#include "modules/flight_control/rate_controller/pid_controller.hpp"

TEST(MyControllerTest, ClampingBehavior) {
    apx::control::PidController pid(1.0f, 0.5f, 0.05f, 1.0f);
    pid.set_output_limits(-0.5f, 0.5f);
    
    float out = pid.compute(10.0f, 0.0025f);
    EXPECT_LE(out, 0.5f);
    EXPECT_GE(out, -0.5f);
}
```

Add your test file to `tests/unit/CMakeLists.txt` and re-run `run_unit_tests`.

---

## 5. Integration Simulation Tests

Automated Python flight scripts test real-time MAVLink telemetry and control:

```bash
# Test guided takeoff, square trajectory, and landing
python3 sitl/test_flight.py --vehicle quad_x

# Test APX Flight Manager mission execution and sprayer pump control
python3 sitl/test_gcs_mission_and_joystick.py
```
