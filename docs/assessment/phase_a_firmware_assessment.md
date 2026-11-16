# APX Phase A — Existing Firmware Assessment (Deliverable 1)

Repository state: commit `0d3970e` (working tree clean at time of inspection).
Method: full read of the build system, platform, kernel, HAL, estimator, sensor,
control, mode, safety, MAVLink, parameter, logging, simulation and test code;
the C++ unit tests were built and run. Findings marked **[V]** were checked
directly against the code for this report; the rest come from the same review
and cite file:line so they can be checked the same way.

## 1. Summary

APX's flight behaviour today is that of a **controller flying on perfect
simulator truth**. No state estimator runs anywhere, the STM32H7 build does not
execute the flight stack, and none of the failsafes, arming checks, sensor
voting or vibration filtering described in the README is connected to the
running code.

This matters for the master plan: almost every requested improvement
(estimator choice, lane redundancy, fault detection, disturbance rejection,
STM32H7 timing) can only be *measured* once the loop is closed through
simulated sensors and a real estimator. Comparing INDI against PID, or an
invariant EKF against an ESKF, on noise-free truth would produce numbers with
no bearing on the real vehicle. The highest-value next step is therefore
foundational, not an advanced algorithm.

## 2. Hardware and platform

| Item | Finding |
|---|---|
| MCU | STM32H743 on all boards (`platforms/stm32h7/boards/*/hwdef.dat:3`) **[V]** |
| Memory (v1 linker script) | Flash 2 MB, DTCM 128 KB, AXI SRAM 512 KB; SRAM1–4 and ITCM declared only on pro/custom boards and unused |
| FPU | `-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard` (double-precision capable); no `-fsingle-precision-constant` |
| Clock | **No PLL configuration**: `SystemInit()` only enables the FPU and VTOR, so the core runs on HSI (~64 MHz) instead of 400–480 MHz. Board files disagree on the crystal (8 / 24 / 25 MHz) |
| RTOS | None (bare metal). Zephyr files exist but are not built |
| HAL | Interfaces only; `hal/stm32h7/` is empty. No SPI/I2C/UART/PWM/DShot implementation |
| Firmware entry | `apx_main_stm32h7.cpp`: a free-running loop calling attitude → rate → mixer on constant synthetic inputs. No estimator, modes, MAVLink or outputs |
| Existing image | Pro board ELF ≈ 8.5 KB code — a test loop, not a flight stack **[V]** |
| Fault handlers | Every handler, including HardFault, is a weak alias to an infinite loop |

**Consequence:** no STM32H7 CPU, stack or WCET figure for the flight stack can be
measured today, and none should be quoted. *(Correction, step 1: the pinned Arm
GNU Toolchain 15.2.rel1 was already present under `platforms/stm32h7/toolchain/`
but not on PATH. All three boards now build: ~5 KB flash, 17 KB RAM (v1) for the
current test loop. See `step1_defect_fixes.md`.)*

## 3. Real-time execution

- One loop does all the work, then `delay_us(2500)`. The actual period is
  work time + 2.5 ms, but `dt` is hard-coded to 0.0025 s (`main/apx_main.cpp:224, 350`).
  The real period has not been measured.
- No scheduler (`kernel/task.hpp` has no implementation), no deadline or
  overrun monitor, no CPU-load or stack high-water measurement.
- The bare-metal `now_us()` uses the 32-bit DWT cycle counter assuming 64 MHz,
  with no wrap extension, so it jumps backwards every ~67 s (`kernel/src/timer.cpp:29-62`).
- `Watchdog` exists, but its hardware kick is a stub and nothing instantiates it.
- `ModeManager` is called from both the control loop and the MAVLink network
  thread without a lock (mode changes, guided targets, speed commands).
- Per-tick string-keyed parameter lookups (`MOT_SPIN_ARM` every armed tick)
  and `printf` calls in mode update paths; every mode switch heap-allocates a
  new mode object.

## 4. State estimation and sensors

| Component | Status |
|---|---|
| Estimator in the loop | **None.** SITL copies simulator truth (position, velocity, gyro, quaternion) into `VehicleState` (`main/apx_main.cpp:186-201`) **[V]**. `apx_ekf3` is linked but never called. |
| EKF3 (24 states, float) | Total-state, not error-state. **Covariance propagation is `P += diag(Q)` plus a velocity–position term; there is no F·P·Fᵀ** (`ekf3.cpp:190-215`) **[V]**. Attitude and bias states therefore never correlate with GPS/baro and cannot be corrected by them. Mag prediction does not use the states its H matrix updates. Flow update is a stub. No Joseph form, no alignment, no state NaN/PD checks, GPS validity flags set even when all axes are rejected. |
| UKF (12 states) | Not built. Gyro is integrated without `dt` (`ukf.cpp:235-245`) **[V]**; attitude covariance not propagated; measurement update is not a valid Kalman update. |
| Estimator arbiter (3 lanes) | Not built and would not compile (field-name mismatches). Lanes share one IMU, "NIS" is a signed raw innovation, selection has no hysteresis, and outputs are blended (quaternions linearly). |
| Sensor voter | `compute_deviation()` returns 0 for any non-float type, so every IMU/baro/mag/GPS sample "agrees" and the first valid sensor always wins (`sensor_voter.hpp:61-67`) **[V]**. SensorManager casts every IMU to `Icm42688*` regardless of type. |
| Filters | Not instantiated anywhere. The "Butterworth" LPF uses `alpha = sin(ω)/(2√2)`, i.e. Q = √2, not 0.707 (`noise_filters.hpp:16`) **[V]**. The notch coefficients do not form a notch. No RPM source for the "dynamic notch". |
| Timestamps / delay | Drivers stamp at read time; no GPS/baro delay compensation, no delay parameters. |
| Tests | No estimator, filter or crossbar tests. |

## 5. Control and allocation

- **Cascade:** position-P → velocity PID → acceleration → lean angle →
  quaternion P (swing/twist) → body-rate PID → mixer, all at one rate.
  The rate-loop feedforward exists but every caller passes 0.
- **Integrators restart at zero on every in-flight mode switch**, because
  each switch constructs new controller objects (`mode_manager.cpp:20-34, 63`).
- **Derivative kick on first tick:** `PidController::compute()` starts with
  `m_prev_error = 0` and seeds its filter with that first raw derivative, so
  D = k_d·error/dt on the first tick after any reset.
- **Anti-windup** clamps on the PID's own output only; mixer saturation is not
  fed back, so rate integrators keep winding while the mixer scales roll/pitch.
- **No thrust tilt compensation** (1/cos(tilt)), so altitude sags in turns and
  the hover-thrust learner can absorb tilted-flight thrust.
- **Mixer:** fixed factor tables for QUAD_X, HEX_X and OCTO_X only. QUAD_PLUS,
  HEX_FLAT, OCTO_COAX and OCTO_PLUS return a motor count but a **null table that
  is then dereferenced** (`motor_mixer.cpp:10-13, 110-128`) **[V]**. This is latent
  only because the frame selection currently returns X frames. Per-frame factors
  are not normalised, so the same gains give different authority on each frame.
  No effectiveness matrix, actuator-health notion, thrust linearisation, battery
  compensation or motor-failure handling.
- **Unused parameters:** `ATC_ACCEL_P_MAX`, `FRAME_CLASS`/`FRAME_TYPE`,
  `EK3_ENABLE`, `ARMING_CHECK`, `FS_*`, `BATT_*`.

## 6. Safety and failsafes

| Function | Status |
|---|---|
| Arming checks | None (`can_arm()` always true; `PreflightCheck` is all stubs and unused) |
| RC loss | After 1 s, sticks go neutral (throttle 0.5); no failsafe action. STABILIZE flies on at ~hover thrust |
| GCS loss / battery / geofence / EKF | Parameters registered, nothing implemented; battery reported as a constant 12.6 V / 95 % |
| Crash detection | Implemented: >80° tilt or hard ground impact → disarm. Impact branch requires rel_alt < 0.3 m, which the hex (gear at ~0.6 m) never reaches |
| Land detection | Implemented, but with a hard-coded hover estimate (0.70) instead of `MOT_THST_HOVER` and an altitude threshold relative to home |
| MAVLink disarm | Accepted in flight with no check (stick disarm is now gated) **[V]** |
| `ModeRTL` | Ignores `RTL_ALT` (hard-coded 15 m, `mode_rtl.hpp:39`) **[V]** |

## 7. MAVLink, parameters, logging

- All UDP and TCP links feed **one** parser channel (`m_channel = MAVLINK_COMM_0`)
  **[V]**, so simultaneous links can corrupt each other's frames.
- HEARTBEAT always reports `MAV_TYPE_QUADROTOR` **[V]**, even for hex/octo.
- `SYS_STATUS` reports every sensor present and healthy, unconditionally.
  `GPS_RAW_INT` fix and accuracy fields are constants. ATTITUDE body rates are 0.
- Not sent: `EKF_STATUS_REPORT`, `ESTIMATOR_STATUS`, `VIBRATION`,
  `ATTITUDE_TARGET`, `HOME_POSITION`, `LOCAL_POSITION_NED`, `EXTENDED_SYS_STATE`.
- `mission_type` is ignored, so a fence or rally upload overwrites the mission.
- `REQUEST_DATA_STREAM`, `SET_MESSAGE_INTERVAL`, `PREFLIGHT_STORAGE`, reboot and
  log download are unsupported. `DO_PARACHUTE` and `PREFLIGHT_CALIBRATION`
  return ACCEPTED but do nothing.
- **Parameters are not persisted** (`load/save_to_storage` return true) **[V]**,
  and change callbacks run while the ParamManager mutex is held.
- **No flight logging** is built: `ulog_writer.cpp` is unused and would
  self-deadlock on `open()`; `modules/logging` and `modules/storage` are empty
  or not in the build. (The CSV logger was removed by the revert in `3471aff`.)

## 8. Simulation and tests

| Item | Finding |
|---|---|
| Built-in physics | RK4, 400 Hz, first-order motor lag (τ = 0.02 s), no drag, wind, noise or vibration. `octo_x` silently gets the quad model. *Found in step 1:* its "hex X" was actually a hexa-plus layout not matching the HEX_X mixer, so every built-in hex takeoff flipped (fixed) |
| Gazebo plugin | Thrust ∝ throttle², no motor lag, constant baro, `accel_body` not filled. Hex motor mapping fixed earlier in this session |
| Sensor models | `sitl/sensor_models/` empty; `SensorInjector` discards its noise; `imu_sim.cpp` (noise, bias walk, vibration) exists but is not built |
| Vehicle models | `apx_quad` (2.0 kg), `apx_hex` (14.5 kg); **no octocopter model** |
| Unit tests | 35/35 pass (PID, voter, mixer, complementary filter, path tracker, brake, sprayer, AUTO). None for EKF/UKF, attitude controller, MAVLink, parameters or physics |
| SITL scenario tests | 7 pymavlink scripts (path tracking, speeds, braking, GCS mission/joystick, loiter landing, sprayer). Not run in CI |
| CI | Builds STM32H7 (2 boards), SITL and Linux; runs unit tests only. Release notes state SITL scenarios were verified, which CI does not do |

## 9. Baseline that exists today

These are the only reproducible measurements available, and all are on
**simulator truth with no sensor noise, wind or estimator** (idealised upper
bounds, not vehicle performance):

| Scenario (SITL) | Result |
|---|---|
| AUTO corners mission, quad/hex, 5 m/s | path error max 0.27–0.28 m, final overshoot ≤ 0.13 m |
| AUTO long legs, Gazebo hex | mid-leg cross-track ≤ 0.05 m, max tilt 19.8° |
| DO_CHANGE_SPEED 8 m/s / live 3 m/s | 7.74 / 2.91 m/s cruise |
| Unit tests | 35/35 pass |

Not measurable yet: estimator accuracy and consistency, disturbance rejection
(no wind model), fault detection, any STM32H7 resource or timing figure.

## 10. Gap analysis against the master requirements

| Requirement | Present | Blocking gap |
|---|---|---|
| Stable attitude/rate/position control | Yes, on truth | Untested with noise, delay, vibration or estimator in the loop |
| Robust sensor fusion | No | No estimator in the loop; EKF3 lacks covariance propagation |
| Four-lane redundancy | No | Requires ≥1 correct estimator first; sensor voting is a no-op |
| Fault detection/isolation | No | No innovations or health data to monitor; no failsafe actions |
| Control allocation / rotor failure | Partial (fixed mixer) | No effectiveness matrix; null-table bug on non-X frames |
| Deterministic STM32H7 execution | No | No PLL, HAL, scheduler, timing or deadline monitoring |
| Gazebo validation, quad/hex/octo | Quad/hex only | No octo model; no sensor noise, wind or payload dynamics |
| MAVLink/APX Flight Manager / GCS compatibility | Basic | Shared parser channel; fixed vehicle type; no estimator/health telemetry; params not persisted |
| Logging for analysis | No | Nothing records flight data |

## 11. Recommended order (for decision)

Each step makes the next one measurable.

1. **Fix verified latent defects** (small, low-risk, unit-testable): mixer null
   table, PID first-tick derivative kick, LPF Q, DWT timer wrap, MAVLink
   per-link parser channels, vehicle type in HEARTBEAT, callback-under-lock.
2. **Realistic simulated sensors** in SITL and Gazebo: IMU noise, bias random
   walk, vibration, saturation and latency; baro, mag and GPS models with delay;
   a wind/gust model; seeded randomness. This creates the test environment that
   every later comparison depends on.
3. **Estimator in the loop behind a parameter** (truth vs estimator), starting
   with one well-understood baseline estimator done correctly: full covariance
   propagation, Joseph-form updates, NIS gating, alignment, delayed-measurement
   handling, and consistency tests against simulated truth.
4. **Flight logging and timing instrumentation** (loop period, overruns, WCET
   per module), so both SITL and later hardware runs produce evidence.
5. Only then: **Phase B research scoring and the four-lane / fault-tolerance
   framework**, built on estimator innovations and health data that exist.

The STM32H7 bring-up (PLL, HAL, scheduler, drivers) is a parallel track. It is
required before any resource or WCET claim, and should not be blocked on the
algorithm work.

## 12. Open questions

1. **Toolchain and hardware:** may the ARM toolchain be installed here (CI uses
   `apt install gcc-arm-none-eabi`), and is a physical STM32H743 board with a
   debugger available for cycle-count measurements?
2. **Sensor suite on the real vehicle:** IMU, baro, mag and GPS models and
   counts; rangefinder; ESC telemetry (RPM) for notch filtering; tank-level or
   pump-flow telemetry. Several spraying-disturbance estimators are only
   possible if these exist.
3. **Priority:** SITL estimator-in-the-loop first (recommended) or STM32H7
   bring-up first?
4. **Target vehicle:** is the 14.5 kg hex the primary airframe? Its mass,
   inertia, motor and propeller data define the acceptance thresholds.
