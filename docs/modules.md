# APX Subsystem & Module Catalog

This document provides a comprehensive catalog of all core modules, subsystems, and libraries within the APX codebase.

---

## 1. Flight Control (`modules/flight_control`)

### 1.1 Rate Controller (`rate_controller/`)
- **Purpose:** Fast inner-loop angular rate stabilization for Roll, Pitch, and Yaw.
- **Responsibilities:**
  - Evaluates angular rate errors against gyro measurements.
  - Computes torque setpoints in normalized range `[-1.0, 1.0]`.
  - Implements derivative-on-measurement \( \frac{d\omega}{dt} \) to prevent derivative kick upon setpoint steps.
  - Applies single-pole dynamic low-pass filtering with cutoff frequency set by `set_derivative_lpf_cutoff(hz)`.
- **Key Classes:** [`PidController`](api/control.md#pidcontroller), [`RateController`](api/control.md#ratecontroller)
- **Inputs:** Commanded angular rates (rad/s), Gyro rates (rad/s), Time delta \( \Delta t \).
- **Outputs:** Commanded 3-axis torque setpoints.
- **Loop Rate:** 400 Hz.

### 1.2 Attitude Controller (`attitude_controller/`)
- **Purpose:** Converts desired attitude quaternions into commanded body angular rates.
- **Responsibilities:**
  - Performs swing-twist quaternion error decomposition.
  - Controls thrust-vector tilt and yaw heading as independent degrees of freedom.
  - Implements angular acceleration limits (`m_max_accel_rp`, `m_max_accel_y`) to prevent rate spikes during sharp cornering.
- **Key Classes:** [`AttitudeController`](api/control.md#attitudecontroller)
- **Inputs:** Target quaternion \( q_{desired} \), Current quaternion \( q_{actual} \), \( \Delta t \).
- **Outputs:** Commanded body rates `[roll_rate, pitch_rate, yaw_rate]` (rad/s).

### 1.3 Position Controller (`position_controller/`)
- **Purpose:** 3D position and velocity trajectory tracking.
- **Responsibilities:**
  - Computes horizontal velocity targets from position error, capped by `m_max_speed_ms` (`WPNAV_SPEED`).
  - Converts horizontal velocity error to lean tilt angle setpoints (roll/pitch).
  - Regulates vertical climb/descent rate against vertical limits `m_max_climb_speed_ms` and `m_max_descent_speed_ms`.
  - Adapts hover baseline thrust `m_hover_thrust` (`MOT_THST_HOVER`) with a 10s time constant and periodically persists to `ParamManager`.
- **Key Classes:** [`PositionController`](api/control.md#positioncontroller)
- **Inputs:** Target position/velocity (NED), Actual position/velocity (NED), \( \Delta t \).
- **Outputs:** Target thrust `[0.0, 1.0]`, Roll & Pitch lean angle setpoints.

### 1.4 Motor Mixer (`mixer/`)
- **Purpose:** Maps thrust and rotational torque setpoints to motor PWM and DShot duty values.
- **Responsibilities:**
  - Evaluates geometric mixing matrix for standard frames: Quad X, Hex X, Octo X.
  - Scales roll and pitch first to ensure attitude stabilization is never compromised.
  - Binds yaw authority to available headroom (`yaw_headroom = max(0.15, 1.0 - rp_spread)`).
  - Performs whole-vector desaturation to preserve the orientation of commanded rotations.
- **Key Classes:** [`MotorMixer`](api/control.md#motormixer), `MotorGeometry`
- **Outputs:** Motor output values `[0.0, 1.0]` for channels 1..8.

---

## 2. State Estimation (`modules/estimator`)

### 2.1 Extended Kalman Filter (`ekf3/`)
- **Purpose:** Aerospace-grade 24-state error-state Extended Kalman Filter.
- **Responsibilities:**
  - Fuses IMU, GPS (including RTK), Baro altitude, and Magnetometer heading.
  - Estimates 3D position, velocity, orientation quaternion, gyro bias, accel bias, wind velocity, and baro height offset.
  - Chi-squared innovation gating on sensor inputs for robust outlier rejection.
- **Key Classes:** `EKF3`, `EKF3State`, `NavState`

### 2.2 Complementary Attitude Filter (`attitude_estimator/`)
- **Purpose:** Lightweight Mahony attitude observer.
- **Responsibilities:**
  - Fuses high-rate gyro and accel measurements with optional magnetometer heading.
  - Operates as a reliable fallback when GPS or advanced estimator lanes degrade.
- **Key Classes:** [`ComplementaryFilter`](api/estimator.md#complementaryfilter)

### 2.3 Sensor Prefilters (`filters/`)
- **Purpose:** Signal conditioning prior to state estimation.
- **Responsibilities:**
  - 2nd-order Butterworth low-pass filters on gyro and accelerometer signals.
  - Harmonic notch filters tracking motor RPM from ESC telemetry to eliminate propeller vibration harmonics.
- **Key Classes:** `ImuPrefilter`, `ButterworthLPF`, `NotchFilter`, `HarmonicNotchFilter`

---

## 3. Flight Modes & Mission Navigation (`modules/modes`)

### 3.1 Mode Manager (`mode_manager.cpp`)
- **Purpose:** Flight mode lifecycle coordinator and safety interlock.
- **Responsibilities:**
  - Maintains active mode instance and dispatches update ticks.
  - Validates mode entry prerequisites (e.g., GPS 3D fix for POS_HOLD/AUTO).
  - Maps internal `FlightModeId` to standard MAVLink custom mode telemetry numbers (`0=STABILIZE`, `3=AUTO`, `4=GUIDED`, `17=BRAKE`).
- **Key Classes:** [`ModeManager`](api/modes.md#modemanager), [`FlightMode`](api/modes.md#flightmode)

### 3.2 Implemented Flight Modes
- **STABILIZE:** Pilot controls attitude lean angles directly; throttle maps to collective thrust.
- **ALT_HOLD:** Automatic altitude hold; throttle stick centered commands zero climb rate.
- **POS_HOLD:** GPS-assisted position hold; stick deflection commands ground velocity.
- **GUIDED:** Autonomous waypoint and takeoff commands directed by GCS.
- **AUTO:** Mission execution following programmed MAVLink mission items (takeoff, waypoints, loiter, sprayer activation, condition items):
  - **Heading & Yaw Behaviors (`WP_YAW_BEHAVIOR`):** Supports `0=Constant heading`, `1=Face next WP`, `2=Face next WP except RTL`, `3=Along course`, `4=Fixed / Condition yaw`. Configurable initial fixed heading via `AUTO_YAW_HEAD` (0-360° or `-1` for entry heading).
  - **Condition Yaw (`MAV_CMD_CONDITION_YAW` / 115):** When reached, drone halts horizontal translation, rotates on the spot to desired heading (CW/CCW/shortest, absolute/relative) at specified deg/s, holds until target yaw is achieved and gyro rates settle, then locks heading for all subsequent waypoints (side-slipping/strafing as necessary to follow track without turning), RTL approach, and landing.
  - **Condition Altitude (`MAV_CMD_CONDITION_CHANGE_ALT` / 113):** Holds horizontal coordinates while climbing or descending at target rate until within 0.5m of target altitude before advancing.
  - **Speed Changes (`MAV_CMD_DO_CHANGE_SPEED` / 178):** Dynamically updates horizontal cruise speed or vertical climb/descent speed.
  - **Altitude Changes (`MAV_CMD_DO_CHANGE_ALTITUDE` / 186):** Instantaneously updates target cruising altitude.
  - **Sprayer Control:** Responds to `MAV_CMD_DO_SET_SERVO` (183) and `MAV_CMD_DO_SPRAYER` (216) for precision pump rate dosing.
- **BRAKE:** Pauses autonomous flight, immediately braking and holding stationary NED coordinates.
- **RTL:** Ascends to `RTL_ALT`, flies to home position, loiters, and transitions to Land.
- **LAND:** GPS-assisted controlled descent with multi-staged descent rate and automated ground contact detection. Supports pilot manual repositioning (`LAND_REPOS_SPEED`, default 150 cm/s) allowing horizontal XY nudging and heading adjustment via RC sticks while descending to pinpoint touchdowns on landing pads.

### 3.3 Trajectory & Path Tracker (`path_tracker.hpp`)
- **Purpose:** Smooth S-curve trajectory generation between waypoints.
- **Responsibilities:**
  - Slew-rate velocity shaping and braking distance calculations.
  - Fly-through corners (`WPNAV_BLEND=1`, default): a spiral-arc-spiral blend whose curvature ramps
    linearly in and out (the transition curve used in road/rail design). At the planned corner speed the
    lateral acceleration builds at no more than `WPNAV_JERK` and peaks at no more than `WPNAV_ACCEL`; the
    curve passes within the waypoint radius (`WPNAV_RADIUS` or the mission item's acceptance radius) of the
    waypoint and uses at most 45 % of either leg. The corner speed is the highest speed meeting all of these.
    Along-track speed and acceleration stay continuous into and out of the corner.
  - `WPNAV_BLEND=0`: the older circular arc (lateral acceleration steps in at the arc entry), kept for comparison.
  - Independent heading lock (`set_held_yaw()`) and side-slip navigation when `face_along_track` is disabled.

### 3.4 Agricultural Sprayer Subsystem (`sprayer.hpp`)
- **Purpose:** Agricultural spray pump and atomizing spinner controller.
- **Responsibilities:**
  - Controls spray pump via PWM on auxiliary channel 9 (`1100=OFF`, `1900=MAX`).
  - Interlocks with vehicle arming state and speed-dependent dosing.
  - Responds to `MAV_CMD_DO_SET_SERVO` mission items during autonomous spraying.

---

## 4. Telemetry & Communications (`modules/gcs`)

### 4.1 MAVLink Handler (`mavlink_handler.cpp`)
- **Purpose:** Ground Control Station communication over UART and UDP.
- **Responsibilities:**
  - Encodes and streams standard MAVLink telemetry: `HEARTBEAT`, `ATTITUDE`, `GLOBAL_POSITION_INT`, `SYS_STATUS`, `SERVO_OUTPUT_RAW`, `STATUSTEXT`.
  - Decodes `PARAM_REQUEST_LIST`, `PARAM_SET`, `MISSION_ITEM_INT`, `COMMAND_LONG`, `SET_MODE`.
  - Broadcasts standard MAVLink multicopter autopilot dialect for full compatibility with APX Flight Manager and Ground Control Station.

### 4.2 Parameter Manager (`modules/params/param_manager.cpp`)
- **Purpose:** Type-safe non-volatile parameter storage.
- **Responsibilities:**
  - Stores float, integer, and boolean parameters in persistent key-value format.
  - Dispatches parameter change notifications.
  - Implements parameter search, clamping, and default initialization.

---

## 5. Hardware Abstraction Layer (`hal/`)

The HAL defines abstract, pure virtual interfaces implemented across different execution targets:
- [`IHalUart`](api/hal.md#ihaluart): Serial bus I/O with non-blocking reads and RX callbacks.
- [`IHalSpi`](api/hal.md#ihalspi): Full-duplex SPI transfers and chip-select gating.
- [`IHalI2c`](api/hal.md#ihali2c): Register read/write transactions for I2C sensors.
- [`IHalPwm`](api/hal.md#ihalpwm): Multi-channel timer PWM output for ESCs and servos.
- [`IHalGpio`](api/hal.md#ihalgpio): Pin configuration, digital I/O, and EXTI interrupt callbacks.
- [`IHalTimer`](api/hal.md#ihaltimer): Microsecond and millisecond system timekeeping.

---

## 6. Device Drivers (`drivers/`)

- **IMU:** ICM-42688-P, ICM-42605, ICM-20689, ICM-20948, BMI088, BMI270, BMI323.
- **Barometer:** MS5611, BMP388.
- **Magnetometer:** IST8310, RM3100, LIS3MDL.
- **GNSS / RTK:** u-blox M9, u-blox M10, u-blox ZED-F9P (with RTCM injection).
- **Power Monitors:** INA226 (I2C), Analog ADC voltage/current sensors.
- **RC Receivers:** SBUS (inverted UART), CRSF / ExpressLRS (420 kBaud).
- **Actuators:** DShot (150/300/600), PWM Servo, GPIO Relays, Agricultural Sprayers.
