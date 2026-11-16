# APX Flight Controller — Full Parameter Reference Guide

> **Version**: 0.1.0  
> **Protocol**: MAVLink v2 (Compatible with APX Flight Manager and Ground Control Station)

This document provides a reference for all tuning, configuration, and failsafe parameters in APX.

---

## 1. System Configuration (`SYS_`)

| Parameter | Default | Range | Description |
|---|---|---|---|
| `SYS_VEHICLE` | `quad_x` | Enum | Frame geometry (`quad_x`, `quad_plus`, `hex_x`, `octo_x`) |
| `SYS_ARMING_CHECK` | `1` | `0 - 1` | Preflight safety check enforcement (1=Enforce, 0=Bypass) |
| `SYS_FC_RATE_HZ` | `1000` | `400 - 2000` | Inner rate controller loop frequency in Hz |
| `SYS_DEBUG_LEVEL` | `1` | `0 - 3` | Console log verbosity level (0=Off, 1=Errors, 2=Info, 3=Debug) |

---

## 2. Sensor Redundancy & Voting (`SENS_`)

| Parameter | Default | Range | Description |
|---|---|---|---|
| `SENS_IMU_COUNT` | `2` | `1 - 4` | Active redundant IMU sensor count |
| `SENS_IMU_GATE` | `0.5` | `0.1 - 2.0` | Concord gate in m/s^2 before flagging discordant sensor |
| `SENS_BARO_GATE` | `2.0` | `0.5 - 5.0` | Barometric altitude deviation tolerance in meters |
| `SENS_HEALTH_TMO` | `200000` | `50000 - 1000000` | Stale sensor timeout in microseconds (200ms) |

---

## 3. Aerospace 24-State EKF3 Estimator (`EKF3_`)

| Parameter | Default | Range | Description |
|---|---|---|---|
| `EKF3_GNOISE` | `0.001` | `0.0001 - 0.05` | Gyroscope process noise (rad/s) |
| `EKF3_ANOISE` | `0.01` | `0.001 - 0.5` | Accelerometer process noise (m/s^2) |
| `EKF3_GBNOISE` | `0.0001` | `0.00001 - 0.01` | Gyro bias random walk rate (rad/s^2) |
| `EKF3_ABNOISE` | `0.001` | `0.0001 - 0.05` | Accel bias random walk rate (m/s^3) |
| `EKF3_WNOISE` | `0.05` | `0.005 - 0.5` | Wind estimation process noise (m/s^2) |
| `EKF3_GPSNOISE` | `1.5` | `0.02 - 10.0` | Standard GPS horizontal accuracy (Set to 0.02 for RTK Fixed) |
| `EKF3_GVNOISE` | `0.1` | `0.01 - 1.0` | GPS velocity measurement noise (m/s) |
| `EKF3_BARNOISE` | `1.0` | `0.1 - 5.0` | Barometric altitude noise standard deviation (m) |
| `EKF3_MAGNOISE` | `0.05` | `0.01 - 0.5` | Magnetometer measurement noise (Gauss) |
| `EKF3_GPSGATE` | `5.0` | `1.0 - 20.0` | Chi-squared innovation gate multiplier for GPS |
| `EKF3_BARGATE` | `4.0` | `1.0 - 15.0` | Chi-squared innovation gate multiplier for Baro |

---

## 4. Digital Filter Pipeline (`FILT_`)

| Parameter | Default | Range | Description |
|---|---|---|---|
| `FILT_GYR_LPF` | `50.0` | `10.0 - 200.0` | 2nd-order Butterworth gyro cutoff frequency (Hz) |
| `FILT_ACC_LPF` | `20.0` | `5.0 - 100.0` | 2nd-order Butterworth accelerometer cutoff frequency (Hz) |
| `FILT_NOTCH_EN` | `1` | `0 - 1` | Enable dynamic motor vibration harmonic notch filter |
| `FILT_NOTCH_FREQ`| `80.0` | `20.0 - 300.0` | Fundamental motor notch center frequency (Hz) |
| `FILT_NOTCH_BW` | `20.0` | `5.0 - 60.0` | Notch filter attenuation bandwidth (Hz) |
| `FILT_NOTCH_HARM`| `2` | `1 - 4` | Number of motor RPM harmonics to filter |
| `FILT_NOTCH_TRK` | `1` | `0 - 1` | Enable dynamic RPM tracking from DShot ESC telemetry |

---

## 5. Rate & Attitude PID Controllers (`RATE_`, `ATT_`)

| Parameter | Default | Range | Description |
|---|---|---|---|
| `RATE_ROLL_P` | `0.135` | `0.02 - 0.5` | Roll axis rate proportional gain |
| `RATE_ROLL_I` | `0.135` | `0.02 - 0.5` | Roll axis rate integral gain |
| `RATE_ROLL_D` | `0.003` | `0.0005 - 0.05` | Roll axis rate derivative gain |
| `RATE_PITCH_P`| `0.135` | `0.02 - 0.5` | Pitch axis rate proportional gain |
| `RATE_PITCH_I`| `0.135` | `0.02 - 0.5` | Pitch axis rate integral gain |
| `RATE_PITCH_D`| `0.003` | `0.0005 - 0.05` | Pitch axis rate derivative gain |
| `RATE_YAW_P` | `0.180` | `0.05 - 1.0` | Yaw axis rate proportional gain |
| `RATE_YAW_I` | `0.018` | `0.005 - 0.2` | Yaw axis rate integral gain |
| `ATT_ROLL_P` | `4.5` | `1.0 - 12.0` | Roll attitude P gain (quaternion error) |
| `ATT_PITCH_P` | `4.5` | `1.0 - 12.0` | Pitch attitude P gain |
| `ATT_YAW_P` | `4.0` | `1.0 - 10.0` | Yaw attitude P gain |
| `ATT_TILT_MAX`| `35.0` | `10.0 - 60.0` | Maximum tilt angle limit in degrees |

---

## 6. Agricultural Flight Modes (`AGRI_`)

| Parameter | Default | Range | Description |
|---|---|---|---|
| `AGRI_PUMP_RATE` | `0.6` | `0.0 - 1.0` | Nominal spray pump throttle (0.0=Off, 1.0=Full) |
| `AGRI_SPEED_COMP`| `1` | `0 - 1` | Proportional spray volume compensation based on groundspeed |
| `AGRI_SPRAY_ALT` | `2.5` | `0.8 - 10.0` | Target AGL altitude for crop dusting (meters) |
| `AGRI_ZZ_ROW_SP` | `5.0` | `1.0 - 25.0` | Parallel row spacing for autonomous ZigZag coverage (meters) |
| `AGRI_ZZ_TURN_SPD`| `2.0` | `0.5 - 6.0` | Boundary turnaround velocity in m/s |
| `AGRI_ZZ_ALT_LK` | `0` | `0 - 1` | 0=Pilot manual altitude override enabled, 1=Lock to target |

---

## 7. Failsafe & Recovery (`SAFE_`)

| Parameter | Default | Range | Description |
|---|---|---|---|
| `SAFE_FS_RC` | `2` | `0 - 3` | RC loss action (0=Warn, 1=Land, 2=RTL, 3=Hold) |
| `SAFE_FS_BATT` | `2` | `0 - 3` | Low battery threshold action (Default: Return to Launch) |
| `SAFE_BATT_WARN`| `20.0` | `5.0 - 40.0` | Low battery percentage warning threshold (%) |
| `SAFE_BATT_CRIT`| `10.0` | `3.0 - 25.0` | Critical battery percentage landing trigger (%) |
| `SAFE_RTL_ALT` | `15.0` | `5.0 - 100.0` | Climb-to altitude before returning home (meters) |
| `SAFE_GEO_EN` | `1` | `0 - 1` | Geofence protection boundary (1=Enabled) |
| `SAFE_GEO_RAD` | `500.0` | `50.0 - 5000.0`| Maximum flight radius from takeoff point (meters) |
