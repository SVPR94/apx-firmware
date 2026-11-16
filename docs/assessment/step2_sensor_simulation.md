# Step 2 — Simulated Sensors, Wind and Drag

Roadmap step 2 (after the Phase A assessment): realistic simulated sensors
in SITL and Gazebo that match the target vehicle's hardware.

**Target hardware:**

- 4 ICs, each with gyro, accel, mag and baro;
- up to 4 CAN GNSS receivers.

**Scope boundary:** the control loop still flies on simulator truth. Step 3
closes it through an estimator fed by these sensors. User guide:
[docs/simulated_sensors.md](../simulated_sensors.md).

## Changes

| Area | Change |
|---|---|
| `sitl/sensors/` (new) | `SimSensorSuite`: 1–4 IMU/baro/mag lanes and 0–4 GNSS receivers from truth; seeded per-instance error processes; `ideal`/`realistic`/`harsh` profiles; CSV logger. Fixed-size buffers, no heap allocation in the update path. |
| `sitl/physics/wind_model.hpp` (new) | Mean wind plus seeded Gauss-Markov gusts. |
| `sitl/physics/multirotor_dynamics` | Body drag against the wind (`drag_area_m2`), ground friction while thrust < weight, airspeed. |
| `sitl/physics/builtin_physics_runner.cpp` | `--wind=`, `--seed=`; per-airframe drag area; airspeed relative to the air. |
| `sitl/gazebo_plugin` | Wind and drag (`<drag_area_m2>`, `APX_SIM_WIND`, `APX_SIM_SEED`). Fills the previously missing `accel_body` (specific force). Baro pressure now follows altitude instead of a constant sea-level value. |
| `sitl/models/apx_hex`, `apx_quad` | `<drag_area_m2>` 0.3 / 0.04 (estimates). |
| `main/apx_main.cpp` | `--sensor-lanes=`, `--gps-count=`, `--sensor-profile=`, `--sim-seed=`, `--sensor-log=`; runs the suite on each new truth sample. |
| `modules/gcs/mavlink_handler` | `HIGHRES_IMU` per lane (`id` = lane). `GPS_RAW_INT`/`GPS2_RAW` come from simulated receivers 1 and 2 (they were built from vehicle truth). `NO_GPS` when there are no receivers. |
| `sitl/sim_vehicle.py` | `--sensor-lanes`, `--gps-count`, `--sensor-profile`, `--seed`, `--wind`, `--sensor-log`. |
| Tests | `tests/unit/test_sim_sensors.cpp`: 20 tests. |

`sitl/sitl_bridge/sensor_injector.*` is an older stub that computed noisy
values and discarded them. Nothing uses it; it is superseded by
`sitl/sensors/` and can be removed.

## Results

**Unit tests:** 67/67 pass (47 existing + 20 new).

**End-to-end SITL** with the CSV log analysed against truth (`realistic`,
wind 6 m/s from 270° with 1.5 m/s gusts, GUIDED takeoff to 10 m, hover,
land):

| | Built-in hex | Gazebo `apx_hex` |
|---|---|---|
| Setup | 3 lanes, 4 GNSS, seed 3 | 4 lanes, 2 GNSS, seed 5 |
| `HIGHRES_IMU` ids seen | 0, 1, 2 | 0, 1, 2, 3 |
| `GPS_RAW_INT` / `GPS2_RAW` | 3D fix, 20 / 19 sats | 3D fix, 19 / 17 sats |
| IMU / baro / mag / GNSS rate | 373 / 48.7 / 94.8 / 10.0 Hz | 333 / 50.0 / 90.9 / 10.0 Hz |
| Baro / GNSS latency | 10.7 / 151 ms | 11.0 / 151 ms |
| Gyro σ at rest (all lanes) | 0.0008 rad/s | 0.0008 rad/s |
| Airspeed / groundspeed in hover | 6.1 / 0.05 m/s | 5.5 / 0.04 m/s |
| Hover altitude, position spread | 9.99 m, 0.18 m | 10.02 m, 0.15 m |
| Landed and disarmed | yes | yes |

Three Gazebo truth checks:

- **Parked specific force:** (0.000, −0.000, −9.806) m/s².
- **Hover:** body-x specific force of 0.36 m/s². That is the drag term the
  accelerometer should see, about 0.38 m/s² expected at 5.5 m/s airspeed
  with CdA 0.3.
- **Rates below nominal:** the IMU rate is set by how often new truth
  arrives (built-in runner ~373 Hz; Gazebo, through the firmware loop,
  ~333 Hz).

**Flight regression:** all pass with drag active.

| Run | Result |
|---|---|
| AUTO corners, hex | 0.22 / 0.25 / 0.28 m |
| AUTO corners, octo | 0.22 / 0.24 / 0.27 m |
| AUTO corners, quad | 0.22 / 0.25 / 0.28 m |
| AUTO corners, Gazebo hex | 0.21 / 0.27 / 0.30 m |
| Survey with `DO_CHANGE_SPEED` 3 m/s and RTL | pass |
| Loiter landing / auto-disarm | 6/6 |
| Waypoint braking | pass |
| Per-item mission speed | pass |
| GCS mission and joystick | pass |
| Sprayer (Gazebo) | 3/3 |

Two observations from the regression:

- **Cruise speed:** mid-leg cruise is 4.75–4.89 m/s against 5.0 commanded.
  An A/B run with drag set to zero gave the same 4.80 m/s, so this is
  existing path-tracker behaviour, not caused by drag.
- **GCS test warning:** "Vehicle still armed after land timeout". It also
  appeared in the step-1 regression, so it isn't new.

**STM32H7 (MinSizeRel):** no relink. Images are unchanged:

- `apx_h743_v1`: 5,092 B flash;
- `apx_h743_pro` / `custom_h7_fc`: 4,944 B flash.

## Not done in this step

- Sensor fault injection (stuck, dead, bias step, intermittent bus). Needed
  for the four-lane redundancy work; planned with step 5.
- Feeding these sensors into `SensorManager`/the estimator (step 3).
  `SensorManager` currently `static_cast`s every driver to one concrete
  type, and the voter's deviation for struct types is always 0. Both must
  be fixed before it can vote between lanes.
- Temperature drift, GNSS geoid separation, RTK.
