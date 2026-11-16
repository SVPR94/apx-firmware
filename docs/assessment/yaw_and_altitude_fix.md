# Yaw Swing and Over-Steady Altitude

Two problems were reported from SITL flights:

1. Altitude looked unrealistically steady.
2. The nose swung about ±4° in yaw while roll and pitch were fine.

## 1. Yaw swing

**Reproduction.** Yaw was measured from 400 Hz simulator truth on the
straight legs of the AUTO corners mission (more than 10 m from any
waypoint), relative to the leg heading:

| Airframe | Yaw swing, peak-to-peak |
|---|---|
| Quad | up to 5.2° |
| Built-in hex | up to 15.9° |
| Octo | up to 26.3° |
| Gazebo `apx_hex` (14.5 kg) | 15.9–24.4° |

In a hover at constant heading the swing was 0.00°. It appears after a
heading change and decays slowly: period about 3.2 s, damping ratio about
0.2.

**Cause.** The yaw rate loop was slower than the yaw angle loop above it.
A multirotor yaws only through rotor reaction torque, which in both
simulators is 2 % of thrust times the rotor arm. With the mixer's ±1 yaw
factor, `ATC_RAT_YAW_P` = 0.18 closed the yaw rate loop at about:

- 1.0 rad/s on the built-in hex (Izz 0.22 kg·m²);
- 1.3 rad/s on the Gazebo hex (Izz 1.23 kg·m²).

Above that loop, `ATC_ANG_YAW_P` = 4.5/s. The cascade then behaves like a
second-order system with natural frequency ≈ √(4.5 × 1.0) = 2.1 rad/s and
damping ≈ 0.24. That predicts a period of 3.0 s, against 3.2 s measured.

**Fix.** `ATC_RAT_YAW_P` 0.18 → 0.6 and `ATC_RAT_YAW_I` 0.018 → 0.06. The
upper parameter limit was raised to 2.0. The same change was made to the
fallback values in STABILIZE, ALT_HOLD and the navigation cascade.

The gain was chosen by A/B flights on the Gazebo hex:

| Gains | Swing on straight legs |
|---|---|
| Rate P 0.18 (old) | 16–24° |
| Rate P 0.6 | 1.2–1.6° |
| Rate P 1.0, and rate P 1.0 with angle P 3.0 | no better |

Corner smoothness, roll/pitch rates and mission time were unchanged.

**Result** (default parameters, corners mission):

| Airframe | Before | After |
|---|---|---|
| Quad | ≤5.2° | ≤2.4° |
| Built-in hex | ≤15.9° | ≤2.1° |
| Octo | ≤26.3° | ≤1.5° |
| Gazebo hex | 15.9–24.4° | 0.9–1.5° |

Peak yaw rate on the legs fell from up to 36°/s to 2–4°/s. These gains are
tuned on the SITL airframes; tune yaw on the real vehicle before relying
on them.

## 2. Over-steady altitude

**Cause.** The firmware flew on, and reported, the simulator's true
height. In calm air there was no sensor error anywhere in the loop, so in
a settled hover:

- true height moved 5–18 mm peak-to-peak;
- the displayed altitude moved 1–7 mm.

**Fix.** There is a new vertical Kalman filter,
`modules/estimator/altitude_estimator`. It is firmware code with no SITL
dependencies and no heap allocation.

- **States:** height, climb rate, and accelerometer bias along the vertical.
- **Prediction:** the mean of the IMU lanes' specific force, rotated to NED.
- **Correction:** every baro lane, each referenced to its own 1 s startup
  average, with a 5σ innovation gate.
- **Arming:** re-zeroes the height, like a baro ground calibration.

In SITL the altitude controller flies on its height and climb rate, and
`GLOBAL_POSITION_INT`/`VFR_HUD` report them. Attitude and horizontal
position are still simulator truth.

`SIM_TRUTH_ALT=1` restores the truth source (default 0).

**Unit tests** (`tests/unit/test_altitude_estimator.cpp`, 6 tests):

- the start-up reference period;
- tracking a climb within 2 cm and 0.02 m/s;
- learning a 0.3 m/s² accel bias;
- smoothing 0.3 m baro noise to below half of it, without freezing;
- rejecting a 30 m glitch while accepting a lasting shift;
- re-zeroing.

**Result** (settled 40 s hover, GUIDED then LOITER, no wind):

| | Before | After |
|---|---|---|
| True height p-p / σ (Gazebo hex) | 0.018 m / 0.006 m | 0.244 m / 0.053 m |
| True height p-p / σ (built-in hex) | 0.005 m / 0.002 m | 0.246 m / 0.059 m |
| Displayed altitude p-p | 0.001–0.007 m | 0.09–0.13 m |
| Cruise altitude error, AUTO corners | 0.01 m | 0.08–0.09 m |

Landing detection still works. After landing, the display reads a few
centimetres from zero (baro drift), as on hardware. In Gazebo the display
now reads 0 at the arming point, where it used to read the model's 0.6 m
spawn height.

## Regression

All pass with both changes as defaults:

- **Unit tests:** 79/79.
- **AUTO corners:** quad, hex, octo and Gazebo hex.
- **Other AUTO missions:** survey with `DO_CHANGE_SPEED` and RTL, long
  legs, live speed change.
- **Other tests:** waypoint braking (altitude dip 0.20 m, limit 0.45 m),
  per-item mission speed, GCS mission and joystick, loiter landing and
  auto-disarm, sprayer (Gazebo).
- **STM32H7 boards:** compile; images unchanged.

**Invalid runs.** The first corners runs for quad, octo and Gazebo
overlapped laptop suspend/resume cycles (system log: 21:34–21:38, around
22:24 and 23:26). The Gazebo run "never reached cruise" and the octo run
"didn't land" because wall-clock time jumped. Those results were discarded
and all four corners runs repeated with no suspend; they pass.
