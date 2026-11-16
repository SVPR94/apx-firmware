# Simulated Sensors and Wind (SITL)

SITL generates the sensor data a real APX board would read: up to four
IMU/baro/mag lanes and up to four GNSS receivers, each with its own seeded
error processes. Wind with gusts acts on the vehicle through body drag.

**What flies on what:**

- **Height and climb rate** come from the vertical estimator
  (`modules/estimator/altitude_estimator`). It's a Kalman filter driven by
  the mean of the IMU lanes and corrected by every baro lane, re-zeroed on
  arming like a baro ground calibration. The altitude controller flies on
  it and telemetry reports it, so baro noise and drift show up as they do
  on a real vehicle. Set `SIM_TRUTH_ALT=1` to fly and report simulator
  truth instead.
- **Everything else** (attitude, horizontal position and velocity) is
  still simulator truth. A full estimator in the loop is the next step.

## Command line

```bash
python3 sitl/sim_vehicle.py --vehicle hex_x --sim builtin \
    --sensor-lanes 4 --gps-count 2 --sensor-profile realistic \
    --seed 7 --wind 6,270,1.5 --sensor-log /tmp/apx_sensors.csv
```

| Option | Values | Default | Meaning |
|---|---|---|---|
| `--sensor-lanes N` | 1–4 | 4 | IMU+baro+mag lanes, one IC each |
| `--gps-count N` | 0–4 | 1 | GNSS receivers (e.g. CAN Here3/Here4) |
| `--sensor-profile P` | `ideal`, `realistic`, `harsh` | `realistic` | Error level (see below) |
| `--seed N` | integer | 1 | Seed for all sensor errors and gusts |
| `--wind S,D[,G]` | m/s, deg, m/s | calm | Mean speed, direction it blows **from** (0 = north, 270 = west), gust sigma |
| `--sensor-log FILE` | path | off | CSV of truth and every sensor sample |

The flight-controller binary takes the same settings directly:
`--sensor-lanes=`, `--gps-count=`, `--sensor-profile=`, `--sim-seed=`,
`--sensor-log=`. Out-of-range values are rejected at startup. The built-in
physics runner takes `--wind=` and `--seed=`. The Gazebo plugin reads
`APX_SIM_WIND` and `APX_SIM_SEED` from the environment, and
`sim_vehicle.py` sets both.

### Reproducibility

Every sensor instance has its own random stream, keyed by
(seed, sensor type, instance).

- **Same seed:** the same noise, biases and gust sequence.
- **Lane count:** changing it doesn't change the errors of the other lanes.
- **Unit tests:** the models are bit-for-bit repeatable.
- **Full SITL flights:** not bit-identical between runs. Physics and
  firmware exchange UDP packets in real time, so sample timing varies
  slightly.

## Models

Code: `sitl/sensors/sim_sensors.{hpp,cpp}`, `sitl/physics/wind_model.hpp`.

The default numbers are order-of-magnitude values for consumer MEMS parts
and standard-precision GNSS. They represent the error left **after
calibration**. They are not taken from a datasheet: replace them with the
figures for the parts you actually fit.

| Sensor | Rate | Error sources |
|---|---|---|
| IMU (gyro + accel) | 400 Hz | White noise from a noise density, turn-on bias, bias random walk, scale error, motor vibration, range clipping (±2000 °/s, ±16 g), 16-bit quantisation |
| Baro | 50 Hz | ISA pressure, white noise, turn-on offset (~2.5 m), Gauss-Markov drift (~0.5 m, τ 120 s), static-port dynamic-pressure error, 10 ms latency |
| Mag | 100 Hz | Local earth field rotated into the body, hard/soft iron residual, motor interference proportional to throttle, white noise |
| GNSS | 10 Hz | Common-mode error shared by all receivers (σ 0.8 m, τ 60 s), per-receiver error (σ 0.4 m, τ 30 s), white noise, velocity noise, antenna lever arm, 150 ms latency, optional outages |

Model notes:

- **Vibration** comes from a single source shared by all IMUs: the motors.
  Each motor adds a once-per-revolution line and a blade-pass line. Rotor
  speed is modelled as `rotor_max_hz * sqrt(throttle)`. Each IMU sees that
  source through its own random mounting gain, so the lanes see
  correlated but different vibration, as on a real frame.
- **GNSS common-mode error** models the atmospheric and ephemeris error
  that every receiver on the vehicle shares. Voting between receivers
  cannot remove it.
- **Mag earth field:** the defaults are rough values for the default home
  (Pune: 0.42 G, declination −0.5°, inclination 29°). They are not a WMM
  evaluation; set `MagModelConfig` for other locations.
- **Not modelled yet:** sensor faults (stuck, dead, bias step), temperature
  drift, GNSS geoid separation, RTK.

### Profiles

| Profile | Use |
|---|---|
| `ideal` | Every error source off (rates and range limits kept). Sensors equal truth. |
| `realistic` | The defaults above. |
| `harsh` | 2× noise, 2–3× bias walk, 3× vibration and motor interference, 2× GNSS error, fewer satellites, plus random GNSS outages (MTBF 120 s, 2–8 s each). |

### Wind and drag

Wind is a mean vector plus first-order Gauss-Markov gusts per axis (τ 2 s,
vertical σ half of horizontal). That's the shape of a first-order Dryden
filter, but without Dryden's altitude- and airspeed-dependent length
scales.

The vehicle feels it through body drag, `F = -½ ρ CdA |v_air| v_air`,
where `v_air` is the velocity relative to the air. Accelerometers measure
this drag.

| Airframe | CdA (m²) |
|---|---|
| Built-in quad | 0.04 |
| Built-in hex | 0.05 |
| Built-in octo | 0.07 |
| Gazebo `apx_hex` (14.5 kg) | 0.3 (`<drag_area_m2>` in the SDF) |

These CdA values are estimates. Replace them with values identified from
flight logs.

Drag is active even without wind. On the ground with less thrust than
weight, the built-in physics holds the vehicle in place, so wind doesn't
push a parked vehicle.

### Timing in SITL

Sensors are sampled from the latest physics truth each time it changes.
An effective rate can therefore fall below nominal when truth arrives
less often than the nominal rate:

- the built-in runner delivers ~373 Hz;
- with Gazebo, the firmware loop reads ~333 Hz.

Measured rates are in the validation table below.

## Telemetry

| MAVLink message | Content |
|---|---|
| `HIGHRES_IMU` (10 Hz per lane, `id` = lane) | Accel, gyro, mag, pressure, pressure altitude, temperature |
| `GPS_RAW_INT` | Receiver 1 (fix, satellites, HDOP as `eph`) |
| `GPS2_RAW` | Receiver 2 |

MAVLink has no standard message for a third or fourth receiver; use the CSV
log for those. With `--gps-count 0`, `GPS_RAW_INT` reports `NO_GPS`.

## CSV log

Row formats are documented in `sitl/sensors/sim_sensor_log.hpp`:

- `T`: truth;
- `I`/`B`/`M`/`G`: IMU, baro, mag and GNSS samples, with their lane.

Each sensor row carries two times:

- **Write time:** when the flight code first sees the sample.
- **Sample time:** when the sample was taken.

The difference between them is the modelled latency.

## Validation

Unit tests in `tests/unit/test_sim_sensors.cpp`:

- profiles and lane clamping;
- `ideal` equals truth;
- seed replay;
- lane independence;
- noise density and bias random-walk statistics;
- vibration frequency;
- clipping;
- baro and GNSS latency;
- GNSS common-mode sharing and outages;
- mag rotation;
- wind direction, gust statistics and replay;
- drag direction.

Results from end-to-end SITL runs with the CSV log (`realistic` profile,
wind 6 m/s from 270° with 1.5 m/s gusts, GUIDED hover at 10 m):

| | Built-in hex, 3 lanes, 4 GNSS, seed 3 | Gazebo `apx_hex`, 4 lanes, 2 GNSS, seed 5 |
|---|---|---|
| IMU rate | 373 Hz | 333 Hz |
| Baro rate | 48.7 Hz | 50.0 Hz |
| Mag rate | 94.8 Hz | 90.9 Hz |
| GNSS rate | 10.0 Hz | 10.0 Hz |
| Baro latency | 10.7 ms | 11.0 ms |
| GNSS latency | 151 ms | 151 ms |
| Gyro σ at rest | 0.0008 rad/s | 0.0008 rad/s |
| Baro offset per lane | +2.7 / +0.8 / +2.6 m | +2.0 / +0.6 / −1.3 / −0.2 m |
| GNSS north error (mean, shared) | +1.1 … +1.8 m | +0.7 … +1.1 m |
| Airspeed / groundspeed in hover | 6.1 / 0.05 m/s | 5.5 / 0.04 m/s |
| Hover position spread | 0.18 m | 0.15 m |
