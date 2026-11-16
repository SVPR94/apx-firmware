# State Estimation API Reference

Namespace: `apx::estimator`

## Summary

This page documents all public classes, data structures, and enumerations for `State Estimation`.

| Symbol | Kind | Declared In | Description |
|---|---|---|---|
| [`AlphaBetaTracker`](#alphabetatracker) | `class` | `modules/estimator/filters/noise_filters.hpp` | ─── Alpha-Beta Tracker (aerospace position/velocity estimator) ──────────────── |
| [`AttitudeState`](#attitudestate) | `struct` | `modules/estimator/attitude_estimator/complementary_filter.hpp` | Struct definition. |
| [`BaroMeas`](#baromeas) | `struct` | `modules/estimator/ekf3/ekf3_types.hpp` | Struct definition. |
| [`BaroMeasurement`](#baromeasurement) | `struct` | `modules/estimator/ekf2/ekf2_types.hpp` | Struct definition. |
| [`ButterworthLPF`](#butterworthlpf) | `class` | `modules/estimator/filters/noise_filters.hpp` | ─── 2nd-order Butterworth Low-Pass Filter ──────────────────────────────────── |
| [`ComplementaryFilter`](#complementaryfilter) | `class` | `modules/estimator/attitude_estimator/complementary_filter.hpp` | Class definition. |
| [`EKF2`](#ekf2) | `class` | `modules/estimator/ekf2/ekf2.hpp` | Class definition. |
| [`EKF2State`](#ekf2state) | `struct` | `modules/estimator/ekf2/ekf2_types.hpp` | Struct definition. |
| [`EKF3`](#ekf3) | `class` | `modules/estimator/ekf3/ekf3.hpp` | Class definition. |
| [`EKF3InitParams`](#ekf3initparams) | `struct` | `modules/estimator/ekf3/ekf3_types.hpp` | Struct definition. |
| [`EKF3State`](#ekf3state) | `struct` | `modules/estimator/ekf3/ekf3_types.hpp` | Struct definition. |
| [`EstimatorArbiter`](#estimatorarbiter) | `class` | `modules/estimator/estimator_master/estimator_arbiter.hpp` | Class definition. |
| [`EstimatorLaneId`](#estimatorlaneid) | `enum` | `modules/estimator/estimator_master/estimator_master_types.hpp` | Enum definition. |
| [`FlowMeas`](#flowmeas) | `struct` | `modules/estimator/ekf3/ekf3_types.hpp` | Gauss |
| [`FusedNavSolution`](#fusednavsolution) | `struct` | `modules/estimator/estimator_master/estimator_master_types.hpp` | Struct definition. |
| [`GpsMeas`](#gpsmeas) | `struct` | `modules/estimator/ekf3/ekf3_types.hpp` | Struct definition. |
| [`GpsMeasurement`](#gpsmeasurement) | `struct` | `modules/estimator/ekf2/ekf2_types.hpp` | Struct definition. |
| [`HarmonicNotchFilter`](#harmonicnotchfilter) | `class` | `modules/estimator/filters/noise_filters.hpp` | ─── Harmonic Notch Filter ───────────────────────────────────────────────────── |
| [`IMUPrefilter`](#imuprefilter) | `class` | `modules/estimator/filters/noise_filters.hpp` | Class definition. |
| [`IMUPrefilterParams`](#imuprefilterparams) | `struct` | `modules/estimator/filters/noise_filters.hpp` | ─── IMU Pre-filter Pipeline ────────────────────────────────────────────────── |
| [`LaneDiagnostics`](#lanediagnostics) | `struct` | `modules/estimator/estimator_master/estimator_master_types.hpp` | Struct definition. |
| [`LaneHealthStatus`](#lanehealthstatus) | `enum` | `modules/estimator/estimator_master/estimator_master_types.hpp` | Enum definition. |
| [`MagMeas`](#magmeas) | `struct` | `modules/estimator/ekf3/ekf3_types.hpp` | Struct definition. |
| [`MagMeasurement`](#magmeasurement) | `struct` | `modules/estimator/ekf2/ekf2_types.hpp` | Struct definition. |
| [`MeasurementType`](#measurementtype) | `enum` | `modules/estimator/ekf2/ekf2_types.hpp` | Enum definition. |
| [`MedianFilter`](#medianfilter) | `class` | `modules/estimator/filters/noise_filters.hpp` | Class definition. |
| [`NavState`](#navstate) | `struct` | `modules/estimator/ekf3/ekf3_types.hpp` | Struct definition. |
| [`NotchFilter`](#notchfilter) | `class` | `modules/estimator/filters/noise_filters.hpp` | ─── IIR Notch Filter (bilinear transform) ───────────────────────────────────── |
| [`SensorCrossbar`](#sensorcrossbar) | `class` | `modules/estimator/estimator_master/sensor_crossbar.hpp` | Class definition. |
| [`SensorHealthMetrics`](#sensorhealthmetrics) | `struct` | `modules/estimator/estimator_master/sensor_crossbar.hpp` | Struct definition. |
| [`StateIdx`](#stateidx) | `enum` | `modules/estimator/ekf2/ekf2_types.hpp` | State: [q0,q1,q2,q3, vx,vy,vz, px,py,pz, bg_x,bg_y,bg_z, ba_x,ba_y,ba_z] |
| [`UKF`](#ukf) | `class` | `modules/estimator/ukf/ukf.hpp` | Class definition. |
| [`UkfConfig`](#ukfconfig) | `struct` | `modules/estimator/ukf/ukf_types.hpp` | Struct definition. |
| [`UkfNavState`](#ukfnavstate) | `struct` | `modules/estimator/ukf/ukf_types.hpp` | 25 sigma points |

---

## AlphaBetaTracker
**Kind:** `class`  
**Namespace:** `apx::filters`  
**Header:** `modules/estimator/filters/noise_filters.hpp`  

> ─── Alpha-Beta Tracker (aerospace position/velocity estimator) ────────────────

---

## AttitudeState
**Kind:** `struct`  
**Namespace:** `apx::estimator`  
**Header:** `modules/estimator/attitude_estimator/complementary_filter.hpp`  

---

## BaroMeas
**Kind:** `struct`  
**Namespace:** `apx::estimator`  
**Header:** `modules/estimator/ekf3/ekf3_types.hpp`  

---

## BaroMeasurement
**Kind:** `struct`  
**Namespace:** `apx::estimator`  
**Header:** `modules/estimator/ekf2/ekf2_types.hpp`  

---

## ButterworthLPF
**Kind:** `class`  
**Namespace:** `apx::filters`  
**Header:** `modules/estimator/filters/noise_filters.hpp`  

> ─── 2nd-order Butterworth Low-Pass Filter ────────────────────────────────────

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `cos(omega)` | Member function. |
| `` | `reset()` | Member function. |

---

## ComplementaryFilter
**Kind:** `class`  
**Namespace:** `apx::estimator`  
**Header:** `modules/estimator/attitude_estimator/complementary_filter.hpp`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `ComplementaryFilter()` | Member function. |
| `void` | `update(const float gyro_rads[3], const float accel_ms2[3], const float* mag_gauss, // nullptr if no mag float dt_s)` | Member function. |
| `void` | `reset()` | Member function. |

---

## EKF2
**Kind:** `class`  
**Namespace:** `apx::estimator`  
**Header:** `modules/estimator/ekf2/ekf2.hpp`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `EKF2()` | Member function. |
| `void` | `init()` | Member function. |
| `void` | `predict(const float gyro_rads[3], const float accel_ms2[3], float dt_s)` | Member function. |
| `void` | `update_gps(const GpsMeasurement& m)` | Member function. |
| `void` | `update_baro(const BaroMeasurement& m)` | Member function. |
| `void` | `update_mag(const MagMeasurement& m)` | Member function. |
| `bool` | `is_healthy()` | Member function. |

---

## EKF2State
**Kind:** `struct`  
**Namespace:** `apx::estimator`  
**Header:** `modules/estimator/ekf2/ekf2_types.hpp`  

---

## EKF3
**Kind:** `class`  
**Namespace:** `apx::estimator`  
**Header:** `modules/estimator/ekf3/ekf3.hpp`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `EKF3()` | Member function. |
| `void` | `init(const EKF3InitParams& params = EKF3InitParams{})` | Member function. |
| `void` | `set_home(double lat_deg, double lon_deg, float alt_m)` | Member function. |
| `void` | `predict(const float gyro_rads[3], const float accel_ms2[3], float dt_s)` | IMU prediction step (1kHz) |
| `void` | `update_gps(const GpsMeas& m)` | Measurement update steps |
| `void` | `update_gps_rtk(const GpsMeas& m)` | Member function. |
| `void` | `update_baro(const BaroMeas& m)` | Member function. |
| `void` | `update_mag(const MagMeas& m)` | Member function. |
| `void` | `update_flow(const FlowMeas& m)` | Member function. |
| `bool` | `is_healthy()` | Member function. |

---

## EKF3InitParams
**Kind:** `struct`  
**Namespace:** `apx::estimator`  
**Header:** `modules/estimator/ekf3/ekf3_types.hpp`  

---

## EKF3State
**Kind:** `struct`  
**Namespace:** `apx::estimator`  
**Header:** `modules/estimator/ekf3/ekf3_types.hpp`  

---

## EstimatorArbiter
**Kind:** `class`  
**Namespace:** `apx::estimator`  
**Header:** `modules/estimator/estimator_master/estimator_arbiter.hpp`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `void` | `init(double home_lat, double home_lon, float home_alt)` | Member function. |
| `void` | `predict(const float gyro_body[3], const float accel_body[3], float dt_s, uint64_t now_us)` | High-rate IMU predict step (400 Hz) |
| `void` | `update_gps(double lat_deg, double lon_deg, float alt_m, const float vel_ned_ms[3], float hacc_m, float vacc_m, uint64_t now_us)` | Asynchronous sensor measurement updates |
| `void` | `update_baro(float pressure_pa, float altitude_m, uint64_t now_us)` | Member function. |
| `void` | `update_mag(const float mag_gauss[3], uint64_t now_us)` | Member function. |

---

## EstimatorLaneId
**Kind:** `enum`  
**Namespace:** `apx::estimator`  
**Header:** `modules/estimator/estimator_master/estimator_master_types.hpp`  

### Enumerators
| Value | Description |
|---|---|
| `LANE_EKF3` | Enumerator value |
| `LANE_UKF` | Enumerator value |
| `LANE_INVARIANT` | Enumerator value |
| `COUNT` | Enumerator value |

---

## FlowMeas
**Kind:** `struct`  
**Namespace:** `apx::estimator`  
**Header:** `modules/estimator/ekf3/ekf3_types.hpp`  

> Gauss

---

## FusedNavSolution
**Kind:** `struct`  
**Namespace:** `apx::estimator`  
**Header:** `modules/estimator/estimator_master/estimator_master_types.hpp`  

---

## GpsMeas
**Kind:** `struct`  
**Namespace:** `apx::estimator`  
**Header:** `modules/estimator/ekf3/ekf3_types.hpp`  

---

## GpsMeasurement
**Kind:** `struct`  
**Namespace:** `apx::estimator`  
**Header:** `modules/estimator/ekf2/ekf2_types.hpp`  

---

## HarmonicNotchFilter
**Kind:** `class`  
**Namespace:** `apx::filters`  
**Header:** `modules/estimator/filters/noise_filters.hpp`  

> ─── Harmonic Notch Filter ─────────────────────────────────────────────────────

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `update_fundamental(fundamental_hz)` | Member function. |
| `` | `apply(x)` | Member function. |
| `` | `reset()` | Member function. |

---

## IMUPrefilter
**Kind:** `class`  
**Namespace:** `apx::filters`  
**Header:** `modules/estimator/filters/noise_filters.hpp`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `init(p.gyro_lpf_hz, sample_rate_hz)` | Member function. |
| `` | `init(p.accel_lpf_hz, sample_rate_hz)` | Member function. |
| `` | `init(p.notch_freq_hz, p.notch_harmonics, p.notch_bw_hz, sample_rate_hz)` | Member function. |
| `` | `update_fundamental(freq_hz)` | Member function. |
| `` | `apply(g)` | Member function. |
| `` | `apply(g)` | Member function. |
| `` | `apply(accel_in[i])` | Member function. |

---

## IMUPrefilterParams
**Kind:** `struct`  
**Namespace:** `apx::filters`  
**Header:** `modules/estimator/filters/noise_filters.hpp`  

> ─── IMU Pre-filter Pipeline ──────────────────────────────────────────────────

---

## LaneDiagnostics
**Kind:** `struct`  
**Namespace:** `apx::estimator`  
**Header:** `modules/estimator/estimator_master/estimator_master_types.hpp`  

---

## LaneHealthStatus
**Kind:** `enum`  
**Namespace:** `apx::estimator`  
**Header:** `modules/estimator/estimator_master/estimator_master_types.hpp`  

### Enumerators
| Value | Description |
|---|---|
| `OPTIMAL` | Enumerator value |
| `covariance well conditioned
    ACCEPTABLE` | Enumerator value |
| `nominal operation
    DEGRADED` | Enumerator value |
| `rapidly growing covariance
    FAULTED` | Enumerator value |
| `divergence or timeout` | Enumerator value |

---

## MagMeas
**Kind:** `struct`  
**Namespace:** `apx::estimator`  
**Header:** `modules/estimator/ekf3/ekf3_types.hpp`  

---

## MagMeasurement
**Kind:** `struct`  
**Namespace:** `apx::estimator`  
**Header:** `modules/estimator/ekf2/ekf2_types.hpp`  

---

## MeasurementType
**Kind:** `enum`  
**Namespace:** `apx::estimator`  
**Header:** `modules/estimator/ekf2/ekf2_types.hpp`  

### Enumerators
| Value | Description |
|---|---|
| `GPS_POS` | Enumerator value |
| `GPS_VEL` | Enumerator value |
| `BARO_ALT` | Enumerator value |
| `MAG_HEADING` | Enumerator value |
| `FLOW_VEL` | Enumerator value |

---

## MedianFilter
**Kind:** `class`  
**Namespace:** `apx::filters`  
**Header:** `modules/estimator/filters/noise_filters.hpp`  

---

## NavState
**Kind:** `struct`  
**Namespace:** `apx::estimator`  
**Header:** `modules/estimator/ekf3/ekf3_types.hpp`  

---

## NotchFilter
**Kind:** `class`  
**Namespace:** `apx::filters`  
**Header:** `modules/estimator/filters/noise_filters.hpp`  

> ─── IIR Notch Filter (bilinear transform) ─────────────────────────────────────

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `update_coefficients()` | Member function. |
| `` | `reset()` | Member function. |
| `` | `update_coefficients()` | Member function. |

---

## SensorCrossbar
**Kind:** `class`  
**Namespace:** `apx::estimator`  
**Header:** `modules/estimator/estimator_master/sensor_crossbar.hpp`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `SensorCrossbar()` | Member function. |
| `void` | `report_imu_metrics(int index, float vibration, bool clipped, uint64_t now_us)` | Update real-time health metrics per sensor instance |
| `void` | `report_baro_metrics(int index, float pressure_noise, uint64_t now_us)` | Member function. |
| `void` | `report_mag_metrics(int index, float field_norm_err, uint64_t now_us)` | Member function. |
| `void` | `report_gps_metrics(int index, float hacc, uint8_t num_sats, uint64_t now_us)` | Member function. |
| `void` | `update_routing(uint64_t now_us)` | Evaluate sensor qualities and re-route matrix dynamically |
| `int` | `get_routed_imu(EstimatorLaneId lane)` | Dynamic routing queries for estimator lanes |
| `int` | `get_routed_baro(EstimatorLaneId lane)` | Member function. |
| `int` | `get_routed_mag(EstimatorLaneId lane)` | Member function. |
| `int` | `get_routed_gps(EstimatorLaneId lane)` | Member function. |
| `void` | `force_route_imu(EstimatorLaneId lane, int imu_idx)` | Manual override or testing injection |

---

## SensorHealthMetrics
**Kind:** `struct`  
**Namespace:** `apx::estimator`  
**Header:** `modules/estimator/estimator_master/sensor_crossbar.hpp`  

---

## StateIdx
**Kind:** `enum`  
**Namespace:** `apx::estimator`  
**Header:** `modules/estimator/ekf2/ekf2_types.hpp`  

> State: [q0,q1,q2,q3, vx,vy,vz, px,py,pz, bg_x,bg_y,bg_z, ba_x,ba_y,ba_z]

### Enumerators
| Value | Description |
|---|---|
| `Q0` | Enumerator value |
| `Q1` | Enumerator value |
| `Q2` | Enumerator value |
| `Q3` | Enumerator value |
| `VX` | Enumerator value |
| `VY` | Enumerator value |
| `VZ` | Enumerator value |
| `PX` | Enumerator value |
| `PY` | Enumerator value |
| `PZ` | Enumerator value |
| `BGX` | Enumerator value |
| `BGY` | Enumerator value |
| `BGZ` | Enumerator value |
| `BAX` | Enumerator value |
| `BAY` | Enumerator value |
| `BAZ` | Enumerator value |

---

## UKF
**Kind:** `class`  
**Namespace:** `apx::estimator`  
**Header:** `modules/estimator/ukf/ukf.hpp`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `UKF()` | Member function. |
| `void` | `init(const UkfConfig& config = UkfConfig{})` | Member function. |
| `void` | `set_home(double lat_deg, double lon_deg, float alt_m)` | Member function. |
| `void` | `predict(const float gyro_rads[3], const float accel_ms2[3], float dt_s)` | Dynamic prediction step using Unscented Transform (100Hz - 400Hz) |
| `void` | `update_gps(double lat_deg, double lon_deg, float alt_m, const float vel_ned_ms[3], float hacc_m, float vacc_m)` | Measurement update steps |
| `void` | `update_baro(float altitude_m)` | Member function. |
| `void` | `update_mag(const float mag_gauss[3])` | Member function. |
| `void` | `reset()` | Member function. |

---

## UkfConfig
**Kind:** `struct`  
**Namespace:** `apx::estimator`  
**Header:** `modules/estimator/ukf/ukf_types.hpp`  

---

## UkfNavState
**Kind:** `struct`  
**Namespace:** `apx::estimator`  
**Header:** `modules/estimator/ukf/ukf_types.hpp`  

> 25 sigma points

---
