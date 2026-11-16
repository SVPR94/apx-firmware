# Flight Control API Reference

Namespace: `apx::control`

## Summary

This page documents all public classes, data structures, and enumerations for `Flight Control`.

| Symbol | Kind | Declared In | Description |
|---|---|---|---|
| [`AttitudeController`](#attitudecontroller) | `class` | `modules/flight_control/attitude_controller/attitude_controller.hpp` | Class definition. |
| [`FrameType`](#frametype) | `enum` | `modules/flight_control/mixer/motor_geometry.hpp` | Enum definition. |
| [`MotorConfig`](#motorconfig) | `struct` | `modules/flight_control/mixer/motor_geometry.hpp` | Struct definition. |
| [`MotorMixer`](#motormixer) | `class` | `modules/flight_control/mixer/motor_mixer.hpp` | Class definition. |
| [`PidController`](#pidcontroller) | `class` | `modules/flight_control/rate_controller/pid_controller.hpp` | Class definition. |
| [`PositionController`](#positioncontroller) | `class` | `modules/flight_control/position_controller/position_controller.hpp` | Class definition. |
| [`RateController`](#ratecontroller) | `class` | `modules/flight_control/rate_controller/rate_controller.hpp` | Class definition. |

---

## AttitudeController
**Kind:** `class`  
**Namespace:** `apx::control`  
**Header:** `modules/flight_control/attitude_controller/attitude_controller.hpp`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `AttitudeController()` | Member function. |
| `void` | `update(const float q_desired[4], const float q_actual[4], float dt_s, float desired_rate_rads[3])` | Member function. |
| `void` | `set_proportional_gain(float p_roll, float p_pitch, float p_yaw)` | Member function. |
| `void` | `set_rate_limits(float max_roll_pitch, float max_yaw)` | Member function. |
| `void` | `set_accel_limits(float max_accel_rp_rads2, float max_accel_y_rads2)` | pre-existing unslewed behavior. |
| `void` | `reset()` | from a stale previous-rate value instead of starting fresh. |

---

## FrameType
**Kind:** `enum`  
**Namespace:** `apx::control`  
**Header:** `modules/flight_control/mixer/motor_geometry.hpp`  

### Enumerators
| Value | Description |
|---|---|
| `QUAD_X` | Enumerator value |
| `QUAD_PLUS` | Enumerator value |
| `HEX_X` | Enumerator value |
| `HEX_FLAT` | Enumerator value |
| `OCTO_X` | Enumerator value |
| `OCTO_COAX` | Enumerator value |
| `OCTO_PLUS` | Enumerator value |

---

## MotorConfig
**Kind:** `struct`  
**Namespace:** `apx::control`  
**Header:** `modules/flight_control/mixer/motor_geometry.hpp`  

---

## MotorMixer
**Kind:** `class`  
**Namespace:** `apx::control`  
**Header:** `modules/flight_control/mixer/motor_mixer.hpp`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `MotorMixer()` | Member function. |
| `void` | `mix(float thrust, float roll, float pitch, float yaw, FrameType frame, float outputs[8])` | Member function. |

---

## PidController
**Kind:** `class`  
**Namespace:** `apx::control`  
**Header:** `modules/flight_control/rate_controller/pid_controller.hpp`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `PidController(float kp, float ki, float kd, float output_limit = 1.0f)` | Member function. |
| `float` | `compute(float error, float dt_s)` | Member function. |
| `float` | `compute_with_derivative(float error, float measured_derivative, float dt_s)` | Member function. |
| `void` | `reset()` | Member function. |
| `void` | `set_gains(float kp, float ki, float kd)` | Member function. |
| `void` | `set_output_limits(float min, float max)` | Member function. |
| `void` | `set_integral_limit(float limit)` | Member function. |
| `void` | `set_derivative_lpf_cutoff(float hz)` | tracks accurately regardless of the update rate. |

---

## PositionController
**Kind:** `class`  
**Namespace:** `apx::control`  
**Header:** `modules/flight_control/position_controller/position_controller.hpp`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `PositionController()` | Member function. |
| `void` | `update_horizontal(const float pos_target[2], const float pos_actual[2], const float vel_target[2], const float vel_actual[2], float dt, float accel_target[2])` | Member function. |
| `void` | `update_altitude(float alt_target, float alt_actual, float vel_target, float vel_actual, float dt, float& thrust_target)` | Member function. |
| `void` | `reset()` | Member function. |
| `void` | `set_horizontal_velocity_gains(float kp, float ki, float kd)` | Member function. |
| `void` | `set_climb_rate_gains(float kp, float ki, float kd)` | Member function. |

---

## RateController
**Kind:** `class`  
**Namespace:** `apx::control`  
**Header:** `modules/flight_control/rate_controller/rate_controller.hpp`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `RateController()` | Member function. |
| `void` | `update(const float desired_rate_rads[3], const float measured_rate_rads[3], float dt_s, float output[3])` | Member function. |
| `void` | `set_gains(int axis, float kp, float ki, float kd, float ff)` | Member function. |
| `void` | `reset()` | Member function. |

---
