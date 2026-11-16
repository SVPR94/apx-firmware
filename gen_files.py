# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
import os

def write_file(path, content):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as f:
        f.write(content.strip() + '\n')

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
files = {}

files[os.path.join(ROOT_DIR, 'modules/estimator/attitude_estimator/complementary_filter.hpp'] = """
#pragma once
#include <array>
#include <cstdint>
#include <cmath>

namespace apx::estimator {

struct AttitudeState {
    float quaternion[4]; // w,x,y,z
    float euler_rpy[3];  // roll, pitch, yaw in radians
    float angular_rate[3]; // corrected gyro rates
};

class ComplementaryFilter {
public:
    ComplementaryFilter();
    void update(const float gyro_rads[3], const float accel_ms2[3],
                const float* mag_gauss, // nullptr if no mag
                float dt_s);
    const AttitudeState& get_state() const { return m_state; }
    void set_gains(float kp, float ki) { m_kp = kp; m_ki = ki; }
    void reset();
private:
    AttitudeState m_state;
    float m_integrator[3]{0,0,0};
    float m_kp{2.0f};
    float m_ki{0.005f};
    void quaternion_multiply(const float a[4], const float b[4], float out[4]) const;
    void quaternion_normalize(float q[4]) const;
    void quaternion_to_euler(const float q[4], float rpy[3]) const;
};
} // namespace apx::estimator
"""

files[os.path.join(ROOT_DIR, 'modules/estimator/attitude_estimator/complementary_filter.cpp'] = """
#include "complementary_filter.hpp"

namespace apx::estimator {

ComplementaryFilter::ComplementaryFilter() {
    reset();
}

void ComplementaryFilter::reset() {
    m_state.quaternion[0] = 1.0f;
    m_state.quaternion[1] = 0.0f;
    m_state.quaternion[2] = 0.0f;
    m_state.quaternion[3] = 0.0f;
    for (int i=0; i<3; ++i) {
        m_state.euler_rpy[i] = 0.0f;
        m_state.angular_rate[i] = 0.0f;
        m_integrator[i] = 0.0f;
    }
}

void ComplementaryFilter::update(const float gyro_rads[3], const float accel_ms2[3], const float* mag_gauss, float dt_s) {
    float ax = accel_ms2[0], ay = accel_ms2[1], az = accel_ms2[2];
    float norm = std::sqrt(ax*ax + ay*ay + az*az);
    float ex = 0, ey = 0, ez = 0;
    
    if (norm > 0.0f) {
        ax /= norm; ay /= norm; az /= norm;
        float vx = 2.0f * (m_state.quaternion[1]*m_state.quaternion[3] - m_state.quaternion[0]*m_state.quaternion[2]);
        float vy = 2.0f * (m_state.quaternion[0]*m_state.quaternion[1] + m_state.quaternion[2]*m_state.quaternion[3]);
        float vz = m_state.quaternion[0]*m_state.quaternion[0] - m_state.quaternion[1]*m_state.quaternion[1] - m_state.quaternion[2]*m_state.quaternion[2] + m_state.quaternion[3]*m_state.quaternion[3];
        
        ex = (ay*vz - az*vy);
        ey = (az*vx - ax*vz);
        ez = (ax*vy - ay*vx);
    }
    
    m_integrator[0] += m_ki * ex * dt_s;
    m_integrator[1] += m_ki * ey * dt_s;
    m_integrator[2] += m_ki * ez * dt_s;
    
    float gx = gyro_rads[0] + m_kp * ex + m_integrator[0];
    float gy = gyro_rads[1] + m_kp * ey + m_integrator[1];
    float gz = gyro_rads[2] + m_kp * ez + m_integrator[2];
    
    m_state.angular_rate[0] = gx;
    m_state.angular_rate[1] = gy;
    m_state.angular_rate[2] = gz;
    
    gx *= (0.5f * dt_s);
    gy *= (0.5f * dt_s);
    gz *= (0.5f * dt_s);
    
    float qa[4] = {m_state.quaternion[0], m_state.quaternion[1], m_state.quaternion[2], m_state.quaternion[3]};
    m_state.quaternion[0] += (-qa[1]*gx - qa[2]*gy - qa[3]*gz);
    m_state.quaternion[1] += ( qa[0]*gx + qa[2]*gz - qa[3]*gy);
    m_state.quaternion[2] += ( qa[0]*gy - qa[1]*gz + qa[3]*gx);
    m_state.quaternion[3] += ( qa[0]*gz + qa[1]*gy - qa[2]*gx);
    
    quaternion_normalize(m_state.quaternion);
    quaternion_to_euler(m_state.quaternion, m_state.euler_rpy);
}

void ComplementaryFilter::quaternion_multiply(const float a[4], const float b[4], float out[4]) const {
    out[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    out[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    out[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    out[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}

void ComplementaryFilter::quaternion_normalize(float q[4]) const {
    float norm = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (norm > 0.0f) {
        q[0] /= norm; q[1] /= norm; q[2] /= norm; q[3] /= norm;
    }
}

void ComplementaryFilter::quaternion_to_euler(const float q[4], float rpy[3]) const {
    rpy[0] = std::atan2(2.0f * (q[0]*q[1] + q[2]*q[3]), 1.0f - 2.0f * (q[1]*q[1] + q[2]*q[2]));
    rpy[1] = std::asin(2.0f * (q[0]*q[2] - q[3]*q[1]));
    rpy[2] = std::atan2(2.0f * (q[0]*q[3] + q[1]*q[2]), 1.0f - 2.0f * (q[2]*q[2] + q[3]*q[3]));
}

} // namespace apx::estimator
"""

files[os.path.join(ROOT_DIR, 'modules/estimator/ekf2/ekf2_types.hpp'] = """
#pragma once
#include <array>
#include <cstdint>
namespace apx::estimator {

constexpr int EKF_STATE_SIZE = 16;
constexpr int EKF_COV_SIZE  = EKF_STATE_SIZE;

// State: [q0,q1,q2,q3, vx,vy,vz, px,py,pz, bg_x,bg_y,bg_z, ba_x,ba_y,ba_z]
enum StateIdx { Q0=0,Q1,Q2,Q3, VX,VY,VZ, PX,PY,PZ, BGX,BGY,BGZ, BAX,BAY,BAZ };

struct EKF2State {
    std::array<float, EKF_STATE_SIZE> x{}; // state vector
    std::array<float, EKF_COV_SIZE * EKF_COV_SIZE> P{}; // covariance (16x16 diagonal init)
};

enum class MeasurementType { GPS_POS, GPS_VEL, BARO_ALT, MAG_HEADING, FLOW_VEL };

struct GpsMeasurement { float lat_deg, lon_deg, alt_m; };
struct BaroMeasurement { float altitude_m; };
struct MagMeasurement  { float heading_rad; };

} // namespace apx::estimator
"""

files[os.path.join(ROOT_DIR, 'modules/estimator/ekf2/ekf2.hpp'] = """
#pragma once
#include "ekf2_types.hpp"
namespace apx::estimator {
class EKF2 {
public:
    EKF2();
    void init();
    void predict(const float gyro_rads[3], const float accel_ms2[3], float dt_s);
    void update_gps(const GpsMeasurement& m);
    void update_baro(const BaroMeasurement& m);
    void update_mag(const MagMeasurement& m);
    const EKF2State& get_state() const { return m_state; }
    bool is_healthy() const;
private:
    EKF2State m_state;
    float m_gyro_noise{0.001f};
    float m_accel_noise{0.01f};
    float m_gps_pos_noise{3.0f};
    float m_baro_noise{2.0f};
    bool m_initialized{false};
    // Matrices: diagonal approximation for efficiency
    void state_transition(const float gyro[3], const float accel[3], float dt);
    void covariance_predict(float dt);
    float innovation_check(float innov, float R_diag) const;
};
} // namespace apx::estimator
"""

files[os.path.join(ROOT_DIR, 'modules/estimator/ekf2/ekf2.cpp'] = """
#include "ekf2.hpp"

namespace apx::estimator {

EKF2::EKF2() {
    init();
}

void EKF2::init() {
    m_state.x.fill(0.0f);
    m_state.x[Q0] = 1.0f; // Identity quaternion
    m_state.P.fill(0.0f);
    for (int i = 0; i < EKF_STATE_SIZE; ++i) {
        m_state.P[i * EKF_STATE_SIZE + i] = 1.0f; // Initial diagonal cov
    }
    m_initialized = true;
}

void EKF2::predict(const float gyro_rads[3], const float accel_ms2[3], float dt_s) {
    if (!m_initialized) return;
    state_transition(gyro_rads, accel_ms2, dt_s);
    covariance_predict(dt_s);
}

void EKF2::state_transition(const float gyro[3], const float accel[3], float dt) {
    // quaternion integration
    float q0 = m_state.x[Q0];
    float q1 = m_state.x[Q1];
    float q2 = m_state.x[Q2];
    float q3 = m_state.x[Q3];
    
    float wx = gyro[0] - m_state.x[BGX];
    float wy = gyro[1] - m_state.x[BGY];
    float wz = gyro[2] - m_state.x[BGZ];
    
    m_state.x[Q0] += 0.5f * (-q1*wx - q2*wy - q3*wz) * dt;
    m_state.x[Q1] += 0.5f * ( q0*wx - q3*wy + q2*wz) * dt;
    m_state.x[Q2] += 0.5f * ( q3*wx + q0*wy - q1*wz) * dt;
    m_state.x[Q3] += 0.5f * (-q2*wx + q1*wy + q0*wz) * dt;
    
    // Normalize quaternion
    float norm = __builtin_sqrtf(m_state.x[Q0]*m_state.x[Q0] + m_state.x[Q1]*m_state.x[Q1] + m_state.x[Q2]*m_state.x[Q2] + m_state.x[Q3]*m_state.x[Q3]);
    if (norm > 0) {
        m_state.x[Q0] /= norm; m_state.x[Q1] /= norm; m_state.x[Q2] /= norm; m_state.x[Q3] /= norm;
    }
    
    // Simple position and velocity integration
    float ax = accel[0] - m_state.x[BAX];
    float ay = accel[1] - m_state.x[BAY];
    float az = accel[2] - m_state.x[BAZ];
    
    // Body to earth frame transform simplified here for brevity
    m_state.x[PX] += m_state.x[VX] * dt;
    m_state.x[PY] += m_state.x[VY] * dt;
    m_state.x[PZ] += m_state.x[VZ] * dt;
    
    m_state.x[VX] += ax * dt;
    m_state.x[VY] += ay * dt;
    m_state.x[VZ] += (az - 9.81f) * dt;
}

void EKF2::covariance_predict(float dt) {
    // Add process noise to diagonal elements
    for (int i = 0; i < 4; ++i) m_state.P[i * EKF_STATE_SIZE + i] += m_gyro_noise * dt;
    for (int i = 4; i < 7; ++i) m_state.P[i * EKF_STATE_SIZE + i] += m_accel_noise * dt;
    for (int i = 7; i < 10; ++i) m_state.P[i * EKF_STATE_SIZE + i] += m_accel_noise * dt * dt;
}

void EKF2::update_baro(const BaroMeasurement& m) {
    if (!m_initialized) return;
    float innov = m.altitude_m - (-m_state.x[PZ]); // Assuming NED frame PZ is down
    float H = -1.0f;
    float R = m_baro_noise;
    float P_zz = m_state.P[PZ * EKF_STATE_SIZE + PZ];
    float S = H * P_zz * H + R;
    if (S == 0.0f) return;
    float K = P_zz * H / S;
    
    m_state.x[PZ] += K * innov;
    m_state.P[PZ * EKF_STATE_SIZE + PZ] *= (1.0f - K * H);
}

void EKF2::update_gps(const GpsMeasurement& m) {
    // Simplified update for GPS
}

void EKF2::update_mag(const MagMeasurement& m) {
    // Simplified update for Mag
}

bool EKF2::is_healthy() const {
    return m_initialized;
}

} // namespace apx::estimator
"""

files[os.path.join(ROOT_DIR, 'modules/flight_control/rate_controller/pid_controller.hpp'] = """
#pragma once
namespace apx::control {
class PidController {
public:
    PidController() = default;
    PidController(float kp, float ki, float kd, float output_limit = 1.0f);
    float compute(float error, float dt_s);
    float compute_with_derivative(float error, float measured_derivative, float dt_s);
    void reset();
    void set_gains(float kp, float ki, float kd);
    void set_output_limits(float min, float max);
    void set_integral_limit(float limit);
    void set_derivative_lpf_cutoff(float hz); // low-pass filter on derivative
    float get_integral() const { return m_integral; }
private:
    float m_kp{0}, m_ki{0}, m_kd{0};
    float m_integral{0};
    float m_prev_error{0};
    float m_deriv_filtered{0};
    float m_deriv_alpha{0.8f}; // LPF coefficient
    float m_output_min{-1.0f}, m_output_max{1.0f};
    float m_integral_limit{1.0f};
    float clamp(float val, float min, float max) const;
};
} // namespace apx::control
"""

files[os.path.join(ROOT_DIR, 'modules/flight_control/rate_controller/pid_controller.cpp'] = """
#include "pid_controller.hpp"

namespace apx::control {

PidController::PidController(float kp, float ki, float kd, float output_limit) 
    : m_kp(kp), m_ki(ki), m_kd(kd), m_output_min(-output_limit), m_output_max(output_limit) {}

float PidController::compute(float error, float dt_s) {
    if (dt_s <= 0.0f) return 0.0f;
    float derivative = (error - m_prev_error) / dt_s;
    m_prev_error = error;
    return compute_with_derivative(error, -derivative, dt_s); // Note: typical practice uses negative measured rate for D
}

float PidController::compute_with_derivative(float error, float measured_derivative, float dt_s) {
    if (dt_s <= 0.0f) return 0.0f;
    
    m_integral += error * m_ki * dt_s;
    m_integral = clamp(m_integral, -m_integral_limit, m_integral_limit);
    
    m_deriv_filtered = m_deriv_alpha * m_deriv_filtered + (1.0f - m_deriv_alpha) * measured_derivative;
    
    float output = (error * m_kp) + m_integral - (m_deriv_filtered * m_kd); // D term acts on measurement to avoid kick
    return clamp(output, m_output_min, m_output_max);
}

void PidController::reset() {
    m_integral = 0.0f;
    m_prev_error = 0.0f;
    m_deriv_filtered = 0.0f;
}

void PidController::set_gains(float kp, float ki, float kd) {
    m_kp = kp; m_ki = ki; m_kd = kd;
}

void PidController::set_output_limits(float min, float max) {
    m_output_min = min;
    m_output_max = max;
}

void PidController::set_integral_limit(float limit) {
    m_integral_limit = limit;
}

void PidController::set_derivative_lpf_cutoff(float hz) {
    // simplistic calculation
    m_deriv_alpha = 0.8f; // To be computed properly with dt
}

float PidController::clamp(float val, float min, float max) const {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

} // namespace apx::control
"""

files[os.path.join(ROOT_DIR, 'modules/flight_control/rate_controller/rate_controller.hpp'] = """
#pragma once
#include "pid_controller.hpp"
namespace apx::control {

class RateController {
public:
    RateController();
    void update(const float desired_rate_rads[3], const float measured_rate_rads[3], float dt_s, float output[3]);
    void set_gains(int axis, float kp, float ki, float kd, float ff);
    void reset();
private:
    PidController m_pid[3];
    float m_ff_gain[3]{0,0,0};
};

} // namespace apx::control
"""

files[os.path.join(ROOT_DIR, 'modules/flight_control/rate_controller/rate_controller.cpp'] = """
#include "rate_controller.hpp"

namespace apx::control {

RateController::RateController() {
    for (int i=0; i<3; ++i) {
        m_pid[i].set_output_limits(-1.0f, 1.0f);
    }
}

void RateController::update(const float desired_rate_rads[3], const float measured_rate_rads[3], float dt_s, float output[3]) {
    for (int i=0; i<3; ++i) {
        float error = desired_rate_rads[i] - measured_rate_rads[i];
        float pid_out = m_pid[i].compute_with_derivative(error, measured_rate_rads[i], dt_s);
        float ff_out = m_ff_gain[i] * desired_rate_rads[i];
        
        output[i] = pid_out + ff_out;
        if (output[i] > 1.0f) output[i] = 1.0f;
        if (output[i] < -1.0f) output[i] = -1.0f;
    }
}

void RateController::set_gains(int axis, float kp, float ki, float kd, float ff) {
    if (axis >= 0 && axis < 3) {
        m_pid[axis].set_gains(kp, ki, kd);
        m_ff_gain[axis] = ff;
    }
}

void RateController::reset() {
    for(int i=0; i<3; ++i) m_pid[i].reset();
}

} // namespace apx::control
"""

files[os.path.join(ROOT_DIR, 'modules/flight_control/attitude_controller/attitude_controller.hpp'] = """
#pragma once

namespace apx::control {

class AttitudeController {
public:
    AttitudeController();
    void update(const float q_desired[4], const float q_actual[4], float desired_rate_rads[3]);
    void set_proportional_gain(float p_roll, float p_pitch, float p_yaw);
    void set_rate_limits(float max_roll_pitch, float max_yaw);
private:
    float m_p_gain[3]{4.5f, 4.5f, 4.5f};
    float m_max_rate_rp{1.57f};
    float m_max_rate_y{0.78f};
    float clamp(float val, float min, float max) const;
};

} // namespace apx::control
"""

files[os.path.join(ROOT_DIR, 'modules/flight_control/attitude_controller/attitude_controller.cpp'] = """
#include "attitude_controller.hpp"
#include <cmath>

namespace apx::control {

AttitudeController::AttitudeController() {}

void AttitudeController::update(const float q_desired[4], const float q_actual[4], float desired_rate_rads[3]) {
    // q_err = q_desired * conj(q_actual)
    float qa[4] = {q_actual[0], -q_actual[1], -q_actual[2], -q_actual[3]};
    float qe[4];
    qe[0] = q_desired[0]*qa[0] - q_desired[1]*qa[1] - q_desired[2]*qa[2] - q_desired[3]*qa[3];
    qe[1] = q_desired[0]*qa[1] + q_desired[1]*qa[0] + q_desired[2]*qa[3] - q_desired[3]*qa[2];
    qe[2] = q_desired[0]*qa[2] - q_desired[1]*qa[3] + q_desired[2]*qa[0] + q_desired[3]*qa[1];
    qe[3] = q_desired[0]*qa[3] + q_desired[1]*qa[2] - q_desired[2]*qa[1] + q_desired[3]*qa[0];
    
    // Convert to axis-angle
    if (qe[0] < 0.0f) {
        qe[0] = -qe[0]; qe[1] = -qe[1]; qe[2] = -qe[2]; qe[3] = -qe[3];
    }
    
    float angle = 2.0f * std::acos(clamp(qe[0], -1.0f, 1.0f));
    float sin_half = std::sin(angle / 2.0f);
    
    float axis[3] = {0,0,0};
    if (sin_half > 1e-6f) {
        axis[0] = qe[1] / sin_half;
        axis[1] = qe[2] / sin_half;
        axis[2] = qe[3] / sin_half;
    }
    
    desired_rate_rads[0] = clamp(m_p_gain[0] * angle * axis[0], -m_max_rate_rp, m_max_rate_rp);
    desired_rate_rads[1] = clamp(m_p_gain[1] * angle * axis[1], -m_max_rate_rp, m_max_rate_rp);
    desired_rate_rads[2] = clamp(m_p_gain[2] * angle * axis[2], -m_max_rate_y, m_max_rate_y);
}

void AttitudeController::set_proportional_gain(float p_roll, float p_pitch, float p_yaw) {
    m_p_gain[0] = p_roll; m_p_gain[1] = p_pitch; m_p_gain[2] = p_yaw;
}

void AttitudeController::set_rate_limits(float max_roll_pitch, float max_yaw) {
    m_max_rate_rp = max_roll_pitch;
    m_max_rate_y = max_yaw;
}

float AttitudeController::clamp(float val, float min, float max) const {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

} // namespace apx::control
"""

files[os.path.join(ROOT_DIR, 'modules/flight_control/mixer/motor_geometry.hpp'] = """
#pragma once

namespace apx::control {

enum class FrameType {
    QUAD_X,
    QUAD_PLUS,
    HEX_X,
    HEX_FLAT,
    OCTO_X,
    OCTO_COAX,
    OCTO_PLUS
};

struct MotorConfig {
    float roll_factor;
    float pitch_factor;
    float yaw_factor;
    int index;
};

constexpr MotorConfig QUAD_X_MOTORS[4] = {
    { 1.0f, -1.0f,  1.0f, 0}, // FR
    {-1.0f,  1.0f,  1.0f, 1}, // BL
    { 1.0f,  1.0f, -1.0f, 2}, // FL
    {-1.0f, -1.0f, -1.0f, 3}  // BR
};

constexpr MotorConfig HEX_X_MOTORS[6] = {
    { 0.866f, -0.5f,  1.0f, 0},
    {-0.866f,  0.5f,  1.0f, 1},
    { 0.866f,  0.5f, -1.0f, 2},
    {-0.866f, -0.5f, -1.0f, 3},
    { 0.0f,   -1.0f, -1.0f, 4},
    { 0.0f,    1.0f,  1.0f, 5}
};

constexpr MotorConfig OCTO_X_MOTORS[8] = {
    { 0.383f, -0.924f,  1.0f, 0},
    { 0.924f, -0.383f, -1.0f, 1},
    { 0.924f,  0.383f,  1.0f, 2},
    { 0.383f,  0.924f, -1.0f, 3},
    {-0.383f,  0.924f,  1.0f, 4},
    {-0.924f,  0.383f, -1.0f, 5},
    {-0.924f, -0.383f,  1.0f, 6},
    {-0.383f, -0.924f, -1.0f, 7}
};

} // namespace apx::control
"""

files[os.path.join(ROOT_DIR, 'modules/flight_control/mixer/motor_mixer.hpp'] = """
#pragma once
#include "motor_geometry.hpp"

namespace apx::control {

class MotorMixer {
public:
    MotorMixer();
    void mix(float thrust, float roll, float pitch, float yaw, FrameType frame, float outputs[8]);
private:
    int get_motor_count(FrameType frame) const;
    const MotorConfig* get_motor_configs(FrameType frame) const;
};

} // namespace apx::control
"""

files[os.path.join(ROOT_DIR, 'modules/flight_control/mixer/motor_mixer.cpp'] = """
#include "motor_mixer.hpp"

namespace apx::control {

MotorMixer::MotorMixer() {}

void MotorMixer::mix(float thrust, float roll, float pitch, float yaw, FrameType frame, float outputs[8]) {
    int count = get_motor_count(frame);
    const MotorConfig* configs = get_motor_configs(frame);
    
    if (count == 0) return;

    float min_out = 1.0f;
    float max_out = 0.0f;
    
    for (int i = 0; i < count; ++i) {
        outputs[i] = thrust + roll * configs[i].roll_factor + pitch * configs[i].pitch_factor + yaw * configs[i].yaw_factor;
        if (outputs[i] < min_out) min_out = outputs[i];
        if (outputs[i] > max_out) max_out = outputs[i];
    }
    
    // Desaturation
    float boost = 0.0f;
    if (min_out < 0.0f) {
        boost = -min_out;
    }
    if (max_out + boost > 1.0f) {
        boost -= (max_out + boost - 1.0f);
    }
    
    for (int i = 0; i < count; ++i) {
        outputs[i] += boost;
        if (outputs[i] < 0.0f) outputs[i] = 0.0f;
        if (outputs[i] > 1.0f) outputs[i] = 1.0f;
    }
}

int MotorMixer::get_motor_count(FrameType frame) const {
    switch (frame) {
        case FrameType::QUAD_X: case FrameType::QUAD_PLUS: return 4;
        case FrameType::HEX_X: case FrameType::HEX_FLAT: return 6;
        case FrameType::OCTO_X: case FrameType::OCTO_COAX: case FrameType::OCTO_PLUS: return 8;
        default: return 0;
    }
}

const MotorConfig* MotorMixer::get_motor_configs(FrameType frame) const {
    switch (frame) {
        case FrameType::QUAD_X: return QUAD_X_MOTORS;
        case FrameType::HEX_X: return HEX_X_MOTORS;
        case FrameType::OCTO_X: return OCTO_X_MOTORS;
        default: return nullptr;
    }
}

} // namespace apx::control
"""

files[os.path.join(ROOT_DIR, 'modules/flight_control/position_controller/position_controller.hpp'] = """
#pragma once
#include "../rate_controller/pid_controller.hpp"

namespace apx::control {

class PositionController {
public:
    PositionController();
    void update_horizontal(const float pos_target[2], const float pos_actual[2], 
                           const float vel_target[2], const float vel_actual[2],
                           float dt, float accel_target[2]);
    void update_altitude(float alt_target, float alt_actual, 
                         float vel_target, float vel_actual,
                         float dt, float& thrust_target);
    void reset();
private:
    PidController m_vel_x_pid;
    PidController m_vel_y_pid;
    PidController m_vel_z_pid;
    float m_pos_p{1.0f};
    float m_alt_p{1.0f};
    float m_max_tilt_rad{0.523f}; // 30 degrees
};

} // namespace apx::control
"""

files[os.path.join(ROOT_DIR, 'modules/flight_control/position_controller/position_controller.cpp'] = """
#include "position_controller.hpp"
#include <cmath>

namespace apx::control {

PositionController::PositionController() {
    m_vel_x_pid.set_gains(1.0f, 0.1f, 0.0f);
    m_vel_y_pid.set_gains(1.0f, 0.1f, 0.0f);
    m_vel_z_pid.set_gains(1.5f, 0.2f, 0.05f);
    m_vel_x_pid.set_output_limits(-10.0f, 10.0f);
    m_vel_y_pid.set_output_limits(-10.0f, 10.0f);
    m_vel_z_pid.set_output_limits(0.0f, 1.0f); // Thrust
}

void PositionController::update_horizontal(const float pos_target[2], const float pos_actual[2], 
                                           const float vel_target[2], const float vel_actual[2],
                                           float dt, float accel_target[2]) {
    float vel_sp_x = (pos_target[0] - pos_actual[0]) * m_pos_p + vel_target[0];
    float vel_sp_y = (pos_target[1] - pos_actual[1]) * m_pos_p + vel_target[1];
    
    accel_target[0] = m_vel_x_pid.compute(vel_sp_x - vel_actual[0], dt);
    accel_target[1] = m_vel_y_pid.compute(vel_sp_y - vel_actual[1], dt);
}

void PositionController::update_altitude(float alt_target, float alt_actual, 
                                         float vel_target, float vel_actual,
                                         float dt, float& thrust_target) {
    float vel_sp_z = (alt_target - alt_actual) * m_alt_p + vel_target;
    thrust_target = m_vel_z_pid.compute(vel_sp_z - vel_actual, dt);
}

void PositionController::reset() {
    m_vel_x_pid.reset();
    m_vel_y_pid.reset();
    m_vel_z_pid.reset();
}

} // namespace apx::control
"""

for path, content in files.items():
    write_file(path, content)
