// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "eskf15.hpp"
#include <cmath>
#include <cstring>
#include <algorithm>

namespace apx::estimator {

static constexpr float kGravity = 9.80665f;
static constexpr float kPi = 3.14159265358979323846f;
static constexpr float kDegToRad = kPi / 180.0f;
static constexpr float kRadToDeg = 180.0f / kPi;
static constexpr double kEarthRadiusM = 6378137.0; // WGS-84 equatorial radius

// Helper to wrap angle to [-pi, pi]
static inline float wrap_pi(float angle) {
    while (angle > kPi)  angle -= 2.0f * kPi;
    while (angle < -kPi) angle += 2.0f * kPi;
    return angle;
}

// ─── Quaternion & Attitude Mathematics ────────────────────────────────────────

void ESKF15::quat_multiply(const float a[4], const float b[4], float out[4]) {
    out[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    out[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    out[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    out[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}

void ESKF15::quat_normalize(float q[4]) {
    float mag_sq = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
    if (mag_sq < 1.0e-8f || !std::isfinite(mag_sq)) {
        q[0] = 1.0f; q[1] = 0.0f; q[2] = 0.0f; q[3] = 0.0f;
        return;
    }
    float inv_mag = 1.0f / std::sqrt(mag_sq);
    q[0] *= inv_mag;
    q[1] *= inv_mag;
    q[2] *= inv_mag;
    q[3] *= inv_mag;
}

void ESKF15::quat_to_dcm(const float q[4], float R[9]) {
    float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
    R[0] = 1.0f - 2.0f*(qy*qy + qz*qz);
    R[1] = 2.0f*(qx*qy - qw*qz);
    R[2] = 2.0f*(qx*qz + qw*qy);

    R[3] = 2.0f*(qx*qy + qw*qz);
    R[4] = 1.0f - 2.0f*(qx*qx + qz*qz);
    R[5] = 2.0f*(qy*qz - qw*qx);

    R[6] = 2.0f*(qx*qz - qw*qy);
    R[7] = 2.0f*(qy*qz + qw*qx);
    R[8] = 1.0f - 2.0f*(qx*qx + qy*qy);
}

void ESKF15::quat_to_euler(const float q[4], float rpy[3]) {
    float qw = q[0], qx = q[1], qy = q[2], qz = q[3];

    // Roll (X-axis rotation)
    float sinr_cosp = 2.0f * (qw*qx + qy*qz);
    float cosr_cosp = 1.0f - 2.0f * (qx*qx + qy*qy);
    rpy[0] = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (Y-axis rotation)
    float sinp = 2.0f * (qw*qy - qz*qx);
    if (std::abs(sinp) >= 1.0f) {
        rpy[1] = std::copysign(kPi / 2.0f, sinp); // Gimbal lock clamp
    } else {
        rpy[1] = std::asin(sinp);
    }

    // Yaw (Z-axis rotation)
    float siny_cosp = 2.0f * (qw*qz + qx*qy);
    float cosy_cosp = 1.0f - 2.0f * (qy*qy + qz*qz);
    rpy[2] = std::atan2(siny_cosp, cosy_cosp);
}

void ESKF15::quat_from_euler(float roll, float pitch, float yaw, float q[4]) {
    float cr = std::cos(roll * 0.5f),  sr = std::sin(roll * 0.5f);
    float cp = std::cos(pitch * 0.5f), sp = std::sin(pitch * 0.5f);
    float cy = std::cos(yaw * 0.5f),   sy = std::sin(yaw * 0.5f);

    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
    quat_normalize(q);
}

void ESKF15::geodetic_to_ned(double lat, double lon, float alt, float ned[3]) const {
    double dlat = (lat - m_home_lat) * kDegToRad;
    double dlon = (lon - m_home_lon) * kDegToRad;
    double cos_lat = std::cos(m_home_lat * kDegToRad);
    ned[0] = static_cast<float>(dlat * kEarthRadiusM);
    ned[1] = static_cast<float>(dlon * kEarthRadiusM * cos_lat);
    ned[2] = -(alt - m_home_alt); // Down is positive
}

// ─── Initialization & Reset ──────────────────────────────────────────────────

ESKF15::ESKF15() {
    reset();
}

void ESKF15::init(const ESKFTuningParams& params) {
    m_params = params;
    reset();
    m_initialized = true;
}

void ESKF15::reset() {
    // Nominal state reset
    for (int i = 0; i < 3; ++i) {
        m_state.pos_ned[i] = 0.0f;
        m_state.vel_ned[i] = 0.0f;
        m_state.accel_bias[i] = 0.0f;
        m_state.gyro_bias[i] = 0.0f;
    }
    m_state.quat[0] = 1.0f;
    m_state.quat[1] = 0.0f;
    m_state.quat[2] = 0.0f;
    m_state.quat[3] = 0.0f;

    // Covariance initialization (diagonal)
    m_P.fill(0.0f);
    auto set_p = [this](int idx, float var) {
        m_P[idx * ESKF15_STATE_DIM + idx] = var;
    };

    // Position variances (10 m initial uncertainty)
    set_p(STATE_DPN, 100.0f);
    set_p(STATE_DPE, 100.0f);
    set_p(STATE_DPD, 100.0f);

    // Velocity variances (1 m/s initial uncertainty)
    set_p(STATE_DVN, 1.0f);
    set_p(STATE_DVE, 1.0f);
    set_p(STATE_DVD, 1.0f);

    // Attitude tilt variances (~5.7 deg initial uncertainty)
    set_p(STATE_DTH_X, 0.01f);
    set_p(STATE_DTH_Y, 0.01f);
    set_p(STATE_DTH_Z, 0.04f); // Yaw uncertainty higher (~11.5 deg)

    // Accelerometer bias variances (0.1 m/s^2)
    set_p(STATE_DBA_X, 0.01f);
    set_p(STATE_DBA_Y, 0.01f);
    set_p(STATE_DBA_Z, 0.01f);

    // Gyroscope bias variances (0.01 rad/s)
    set_p(STATE_DBG_X, 0.0001f);
    set_p(STATE_DBG_Y, 0.0001f);
    set_p(STATE_DBG_Z, 0.0001f);

    m_predict_count = 0;
    m_gps_update_count = 0;
    m_baro_update_count = 0;
    m_mag_update_count = 0;

    m_nav.attitude_valid = false;
    m_nav.velocity_valid = false;
    m_nav.position_valid = false;
    m_nav.healthy = true;

    update_nav_state();
}

void ESKF15::set_home(double lat_deg, double lon_deg, float alt_m) {
    m_home_lat = lat_deg;
    m_home_lon = lon_deg;
    m_home_alt = alt_m;
    m_home_set = true;
    m_state.pos_ned[0] = 0.0f;
    m_state.pos_ned[1] = 0.0f;
    m_state.pos_ned[2] = 0.0f;
    update_nav_state();
}

void ESKF15::align_tilt_from_accel(const float accel[3]) {
    float norm = std::sqrt(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
    if (norm < 5.0f || norm > 15.0f) return; // Don't align if under violent motion

    // On ground at rest, specific force opposes gravity: a_b ≈ -R^T * g_ned = [0, 0, -g]
    // Normalized accel:
    float ax = accel[0] / norm;
    float ay = accel[1] / norm;
    float az = accel[2] / norm;

    // Pitch: positive pitch tilts nose up, a_x increases
    float pitch = std::atan2(-ax, -az);
    // Roll: positive roll tilts right wing down, a_y decreases
    float roll = std::atan2(ay, -az);

    float current_rpy[3];
    quat_to_euler(m_state.quat, current_rpy);
    quat_from_euler(roll, pitch, current_rpy[2], m_state.quat);
    m_nav.attitude_valid = true;
    update_nav_state();
}

void ESKF15::align_yaw_from_mag(const float mag_body[3], float declination_rad) {
    float rpy[3];
    quat_to_euler(m_state.quat, rpy);
    float roll = rpy[0], pitch = rpy[1];

    // Tilt-compensate magnetometer body vector into horizontal earth frame
    float cr = std::cos(roll),  sr = std::sin(roll);
    float cp = std::cos(pitch), sp = std::sin(pitch);

    float mag_x = mag_body[0]*cp + mag_body[1]*sr*sp + mag_body[2]*cr*sp;
    float mag_y = mag_body[1]*cr - mag_body[2]*sr;

    if (std::abs(mag_x) < 1.0e-5f && std::abs(mag_y) < 1.0e-5f) return;

    // Magnetic yaw:
    float yaw_mag = std::atan2(-mag_y, mag_x);
    // True yaw:
    float yaw_true = wrap_pi(yaw_mag + declination_rad);

    quat_from_euler(roll, pitch, yaw_true, m_state.quat);
    m_nav.attitude_valid = true;
    update_nav_state();
}

// ─── High-Rate Prediction Step (Strapdown Integration) ────────────────────────

void ESKF15::predict(const float gyro_rads[3], const float accel_ms2[3], float dt_s) {
    if (!m_initialized) {
        init();
    }
    if (dt_s <= 0.0f || dt_s > 0.1f) return; // Guard against crazy timing leaps

    // 1. Correct sensor measurements with estimated biases
    float wx = gyro_rads[0] - m_state.gyro_bias[0];
    float wy = gyro_rads[1] - m_state.gyro_bias[1];
    float wz = gyro_rads[2] - m_state.gyro_bias[2];

    float ax = accel_ms2[0] - m_state.accel_bias[0];
    float ay = accel_ms2[1] - m_state.accel_bias[1];
    float az = accel_ms2[2] - m_state.accel_bias[2];

    // 2. Nominal Attitude Integration via Quaternion Exponential (first-order)
    float omega_norm = std::sqrt(wx*wx + wy*wy + wz*wz);
    float dq[4];
    if (omega_norm > 1.0e-6f) {
        float half_theta = 0.5f * omega_norm * dt_s;
        float sin_ht = std::sin(half_theta) / omega_norm;
        dq[0] = std::cos(half_theta);
        dq[1] = wx * sin_ht;
        dq[2] = wy * sin_ht;
        dq[3] = wz * sin_ht;
    } else {
        dq[0] = 1.0f;
        dq[1] = 0.5f * wx * dt_s;
        dq[2] = 0.5f * wy * dt_s;
        dq[3] = 0.5f * wz * dt_s;
    }

    float q_new[4];
    quat_multiply(m_state.quat, dq, q_new);
    quat_normalize(q_new);
    for (int i = 0; i < 4; ++i) m_state.quat[i] = q_new[i];

    // 3. Nominal Velocity and Position Integration
    float R[9];
    quat_to_dcm(m_state.quat, R);

    // Specific force in NED: f_ned = R * a_body
    float fn = R[0]*ax + R[1]*ay + R[2]*az;
    float fe = R[3]*ax + R[4]*ay + R[5]*az;
    float fd = R[6]*ax + R[7]*ay + R[8]*az;

    // Linear acceleration in NED = f_ned + [0, 0, g]
    float an = fn;
    float ae = fe;
    float ad = fd + kGravity;

    // Update position and velocity
    m_state.pos_ned[0] += m_state.vel_ned[0] * dt_s + 0.5f * an * dt_s * dt_s;
    m_state.pos_ned[1] += m_state.vel_ned[1] * dt_s + 0.5f * ae * dt_s * dt_s;
    m_state.pos_ned[2] += m_state.vel_ned[2] * dt_s + 0.5f * ad * dt_s * dt_s;

    m_state.vel_ned[0] += an * dt_s;
    m_state.vel_ned[1] += ae * dt_s;
    m_state.vel_ned[2] += ad * dt_s;

    // 4. Error-State Covariance Propagation: P = F * P * F^T + Q
    // We compute the discrete transition blocks:
    // F_pv = I * dt
    // F_v_th = - [f_ned]x * dt
    // F_v_ba = - R * dt
    // F_th_th = I - [w]x * dt
    // F_th_bg = - I * dt
    // F_ba_ba = I, F_bg_bg = I

    // Construct skew-symmetric cross-product matrix [-f_ned]x:
    // [-f]x = [  0   fd  -fe]
    //         [-fd   0    fn]
    //         [ fe  -fn   0 ]
    float f_skew[3][3] = {
        {  0.0f,  fd*dt_s, -fe*dt_s },
        { -fd*dt_s,  0.0f,  fn*dt_s },
        {  fe*dt_s, -fn*dt_s,  0.0f }
    };

    float R_dt[3][3];
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            R_dt[r][c] = -R[r*3 + c] * dt_s;
        }
    }

    // Temporary covariance copy
    std::array<float, ESKF15_STATE_DIM * ESKF15_STATE_DIM> P_old = m_P;
    auto get_p = [&P_old](int r, int c) -> float {
        return P_old[r * ESKF15_STATE_DIM + c];
    };

    // Propagate P with F (sparse analytical matrix multiplication for maximum efficiency):
    // Position rows (DPN, DPE, DPD):
    for (int axis = 0; axis < 3; ++axis) {
        int p_idx = STATE_DPN + axis;
        int v_idx = STATE_DVN + axis;
        for (int col = 0; col < ESKF15_STATE_DIM; ++col) {
            m_P[p_idx * ESKF15_STATE_DIM + col] = get_p(p_idx, col) + get_p(v_idx, col) * dt_s;
        }
    }
    // Complete position columns:
    for (int row = 0; row < ESKF15_STATE_DIM; ++row) {
        for (int axis = 0; axis < 3; ++axis) {
            int p_idx = STATE_DPN + axis;
            int v_idx = STATE_DVN + axis;
            m_P[row * ESKF15_STATE_DIM + p_idx] += m_P[row * ESKF15_STATE_DIM + v_idx] * dt_s;
        }
    }

    // Velocity rows (DVN, DVE, DVD):
    for (int axis = 0; axis < 3; ++axis) {
        int v_idx = STATE_DVN + axis;
        for (int col = 0; col < ESKF15_STATE_DIM; ++col) {
            float val = get_p(v_idx, col);
            for (int k = 0; k < 3; ++k) {
                val += f_skew[axis][k] * get_p(STATE_DTH_X + k, col);
                val += R_dt[axis][k] * get_p(STATE_DBA_X + k, col);
            }
            m_P[v_idx * ESKF15_STATE_DIM + col] = val;
        }
    }
    // Complete velocity columns:
    for (int row = 0; row < ESKF15_STATE_DIM; ++row) {
        for (int axis = 0; axis < 3; ++axis) {
            int v_idx = STATE_DVN + axis;
            float sum = 0.0f;
            for (int k = 0; k < 3; ++k) {
                sum += m_P[row * ESKF15_STATE_DIM + (STATE_DTH_X + k)] * f_skew[axis][k];
                sum += m_P[row * ESKF15_STATE_DIM + (STATE_DBA_X + k)] * R_dt[axis][k];
            }
            m_P[row * ESKF15_STATE_DIM + v_idx] += sum;
        }
    }

    // Attitude rows (DTH_X, DTH_Y, DTH_Z):
    for (int axis = 0; axis < 3; ++axis) {
        int th_idx = STATE_DTH_X + axis;
        int bg_idx = STATE_DBG_X + axis;
        for (int col = 0; col < ESKF15_STATE_DIM; ++col) {
            m_P[th_idx * ESKF15_STATE_DIM + col] -= get_p(bg_idx, col) * dt_s;
        }
    }
    for (int row = 0; row < ESKF15_STATE_DIM; ++row) {
        for (int axis = 0; axis < 3; ++axis) {
            int th_idx = STATE_DTH_X + axis;
            int bg_idx = STATE_DBG_X + axis;
            m_P[row * ESKF15_STATE_DIM + th_idx] -= m_P[row * ESKF15_STATE_DIM + bg_idx] * dt_s;
        }
    }

    // 5. Add Continuous Process Noise Q:
    float dt2 = dt_s * dt_s;
    float dt3_div3 = dt2 * dt_s / 3.0f;

    float q_vel = m_params.q_accel_noise * m_params.q_accel_noise;
    float q_pos = q_vel * dt3_div3;
    float q_att = m_params.q_gyro_noise * m_params.q_gyro_noise * dt_s;
    float q_acc_bias = m_params.q_accel_bias_walk * m_params.q_accel_bias_walk * dt_s;
    float q_gyr_bias = m_params.q_gyro_bias_walk * m_params.q_gyro_bias_walk * dt_s;

    for (int i = 0; i < 3; ++i) {
        m_P[(STATE_DPN + i) * ESKF15_STATE_DIM + (STATE_DPN + i)] += q_pos;
        m_P[(STATE_DVN + i) * ESKF15_STATE_DIM + (STATE_DVN + i)] += q_vel * dt_s;
        m_P[(STATE_DTH_X + i) * ESKF15_STATE_DIM + (STATE_DTH_X + i)] += q_att;
        m_P[(STATE_DBA_X + i) * ESKF15_STATE_DIM + (STATE_DBA_X + i)] += q_acc_bias;
        m_P[(STATE_DBG_X + i) * ESKF15_STATE_DIM + (STATE_DBG_X + i)] += q_gyr_bias;
    }

    ++m_predict_count;
    condition_covariance();
    update_nav_state();
}

// ─── Sequential Scalar Measurement Engine ─────────────────────────────────────

bool ESKF15::update_scalar(int state_idx, float innovation, float R_var, float gate_sigma) {
    if (state_idx < 0 || state_idx >= ESKF15_STATE_DIM) return false;

    float P_meas = m_P[state_idx * ESKF15_STATE_DIM + state_idx];
    float S = P_meas + R_var;
    if (S < 1.0e-8f || !std::isfinite(S)) return false;

    // Chi-Squared outlier rejection
    float gate_sq = gate_sigma * gate_sigma;
    if ((innovation * innovation) / S > gate_sq) {
        return false; // Outlier rejected!
    }

    // Compute Kalman Gain Vector K = P * H^T / S
    // Since H is [0 ... 1 ... 0] at state_idx, P * H^T is column state_idx of P
    float K[ESKF15_STATE_DIM];
    for (int i = 0; i < ESKF15_STATE_DIM; ++i) {
        K[i] = m_P[i * ESKF15_STATE_DIM + state_idx] / S;
    }

    // Calculate Error State: delta_x = K * innovation
    float dx[ESKF15_STATE_DIM];
    for (int i = 0; i < ESKF15_STATE_DIM; ++i) {
        dx[i] = K[i] * innovation;
    }

    // Covariance update: P = P - K * S * K^T
    // Mathematically equivalent to (I - K*H)*P, guaranteed symmetric!
    for (int r = 0; r < ESKF15_STATE_DIM; ++r) {
        for (int c = 0; c <= r; ++c) {
            float p_new = m_P[r * ESKF15_STATE_DIM + c] - K[r] * K[c] * S;
            m_P[r * ESKF15_STATE_DIM + c] = p_new;
            m_P[c * ESKF15_STATE_DIM + r] = p_new; // Symmetrize
        }
    }

    // Inject Error State into Nominal State
    inject_error_state(dx);
    condition_covariance();
    update_nav_state();
    return true;
}

bool ESKF15::update_arbitrary(const float H[ESKF15_STATE_DIM], float innovation, float R_var, float gate_sigma) {
    // Compute P * H^T (15-element vector)
    float PHt[ESKF15_STATE_DIM]{0.0f};
    for (int r = 0; r < ESKF15_STATE_DIM; ++r) {
        for (int c = 0; c < ESKF15_STATE_DIM; ++c) {
            PHt[r] += m_P[r * ESKF15_STATE_DIM + c] * H[c];
        }
    }

    // Innovation variance S = H * P * H^T + R
    float S = R_var;
    for (int i = 0; i < ESKF15_STATE_DIM; ++i) {
        S += H[i] * PHt[i];
    }
    if (S < 1.0e-8f || !std::isfinite(S)) return false;

    // Chi-Squared gate
    if ((innovation * innovation) / S > (gate_sigma * gate_sigma)) {
        return false;
    }

    // Kalman gain K = PHt / S
    float K[ESKF15_STATE_DIM];
    for (int i = 0; i < ESKF15_STATE_DIM; ++i) {
        K[i] = PHt[i] / S;
    }

    float dx[ESKF15_STATE_DIM];
    for (int i = 0; i < ESKF15_STATE_DIM; ++i) {
        dx[i] = K[i] * innovation;
    }

    // Covariance update: P = P - K * S * K^T
    for (int r = 0; r < ESKF15_STATE_DIM; ++r) {
        for (int c = 0; c <= r; ++c) {
            float p_new = m_P[r * ESKF15_STATE_DIM + c] - K[r] * K[c] * S;
            m_P[r * ESKF15_STATE_DIM + c] = p_new;
            m_P[c * ESKF15_STATE_DIM + r] = p_new;
        }
    }

    inject_error_state(dx);
    condition_covariance();
    update_nav_state();
    return true;
}

void ESKF15::inject_error_state(const float dx[ESKF15_STATE_DIM]) {
    // 1. Position injection
    m_state.pos_ned[0] += dx[STATE_DPN];
    m_state.pos_ned[1] += dx[STATE_DPE];
    m_state.pos_ned[2] += dx[STATE_DPD];

    // 2. Velocity injection
    m_state.vel_ned[0] += dx[STATE_DVN];
    m_state.vel_ned[1] += dx[STATE_DVE];
    m_state.vel_ned[2] += dx[STATE_DVD];

    // 3. Attitude quaternion injection via small-angle rotation vector
    // q = q ⊗ [1, 0.5 * d_theta]
    float dth_norm = std::sqrt(dx[STATE_DTH_X]*dx[STATE_DTH_X] +
                               dx[STATE_DTH_Y]*dx[STATE_DTH_Y] +
                               dx[STATE_DTH_Z]*dx[STATE_DTH_Z]);
    float dq[4];
    if (dth_norm > 1.0e-6f) {
        float half_angle = 0.5f * dth_norm;
        float factor = std::sin(half_angle) / dth_norm;
        dq[0] = std::cos(half_angle);
        dq[1] = dx[STATE_DTH_X] * factor;
        dq[2] = dx[STATE_DTH_Y] * factor;
        dq[3] = dx[STATE_DTH_Z] * factor;
    } else {
        dq[0] = 1.0f;
        dq[1] = 0.5f * dx[STATE_DTH_X];
        dq[2] = 0.5f * dx[STATE_DTH_Y];
        dq[3] = 0.5f * dx[STATE_DTH_Z];
    }
    float q_injected[4];
    quat_multiply(m_state.quat, dq, q_injected);
    quat_normalize(q_injected);
    for (int i = 0; i < 4; ++i) m_state.quat[i] = q_injected[i];

    // 4. Bias injection with saturation clamping
    m_state.accel_bias[0] = std::clamp(m_state.accel_bias[0] + dx[STATE_DBA_X],
                                       -m_params.max_accel_bias, m_params.max_accel_bias);
    m_state.accel_bias[1] = std::clamp(m_state.accel_bias[1] + dx[STATE_DBA_Y],
                                       -m_params.max_accel_bias, m_params.max_accel_bias);
    m_state.accel_bias[2] = std::clamp(m_state.accel_bias[2] + dx[STATE_DBA_Z],
                                       -m_params.max_accel_bias, m_params.max_accel_bias);

    m_state.gyro_bias[0] = std::clamp(m_state.gyro_bias[0] + dx[STATE_DBG_X],
                                      -m_params.max_gyro_bias, m_params.max_gyro_bias);
    m_state.gyro_bias[1] = std::clamp(m_state.gyro_bias[1] + dx[STATE_DBG_Y],
                                      -m_params.max_gyro_bias, m_params.max_gyro_bias);
    m_state.gyro_bias[2] = std::clamp(m_state.gyro_bias[2] + dx[STATE_DBG_Z],
                                      -m_params.max_gyro_bias, m_params.max_gyro_bias);
}

// ─── Asynchronous Measurement Updates ─────────────────────────────────────────

void ESKF15::update_baro(float baro_alt_m) {
    if (!std::isfinite(baro_alt_m)) return;

    // Measurement model: z = Down (positive down).
    // If home is configured, baro_alt_m is MSL altitude; convert to relative NED Down.
    float rel_alt = m_home_set ? (baro_alt_m - m_home_alt) : baro_alt_m;
    float z_meas = -rel_alt;
    float innovation = z_meas - m_state.pos_ned[2];
    float R_var = m_params.r_baro_alt * m_params.r_baro_alt;

    if (update_scalar(STATE_DPD, innovation, R_var, m_params.gate_baro)) {
        ++m_baro_update_count;
    }
}

void ESKF15::update_gps(const ESKFGpsSample& gps) {
    if (!gps.valid || gps.fix_type < 3) return;

    // Auto-set home on first high-quality 3D GPS fix if not already configured
    if (!m_home_set) {
        set_home(gps.lat_deg, gps.lon_deg, gps.alt_msl_m);
        return;
    }

    // Convert Geodetic lat/lon/alt to local NED tangent coordinates
    float gps_ned[3];
    geodetic_to_ned(gps.lat_deg, gps.lon_deg, gps.alt_msl_m, gps_ned);

    // Dynamic measurement noise based on reported accuracy / HDOP
    float r_horiz = std::max(m_params.r_gps_pos_horiz, gps.h_acc);
    float r_vert  = std::max(m_params.r_gps_pos_vert, gps.v_acc);
    float r_vel   = std::max(m_params.r_gps_vel, gps.s_acc);

    float R_pos_horiz = r_horiz * r_horiz;
    float R_pos_vert  = r_vert * r_vert;
    float R_vel       = r_vel * r_vel;

    // 1. Sequential Position Updates
    update_scalar(STATE_DPN, gps_ned[0] - m_state.pos_ned[0], R_pos_horiz, m_params.gate_gps_pos);
    update_scalar(STATE_DPE, gps_ned[1] - m_state.pos_ned[1], R_pos_horiz, m_params.gate_gps_pos);
    update_scalar(STATE_DPD, gps_ned[2] - m_state.pos_ned[2], R_pos_vert,  m_params.gate_gps_pos);

    // 2. Sequential Velocity Updates
    update_scalar(STATE_DVN, gps.vel_ned[0] - m_state.vel_ned[0], R_vel, m_params.gate_gps_vel);
    update_scalar(STATE_DVE, gps.vel_ned[1] - m_state.vel_ned[1], R_vel, m_params.gate_gps_vel);
    update_scalar(STATE_DVD, gps.vel_ned[2] - m_state.vel_ned[2], R_vel, m_params.gate_gps_vel);

    ++m_gps_update_count;
    m_nav.position_valid = true;
    m_nav.velocity_valid = true;
}

void ESKF15::update_mag(const float mag_body[3], float declination_rad) {
    float rpy[3];
    quat_to_euler(m_state.quat, rpy);
    float roll = rpy[0], pitch = rpy[1], yaw = rpy[2];

    float cr = std::cos(roll),  sr = std::sin(roll);
    float cp = std::cos(pitch), sp = std::sin(pitch);

    float mag_x = mag_body[0]*cp + mag_body[1]*sr*sp + mag_body[2]*cr*sp;
    float mag_y = mag_body[1]*cr - mag_body[2]*sr;

    if (std::abs(mag_x) < 1.0e-5f && std::abs(mag_y) < 1.0e-5f) return;

    float yaw_mag = std::atan2(-mag_y, mag_x);
    float yaw_measured = wrap_pi(yaw_mag + declination_rad);
    float innovation = wrap_pi(yaw_measured - yaw);

    float R_var = m_params.r_mag_heading * m_params.r_mag_heading;

    // Measurement Jacobian for yaw: H = [0, 0, 0,  0, 0, 0,  0, 0, 1,  0, 0, 0,  0, 0, 0]
    if (update_scalar(STATE_DTH_Z, innovation, R_var, m_params.gate_mag)) {
        ++m_mag_update_count;
        m_nav.attitude_valid = true;
    }
}

void ESKF15::update_zupt(float vel_sigma) {
    float R_var = vel_sigma * vel_sigma;
    // Zero-velocity update: measurement is 0.0 m/s
    update_scalar(STATE_DVN, 0.0f - m_state.vel_ned[0], R_var, 3.0f);
    update_scalar(STATE_DVE, 0.0f - m_state.vel_ned[1], R_var, 3.0f);
    update_scalar(STATE_DVD, 0.0f - m_state.vel_ned[2], R_var, 3.0f);
}

// ─── Numerical Conditioning & Health ──────────────────────────────────────────

void ESKF15::condition_covariance() {
    bool has_nan = false;

    for (int r = 0; r < ESKF15_STATE_DIM; ++r) {
        // Enforce variance floor (positive definiteness)
        float& var = m_P[r * ESKF15_STATE_DIM + r];
        if (!std::isfinite(var) || var < 1.0e-7f) {
            var = 1.0e-7f;
        }

        // Symmetrize
        for (int c = 0; c < r; ++c) {
            float avg = 0.5f * (m_P[r * ESKF15_STATE_DIM + c] + m_P[c * ESKF15_STATE_DIM + r]);
            if (!std::isfinite(avg)) {
                avg = 0.0f;
                has_nan = true;
            }
            m_P[r * ESKF15_STATE_DIM + c] = avg;
            m_P[c * ESKF15_STATE_DIM + r] = avg;
        }
    }

    if (has_nan) {
        reset();
    }
}

void ESKF15::update_nav_state() {
    for (int i = 0; i < 3; ++i) {
        m_nav.pos_ned[i] = m_state.pos_ned[i];
        m_nav.vel_ned[i] = m_state.vel_ned[i];
        m_nav.accel_bias[i] = m_state.accel_bias[i];
        m_nav.gyro_bias[i] = m_state.gyro_bias[i];

        m_nav.pos_var[i] = m_P[(STATE_DPN + i) * ESKF15_STATE_DIM + (STATE_DPN + i)];
        m_nav.vel_var[i] = m_P[(STATE_DVN + i) * ESKF15_STATE_DIM + (STATE_DVN + i)];
        m_nav.att_var[i] = m_P[(STATE_DTH_X + i) * ESKF15_STATE_DIM + (STATE_DTH_X + i)];
    }

    for (int i = 0; i < 4; ++i) {
        m_nav.quat[i] = m_state.quat[i];
    }
    quat_to_euler(m_state.quat, m_nav.euler_rpy);

    if (m_home_set) {
        double dlat = static_cast<double>(m_state.pos_ned[0]) / 111319.5;
        m_nav.lat_deg = m_home_lat + dlat;
        double lat_rad = m_home_lat * 0.017453292519943295;
        double cos_lat = std::cos(lat_rad);
        if (std::abs(cos_lat) > 1.0e-6) {
            m_nav.lon_deg = m_home_lon + (static_cast<double>(m_state.pos_ned[1]) / (111319.5 * cos_lat));
        } else {
            m_nav.lon_deg = m_home_lon;
        }
        m_nav.alt_msl_m = m_home_alt - m_state.pos_ned[2];
    }

    m_nav.attitude_valid = (m_P[STATE_DTH_X * ESKF15_STATE_DIM + STATE_DTH_X] < 0.25f);
    m_nav.velocity_valid = (m_P[STATE_DVN * ESKF15_STATE_DIM + STATE_DVN] < 5.0f);
    m_nav.position_valid = (m_P[STATE_DPN * ESKF15_STATE_DIM + STATE_DPN] < 50.0f);

    m_nav.healthy = std::isfinite(m_nav.pos_ned[0]) &&
                    std::isfinite(m_nav.vel_ned[0]) &&
                    std::isfinite(m_nav.euler_rpy[0]);
}

} // namespace apx::estimator
