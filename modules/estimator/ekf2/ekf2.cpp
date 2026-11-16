// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
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
