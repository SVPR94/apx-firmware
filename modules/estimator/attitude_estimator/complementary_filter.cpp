// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
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
