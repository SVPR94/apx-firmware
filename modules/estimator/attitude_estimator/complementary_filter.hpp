// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
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
