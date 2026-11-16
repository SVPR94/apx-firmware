// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
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
