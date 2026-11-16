// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include <array>
#include <cstdint>
#include <cmath>

namespace apx::estimator {

constexpr int UKF_STATE_DIM = 12; // Error state dimension (3 rot, 3 vel, 3 pos, 3 gyro bias)
constexpr int UKF_SIGMA_COUNT = 2 * UKF_STATE_DIM + 1; // 25 sigma points

struct UkfNavState {
    float quat[4]{1.0f, 0.0f, 0.0f, 0.0f}; // w, x, y, z
    float euler_rpy[3]{0.0f, 0.0f, 0.0f};  // roll, pitch, yaw (rad)
    float vel_ned[3]{0.0f, 0.0f, 0.0f};    // m/s
    float pos_ned[3]{0.0f, 0.0f, 0.0f};    // m
    float gyro_bias[3]{0.0f, 0.0f, 0.0f};  // rad/s
    float accel_bias[3]{0.0f, 0.0f, 0.0f}; // rad/s^2 (estimated or tracked)

    // Uncertainty diagnostics (variance diagonals of P)
    float att_var[3]{1e-3f, 1e-3f, 1e-3f};
    float vel_var[3]{1e-2f, 1e-2f, 1e-2f};
    float pos_var[3]{0.1f, 0.1f, 0.1f};

    // Health metrics
    float nis_gps_pos{0.0f};
    float nis_gps_vel{0.0f};
    float nis_baro{0.0f};
    float nis_mag{0.0f};
    float overall_nis{0.0f};
    bool  healthy{false};
    uint64_t timestamp_us{0};
};

struct UkfConfig {
    float q_gyro_noise{0.0015f};       // rad/s/sqrt(Hz)
    float q_accel_noise{0.020f};       // m/s^2/sqrt(Hz)
    float q_gyro_bias_noise{0.0001f};  // rad/s^2
    float r_gps_pos{1.5f};             // m
    float r_gps_vel{0.15f};            // m/s
    float r_baro_alt{1.2f};            // m
    float r_mag{0.06f};                // Gauss
    float alpha{1e-3f};                // Sigma point spread (1e-4 <= alpha <= 1)
    float beta{2.0f};                  // 2.0 optimal for Gaussian distributions
    float kappa{0.0f};                 // Secondary scaling parameter
};

} // namespace apx::estimator
