// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include <array>
#include <cstdint>
#include <cmath>

namespace apx::estimator {

constexpr int ESKF15_STATE_DIM = 15;

// Error state vector indices (15 error states):
enum ESKFStateIdx : int {
    // Position error in NED (m)
    STATE_DPN = 0,
    STATE_DPE = 1,
    STATE_DPD = 2,
    // Velocity error in NED (m/s)
    STATE_DVN = 3,
    STATE_DVE = 4,
    STATE_DVD = 5,
    // Small angle rotation vector error (rad)
    STATE_DTH_X = 6,
    STATE_DTH_Y = 7,
    STATE_DTH_Z = 8,
    // Accelerometer bias error in body frame (m/s^2)
    STATE_DBA_X = 9,
    STATE_DBA_Y = 10,
    STATE_DBA_Z = 11,
    // Gyroscope bias error in body frame (rad/s)
    STATE_DBG_X = 12,
    STATE_DBG_Y = 13,
    STATE_DBG_Z = 14
};

// Nominal State: 16 variables (true state = nominal ⊕ error)
struct ESKFNominalState {
    float pos_ned[3]{0.0f, 0.0f, 0.0f};      // [m] North, East, Down
    float vel_ned[3]{0.0f, 0.0f, 0.0f};      // [m/s] North, East, Down
    float quat[4]{1.0f, 0.0f, 0.0f, 0.0f};   // Unit quaternion [qw, qx, qy, qz] (body -> NED)
    float accel_bias[3]{0.0f, 0.0f, 0.0f};   // [m/s^2] Body frame accel bias
    float gyro_bias[3]{0.0f, 0.0f, 0.0f};    // [rad/s] Body frame gyro bias
};

// Configurable Tuning and Noise Parameters
struct ESKFTuningParams {
    // Process noise densities (continuous-time spectral densities)
    float q_gyro_noise{5.0e-3f};        // rad/s/sqrt(Hz)
    float q_accel_noise{5.0e-2f};       // m/s^2/sqrt(Hz)
    float q_gyro_bias_walk{1.0e-4f};    // rad/s^2/sqrt(Hz)
    float q_accel_bias_walk{1.0e-3f};   // m/s^3/sqrt(Hz)

    // Measurement noise standard deviations
    float r_gps_pos_horiz{1.2f};        // m (1-sigma horizontal position)
    float r_gps_pos_vert{2.0f};         // m (1-sigma vertical position)
    float r_gps_vel{0.15f};             // m/s (1-sigma 3D velocity)
    float r_baro_alt{1.0f};             // m (1-sigma baro altitude)
    float r_mag_heading{0.08f};         // rad (~4.5 deg 1-sigma heading)
    float r_zupt_vel{0.02f};            // m/s (zero-velocity pseudo-measurement)

    // Innovation outlier rejection gates (chi-squared / Mahalanobis thresholds)
    float gate_gps_pos{4.0f};           // sigma threshold
    float gate_gps_vel{4.0f};
    float gate_baro{4.0f};
    float gate_mag{3.5f};

    // Maximum allowable sensor biases before clamping
    float max_accel_bias{2.5f};         // m/s^2 (~0.25 g)
    float max_gyro_bias{0.15f};         // rad/s (~8.6 deg/s)
};

// Clean Nav State Output Struct
struct ESKFNavState {
    float pos_ned[3]{0.0f, 0.0f, 0.0f};
    float vel_ned[3]{0.0f, 0.0f, 0.0f};
    float quat[4]{1.0f, 0.0f, 0.0f, 0.0f};
    float euler_rpy[3]{0.0f, 0.0f, 0.0f};  // [rad] roll, pitch, yaw
    float gyro_bias[3]{0.0f, 0.0f, 0.0f};
    float accel_bias[3]{0.0f, 0.0f, 0.0f};

    // Geodetic position (valid when home is set)
    double lat_deg{0.0};
    double lon_deg{0.0};
    float alt_msl_m{0.0f};

    // 1-sigma uncertainties from covariance diagonal
    float pos_var[3]{1.0f, 1.0f, 1.0f};
    float vel_var[3]{0.5f, 0.5f, 0.5f};
    float att_var[3]{0.01f, 0.01f, 0.01f};

    bool attitude_valid{false};
    bool velocity_valid{false};
    bool position_valid{false};
    bool healthy{false};
};

struct ESKFGpsSample {
    double lat_deg{0.0};
    double lon_deg{0.0};
    float alt_msl_m{0.0f};
    float vel_ned[3]{0.0f, 0.0f, 0.0f};
    float h_acc{2.0f};
    float v_acc{3.0f};
    float s_acc{0.2f};
    uint8_t fix_type{0}; // 3 = 3D Fix
    uint8_t satellites{0};
    uint64_t timestamp_us{0};
    bool valid{false};
};

struct ESKFBaroSample {
    float altitude_m{0.0f};
    uint64_t timestamp_us{0};
    bool valid{false};
};

struct ESKFMagSample {
    float mag_xyz[3]{0.0f, 0.0f, 0.0f}; // Body frame [Gauss or microTesla]
    float declination_rad{0.0f};
    uint64_t timestamp_us{0};
    bool valid{false};
};

} // namespace apx::estimator
