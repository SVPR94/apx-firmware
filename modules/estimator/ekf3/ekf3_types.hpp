// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once
#include <array>
#include <cstdint>
#include <cmath>

namespace apx::estimator {

constexpr int EKF3_N = 24;

// State indices
enum StateIdx {
    Q0=0,Q1,Q2,Q3,       // 0-3  quaternion attitude
    VN,VE,VD,            // 4-6  velocity NED (m/s)
    PN,PE,PD,            // 7-9  position NED (m)
    BGX,BGY,BGZ,         // 10-12 gyro bias (rad/s)
    BAX,BAY,BAZ,         // 13-15 accel bias (m/s^2)
    WN,WE,WW,            // 16-18 wind velocity (m/s)
    BBIAS,               // 19   baro height bias (m)
    MDECL,               // 20   mag declination bias (rad)
    MBX,MBY,MBZ          // 21-23 mag body field (Gauss)
};

struct EKF3InitParams {
    float q_gyro_noise       = 0.001f;
    float q_accel_noise      = 0.01f;
    float q_gyro_bias_noise  = 0.0001f;
    float q_accel_bias_noise = 0.001f;
    float q_wind_noise       = 0.05f;
    float q_baro_noise       = 0.01f;
    float q_mag_noise        = 0.001f;
    float r_gps_pos          = 1.5f;
    float r_gps_vel          = 0.1f;
    float r_baro_alt         = 1.0f;
    float r_mag              = 0.05f;
    float r_rtk_pos          = 0.02f;
    float r_flow_vel         = 0.1f;
    float innov_gate_gps_pos = 5.0f;
    float innov_gate_gps_vel = 5.0f;
    float innov_gate_baro    = 4.0f;
    float innov_gate_mag     = 3.0f;
};

struct NavState {
    float quaternion[4]{1,0,0,0};
    float euler_rpy[3]{};
    float velocity_ned[3]{};
    float position_ned[3]{};
    float gyro_bias[3]{};
    float accel_bias[3]{};
    float wind_vel[3]{};
    float baro_bias{0.0f};
    float mag_decl{0.0f};
    float mag_body[3]{};
    // Variances
    float att_var[3]{};
    float vel_var[3]{};
    float pos_var[3]{};
    bool  valid{false};
    uint64_t timestamp_us{0};
};

struct EKF3State {
    std::array<float, EKF3_N> x{};
    std::array<float, EKF3_N * EKF3_N> P{};  // 24x24 covariance matrix
};

struct GpsMeas    { double lat_deg, lon_deg; float alt_m; float vel_ned[3]; float hacc, vacc; };
struct BaroMeas   { float altitude_m; };
struct MagMeas    { float mag_body[3]; };  // Gauss
struct FlowMeas   { float flow_x_rad, flow_y_rad; uint8_t quality; float range_m; };

} // namespace apx::estimator
