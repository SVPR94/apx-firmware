// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include "eskf15_types.hpp"
#include <array>
#include <cstdint>

namespace apx::estimator {

class ESKF15 {
public:
    ESKF15();
    ~ESKF15() = default;

    // Initialization and reset
    void init(const ESKFTuningParams& params = ESKFTuningParams{});
    void reset();
    void set_home(double lat_deg, double lon_deg, float alt_m);
    bool has_home() const { return m_home_set; }

    // Core High-Rate Prediction Step (driven by IMU at 200 Hz - 1 kHz)
    void predict(const float gyro_rads[3], const float accel_ms2[3], float dt_s);

    // Asynchronous Sensor Measurement Updates
    void update_baro(float baro_alt_m);
    void update_gps(const ESKFGpsSample& gps);
    void update_mag(const float mag_body[3], float declination_rad = 0.0f);
    void update_zupt(float vel_sigma = 0.02f);

    // Initial coarse alignment helpers
    void align_tilt_from_accel(const float accel[3]);
    void align_yaw_from_mag(const float mag_body[3], float declination_rad);

    // Output State Accessors
    const ESKFNavState& get_nav_state() const { return m_nav; }
    const ESKFNominalState& get_nominal_state() const { return m_state; }

    void get_quaternion(float q[4]) const {
        for (int i = 0; i < 4; ++i) q[i] = m_state.quat[i];
    }

    void get_euler(float& roll, float& pitch, float& yaw) const {
        roll = m_nav.euler_rpy[0];
        pitch = m_nav.euler_rpy[1];
        yaw = m_nav.euler_rpy[2];
    }

    void get_pos_ned(float pos[3]) const {
        for (int i = 0; i < 3; ++i) pos[i] = m_state.pos_ned[i];
    }

    void get_vel_ned(float vel[3]) const {
        for (int i = 0; i < 3; ++i) vel[i] = m_state.vel_ned[i];
    }

    void get_gyro_bias(float bg[3]) const {
        for (int i = 0; i < 3; ++i) bg[i] = m_state.gyro_bias[i];
    }

    void get_accel_bias(float ba[3]) const {
        for (int i = 0; i < 3; ++i) ba[i] = m_state.accel_bias[i];
    }

    // Health and Validity Indicators
    bool is_healthy() const { return m_nav.healthy; }
    bool is_attitude_valid() const { return m_nav.attitude_valid; }
    bool is_velocity_valid() const { return m_nav.velocity_valid; }
    bool is_position_valid() const { return m_nav.position_valid; }

    // Covariance Accessor (for diagnostics / logging)
    float get_covariance(int row, int col) const {
        if (row >= 0 && row < ESKF15_STATE_DIM && col >= 0 && col < ESKF15_STATE_DIM) {
            return m_P[row * ESKF15_STATE_DIM + col];
        }
        return 0.0f;
    }

    // Mathematical Utilities
    static void quat_multiply(const float a[4], const float b[4], float out[4]);
    static void quat_normalize(float q[4]);
    static void quat_to_dcm(const float q[4], float R[9]);
    static void quat_to_euler(const float q[4], float rpy[3]);
    static void quat_from_euler(float roll, float pitch, float yaw, float q[4]);

private:
    ESKFNominalState m_state;
    ESKFTuningParams m_params;
    ESKFNavState m_nav;

    // 15x15 Error-State Covariance Matrix
    std::array<float, ESKF15_STATE_DIM * ESKF15_STATE_DIM> m_P{};

    // Home position for local NED geodetic tangent plane
    double m_home_lat{0.0};
    double m_home_lon{0.0};
    float m_home_alt{0.0f};
    bool m_home_set{false};

    bool m_initialized{false};
    uint64_t m_last_predict_us{0};
    uint32_t m_predict_count{0};
    uint32_t m_gps_update_count{0};
    uint32_t m_baro_update_count{0};
    uint32_t m_mag_update_count{0};

    // Internal Kalman update engine (sequential scalar updates)
    bool update_scalar(int state_idx, float innovation, float R_var, float gate_sigma);
    bool update_arbitrary(const float H[ESKF15_STATE_DIM], float innovation, float R_var, float gate_sigma);

    // Error injection into nominal state and covariance reset
    void inject_error_state(const float dx[ESKF15_STATE_DIM]);
    void reset_error_state_and_covariance();

    // Covariance health and conditioning
    void condition_covariance();
    void update_nav_state();
    void geodetic_to_ned(double lat, double lon, float alt, float ned[3]) const;
};

} // namespace apx::estimator
