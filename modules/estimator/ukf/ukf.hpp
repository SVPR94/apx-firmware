// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include "ukf_types.hpp"
#include <array>
#include <cmath>

namespace apx::estimator {

class UKF {
public:
    UKF();

    void init(const UkfConfig& config = UkfConfig{});
    void set_home(double lat_deg, double lon_deg, float alt_m);

    // Dynamic prediction step using Unscented Transform (100Hz - 400Hz)
    void predict(const float gyro_rads[3], const float accel_ms2[3], float dt_s);

    // Measurement update steps
    void update_gps(double lat_deg, double lon_deg, float alt_m,
                    const float vel_ned_ms[3], float hacc_m, float vacc_m);
    void update_baro(float altitude_m);
    void update_mag(const float mag_gauss[3]);

    const UkfNavState& get_nav_state() const { return m_state; }
    bool is_healthy() const { return m_state.healthy; }
    float get_nis() const { return m_state.overall_nis; }

    void reset();

private:
    UkfConfig m_config;
    UkfNavState m_state;

    // Nominal state vector [13]: quat(4), vel(3), pos(3), gyro_bias(3)
    float m_q[4]{1.0f, 0.0f, 0.0f, 0.0f};
    float m_v[3]{0.0f, 0.0f, 0.0f};
    float m_p[3]{0.0f, 0.0f, 0.0f};
    float m_bg[3]{0.0f, 0.0f, 0.0f};

    // 12x12 error-state covariance matrix (stored row-major, static allocation)
    std::array<float, UKF_STATE_DIM * UKF_STATE_DIM> m_P{};

    // Unscented transform weights
    float m_Wm[UKF_SIGMA_COUNT]{};
    float m_Wc[UKF_SIGMA_COUNT]{};
    float m_gamma{0.0f};

    bool m_initialized{false};
    bool m_home_set{false};
    double m_home_lat{0.0};
    double m_home_lon{0.0};
    float  m_home_alt{0.0f};

    // Geodetic to local NED conversion
    void geodetic_to_ned(double lat, double lon, float alt, float ned[3]) const;

    // Quaternion & rotation matrix helpers
    static void quat_to_dcm(const float q[4], float R[9]);
    static void quat_to_euler(const float q[4], float rpy[3]);
    static void quat_normalize(float q[4]);
    static void quat_multiply(const float a[4], const float b[4], float out[4]);
    static void rot_vec_to_quat(const float r[3], float q[4]);

    // Cholesky square root of symmetric 12x12 matrix (with regularization)
    bool cholesky_12x12(const float* A, float* L) const;

    void compute_weights();
    void enforce_covariance_symmetry();
};

} // namespace apx::estimator
