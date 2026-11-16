// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once
#include "ekf3_types.hpp"

namespace apx::estimator {

class EKF3 {
public:
    EKF3();

    void init(const EKF3InitParams& params = EKF3InitParams{});
    void set_home(double lat_deg, double lon_deg, float alt_m);

    // IMU prediction step (1kHz)
    void predict(const float gyro_rads[3], const float accel_ms2[3], float dt_s);

    // Measurement update steps
    void update_gps(const GpsMeas& m);
    void update_gps_rtk(const GpsMeas& m);
    void update_baro(const BaroMeas& m);
    void update_mag(const MagMeas& m);
    void update_flow(const FlowMeas& m);

    const NavState& get_nav_state() const { return m_nav; }
    bool is_attitude_valid()  const { return m_att_valid; }
    bool is_velocity_valid()  const { return m_vel_valid; }
    bool is_position_valid()  const { return m_pos_valid; }

    // Diagnostics
    float get_innovation_gps_pos() const { return m_innov_gps_pos; }
    float get_innovation_baro()    const { return m_innov_baro; }
    bool  is_healthy() const;

private:
    EKF3State       m_state;
    EKF3InitParams  m_params;
    NavState        m_nav;

    bool m_initialized{false};
    bool m_att_valid{false};
    bool m_vel_valid{false};
    bool m_pos_valid{false};

    // Home position for NED conversion
    double m_home_lat{0}, m_home_lon{0};
    float  m_home_alt{0};
    bool   m_home_set{false};

    // Earth magnetic field (NED)
    float m_earth_mag_ned[3]{0.25f, 0.0f, 0.43f};  // typical mid-latitude

    // Innovation monitoring
    float m_innov_gps_pos{0};
    float m_innov_baro{0};

    // Helper: convert lat/lon/alt to NED relative to home
    void geodetic_to_ned(double lat, double lon, float alt, float ned[3]) const;

    // Quaternion math
    static void quat_from_euler(float roll, float pitch, float yaw, float q[4]);
    static void quat_multiply(const float a[4], const float b[4], float out[4]);
    static void quat_normalize(float q[4]);
    static void quat_to_euler(const float q[4], float rpy[3]);
    static void quat_to_dcm(const float q[4], float R[9]);  // 3x3 column-major

    // Covariance operations
    void covariance_predict(const float F_diag[EKF3_N], float dt_s);
    void measurement_update(const float H[], int Hrows, const float innov[],
                             const float R_diag[], int meas_size,
                             const float gate_factor[]);
    void ensure_symmetric();
    void check_covariance_health();

    // State to nav output
    void state_to_nav();
};

} // namespace apx::estimator
