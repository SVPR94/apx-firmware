// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include "estimator_master_types.hpp"
#include "sensor_crossbar.hpp"
#include "ekf3/ekf3.hpp"
#include "ukf/ukf.hpp"
#include "attitude_estimator/complementary_filter.hpp"

#include <array>
#include <cstdint>

namespace apx::estimator {

class EstimatorArbiter {
public:
    static EstimatorArbiter& get_instance() {
        static EstimatorArbiter inst;
        return inst;
    }

    void init(double home_lat, double home_lon, float home_alt);

    // Dynamic Sensor Crossbar interface
    SensorCrossbar& get_crossbar() { return m_crossbar; }
    const SensorCrossbar& get_crossbar() const { return m_crossbar; }

    // High-rate IMU predict step (400 Hz)
    void predict(const float gyro_body[3], const float accel_body[3], float dt_s, uint64_t now_us);

    // Asynchronous sensor measurement updates
    void update_gps(double lat_deg, double lon_deg, float alt_m,
                    const float vel_ned_ms[3], float hacc_m, float vacc_m, uint64_t now_us);
    void update_baro(float pressure_pa, float altitude_m, uint64_t now_us);
    void update_mag(const float mag_gauss[3], uint64_t now_us);

    // Primary fused output for flight control loops (400 Hz)
    const FusedNavSolution& get_fused_solution() const { return m_fused; }

    // Diagnostics & telemetry
    const std::array<LaneDiagnostics, 3>& get_lane_diagnostics() const { return m_lane_diag; }
    EstimatorLaneId get_primary_lane() const { return m_fused.primary_lane; }

private:
    EstimatorArbiter();

    // Parallel Estimator Instances
    EKF3 m_ekf3;                   // Lane 0: 24-state EKF
    UKF  m_ukf;                    // Lane 1: 15-state Unscented Kalman Filter
    ComplementaryFilter m_comp;    // Lane 2: Fast geometric attitude observer

    SensorCrossbar m_crossbar;
    FusedNavSolution m_fused;
    std::array<LaneDiagnostics, 3> m_lane_diag;

    bool m_initialized{false};
    uint64_t m_last_arbiter_eval_us{0};

    // Algorithm Master Decision Maker & Weight Evaluation
    void evaluate_lanes(uint64_t now_us);

    // Federated Information-Weighted Consensus Blending
    void compute_federated_blend(uint64_t now_us);

    static void quat_normalize(float q[4]);
    static void quat_to_euler(const float q[4], float rpy[3]);
};

} // namespace apx::estimator
