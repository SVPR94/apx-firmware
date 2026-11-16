// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "sensor_crossbar.hpp"
#include <cstdio>
#include <cmath>

namespace apx::estimator {

SensorCrossbar::SensorCrossbar() {
    for (auto& m : m_imu_metrics)  { m.healthy = true; }
    for (auto& m : m_baro_metrics) { m.healthy = true; }
    for (auto& m : m_mag_metrics)  { m.healthy = true; }
    for (auto& m : m_gps_metrics)  { m.healthy = true; }

    // Initial default mapping
    m_lane_imu_map[static_cast<size_t>(EstimatorLaneId::LANE_EKF3)] = 0;
    m_lane_imu_map[static_cast<size_t>(EstimatorLaneId::LANE_UKF)] = 1;      // UKF receives redundant IMU2 by default
    m_lane_imu_map[static_cast<size_t>(EstimatorLaneId::LANE_INVARIANT)] = 0;
}

void SensorCrossbar::report_imu_metrics(int index, float vibration, bool clipped, uint64_t /*now_us*/) {
    if (index < 0 || index >= kMaxImus) return;
    auto& m = m_imu_metrics[index];
    m.vibration_energy = 0.9f * m.vibration_energy + 0.1f * vibration;
    if (clipped) {
        m.clipping_count++;
    }
    // High vibration threshold (> 6.0 m/s^2 peak-to-peak) or clipping marks degraded
    m.healthy = (m.vibration_energy < 8.0f && m.clipping_count < 50);
}

void SensorCrossbar::report_baro_metrics(int index, float pressure_noise, uint64_t /*now_us*/) {
    if (index < 0 || index >= kMaxBaros) return;
    auto& m = m_baro_metrics[index];
    m.vibration_energy = 0.95f * m.vibration_energy + 0.05f * pressure_noise;
    m.healthy = (m.vibration_energy < 15.0f); // 15 Pa threshold
}

void SensorCrossbar::report_mag_metrics(int index, float field_norm_err, uint64_t /*now_us*/) {
    if (index < 0 || index >= kMaxMags) return;
    auto& m = m_mag_metrics[index];
    m.vibration_energy = 0.9f * m.vibration_energy + 0.1f * field_norm_err;
    m.healthy = (m.vibration_energy < 0.25f); // 25% deviation from expected Earth field
}

void SensorCrossbar::report_gps_metrics(int index, float hacc, uint8_t num_sats, uint64_t /*now_us*/) {
    if (index < 0 || index >= kMaxGpss) return;
    auto& m = m_gps_metrics[index];
    m.healthy = (num_sats >= 6 && hacc < 3.5f);
}

int SensorCrossbar::find_best_imu() const {
    int best_idx = 0;
    float lowest_penalty = 1e6f;
    for (int i = 0; i < kMaxImus; ++i) {
        if (!m_imu_metrics[i].healthy) continue;
        float penalty = m_imu_metrics[i].vibration_energy + static_cast<float>(m_imu_metrics[i].clipping_count) * 0.1f;
        if (penalty < lowest_penalty) {
            lowest_penalty = penalty;
            best_idx = i;
        }
    }
    return best_idx;
}

int SensorCrossbar::find_best_baro() const {
    for (int i = 0; i < kMaxBaros; ++i) {
        if (m_baro_metrics[i].healthy) return i;
    }
    return 0;
}

int SensorCrossbar::find_best_mag() const {
    for (int i = 0; i < kMaxMags; ++i) {
        if (m_mag_metrics[i].healthy) return i;
    }
    return 0;
}

int SensorCrossbar::find_best_gps() const {
    for (int i = 0; i < kMaxGpss; ++i) {
        if (m_gps_metrics[i].healthy) return i;
    }
    return 0;
}

void SensorCrossbar::update_routing(uint64_t now_us) {
    if (now_us - m_last_reroute_check_us < 200000) return; // Evaluate every 200ms
    m_last_reroute_check_us = now_us;

    int best_imu = find_best_imu();
    size_t ekf_lane = static_cast<size_t>(EstimatorLaneId::LANE_EKF3);

    // If current primary IMU became unhealthy, hot-rewire to best healthy IMU!
    if (m_lane_imu_map[ekf_lane] != best_imu && !m_imu_metrics[m_lane_imu_map[ekf_lane]].healthy) {
        std::printf("[APX Crossbar] LIVE REWIRING: IMU%d degraded (vib=%.1fm/s2, clips=%u). Routing IMU%d -> Lane 0 (EKF3)\n",
                    m_lane_imu_map[ekf_lane] + 1,
                    (double)m_imu_metrics[m_lane_imu_map[ekf_lane]].vibration_energy,
                    m_imu_metrics[m_lane_imu_map[ekf_lane]].clipping_count,
                    best_imu + 1);
        m_lane_imu_map[ekf_lane] = best_imu;
        m_rewired_event = true;
    }

    int best_mag = find_best_mag();
    if (m_lane_mag_map[ekf_lane] != best_mag && !m_mag_metrics[m_lane_mag_map[ekf_lane]].healthy) {
        std::printf("[APX Crossbar] LIVE REWIRING: Mag%d distorted. Routing Mag%d -> Lane 0\n",
                    m_lane_mag_map[ekf_lane] + 1, best_mag + 1);
        m_lane_mag_map[ekf_lane] = best_mag;
        m_rewired_event = true;
    }
}

int SensorCrossbar::get_routed_imu(EstimatorLaneId lane) const {
    return m_lane_imu_map[static_cast<size_t>(lane)];
}

int SensorCrossbar::get_routed_baro(EstimatorLaneId lane) const {
    return m_lane_baro_map[static_cast<size_t>(lane)];
}

int SensorCrossbar::get_routed_mag(EstimatorLaneId lane) const {
    return m_lane_mag_map[static_cast<size_t>(lane)];
}

int SensorCrossbar::get_routed_gps(EstimatorLaneId lane) const {
    return m_lane_gps_map[static_cast<size_t>(lane)];
}

void SensorCrossbar::force_route_imu(EstimatorLaneId lane, int imu_idx) {
    if (imu_idx >= 0 && imu_idx < kMaxImus) {
        m_lane_imu_map[static_cast<size_t>(lane)] = imu_idx;
        m_rewired_event = true;
    }
}

} // namespace apx::estimator
