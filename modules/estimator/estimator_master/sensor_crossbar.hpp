// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include "estimator_master_types.hpp"
#include <array>
#include <cstdint>

namespace apx::estimator {

struct SensorHealthMetrics {
    float vibration_energy{0.0f};  // High-frequency accel vibration norm (m/s^2)
    uint32_t clipping_count{0};    // Sensor ADC / dynamic range clipping instances
    float thermal_drift_rate{0.0f};// degC/s or bias drift rate
    float innovation_penalty{0.0f};// Penalty from estimator innovation rejections
    bool  healthy{true};
};

class SensorCrossbar {
public:
    static constexpr int kMaxImus = 4;
    static constexpr int kMaxBaros = 4;
    static constexpr int kMaxMags = 2;
    static constexpr int kMaxGpss = 2;

    SensorCrossbar();

    // Update real-time health metrics per sensor instance
    void report_imu_metrics(int index, float vibration, bool clipped, uint64_t now_us);
    void report_baro_metrics(int index, float pressure_noise, uint64_t now_us);
    void report_mag_metrics(int index, float field_norm_err, uint64_t now_us);
    void report_gps_metrics(int index, float hacc, uint8_t num_sats, uint64_t now_us);

    // Evaluate sensor qualities and re-route matrix dynamically
    void update_routing(uint64_t now_us);

    // Dynamic routing queries for estimator lanes
    int get_routed_imu(EstimatorLaneId lane) const;
    int get_routed_baro(EstimatorLaneId lane) const;
    int get_routed_mag(EstimatorLaneId lane) const;
    int get_routed_gps(EstimatorLaneId lane) const;

    // Manual override or testing injection
    void force_route_imu(EstimatorLaneId lane, int imu_idx);

    bool has_rewired_recently() const { return m_rewired_event; }
    void clear_rewired_event() { m_rewired_event = false; }

private:
    std::array<SensorHealthMetrics, kMaxImus>  m_imu_metrics{};
    std::array<SensorHealthMetrics, kMaxBaros> m_baro_metrics{};
    std::array<SensorHealthMetrics, kMaxMags>  m_mag_metrics{};
    std::array<SensorHealthMetrics, kMaxGpss>  m_gps_metrics{};

    // Routing table: Lane -> Physical Sensor Index
    std::array<int, static_cast<size_t>(EstimatorLaneId::COUNT)> m_lane_imu_map{0, 1, 0};
    std::array<int, static_cast<size_t>(EstimatorLaneId::COUNT)> m_lane_baro_map{0, 0, 0};
    std::array<int, static_cast<size_t>(EstimatorLaneId::COUNT)> m_lane_mag_map{0, 0, 0};
    std::array<int, static_cast<size_t>(EstimatorLaneId::COUNT)> m_lane_gps_map{0, 0, 0};

    bool m_rewired_event{false};
    uint64_t m_last_reroute_check_us{0};

    int find_best_imu() const;
    int find_best_baro() const;
    int find_best_mag() const;
    int find_best_gps() const;
};

} // namespace apx::estimator
