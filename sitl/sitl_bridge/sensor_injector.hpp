// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include "sitl_server.hpp"
#include <random>

namespace apx {
namespace sitl {

class SensorInjector {
public:
    SensorInjector();
    ~SensorInjector() = default;

    void process_state(const SitlPhysicsState& state);
    
    void set_imu_noise(double accel_noise, double gyro_noise);
    void set_baro_noise(double pressure_noise);
    void set_gps_noise(double pos_noise);

private:
    std::default_random_engine m_rng;
    std::normal_distribution<double> m_accel_dist;
    std::normal_distribution<double> m_gyro_dist;
    std::normal_distribution<double> m_baro_dist;
    std::normal_distribution<double> m_gps_dist;
};

} // namespace sitl
} // namespace apx
