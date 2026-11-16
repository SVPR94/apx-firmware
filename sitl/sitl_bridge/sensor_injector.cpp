// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "sensor_injector.hpp"
#include <iostream>

namespace apx {
namespace sitl {

SensorInjector::SensorInjector() {
    set_imu_noise(0.05, 0.01);
    set_baro_noise(1.0);
    set_gps_noise(0.2);
}

void SensorInjector::set_imu_noise(double accel_noise, double gyro_noise) {
    m_accel_dist = std::normal_distribution<double>(0.0, accel_noise);
    m_gyro_dist = std::normal_distribution<double>(0.0, gyro_noise);
}

void SensorInjector::set_baro_noise(double pressure_noise) {
    m_baro_dist = std::normal_distribution<double>(0.0, pressure_noise);
}

void SensorInjector::set_gps_noise(double pos_noise) {
    m_gps_dist = std::normal_distribution<double>(0.0, pos_noise);
}

void SensorInjector::process_state(const SitlPhysicsState& state) {
    // Generate noisy IMU data
    double accel[3];
    double gyro[3];
    for (int i = 0; i < 3; ++i) {
        accel[i] = state.accel_body[i] + m_accel_dist(m_rng);
        gyro[i] = state.gyro_body[i] + m_gyro_dist(m_rng);
    }
    
    // Generate noisy Baro data
    double pressure = state.baro_pressure_pa + m_baro_dist(m_rng);
    
    // Generate noisy GPS data
    double lat = state.gps_lat + m_gps_dist(m_rng);
    double lon = state.gps_lon + m_gps_dist(m_rng);
    double alt = state.gps_alt + m_gps_dist(m_rng);
    
    // In a full implementation, these would be injected into the uBus or similar IPC
    // For now, we simulate the injection process.
}

} // namespace sitl
} // namespace apx
