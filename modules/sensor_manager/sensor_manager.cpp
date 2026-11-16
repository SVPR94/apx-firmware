// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "sensor_manager.hpp"
#include "apx/kernel/timer.hpp"
#include <cstring>

namespace apx::sensor {

void SensorManager::register_imu(int idx, apx::drivers::DriverBase* drv) {
    if (idx >= 0 && idx < kMaxSensors) m_imu_drivers[idx] = drv;
}

void SensorManager::register_baro(int idx, apx::drivers::DriverBase* drv) {
    if (idx >= 0 && idx < kMaxSensors) m_baro_drivers[idx] = drv;
}

void SensorManager::register_mag(int idx, apx::drivers::DriverBase* drv) {
    if (idx >= 0 && idx < kMaxSensors) m_mag_drivers[idx] = drv;
}

void SensorManager::register_gps(int idx, apx::drivers::DriverBase* drv) {
    if (idx >= 0 && idx < kMaxSensors) m_gps_drivers[idx] = drv;
}

void SensorManager::update_imu() {
    uint64_t now = apx::kernel::now_us();
    for (int i = 0; i < kMaxSensors; i++) {
        if (!m_imu_drivers[i]) continue;
        m_imu_drivers[i]->update();
        bool healthy = m_imu_drivers[i]->is_healthy();
        if (healthy) {
            m_imu_health[i].mark_updated(now);
        } else {
            m_imu_health[i].mark_failed();
        }
        bool usable = m_imu_health[i].is_usable(now);
        // Cast driver and get data if possible
        auto* imu_drv = static_cast<apx::drivers::Icm42688*>(m_imu_drivers[i]);
        if (imu_drv) {
            m_voter_imu.add_measurement(i, imu_drv->get_data(), usable);
        }
    }
    m_voter_imu.vote();
}

void SensorManager::update_baro() {
    uint64_t now = apx::kernel::now_us();
    for (int i = 0; i < kMaxSensors; i++) {
        if (!m_baro_drivers[i]) continue;
        m_baro_drivers[i]->update();
        bool healthy = m_baro_drivers[i]->is_healthy();
        if (healthy) {
            m_baro_health[i].mark_updated(now);
        } else {
            m_baro_health[i].mark_failed();
        }
        bool usable = m_baro_health[i].is_usable(now);
        auto* baro_drv = static_cast<apx::drivers::Ms5611*>(m_baro_drivers[i]);
        if (baro_drv) {
            m_voter_baro.add_measurement(i, baro_drv->get_data(), usable);
        }
    }
    m_voter_baro.vote();
}

void SensorManager::update_mag() {
    uint64_t now = apx::kernel::now_us();
    for (int i = 0; i < kMaxSensors; i++) {
        if (!m_mag_drivers[i]) continue;
        m_mag_drivers[i]->update();
        bool healthy = m_mag_drivers[i]->is_healthy();
        if (healthy) {
            m_mag_health[i].mark_updated(now);
        } else {
            m_mag_health[i].mark_failed();
        }
        bool usable = m_mag_health[i].is_usable(now);
        auto* mag_drv = static_cast<apx::drivers::Ist8310*>(m_mag_drivers[i]);
        if (mag_drv) {
            m_voter_mag.add_measurement(i, mag_drv->get_data(), usable);
        }
    }
    m_voter_mag.vote();
}

void SensorManager::update_gps() {
    uint64_t now = apx::kernel::now_us();
    for (int i = 0; i < kMaxSensors; i++) {
        if (!m_gps_drivers[i]) continue;
        m_gps_drivers[i]->update();
        bool healthy = m_gps_drivers[i]->is_healthy();
        if (healthy) {
            m_gps_health[i].mark_updated(now);
        } else {
            m_gps_health[i].mark_failed();
        }
        bool usable = m_gps_health[i].is_usable(now);
        auto* gps_drv = static_cast<apx::drivers::UbloxF9P*>(m_gps_drivers[i]);
        if (gps_drv) {
            m_voter_gps.add_measurement(i, gps_drv->get_data(), usable);
        }
    }
    m_voter_gps.vote();
}

void SensorManager::update() {
    update_imu();
    update_baro();
    update_mag();
    update_gps();
}

const apx::drivers::ImuData& SensorManager::get_primary_imu() const {
    if (m_voter_imu.has_winner()) return m_voter_imu.get_best();
    return m_imu_fallback;
}

const apx::drivers::BaroData& SensorManager::get_primary_baro() const {
    if (m_voter_baro.has_winner()) return m_voter_baro.get_best();
    return m_baro_fallback;
}

const apx::drivers::MagData& SensorManager::get_primary_mag() const {
    if (m_voter_mag.has_winner()) return m_voter_mag.get_best();
    return m_mag_fallback;
}

const apx::drivers::GpsData& SensorManager::get_primary_gps() const {
    if (m_voter_gps.has_winner()) return m_voter_gps.get_best();
    return m_gps_fallback;
}

} // namespace apx::sensor
