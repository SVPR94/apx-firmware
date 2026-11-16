// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "can_radar_driver.hpp"
#include "../../../modules/radar/radar_manager.hpp"
#include <cstring>

namespace apx::drivers {

CanRadarDriver::CanRadarDriver(apx::hal::IHalCan& can, uint32_t base_can_id)
    : m_can(can), m_base_can_id(base_can_id) {
    for (int i = 0; i < 10; ++i) {
        m_data.distance_m[i] = 0.0f;
        m_data.angle_deg[i] = (i >= 1 && i <= 8) ? ((i - 1) * 45.0f) : 0.0f;
        m_data.valid[i] = false;
    }
}

bool CanRadarDriver::init() {
    // CAN interface expected to be initialized elsewhere
    return true;
}

void CanRadarDriver::update() {
    apx::hal::CanFrame frame;
    while (m_can.read(&frame) > 0) {
        if (frame.id >= m_base_can_id && frame.id < m_base_can_id + 10) {
            int sector = frame.id - m_base_can_id;
            
            if (frame.dlc >= 4) {
                uint16_t dist_cm = (static_cast<uint16_t>(frame.data[1]) << 8) | frame.data[0];
                uint16_t angle_01deg = (static_cast<uint16_t>(frame.data[3]) << 8) | frame.data[2];
                uint8_t quality = (frame.dlc >= 5) ? frame.data[4] : 100;
                
                m_data.distance_m[sector] = dist_cm * 0.01f;
                m_data.angle_deg[sector] = angle_01deg * 0.1f;
                m_data.valid[sector] = true;
                
                // Ingest into firmware's unified radar manager
                apx::radar::RadarManager::get_instance().feed_sensor_reading(
                    static_cast<uint8_t>(sector), m_data.distance_m[sector], quality, true, 0);
            }
        }
    }
}

ObstacleData CanRadarDriver::get_data() const {
    return m_data;
}

} // namespace apx::drivers
