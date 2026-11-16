// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "mission_executor.hpp"
#include <cmath>

namespace apx::navigator {

MissionExecutor::MissionExecutor() {
}

void MissionExecutor::add_item(const MissionItem& item) {
    m_items.push_back(item);
}

void MissionExecutor::clear_mission() {
    m_items.clear();
    m_active = false;
    m_current_index = 0;
}

void MissionExecutor::start_mission() {
    if (!m_items.empty()) {
        m_active = true;
        m_current_index = 0;
    }
}

void MissionExecutor::stop_mission() {
    m_active = false;
}

bool MissionExecutor::is_active() const {
    return m_active;
}

int32_t MissionExecutor::get_current_index() const {
    return static_cast<int32_t>(m_current_index);
}

MissionItem MissionExecutor::get_current_item() const {
    if (m_current_index < m_items.size()) {
        return m_items[m_current_index];
    }
    return MissionItem{MissionCommand::WAYPOINT, 0, 0, 0.0f, {0}};
}

void MissionExecutor::update(int32_t current_lat, int32_t current_lon, float current_alt) {
    if (!m_active || m_current_index >= m_items.size()) {
        return;
    }

    if (check_waypoint_reached(current_lat, current_lon, current_alt)) {
        advance_mission();
    }
}

bool MissionExecutor::check_waypoint_reached(int32_t current_lat, int32_t current_lon, float current_alt) const {
    const auto& target = m_items[m_current_index];
    
    // Very simplified distance calculation for example
    double lat_diff = (target.lat - current_lat) * 1e-7;
    double lon_diff = (target.lon - current_lon) * 1e-7;
    double lat_m = lat_diff * 111320.0;
    double lon_m = lon_diff * 40000000.0 * std::cos(current_lat * 1e-7 * M_PI / 180.0) / 360.0;
    double alt_diff = target.alt - current_alt;

    double dist_sq = lat_m * lat_m + lon_m * lon_m + alt_diff * alt_diff;
    return dist_sq < (m_acceptance_radius_m * m_acceptance_radius_m);
}

void MissionExecutor::advance_mission() {
    m_current_index++;
    if (m_current_index >= m_items.size()) {
        m_active = false; // Mission complete
    }
}

} // namespace apx::navigator
