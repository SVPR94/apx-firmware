// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include <cstdint>
#include <vector>

namespace apx::navigator {

enum class MissionCommand : uint16_t {
    TAKEOFF = 22,
    WAYPOINT = 16,
    LAND = 21,
    LOITER_TIME = 19,
    RETURN_TO_LAUNCH = 20,
    CONDITION_CHANGE_ALT = 113,
    CONDITION_YAW = 115,
    DO_CHANGE_SPEED = 178,
    DO_SET_SERVO = 183,
    DO_CHANGE_ALTITUDE = 186
};

struct MissionItem {
    MissionCommand command;
    int32_t lat; // 1e7 deg
    int32_t lon; // 1e7 deg
    float alt;   // m
    float params[4];
};

class MissionExecutor {
public:
    MissionExecutor();
    ~MissionExecutor() = default;

    void add_item(const MissionItem& item);
    void clear_mission();
    
    void start_mission();
    void stop_mission();
    void update(int32_t current_lat, int32_t current_lon, float current_alt);

    bool is_active() const;
    int32_t get_current_index() const;
    MissionItem get_current_item() const;

private:
    std::vector<MissionItem> m_items;
    bool m_active = false;
    size_t m_current_index = 0;
    
    float m_acceptance_radius_m = 2.0f; // 2 meters default

    bool check_waypoint_reached(int32_t current_lat, int32_t current_lon, float current_alt) const;
    void advance_mission();
};

} // namespace apx::navigator
