// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include <cstdint>
#include <array>
#include <cmath>
#include <algorithm>
#include "apx/kernel/mutex.hpp"

namespace apx::radar {

// 10 Radar Channels:
// 0: Down (Terrain clearance / AGL)
// 1: Front (0 deg)
// 2: Front-Right (45 deg)
// 3: Right (90 deg)
// 4: Rear-Right (135 deg)
// 5: Rear (180 deg)
// 6: Rear-Left (225 deg)
// 7: Left (270 deg)
// 8: Front-Left (315 deg)
// 9: Top (Ceiling / overhead clearance)
enum class RadarDirection : uint8_t {
    DOWN = 0,
    FRONT = 1,
    FRONT_RIGHT = 2,
    RIGHT = 3,
    REAR_RIGHT = 4,
    REAR = 5,
    REAR_LEFT = 6,
    LEFT = 7,
    FRONT_LEFT = 8,
    TOP = 9
};

constexpr int kNumRadarSensors = 10;

struct RadarSensorConfig {
    bool enabled{false};
    uint8_t orientation{0};          // MAV_SENSOR_ORIENTATION (0=forward, 25=down, 24=up, etc.)
    float mounting_yaw_deg{0.0f};
    float mounting_pitch_deg{0.0f};
    float mounting_roll_deg{0.0f};
    float mounting_pos[3]{0.0f, 0.0f, 0.0f}; // X, Y, Z (m) in body frame
    float min_distance_m{0.2f};
    float max_distance_m{40.0f};
};

struct RadarSensorReading {
    float distance_m{0.0f};          // Measured raw distance along sensor beam
    uint8_t signal_quality{100};     // 0-100%
    bool valid{false};
    bool healthy{false};
    uint64_t last_update_us{0};
};

struct TerrainState {
    bool enabled{false};
    bool active{false};
    float target_clearance_m{2.0f};  // Desired vertical clearance above terrain
    float min_range_m{2.0f};         // Minimum valid altimeter reading range for terrain effect (m)
    float max_range_m{30.0f};        // Maximum valid altimeter reading range for terrain effect (m)
    float vertical_ground_distance_m{0.0f}; // Tilt-compensated vertical ground distance
    float terrain_elevation_ned_z{0.0f};    // Smoothed ground elevation in world NED (Z down)
    bool ground_contact_valid{false};
    bool range_gated{false};         // True if altitude is outside [min_range_m, max_range_m]
    bool tilt_gated{false};          // True if vehicle tilt exceeds max allowed tilt
    float current_tilt_deg{0.0f};
    float max_tilt_deg{35.0f};
    uint8_t terrain_src{0};          // 0 = Downward Radar, 1 = Baro/GPS relative
};

class RadarManager {
public:
    static RadarManager& get_instance();

    RadarManager(const RadarManager&) = delete;
    RadarManager& operator=(const RadarManager&) = delete;

    void init();
    void reload_params();

    // Data ingestion interfaces (same firmware pipeline for SITL, CAN and UART)
    void feed_sitl_radar(const float distances[kNumRadarSensors], uint16_t valid_mask, uint64_t time_us);
    void feed_sensor_reading(uint8_t sensor_idx, float distance_m, uint8_t quality, bool valid, uint64_t time_us);

    // Periodic update with current vehicle attitude and position
    void update(uint64_t time_us, const float pos_ned[3], float roll, float pitch, float yaw, float dt_s);

    // Terrain-following altitude adjustment for navigation cascade / flight modes
    // Given the flight mode's desired altitude setpoint in NED Z (downward positive),
    // returns the effective altitude setpoint respecting terrain clearance.
    float adjust_target_altitude(float mode_target_ned_z, float current_pos_ned_z) const;

    // Status queries
    bool is_radar_system_enabled() const { return m_master_enabled; }
    bool is_terrain_following_active() const { return m_terrain.active && m_terrain.ground_contact_valid; }
    
    RadarSensorReading get_sensor_reading(uint8_t sensor_idx) const;
    RadarSensorConfig get_sensor_config(uint8_t sensor_idx) const;
    TerrainState get_terrain_state() const;

    // Obstacle distance 360-degree array (72 bins, 5 degrees each) for MAVLink OBSTACLE_DISTANCE
    void get_obstacle_distances_cm(uint16_t out_distances_cm[72], float& out_min_m, float& out_max_m) const;

private:
    RadarManager();
    ~RadarManager() = default;

    mutable apx::kernel::Mutex m_mutex;

    bool m_master_enabled{false};
    float m_radar_rate_hz{20.0f};
    uint32_t m_timeout_ms{500};

    std::array<RadarSensorConfig, kNumRadarSensors> m_configs{};
    std::array<RadarSensorReading, kNumRadarSensors> m_readings{};

    TerrainState m_terrain{};
    bool m_terrain_filter_initialized{false};
};

} // namespace apx::radar
