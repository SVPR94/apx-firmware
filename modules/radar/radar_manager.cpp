// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "radar_manager.hpp"
#include "param_manager.hpp"
#include <cstdio>
#include <cstring>

namespace apx::radar {

namespace {
constexpr float kRadToDeg = 57.2957795f;
constexpr float kDegToRad = 0.017453293f;
}

RadarManager& RadarManager::get_instance() {
    static RadarManager instance;
    return instance;
}

RadarManager::RadarManager() {
    // Default orientations matching standard MAVLink sensor rotations:
    // 0: Down -> MAV_SENSOR_ROT_PITCH_270 (25)
    // 1: Front -> MAV_SENSOR_ROT_NONE (0)
    // 2: Front-Right -> MAV_SENSOR_ROT_YAW_45 (1)
    // 3: Right -> MAV_SENSOR_ROT_YAW_90 (2)
    // 4: Rear-Right -> MAV_SENSOR_ROT_YAW_135 (3)
    // 5: Rear -> MAV_SENSOR_ROT_YAW_180 (4)
    // 6: Rear-Left -> MAV_SENSOR_ROT_YAW_225 (5)
    // 7: Left -> MAV_SENSOR_ROT_YAW_270 (6)
    // 8: Front-Left -> MAV_SENSOR_ROT_YAW_315 (7)
    // 9: Top -> MAV_SENSOR_ROT_PITCH_90 (24)
    static const uint8_t kDefaultOrientations[kNumRadarSensors] = {
        25, 0, 1, 2, 3, 4, 5, 6, 7, 24
    };

    static const float kDefaultYaws[kNumRadarSensors] = {
        0.0f, 0.0f, 45.0f, 90.0f, 135.0f, 180.0f, 225.0f, 270.0f, 315.0f, 0.0f
    };

    for (int i = 0; i < kNumRadarSensors; ++i) {
        m_configs[i].enabled = true; // Enabled by default when radar system is active
        m_configs[i].orientation = kDefaultOrientations[i];
        m_configs[i].mounting_yaw_deg = kDefaultYaws[i];
        m_configs[i].mounting_pitch_deg = (i == 0) ? -90.0f : ((i == 9) ? 90.0f : 0.0f);
        m_configs[i].mounting_roll_deg = 0.0f;
        m_configs[i].min_distance_m = 0.2f;
        m_configs[i].max_distance_m = 40.0f;

        m_readings[i].distance_m = 0.0f;
        m_readings[i].signal_quality = 100;
        m_readings[i].valid = false;
        m_readings[i].healthy = false;
        m_readings[i].last_update_us = 0;
    }

    m_terrain.enabled = true;
    m_terrain.active = false;
    m_terrain.target_clearance_m = 2.0f;
    m_terrain.max_tilt_deg = 35.0f;
    m_terrain.terrain_src = 0;
}

void RadarManager::init() {
    reload_params();
}

void RadarManager::reload_params() {
    auto& params = apx::gcs::ParamManager::get_instance();
    apx::kernel::LockGuard lock(m_mutex);

    m_master_enabled = (params.get_int_or("RADAR_ENABLE", 1) != 0);
    m_radar_rate_hz = params.get_float_or("RADAR_RATE", 20.0f);
    m_timeout_ms = static_cast<uint32_t>(params.get_int_or("RADAR_TIMEOUT_MS", 500));

    m_terrain.enabled = (params.get_int_or("TERRAIN_ENABLE", 1) != 0);
    m_terrain.target_clearance_m = params.get_float_or("TERRAIN_ALT_TGT", 2.0f);
    m_terrain.min_range_m = params.get_float_or("TERRAIN_MIN_M", 2.0f);
    m_terrain.max_range_m = params.get_float_or("TERRAIN_MAX_M", 30.0f);
    m_terrain.max_tilt_deg = params.get_float_or("TERRAIN_MAX_TILT", 35.0f);
    m_terrain.terrain_src = static_cast<uint8_t>(params.get_int_or("TERRAIN_SRC", 0));

    for (int i = 0; i < kNumRadarSensors; ++i) {
        char name[17];
        std::snprintf(name, sizeof(name), "RADAR%d_ENABLE", i + 1);
        m_configs[i].enabled = (params.get_int_or(name, 1) != 0);

        std::snprintf(name, sizeof(name), "RADAR%d_ORIENT", i + 1);
        m_configs[i].orientation = static_cast<uint8_t>(params.get_int_or(name, m_configs[i].orientation));

        std::snprintf(name, sizeof(name), "RADAR%d_MIN_CM", i + 1);
        m_configs[i].min_distance_m = params.get_float_or(name, (i == 0) ? 200.0f : 20.0f) * 0.01f;

        std::snprintf(name, sizeof(name), "RADAR%d_MAX_CM", i + 1);
        m_configs[i].max_distance_m = params.get_float_or(name, (i == 0) ? 3000.0f : 4000.0f) * 0.01f;

        std::snprintf(name, sizeof(name), "RADAR%d_POS_X", i + 1);
        m_configs[i].mounting_pos[0] = params.get_float_or(name, 0.0f);

        std::snprintf(name, sizeof(name), "RADAR%d_POS_Y", i + 1);
        m_configs[i].mounting_pos[1] = params.get_float_or(name, 0.0f);

        std::snprintf(name, sizeof(name), "RADAR%d_POS_Z", i + 1);
        m_configs[i].mounting_pos[2] = params.get_float_or(name, 0.0f);
    }
}

void RadarManager::feed_sitl_radar(const float distances[kNumRadarSensors], uint16_t valid_mask, uint64_t time_us) {
    apx::kernel::LockGuard lock(m_mutex);
    for (int i = 0; i < kNumRadarSensors; ++i) {
        if (!m_configs[i].enabled) continue;
        const bool valid = (valid_mask & (1 << i)) != 0;
        const float dist = distances[i];
        m_readings[i].distance_m = dist;
        m_readings[i].signal_quality = 100;
        m_readings[i].healthy = true;
        m_readings[i].last_update_us = time_us;
        m_readings[i].valid = valid && (dist >= m_configs[i].min_distance_m && dist <= m_configs[i].max_distance_m);
    }
}

void RadarManager::feed_sensor_reading(uint8_t sensor_idx, float distance_m, uint8_t quality, bool valid, uint64_t time_us) {
    if (sensor_idx >= kNumRadarSensors) return;
    apx::kernel::LockGuard lock(m_mutex);
    if (!m_configs[sensor_idx].enabled) return;

    m_readings[sensor_idx].distance_m = distance_m;
    m_readings[sensor_idx].signal_quality = quality;
    m_readings[sensor_idx].healthy = true;
    m_readings[sensor_idx].last_update_us = time_us;
    m_readings[sensor_idx].valid = valid && (distance_m >= m_configs[sensor_idx].min_distance_m && distance_m <= m_configs[sensor_idx].max_distance_m);
}

void RadarManager::update(uint64_t time_us, const float pos_ned[3], float roll, float pitch, float yaw, float dt_s) {
    apx::kernel::LockGuard lock(m_mutex);

    // 1. Timeout / Health check
    const uint64_t timeout_us = static_cast<uint64_t>(m_timeout_ms) * 1000ULL;
    for (int i = 0; i < kNumRadarSensors; ++i) {
        if (m_readings[i].last_update_us == 0 || (time_us > m_readings[i].last_update_us + timeout_us)) {
            m_readings[i].healthy = false;
            m_readings[i].valid = false;
        }
    }

    if (!m_master_enabled) {
        m_terrain.active = false;
        return;
    }

    // 2. Downward sensor processing & attitude compensation
    const auto& down_cfg = m_configs[static_cast<int>(RadarDirection::DOWN)];
    auto& down_reading = m_readings[static_cast<int>(RadarDirection::DOWN)];

    if (m_terrain.enabled && down_cfg.enabled && down_reading.healthy) {
        // Vertical projection factor: vz = cos(pitch)*cos(roll)
        const float vz = std::cos(pitch) * std::cos(roll);
        const float tilt_rad = std::acos(std::clamp(vz, -1.0f, 1.0f));
        m_terrain.current_tilt_deg = tilt_rad * kRadToDeg;

        // Attitude compensated vertical ground clearance
        // h_agl = distance * vz + mounting_z_offset
        const float h_agl = down_reading.distance_m * vz + down_cfg.mounting_pos[2];
        m_terrain.vertical_ground_distance_m = h_agl;

        // Check altimeter effect range gating: reading must be valid and within [min_range_m, max_range_m]
        if (!down_reading.valid || h_agl < m_terrain.min_range_m || h_agl > m_terrain.max_range_m) {
            m_terrain.range_gated = true;
            m_terrain.ground_contact_valid = false;
            m_terrain.active = false;
        } else if (m_terrain.current_tilt_deg > m_terrain.max_tilt_deg) {
            // Tilt gating check
            m_terrain.tilt_gated = true;
            m_terrain.range_gated = false;
            // Hold last known ground elevation during steep maneuvers; do not update from angled beam
        } else {
            m_terrain.tilt_gated = false;
            m_terrain.range_gated = false;

            // Current terrain elevation in NED (Z down):
            // z_terrain = current_pos_z + h_agl
            const float raw_terrain_z = pos_ned[2] + h_agl;

            if (!m_terrain_filter_initialized) {
                m_terrain.terrain_elevation_ned_z = raw_terrain_z;
                m_terrain_filter_initialized = true;
            } else {
                // Smooth terrain elevation with low-pass filter to reject surface roughness and spikes
                const float alpha = std::clamp(dt_s * 5.0f, 0.05f, 0.4f);
                m_terrain.terrain_elevation_ned_z += alpha * (raw_terrain_z - m_terrain.terrain_elevation_ned_z);
            }

            m_terrain.ground_contact_valid = true;
            m_terrain.active = true;
        }
    } else {
        // Downward sensor timed out or disabled
        m_terrain.range_gated = false;
        m_terrain.ground_contact_valid = false;
        m_terrain.active = false;
    }
}

float RadarManager::adjust_target_altitude(float mode_target_ned_z, float current_pos_ned_z) const {
    apx::kernel::LockGuard lock(m_mutex);

    // Altimeter effect ONLY applies when active and within reading range [min_range_m, max_range_m]
    if (!m_master_enabled || !m_terrain.enabled || !m_terrain.ground_contact_valid || !m_terrain.active) {
        return mode_target_ned_z; // Return mode setpoint unaltered
    }

    // Target clearance above ground
    float clearance = m_terrain.target_clearance_m;
    if (clearance <= 0.1f) {
        // If target clearance param is 0, use the relative altitude from the mode
        clearance = std::max(-mode_target_ned_z, 1.0f);
    }

    // Commanded altitude in NED: z_target = z_terrain - clearance (climbing moves Z more negative)
    const float terrain_target_z = m_terrain.terrain_elevation_ned_z - clearance;
    return terrain_target_z;
}

RadarSensorReading RadarManager::get_sensor_reading(uint8_t sensor_idx) const {
    apx::kernel::LockGuard lock(m_mutex);
    if (sensor_idx < kNumRadarSensors) {
        return m_readings[sensor_idx];
    }
    return RadarSensorReading{};
}

RadarSensorConfig RadarManager::get_sensor_config(uint8_t sensor_idx) const {
    apx::kernel::LockGuard lock(m_mutex);
    if (sensor_idx < kNumRadarSensors) {
        return m_configs[sensor_idx];
    }
    return RadarSensorConfig{};
}

TerrainState RadarManager::get_terrain_state() const {
    apx::kernel::LockGuard lock(m_mutex);
    return m_terrain;
}

void RadarManager::get_obstacle_distances_cm(uint16_t out_distances_cm[72], float& out_min_m, float& out_max_m) const {
    apx::kernel::LockGuard lock(m_mutex);
    out_min_m = 0.2f;
    out_max_m = 40.0f;

    // Initialize all 72 bins to UINT16_MAX (unknown/no obstacle)
    for (int i = 0; i < 72; ++i) {
        out_distances_cm[i] = UINT16_MAX;
    }

    // Map the 8 horizontal perimeter sensors to their 5-degree bin centers
    // 1: Front (0 deg -> bin 0)
    // 2: Front-Right (45 deg -> bin 9)
    // 3: Right (90 deg -> bin 18)
    // 4: Rear-Right (135 deg -> bin 27)
    // 5: Rear (180 deg -> bin 36)
    // 6: Rear-Left (225 deg -> bin 45)
    // 7: Left (270 deg -> bin 54)
    // 8: Front-Left (315 deg -> bin 63)
    static const struct {
        uint8_t sensor_idx;
        int center_bin;
    } kHorizontalMap[8] = {
        {1, 0},
        {2, 9},
        {3, 18},
        {4, 27},
        {5, 36},
        {6, 45},
        {7, 54},
        {8, 63}
    };

    for (const auto& item : kHorizontalMap) {
        const auto& r = m_readings[item.sensor_idx];
        const auto& cfg = m_configs[item.sensor_idx];
        if (cfg.enabled && r.healthy && r.valid && r.distance_m <= cfg.max_distance_m) {
            const uint16_t dist_cm = static_cast<uint16_t>(std::round(r.distance_m * 100.0f));
            // Fill beam arc (spread 3 bins: center +/- 1)
            for (int b = -1; b <= 1; ++b) {
                int bin = (item.center_bin + b + 72) % 72;
                if (dist_cm < out_distances_cm[bin]) {
                    out_distances_cm[bin] = dist_cm;
                }
            }
        }
    }
}

} // namespace apx::radar
