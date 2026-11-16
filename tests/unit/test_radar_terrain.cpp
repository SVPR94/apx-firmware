// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include <gtest/gtest.h>
#include "radar_manager.hpp"
#include "param_manager.hpp"
#include <cmath>

using apx::radar::RadarManager;
using apx::radar::RadarDirection;
using apx::radar::kNumRadarSensors;

TEST(RadarTerrainTest, DefaultConfigurationsAndOrientations) {
    auto& rm = RadarManager::get_instance();
    rm.init();

    EXPECT_TRUE(rm.is_radar_system_enabled());

    // 10 sensors check
    EXPECT_EQ(rm.get_sensor_config(0).orientation, 25); // Down
    EXPECT_EQ(rm.get_sensor_config(1).orientation, 0);  // Front
    EXPECT_EQ(rm.get_sensor_config(2).orientation, 1);  // Front-Right
    EXPECT_EQ(rm.get_sensor_config(3).orientation, 2);  // Right
    EXPECT_EQ(rm.get_sensor_config(4).orientation, 3);  // Rear-Right
    EXPECT_EQ(rm.get_sensor_config(5).orientation, 4);  // Rear
    EXPECT_EQ(rm.get_sensor_config(6).orientation, 5);  // Rear-Left
    EXPECT_EQ(rm.get_sensor_config(7).orientation, 6);  // Left
    EXPECT_EQ(rm.get_sensor_config(8).orientation, 7);  // Front-Left
    EXPECT_EQ(rm.get_sensor_config(9).orientation, 24); // Top
}

TEST(RadarTerrainTest, DistanceRangeGatingAndTimeout) {
    auto& rm = RadarManager::get_instance();
    rm.init();

    uint64_t t0 = 1000000; // 1s
    // Valid distance (5m)
    rm.feed_sensor_reading(1, 5.0f, 95, true, t0);
    auto r1 = rm.get_sensor_reading(1);
    EXPECT_TRUE(r1.valid);
    EXPECT_TRUE(r1.healthy);
    EXPECT_NEAR(r1.distance_m, 5.0f, 1e-3f);
    EXPECT_EQ(r1.signal_quality, 95);

    // Below min range (0.1m < 0.2m min)
    rm.feed_sensor_reading(1, 0.1f, 95, true, t0);
    EXPECT_FALSE(rm.get_sensor_reading(1).valid);

    // Above max range (50m > 40m max)
    rm.feed_sensor_reading(1, 50.0f, 95, true, t0);
    EXPECT_FALSE(rm.get_sensor_reading(1).valid);

    // Timeout check (timeout is 500ms)
    float pos[3] = {0, 0, -5.0f};
    rm.update(t0 + 600000, pos, 0.0f, 0.0f, 0.0f, 0.02f);
    EXPECT_FALSE(rm.get_sensor_reading(1).healthy);
}

TEST(RadarTerrainTest, AttitudeCompensationAndTiltGating) {
    auto& rm = RadarManager::get_instance();
    rm.init();

    uint64_t t = 2000000;
    float pos_ned[3] = {0.0f, 0.0f, -5.0f}; // 5m altitude above ground (NED Z is -5)

    // Case 1: Level flight (roll = 0, pitch = 0)
    // Downward beam measures exactly 5.0m
    rm.feed_sensor_reading(0, 5.0f, 100, true, t);
    rm.update(t, pos_ned, 0.0f, 0.0f, 0.0f, 0.02f);

    auto terrain = rm.get_terrain_state();
    EXPECT_TRUE(terrain.active);
    EXPECT_TRUE(terrain.ground_contact_valid);
    EXPECT_FALSE(terrain.tilt_gated);
    EXPECT_NEAR(terrain.vertical_ground_distance_m, 5.0f, 0.05f);
    // Ground elevation should be pos_z (-5) + h_agl (5) = 0.0m
    EXPECT_NEAR(terrain.terrain_elevation_ned_z, 0.0f, 0.05f);

    // Case 2: Pitched 30 degrees (tilt = 30 deg < max_tilt 35 deg)
    // Slanted beam to ground measures: 5.0 / cos(30 deg) = 5.7735m
    float pitch_30deg = 30.0f * 0.017453293f;
    float slanted_dist = 5.0f / std::cos(pitch_30deg);
    t += 20000;
    rm.feed_sensor_reading(0, slanted_dist, 100, true, t);
    rm.update(t, pos_ned, 0.0f, pitch_30deg, 0.0f, 0.02f);

    terrain = rm.get_terrain_state();
    EXPECT_FALSE(terrain.tilt_gated);
    // Attitude compensation must recover true 5.0m vertical distance!
    EXPECT_NEAR(terrain.vertical_ground_distance_m, 5.0f, 0.05f);
    EXPECT_NEAR(terrain.terrain_elevation_ned_z, 0.0f, 0.05f);

    // Case 3: Pitched 45 degrees (tilt = 45 deg > max_tilt 35 deg)
    // Tilt gating should activate to prevent drone diving
    float pitch_45deg = 45.0f * 0.017453293f;
    t += 20000;
    rm.feed_sensor_reading(0, 10.0f, 100, true, t);
    rm.update(t, pos_ned, 0.0f, pitch_45deg, 0.0f, 0.02f);

    terrain = rm.get_terrain_state();
    EXPECT_TRUE(terrain.tilt_gated);
    // Terrain elevation should stay held at 0.0m, not jump from the bad 10m beam!
    EXPECT_NEAR(terrain.terrain_elevation_ned_z, 0.0f, 0.1f);
}

TEST(RadarTerrainTest, TerrainFollowingAltitudeAdjustment) {
    auto& rm = RadarManager::get_instance();
    rm.init();

    // Set target clearance to 2.5m
    apx::gcs::ParamManager::get_instance().set_float("TERRAIN_ALT_TGT", 2.5f);
    rm.reload_params();

    uint64_t t = 3000000;
    // Ground at 0m, drone at -5m
    float pos_ned[3] = {0.0f, 0.0f, -5.0f};
    rm.feed_sensor_reading(0, 5.0f, 100, true, t);
    // Run several steps to settle filter
    for (int i = 0; i < 20; ++i) {
        t += 20000;
        rm.feed_sensor_reading(0, 5.0f, 100, true, t);
        rm.update(t, pos_ned, 0.0f, 0.0f, 0.0f, 0.02f);
    }

    // Flight mode asks for -10m setpoint
    // With terrain following active (ground = 0m, target clearance = 2.5m):
    // Effective setpoint should be 0 - 2.5m = -2.5m NED Z
    float adjusted_z = rm.adjust_target_altitude(-10.0f, pos_ned[2]);
    EXPECT_NEAR(adjusted_z, -2.5f, 0.1f);

    // Now drone flies over a hill of 3m height (ground is at -3m NED Z)
    // Ground clearance measured is 2m (since drone is at -5m, hill is at -3m)
    for (int i = 0; i < 30; ++i) {
        t += 20000;
        rm.feed_sensor_reading(0, 2.0f, 100, true, t);
        rm.update(t, pos_ned, 0.0f, 0.0f, 0.0f, 0.02f);
    }

    auto terrain = rm.get_terrain_state();
    EXPECT_NEAR(terrain.terrain_elevation_ned_z, -3.0f, 0.15f);

    // Target altitude adjusts upward to maintain 2.5m clearance over the hill!
    // z_target = -3.0 - 2.5 = -5.5m NED Z
    adjusted_z = rm.adjust_target_altitude(-10.0f, pos_ned[2]);
    EXPECT_NEAR(adjusted_z, -5.5f, 0.2f);
}

TEST(RadarTerrainTest, ObstacleDistanceBinsMapping) {
    auto& rm = RadarManager::get_instance();
    rm.init();

    uint64_t t = 4000000;
    // Front sensor (idx 1): obstacle at 3.5m
    rm.feed_sensor_reading(1, 3.5f, 95, true, t);
    // Right sensor (idx 3): obstacle at 4.2m
    rm.feed_sensor_reading(3, 4.2f, 95, true, t);

    uint16_t distances_cm[72];
    float min_m = 0.0f, max_m = 0.0f;
    rm.get_obstacle_distances_cm(distances_cm, min_m, max_m);

    // Bin 0 is Front (0 deg) -> should be 350 cm
    EXPECT_EQ(distances_cm[0], 350);
    // Bin 18 is Right (90 deg) -> should be 420 cm
    EXPECT_EQ(distances_cm[18], 420);
    // Bin 36 is Rear (180 deg) -> no obstacle reported, should be UINT16_MAX
    EXPECT_EQ(distances_cm[36], UINT16_MAX);
}

TEST(RadarTerrainTest, AltimeterReadingRangeGating) {
    auto& rm = RadarManager::get_instance();
    rm.init();

    // Configure altimeter effect range: 2.0m to 30.0m
    apx::gcs::ParamManager::get_instance().set_float("TERRAIN_MIN_M", 2.0f);
    apx::gcs::ParamManager::get_instance().set_float("TERRAIN_MAX_M", 30.0f);
    apx::gcs::ParamManager::get_instance().set_float("TERRAIN_ALT_TGT", 5.0f);
    rm.reload_params();

    uint64_t t = 5000000;
    float pos_ned[3] = {0.0f, 0.0f, -10.0f};

    // Case 1: Below minimum range (< 2.0m, e.g. 1.2m on takeoff/landing)
    // Altimeter effect MUST NOT be applied!
    rm.feed_sensor_reading(0, 1.2f, 100, true, t);
    rm.update(t, pos_ned, 0.0f, 0.0f, 0.0f, 0.02f);
    auto terrain = rm.get_terrain_state();
    EXPECT_TRUE(terrain.range_gated);
    EXPECT_FALSE(terrain.active);
    EXPECT_FALSE(terrain.ground_contact_valid);
    // Target altitude must remain unchanged from mode target (-10.0m)
    float adjusted_z = rm.adjust_target_altitude(-10.0f, pos_ned[2]);
    EXPECT_FLOAT_EQ(adjusted_z, -10.0f);

    // Case 2: Within valid reading range (2.0m to 30.0m, e.g. 6.0m)
    // Altimeter effect MUST be applied!
    for (int i = 0; i < 20; ++i) {
        t += 20000;
        rm.feed_sensor_reading(0, 6.0f, 100, true, t);
        rm.update(t, pos_ned, 0.0f, 0.0f, 0.0f, 0.02f);
    }
    terrain = rm.get_terrain_state();
    EXPECT_FALSE(terrain.range_gated);
    EXPECT_TRUE(terrain.active);
    EXPECT_TRUE(terrain.ground_contact_valid);
    // Adjusted altitude should lock to ground + target clearance:
    // ground = -10 + 6 = -4.0m, target = -4.0 - 5.0 = -9.0m
    adjusted_z = rm.adjust_target_altitude(-10.0f, pos_ned[2]);
    EXPECT_NEAR(adjusted_z, -9.0f, 0.15f);

    // Case 3: Above maximum range (> 30.0m, e.g. 35.0m)
    // Altimeter effect MUST NOT be applied!
    t += 20000;
    rm.feed_sensor_reading(0, 35.0f, 100, true, t);
    rm.update(t, pos_ned, 0.0f, 0.0f, 0.0f, 0.02f);
    terrain = rm.get_terrain_state();
    EXPECT_TRUE(terrain.range_gated);
    EXPECT_FALSE(terrain.active);
    EXPECT_FALSE(terrain.ground_contact_valid);
    adjusted_z = rm.adjust_target_altitude(-10.0f, pos_ned[2]);
    EXPECT_FLOAT_EQ(adjusted_z, -10.0f);
}
