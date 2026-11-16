// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include <gtest/gtest.h>
#include "mode_auto.hpp"
#include "mode_land.hpp"
#include "param_manager.hpp"
#include <cmath>

using namespace apx::modes;

namespace {
constexpr float kDegToRad = 0.017453293f;
}

class ModeAutoConditionYawTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto& params = apx::gcs::ParamManager::get_instance();
        params.set_float("WPNAV_SPEED", 500.0f);     // 5 m/s
        params.set_float("WPNAV_RADIUS", 100.0f);    // 1 m
        params.set_int("WP_YAW_BEHAVIOR", 2);        // Face next WP
        params.set_float("AUTO_YAW_HEAD", -1.0f);    // Default: use entry
        params.set_float("LAND_SPEED", 50.0f);       // 50 cm/s
        params.set_float("LAND_REPOS_SPEED", 150.0f);// 1.5 m/s
    }
};

TEST_F(ModeAutoConditionYawTest, ConditionYawRotateAndLockHeading) {
    ModeAuto mode_auto;

    // Mission:
    // 0: Waypoint at (10, 0, -10)
    // 1: CONDITION_YAW: target = 90 deg (East), speed = 45 deg/s, dir = 1 (CW), rel = 0 (abs)
    // 2: Waypoint at (10, 20, -10)  -- translates East/Rightwards!
    std::vector<Waypoint> mission;

    Waypoint wp0;
    wp0.seq = 1;
    wp0.command = ModeAuto::kCmdNavWaypoint;
    wp0.pos_ned[0] = 2.0f;
    wp0.pos_ned[1] = 0.0f;
    wp0.pos_ned[2] = -10.0f;
    wp0.accept_radius_m = 1.0f;
    mission.push_back(wp0);

    Waypoint wp1;
    wp1.seq = 2;
    wp1.command = ModeAuto::kCmdConditionYaw;
    wp1.param1 = 90.0f; // 90 deg (pi/2 rad)
    wp1.param2 = 45.0f; // 45 deg/s
    wp1.param3 = 1.0f;  // CW
    wp1.param4 = 0.0f;  // Absolute
    mission.push_back(wp1);

    Waypoint wp2;
    wp2.seq = 3;
    wp2.command = ModeAuto::kCmdNavWaypoint;
    wp2.pos_ned[0] = 2.0f;
    wp2.pos_ned[1] = 20.0f;
    wp2.pos_ned[2] = -10.0f;
    wp2.accept_radius_m = 1.0f;
    mission.push_back(wp2);

    mode_auto.load_mission(mission);

    VehicleState state;
    state.armed = true;
    state.dt_s = 0.02f;
    state.pos_ned[0] = 0.0f;
    state.pos_ned[1] = 0.0f;
    state.pos_ned[2] = -10.0f;
    state.yaw = 0.0f; // Initial yaw North (0 rad)

    mode_auto.on_enter(state);
    EXPECT_FALSE(mode_auto.is_yaw_locked());

    // Step vehicle towards wp0 until reached
    for (int i = 0; i < 500 && mode_auto.current_seq() == 1; ++i) {
        ControlOutput out = mode_auto.update(state);
        state.pos_ned[0] += (2.0f - state.pos_ned[0]) * 0.05f;
    }
    state.pos_ned[0] = 2.0f;
    state.pos_ned[1] = 0.0f;
    state.pos_ned[2] = -10.0f;
    mode_auto.update(state);

    // A nav item follows the CONDITION_YAW, so (as in apx) it does not
    // block: the next leg starts at once and the heading turns meanwhile.
    float target_yaw_rad = 90.0f * kDegToRad;
    EXPECT_EQ(mode_auto.current_seq(), 3);
    EXPECT_TRUE(mode_auto.is_yaw_locked());
    EXPECT_NEAR(mode_auto.locked_yaw(), target_yaw_rad, 0.01f);

    // Commanded heading slews toward the target at <= 45 deg/s (no step).
    float prev = mode_auto.commanded_yaw();
    for (int i = 0; i < 150; ++i) {
        mode_auto.update(state);
        const float yaw_cmd = mode_auto.commanded_yaw();
        EXPECT_LE(yaw_cmd - prev, 45.0f * kDegToRad * state.dt_s + 1e-4f);
        EXPECT_GE(yaw_cmd, prev - 1e-4f); // CW, never backwards
        prev = yaw_cmd;
        state.yaw = yaw_cmd;
        state.pos_ned[1] += 0.05f; // flying east along the leg
    }
    EXPECT_NEAR(mode_auto.commanded_yaw(), target_yaw_rad, 0.01f);
    EXPECT_EQ(mode_auto.current_seq(), 3); // still flying the leg, never held

    // Heading stays locked at 90 degrees for the leg
    mode_auto.update(state);
    EXPECT_NEAR(mode_auto.locked_yaw(), target_yaw_rad, 0.01f);
}

TEST_F(ModeAutoConditionYawTest, ConditionYawAtMissionEndTurnsInPlace) {
    ModeAuto mode_auto;
    // 0: Waypoint (stop), 1: CONDITION_YAW 90 deg - nothing to fly after it,
    // so the vehicle turns in place and the item completes on the heading.
    std::vector<Waypoint> mission;
    Waypoint wp0;
    wp0.seq = 1;
    wp0.command = ModeAuto::kCmdNavWaypoint;
    wp0.pos_ned[0] = 2.0f;
    wp0.pos_ned[2] = -10.0f;
    wp0.accept_radius_m = 1.0f;
    mission.push_back(wp0);
    Waypoint yaw;
    yaw.seq = 2;
    yaw.command = ModeAuto::kCmdConditionYaw;
    yaw.param1 = 90.0f;
    yaw.param2 = 45.0f;
    mission.push_back(yaw);
    mode_auto.load_mission(mission);

    VehicleState state;
    state.armed = true;
    state.dt_s = 0.02f;
    state.pos_ned[2] = -10.0f;
    mode_auto.on_enter(state);
    for (int i = 0; i < 2000 && mode_auto.current_seq() == 1; ++i) {
        mode_auto.update(state);
        state.pos_ned[0] += (2.0f - state.pos_ned[0]) * 0.05f;
    }
    ASSERT_EQ(mode_auto.current_seq(), 2);

    const float target_yaw_rad = 90.0f * kDegToRad;
    for (int i = 0; i < 300 && mode_auto.current_seq() == 2; ++i) {
        mode_auto.update(state);
        state.yaw = mode_auto.commanded_yaw();
        state.gyro_body[2] = 0.0f;
    }
    EXPECT_EQ(mode_auto.current_seq(), -1); // completed -> mission complete
    EXPECT_TRUE(mode_auto.is_yaw_locked());
    EXPECT_NEAR(mode_auto.locked_yaw(), target_yaw_rad, 0.01f);
}

TEST_F(ModeAutoConditionYawTest, ConstantHeadingParamBehavior) {
    auto& params = apx::gcs::ParamManager::get_instance();
    params.set_int("WP_YAW_BEHAVIOR", 0); // 0 = Constant heading
    params.set_float("AUTO_YAW_HEAD", 45.0f); // 45 deg

    ModeAuto mode_auto;
    std::vector<Waypoint> mission;
    Waypoint wp0;
    wp0.seq = 1;
    wp0.command = ModeAuto::kCmdNavWaypoint;
    wp0.pos_ned[0] = 50.0f;
    wp0.pos_ned[1] = 0.0f;
    wp0.pos_ned[2] = -15.0f;
    mission.push_back(wp0);
    mode_auto.load_mission(mission);

    VehicleState state;
    state.armed = true;
    state.dt_s = 0.02f;
    state.pos_ned[0] = 0.0f;
    state.pos_ned[1] = 0.0f;
    state.pos_ned[2] = -15.0f;
    state.yaw = 0.0f;

    mode_auto.on_enter(state);

    // Should immediately be locked to AUTO_YAW_HEAD (45 deg = 0.785398 rad)
    EXPECT_TRUE(mode_auto.is_yaw_locked());
    EXPECT_NEAR(mode_auto.locked_yaw(), 45.0f * kDegToRad, 0.01f);
}

TEST_F(ModeAutoConditionYawTest, ConditionChangeAltClimb) {
    ModeAuto mode_auto;
    std::vector<Waypoint> mission;

    Waypoint wp_alt;
    wp_alt.seq = 1;
    wp_alt.command = ModeAuto::kCmdConditionChangeAlt;
    wp_alt.param1 = 2.0f;  // Climb rate 2 m/s
    wp_alt.param2 = 25.0f; // Target altitude 25m (z = -25m)
    mission.push_back(wp_alt);

    Waypoint wp_end;
    wp_end.seq = 2;
    wp_end.command = ModeAuto::kCmdNavWaypoint;
    wp_end.pos_ned[0] = 10.0f;
    wp_end.pos_ned[1] = 10.0f;
    wp_end.pos_ned[2] = -25.0f;
    mission.push_back(wp_end);

    mode_auto.load_mission(mission);

    VehicleState state;
    state.armed = true;
    state.dt_s = 0.02f;
    state.pos_ned[0] = 0.0f;
    state.pos_ned[1] = 0.0f;
    state.pos_ned[2] = -10.0f; // 10m initial altitude
    state.yaw = 0.0f;

    mode_auto.on_enter(state);
    EXPECT_EQ(mode_auto.current_seq(), 1);

    // Vehicle climbs towards 25m
    for (int i = 0; i < 50 && mode_auto.current_seq() == 1; ++i) {
        ControlOutput out = mode_auto.update(state);
        state.pos_ned[2] -= 0.3f; // climbing
    }

    state.pos_ned[2] = -25.0f;
    mode_auto.update(state);

    // Should now have completed alt condition and moved to next item
    EXPECT_EQ(mode_auto.current_seq(), 2);
}

TEST_F(ModeAutoConditionYawTest, DoChangeSpeedExecution) {
    ModeAuto mode_auto;
    std::vector<Waypoint> mission;

    Waypoint wp_spd;
    wp_spd.seq = 1;
    wp_spd.command = ModeAuto::kCmdDoChangeSpeed;
    wp_spd.param1 = 1.0f;  // Groundspeed
    wp_spd.param2 = 8.5f;  // 8.5 m/s
    mission.push_back(wp_spd);

    Waypoint wp_nav;
    wp_nav.seq = 2;
    wp_nav.command = ModeAuto::kCmdNavWaypoint;
    wp_nav.pos_ned[0] = 100.0f;
    wp_nav.pos_ned[1] = 0.0f;
    wp_nav.pos_ned[2] = -10.0f;
    mission.push_back(wp_nav);

    mode_auto.load_mission(mission);

    VehicleState state;
    state.armed = true;
    state.dt_s = 0.02f;
    state.pos_ned[0] = 0.0f;
    state.pos_ned[1] = 0.0f;
    state.pos_ned[2] = -10.0f;
    state.yaw = 0.0f;

    mode_auto.on_enter(state);

    // DoChangeSpeed is instantaneous, so first update immediately executes it
    // and begins navigation to wp_nav
    mode_auto.update(state);
    EXPECT_EQ(mode_auto.current_seq(), 2);
}

TEST_F(ModeAutoConditionYawTest, ModeLandManualRepositioning) {
    ModeLand mode_land;

    VehicleState state;
    state.armed = true;
    state.dt_s = 0.05f;
    state.pos_ned[0] = 0.0f;
    state.pos_ned[1] = 0.0f;
    state.pos_ned[2] = -10.0f;
    state.yaw = 0.0f; // Facing North
    state.rc_valid = true;

    mode_land.on_enter(state);
    EXPECT_NEAR(mode_land.target_yaw(), 0.0f, 0.001f);
    EXPECT_NEAR(mode_land.target_pos_ned()[0], 0.0f, 0.001f);
    EXPECT_NEAR(mode_land.target_pos_ned()[1], 0.0f, 0.001f);

    // Pilot pushes pitch stick forward (0.8) and yaw stick right (0.5)
    state.rc_pitch = 0.8f;
    state.rc_roll = 0.0f;
    state.rc_yaw = 0.5f;

    mode_land.update(state);

    // Target North position should have nudged forward (> 0)
    EXPECT_GT(mode_land.target_pos_ned()[0], 0.0f);
    // Target yaw should have slewed CW (> 0)
    EXPECT_GT(mode_land.target_yaw(), 0.0f);
}

TEST_F(ModeAutoConditionYawTest, ConditionYawHighRateSmoothDeceleration) {
    ModeAuto mode_auto;
    std::vector<Waypoint> mission;

    Waypoint wp0;
    wp0.seq = 1;
    wp0.command = ModeAuto::kCmdNavWaypoint;
    wp0.pos_ned[0] = 1.0f;
    wp0.pos_ned[1] = 0.0f;
    wp0.pos_ned[2] = -10.0f;
    wp0.accept_radius_m = 1.0f;
    mission.push_back(wp0);

    // CONDITION_YAW with high yaw rate (120 deg/s) to 90 deg
    Waypoint wp_yaw;
    wp_yaw.seq = 2;
    wp_yaw.command = ModeAuto::kCmdConditionYaw;
    wp_yaw.param1 = 90.0f;  // 90 deg target
    wp_yaw.param2 = 120.0f; // 120 deg/s high rate
    wp_yaw.param3 = 1.0f;   // CW
    wp_yaw.param4 = 0.0f;   // Absolute
    mission.push_back(wp_yaw);

    Waypoint wp_next;
    wp_next.seq = 3;
    wp_next.command = ModeAuto::kCmdNavWaypoint;
    wp_next.pos_ned[0] = 1.0f;
    wp_next.pos_ned[1] = 20.0f;
    wp_next.pos_ned[2] = -10.0f;
    mission.push_back(wp_next);

    mode_auto.load_mission(mission);

    VehicleState state;
    state.armed = true;
    state.dt_s = 0.02f;
    state.pos_ned[0] = 1.0f;
    state.pos_ned[1] = 0.0f;
    state.pos_ned[2] = -10.0f;
    state.yaw = 0.0f;

    mode_auto.on_enter(state);
    // Complete wp0; the CONDITION_YAW runs while flying to wp_next.
    mode_auto.update(state);
    EXPECT_EQ(mode_auto.current_seq(), 3);

    float target_yaw_rad = 90.0f * kDegToRad;
    float prev_rate = 0.0f;
    float max_rate = 0.0f;
    bool deceleration_observed = false;
    float prev_cmd = mode_auto.commanded_yaw();
    for (int i = 0; i < 150; ++i) {
        mode_auto.update(state);
        const float cmd = mode_auto.commanded_yaw();
        const float rate = (cmd - prev_cmd) / state.dt_s;
        max_rate = std::max(max_rate, rate);
        // eases into the target: rate falls while the error is small
        if (i > 5 && std::fabs(target_yaw_rad - cmd) < 0.20f && rate < prev_rate) {
            deceleration_observed = true;
        }
        EXPECT_GE(rate, -1e-3f); // never turns back / loops
        prev_rate = rate;
        prev_cmd = cmd;
        state.yaw = cmd;
    }
    EXPECT_LE(max_rate, 120.0f * kDegToRad + 1e-3f);
    EXPECT_TRUE(deceleration_observed);
    EXPECT_NEAR(mode_auto.commanded_yaw(), target_yaw_rad, 0.01f);

    // Settled and locked exact heading without looping
    EXPECT_TRUE(mode_auto.is_yaw_locked());
    EXPECT_NEAR(mode_auto.locked_yaw(), target_yaw_rad, 0.02f);
    EXPECT_EQ(mode_auto.current_seq(), 3);
}

TEST_F(ModeAutoConditionYawTest, ModeLandForwardBrakingAndStoppedPosition) {
    ModeLand mode_land;

    VehicleState state;
    state.armed = true;
    state.dt_s = 0.02f;
    state.pos_ned[0] = 50.0f;
    state.pos_ned[1] = 10.0f;
    state.pos_ned[2] = -20.0f;
    state.yaw = 0.0f;
    // Vehicle is traveling forward at 4.0 m/s East when LAND command is received
    state.vel_ned[0] = 0.0f;
    state.vel_ned[1] = 4.0f;
    state.vel_ned[2] = 0.0f;

    mode_land.on_enter(state);

    // Initial phase must be Braking (not descending immediately at command reception point)
    EXPECT_EQ(mode_land.phase(), LandPhase::Braking);

    // Projected stopping position must be ahead along travel direction (East > 10.0m)
    // Stopping distance at 4 m/s and 1.5 m/s^2 is ~ 4^2 / (2 * 1.5) = 5.33m -> East ~ 15.33m
    EXPECT_NEAR(mode_land.target_pos_ned()[0], 50.0f, 0.1f);
    EXPECT_GT(mode_land.target_pos_ned()[1], 14.0f);

    // Simulate vehicle decelerating to a complete stop
    for (int i = 0; i < 200 && mode_land.phase() == LandPhase::Braking; ++i) {
        mode_land.update(state);
        // Vehicle slows down forward
        if (state.vel_ned[1] > 0.05f) {
            state.vel_ned[1] -= 1.5f * state.dt_s;
            state.pos_ned[1] += state.vel_ned[1] * state.dt_s;
        } else {
            state.vel_ned[1] = 0.0f;
        }
    }

    state.vel_ned[1] = 0.0f;
    mode_land.update(state);

    // Once vehicle has safely stopped, phase must transition to Descend,
    // and touchdown position must be locked at the exact stopped coordinates!
    EXPECT_EQ(mode_land.phase(), LandPhase::Descend);
    EXPECT_NEAR(mode_land.target_pos_ned()[1], state.pos_ned[1], 0.2f);
}
