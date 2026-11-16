// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include <gtest/gtest.h>
#include "mode_brake.hpp"
#include "mode_auto.hpp"
#include "mode_manager.hpp"
#include "sprayer.hpp"
#include "param_manager.hpp"

using namespace apx::modes;

TEST(ModeBrakeTest, ModePropertiesAndNumbering) {
    ModeBrake brake;
    EXPECT_EQ(brake.id(), FlightModeId::BRAKE);
    EXPECT_EQ(brake.get_name(), "BRAKE");
    EXPECT_TRUE(brake.is_position_required());
    EXPECT_TRUE(brake.is_gps_required());

    // Custom mode 17 mapping
    EXPECT_EQ(ModeManager::to_custom_mode_number(FlightModeId::BRAKE), 17u);
    auto mode_opt = ModeManager::from_custom_mode_number(17);
    ASSERT_TRUE(mode_opt.has_value());
    EXPECT_EQ(*mode_opt, FlightModeId::BRAKE);
}

TEST(ModeBrakeTest, EnterAndHoldStationary) {
    ModeBrake brake;
    VehicleState state;
    state.armed = true;
    state.dt_s = 0.0025f;
    state.pos_ned[0] = 10.0f;
    state.pos_ned[1] = 20.0f;
    state.pos_ned[2] = -5.0f; // 5m altitude
    state.vel_ned[0] = 0.0f;
    state.vel_ned[1] = 0.0f;
    state.vel_ned[2] = 0.0f;
    state.yaw = 1.2f;

    brake.on_enter(state);
    ControlOutput out = brake.update(state);
    // Should command thrust to hold hover altitude
    EXPECT_GT(out.thrust, 0.0f);
}

TEST(ModeBrakeTest, IgnoreStickInputs) {
    ModeBrake brake;
    VehicleState state;
    state.armed = true;
    state.dt_s = 0.0025f;
    state.pos_ned[0] = 0.0f;
    state.pos_ned[1] = 0.0f;
    state.pos_ned[2] = -5.0f;
    state.yaw = 0.0f;

    brake.on_enter(state);
    ControlOutput out1 = brake.update(state);

    // Pilot applies full pitch, roll, and yaw sticks
    state.rc_valid = true;
    state.rc_pitch = 1.0f;
    state.rc_roll = -1.0f;
    state.rc_yaw = 0.8f;
    state.rc_throttle = 1.0f;

    ControlOutput out2 = brake.update(state);

    // Outputs should be identical because pilot sticks are ignored in BRAKE mode to pause drone
    EXPECT_NEAR(out1.roll_cmd, out2.roll_cmd, 1e-4f);
    EXPECT_NEAR(out1.pitch_cmd, out2.pitch_cmd, 1e-4f);
    EXPECT_NEAR(out1.thrust, out2.thrust, 1e-4f);
}

TEST(SprayerTest, ChannelMappingAndPwm) {
    apx::gcs::ParamManager::get_instance().init_defaults();
    auto& sprayer = Sprayer::instance();
    sprayer.load_params();

    // Pump on Servo 9 (0-indexed 8), Spinner on Servo 10 (0-indexed 9)
    EXPECT_EQ(sprayer.pump_channel(), 8);
    EXPECT_EQ(sprayer.spinner_channel(), 9);

    // Default off PWM is 1100 us
    EXPECT_EQ(sprayer.pump_pwm(), 1100);
    EXPECT_FALSE(sprayer.spraying());

    // Manual PWM control
    sprayer.set_pump_pwm_manual(1900);
    EXPECT_EQ(sprayer.pump_pwm(), 1900);
    EXPECT_TRUE(sprayer.spraying());
    EXPECT_FLOAT_EQ(sprayer.pump_fraction(), 1.0f);

    sprayer.set_pump_pwm_manual(1500);
    EXPECT_EQ(sprayer.pump_pwm(), 1500);
    EXPECT_TRUE(sprayer.spraying());
    EXPECT_NEAR(sprayer.pump_fraction(), 0.5f, 0.01f);

    sprayer.set_pump_pwm_manual(1100);
    EXPECT_EQ(sprayer.pump_pwm(), 1100);
    EXPECT_FALSE(sprayer.spraying());
    EXPECT_FLOAT_EQ(sprayer.pump_fraction(), 0.0f);

    // Set flow rate fraction
    sprayer.set_flow_rate(1.0f);
    EXPECT_EQ(sprayer.pump_pwm(), 1900);
    EXPECT_TRUE(sprayer.spraying());

    sprayer.set_flow_rate(0.0f);
    EXPECT_EQ(sprayer.pump_pwm(), 1100);
    EXPECT_FALSE(sprayer.spraying());
}

TEST(SprayerTest, ManualControlPersistsWhileDisarmed) {
    auto& sprayer = Sprayer::instance();
    sprayer.load_params();

    VehicleState state;
    state.armed = false;
    state.dt_s = 0.0025f;

    // Verify initial update while disarmed does not crash or enable sprayer
    sprayer.update(state);
    EXPECT_EQ(sprayer.pump_pwm(), 1100);
    EXPECT_FALSE(sprayer.spraying());

    // Manual DO_SET_SERVO command on bench (disarmed)
    sprayer.set_pump_pwm_manual(1900);
    EXPECT_EQ(sprayer.pump_pwm(), 1900);
    EXPECT_TRUE(sprayer.spraying());

    // Multiple control loop updates while still disarmed must NOT wipe manual PWM
    for (int i = 0; i < 20; ++i) {
        sprayer.update(state);
    }
    EXPECT_EQ(sprayer.pump_pwm(), 1900);
    EXPECT_TRUE(sprayer.spraying());

    // Can manually turn off
    sprayer.set_pump_pwm_manual(1100);
    EXPECT_EQ(sprayer.pump_pwm(), 1100);
    EXPECT_FALSE(sprayer.spraying());
}

TEST(SprayerTest, DisarmTransitionEdgeShutsOffSprayer) {
    auto& sprayer = Sprayer::instance();
    sprayer.load_params();

    VehicleState state;
    state.armed = true;
    state.dt_s = 0.0025f;

    sprayer.update(state);
    sprayer.set_pump_pwm_manual(1900);
    sprayer.set_spinner_pwm_manual(1800);
    EXPECT_EQ(sprayer.pump_pwm(), 1900);
    EXPECT_EQ(sprayer.spinner_pwm(), 1800);
    EXPECT_TRUE(sprayer.spraying());

    // Now transition to disarmed (e.g. landed / emergency stop)
    state.armed = false;
    sprayer.update(state);

    EXPECT_EQ(sprayer.pump_pwm(), 1100);
    EXPECT_EQ(sprayer.spinner_pwm(), 1100);
    EXPECT_FALSE(sprayer.spraying());
    EXPECT_FALSE(sprayer.manual_control());
}

TEST(SprayerTest, SpinnerManualControl) {
    auto& sprayer = Sprayer::instance();
    sprayer.load_params();

    EXPECT_EQ(sprayer.spinner_pwm(), 1100);
    sprayer.set_spinner_pwm_manual(1750);
    EXPECT_EQ(sprayer.spinner_pwm(), 1750);

    // Resetting manual control
    sprayer.clear_manual_control();
    EXPECT_EQ(sprayer.spinner_pwm(), 1100);
}

TEST(ModeAutoTest, DoSetServoWaypointControlsSprayer) {
    auto& sprayer = Sprayer::instance();
    sprayer.load_params();

    ModeAuto auto_mode;
    VehicleState state;
    state.armed = true;
    state.dt_s = 0.0025f;
    state.pos_ned[2] = -10.0f; // 10m altitude

    // Waypoint 0: DO_SET_SERVO ch 9 pwm 1900
    // Waypoint 1: NAV_WAYPOINT 10m away
    Waypoint wp0;
    wp0.seq = 1;
    wp0.command = ModeAuto::kCmdDoSetServo;
    wp0.param1 = 9.0f;
    wp0.param2 = 1900.0f;

    Waypoint wp1;
    wp1.seq = 2;
    wp1.command = ModeAuto::kCmdNavWaypoint;
    wp1.pos_ned[0] = 10.0f;
    wp1.pos_ned[1] = 0.0f;
    wp1.pos_ned[2] = -10.0f;
    wp1.accept_radius_m = 1.0f;

    auto_mode.load_mission({wp0, wp1});
    auto_mode.on_enter(state);
    auto_mode.update(state);

    EXPECT_EQ(sprayer.pump_pwm(), 1900);
    EXPECT_TRUE(sprayer.spraying());

    // Next item turns it off
    Waypoint wp_off;
    wp_off.seq = 3;
    wp_off.command = ModeAuto::kCmdDoSetServo;
    wp_off.param1 = 9.0f;
    wp_off.param2 = 1100.0f;

    auto_mode.load_mission({wp_off, wp1});
    auto_mode.on_enter(state);
    auto_mode.update(state);

    EXPECT_EQ(sprayer.pump_pwm(), 1100);
    EXPECT_FALSE(sprayer.spraying());
}
