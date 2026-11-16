// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

namespace apx::modes {

// Snapshot of everything a FlightMode needs to make a decision each tick.
// Position/velocity are NED, relative to the home position established at
// boot (see apx_main.cpp --home-lat/--home-lon/--home-alt). Attitude is
// both euler (for logging/simple checks) and quaternion (for the
// attitude controller, which needs it to avoid gimbal-lock issues that
// naive euler-error math has).
struct VehicleState {
    bool armed{false};
    float dt_s{0.01f};

    float pos_ned[3]{0, 0, 0};   // north, east, down (m)
    float vel_ned[3]{0, 0, 0};   // m/s
    float home_ned[3]{0, 0, 0};  // home (set on arming/GCS), same frame as pos_ned

    float roll{0}, pitch{0}, yaw{0}; // rad
    float quat[4]{1, 0, 0, 0};       // w,x,y,z
    float gyro_body[3]{0, 0, 0};     // rad/s

    // Normalized pilot/GCS stick inputs:
    // roll, pitch, yaw: -1.0 .. +1.0
    // throttle: 0.0 .. 1.0 (0.5 = hover center / neutral)
    float rc_roll{0.0f};
    float rc_pitch{0.0f};
    float rc_yaw{0.0f};
    float rc_throttle{0.5f};
    bool  rc_valid{false};
};

// What a FlightMode produces each tick. thrust/roll_cmd/pitch_cmd/yaw_cmd
// feed straight into MotorMixer::mix() (same 0..1 / -1..1 convention it
// already used for the thrust-only case). request_disarm is how the land
// detector in ModeLand tells the caller to cut power once touchdown is
// confirmed - see mode_manager.cpp.
struct ControlOutput {
    float thrust{0.0f};     // 0..1
    float roll_cmd{0.0f};   // -1..1
    float pitch_cmd{0.0f};  // -1..1
    float yaw_cmd{0.0f};    // -1..1
    bool request_disarm{false};
    // Set by ModeManager's CrashDetector (see crash_detector.hpp), not by
    // any individual mode - distinct from an ordinary request_disarm (e.g.
    // LAND's normal touchdown) so the caller can log/alert differently.
    bool crash_detected{false};
};

// A GCS-commanded "go here" target (SET_POSITION_TARGET_LOCAL_NED/
// GLOBAL_INT, or MAV_CMD_DO_REPOSITION), already resolved to NED-relative-
// to-home by the caller so modes never need to know about MAVLink frames.
struct GuidedTarget {
    float pos_ned[3]{0, 0, 0};
    bool has_yaw{false};
    float yaw{0};
};

} // namespace apx::modes
