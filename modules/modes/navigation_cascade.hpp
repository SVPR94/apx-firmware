// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include "position_controller/position_controller.hpp"
#include "attitude_controller/attitude_controller.hpp"
#include "rate_controller/rate_controller.hpp"
#include "vehicle_state.hpp"

namespace apx::modes {

// Standard aerospace ZYX (yaw-pitch-roll intrinsic) euler -> quaternion
// (w,x,y,z). Matches the convention apx_gazebo_plugin.cpp uses for
// state.quat. Shared so any mode building its own attitude target (not
// just NavigationCascade) uses the same convention.
void euler_to_quat(float roll, float pitch, float yaw, float q[4]);

// Shared Position -> Attitude -> Rate -> mixer-command cascade used by every
// mode that needs to hold or navigate to a 3D point (ModeGuided,
// ModePosHold, ModeLoiter, ModeLand). This is the one place that pipeline
// lives - a new position-aware mode only has to decide *what* target to
// feed in each tick (see mode_manager.hpp/.cpp for how modes themselves
// are registered/switched), not re-derive how a target becomes a mixer
// command.
class NavigationCascade {
public:
    NavigationCascade();

    // target_pos_ned: north/east/down target (m), relative to home.
    // has_yaw/target_yaw: if false, yaw is left uncontrolled (yaw_cmd=0),
    // holding whatever heading the vehicle already has.
    // use_forced_climb_rate/forced_climb_rate_ms: when set, replaces the
    // normal "hold target_pos_ned[2]" altitude behavior with a plain climb-
    // rate command - used by ModeLand for a fixed descent rate instead of
    // holding an altitude.
    // vel_ff_ne: optional north/east feedforward velocity (m/s) - the
    // speed the caller's own trajectory plan already intends to be moving
    // at this tick (e.g. PathTracker's along-track planned speed),
    // separate from whatever velocity the position-error P term alone
    // would imply. nullptr (the default) means "no feedforward" - every
    // caller except ModeAuto holds a fixed point and has no planned
    // velocity to offer, so they're unaffected by this parameter existing.
    // accel_ff_ne: optional north/east feedforward acceleration (m/s^2)
    // from the same trajectory plan (e.g. centripetal accel through a
    // corner arc). When vel_ff_ne is given the target is treated as an
    // already-shaped trajectory and is not slew-limited again.
    ControlOutput compute(const float target_pos_ned[3], bool has_yaw, float target_yaw,
                          const VehicleState& state,
                          bool use_forced_climb_rate = false, float forced_climb_rate_ms = 0.0f,
                          const float vel_ff_ne[2] = nullptr,
                          const float accel_ff_ne[2] = nullptr);

    void reset();

    // Overrides for modes with their own limits (ModeAuto: WPNAV_SPEED_UP/DN
    // and a lean-angle based accel limit so the controller keeps authority
    // beyond what the trajectory itself plans to use).
    void set_max_horizontal_accel(float accel_ms2) { m_max_accel_ms2 = accel_ms2; }
    void set_max_horizontal_speed(float speed_ms) { m_pos_ctrl.set_max_horizontal_speed(speed_ms); }
    void set_max_vertical_speed(float up_ms, float down_ms) { m_pos_ctrl.set_max_vertical_speed(up_ms, down_ms); }
    float get_max_tilt_rad() const { return m_max_tilt_rad; }

private:
    control::PositionController m_pos_ctrl;
    control::AttitudeController m_att_ctrl;
    control::RateController m_rate_ctrl;

    // A target handed to compute() is used as a reference *trajectory*,
    // not a step input: this tracks a smoothed target that moves toward
    // the real one at a bounded rate (see compute()). A raw step input -
    // even a small one, even with heavily eased gains - reliably diverged
    // into a flip in live SITL testing; slewing the reference instead of
    // the vehicle's response to it is what actually fixed that, not
    // further gain reduction.
    float m_smoothed_target_ne[2]{0, 0};
    bool m_smoothed_target_init{false};

    // Read from ParamManager at construction time (ANGLE_MAX,
    // WPNAV_ACCEL, WPNAV_SPEED, plus a safety-tilt margin derived from
    // ANGLE_MAX) - see navigation_cascade.cpp's constructor. A fresh
    // NavigationCascade is created every time a mode using it is
    // (re-)entered (see mode_manager.cpp's create_mode()), so a parameter
    // change takes effect on the next mode entry, not instantaneously
    // mid-flight - deliberate, to avoid a live gain/limit step under an
    // active controller.
    float m_max_tilt_rad{0.436f};
    float m_max_accel_ms2{3.0f};
    float m_target_slew_rate_ms{2.0f};
    float m_safety_tilt_rad{0.698f};
    float m_filtered_pitch_target{0.0f};
    float m_filtered_roll_target{0.0f};
    float m_max_tilt_slew_rads{0.785f}; // 45 deg/s max tilt slew rate to prevent abrupt braking tilt steps
};

} // namespace apx::modes
