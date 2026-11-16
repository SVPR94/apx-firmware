// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "mode_stabilize.hpp"
#include "navigation_cascade.hpp"
#include "param_manager.hpp"
#include <cstdio>
#include <cmath>
#include <algorithm>

namespace apx::modes {

namespace {
constexpr float kIdleThrottleWhenArmed = 0.08f;
constexpr float kDegToRad = 0.017453293f;
}

ModeStabilize::ModeStabilize() {
    auto& params = apx::gcs::ParamManager::get_instance();

    m_rate_ctrl.set_gains(0,
        params.get_float_or("ATC_RAT_RLL_P", 0.135f),
        params.get_float_or("ATC_RAT_RLL_I", 0.135f),
        params.get_float_or("ATC_RAT_RLL_D", 0.0036f), 0.0f);
    m_rate_ctrl.set_gains(1,
        params.get_float_or("ATC_RAT_PIT_P", 0.135f),
        params.get_float_or("ATC_RAT_PIT_I", 0.135f),
        params.get_float_or("ATC_RAT_PIT_D", 0.0036f), 0.0f);
    m_rate_ctrl.set_gains(2,
        params.get_float_or("ATC_RAT_YAW_P", 0.600f),
        params.get_float_or("ATC_RAT_YAW_I", 0.060f),
        params.get_float_or("ATC_RAT_YAW_D", 0.0f), 0.0f);

    m_att_ctrl.set_proportional_gain(
        params.get_float_or("ATC_ANG_RLL_P", 4.5f),
        params.get_float_or("ATC_ANG_PIT_P", 4.5f),
        params.get_float_or("ATC_ANG_YAW_P", 4.5f));
    // AttitudeController shares one roll/pitch rate ceiling - see
    // navigation_cascade.cpp's constructor for the same ATC_RATE_R_MAX vs
    // P_MAX mismatch note.
    m_att_ctrl.set_rate_limits(
        params.get_float_or("ATC_RATE_R_MAX", 90.0f) * kDegToRad,
        params.get_float_or("ATC_RATE_Y_MAX", 45.0f) * kDegToRad);
    m_att_ctrl.set_accel_limits(
        params.get_float_or("ATC_ACCEL_R_MAX", 300.0f) * kDegToRad,
        params.get_float_or("ATC_ACCEL_Y_MAX", 150.0f) * kDegToRad);

    // Pilot max lean angle - ANGLE_MAX (centidegrees)
    // parameter for manual-lean modes.
    m_max_tilt_rad = static_cast<float>(params.get_int_or("ANGLE_MAX", 2500)) * 0.01f * kDegToRad;
    // Yaw-stick heading-target ramp rate - capped at (not above)
    // m_att_ctrl's own real max yaw rate above, same reasoning as
    // mode_loiter.hpp's kMaxYawRateRads fix: a target ramping faster than
    // the controller can physically achieve just runs the heading target
    // further and further ahead of the real, physical yaw for as long as
    // the stick is held, with no bound.
    m_max_yaw_rate_rad = params.get_float_or("ATC_RATE_Y_MAX", 45.0f) * kDegToRad;
}

void ModeStabilize::on_enter(const VehicleState& state) {
    m_target_yaw = state.yaw;
    m_rate_ctrl.reset();
    m_att_ctrl.reset();
    std::printf("[APX Mode] Entering STABILIZE\n");
}

void ModeStabilize::on_exit() {
    m_rate_ctrl.reset();
    m_att_ctrl.reset();
}

ControlOutput ModeStabilize::update(const VehicleState& state) {
    ControlOutput out;
    if (!state.armed) {
        m_rate_ctrl.reset();
        m_att_ctrl.reset();
        m_target_yaw = state.yaw;
        return out;
    }

    // Collective thrust from throttle stick, with idle floor when armed
    float stick_thr = state.rc_throttle;
    if (stick_thr < 0.05f) {
        out.thrust = kIdleThrottleWhenArmed;
    } else {
        out.thrust = kIdleThrottleWhenArmed + stick_thr * (0.90f - kIdleThrottleWhenArmed);
    }

    // Yaw stick slews heading target
    if (std::abs(state.rc_yaw) > 0.05f) {
        m_target_yaw += state.rc_yaw * m_max_yaw_rate_rad * state.dt_s;
        while (m_target_yaw > 3.14159265f)  m_target_yaw -= 6.2831853f;
        while (m_target_yaw < -3.14159265f) m_target_yaw += 6.2831853f;
    }

    // Roll and pitch stick deflection -> attitude target
    float desired_roll = state.rc_roll * m_max_tilt_rad;
    float desired_pitch = state.rc_pitch * m_max_tilt_rad;

    float q_desired[4];
    euler_to_quat(desired_roll, desired_pitch, m_target_yaw, q_desired);

    float desired_rate[3];
    m_att_ctrl.update(q_desired, state.quat, state.dt_s, desired_rate);

    float rate_out[3];
    m_rate_ctrl.update(desired_rate, state.gyro_body, state.dt_s, rate_out);

    out.roll_cmd = rate_out[0];
    out.pitch_cmd = rate_out[1];
    out.yaw_cmd = rate_out[2];
    return out;
}

} // namespace apx::modes
