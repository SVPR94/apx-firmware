// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "mode_althold.hpp"
#include "navigation_cascade.hpp"
#include "param_manager.hpp"
#include <cstdio>
#include <cmath>

namespace apx::modes {

namespace {
constexpr float kDegToRad = 0.017453293f;
}

ModeAltHold::ModeAltHold() {
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
    m_att_ctrl.set_rate_limits(
        params.get_float_or("ATC_RATE_R_MAX", 90.0f) * kDegToRad,
        params.get_float_or("ATC_RATE_Y_MAX", 45.0f) * kDegToRad);
    m_att_ctrl.set_accel_limits(
        params.get_float_or("ATC_ACCEL_R_MAX", 300.0f) * kDegToRad,
        params.get_float_or("ATC_ACCEL_Y_MAX", 150.0f) * kDegToRad);

    m_pos_ctrl.set_altitude_gain(params.get_float_or("PSC_POSZ_P", 1.0f));
    m_pos_ctrl.set_climb_rate_gains(
        params.get_float_or("PSC_VELZ_P", 1.5f),
        params.get_float_or("PSC_VELZ_I", 0.2f),
        params.get_float_or("PSC_VELZ_D", 0.05f));
    m_pos_ctrl.set_hover_thrust(params.get_float_or("MOT_THST_HOVER", 0.5f));

    m_max_tilt_rad = static_cast<float>(params.get_int_or("ANGLE_MAX", 2500)) * 0.01f * kDegToRad;
    m_max_yaw_rate_rad = params.get_float_or("ATC_RATE_Y_MAX", 45.0f) * kDegToRad;
    m_climb_speed_ms = params.get_float_or("PILOT_SPEED_UP", 250.0f) / 100.0f;
    m_desc_speed_ms = params.get_float_or("PILOT_SPEED_DN", 150.0f) / 100.0f;
    // See PositionController::update_altitude()'s comment / NavigationCascade's
    // matching wiring - without this, a climb command issued while still
    // on the ground built an unbounded velocity-setpoint error that
    // dumped as a thrust kick the instant the vehicle broke ground.
    m_pos_ctrl.set_max_vertical_speed(m_climb_speed_ms, m_desc_speed_ms);
}

void ModeAltHold::on_enter(const VehicleState& state) {
    m_target_rel_alt_m = -state.pos_ned[2];
    m_hold_yaw = state.yaw;
    m_rate_ctrl.reset();
    m_att_ctrl.reset();
    m_pos_ctrl.reset();
    std::printf("[APX Mode] Entering ALT_HOLD (hold %.2fm)\n", (double)m_target_rel_alt_m);
}

void ModeAltHold::on_exit() {
    m_rate_ctrl.reset();
    m_att_ctrl.reset();
    m_pos_ctrl.reset();
}

ControlOutput ModeAltHold::update(const VehicleState& state) {
    ControlOutput out;
    if (!state.armed) {
        m_rate_ctrl.reset();
        m_att_ctrl.reset();
        m_pos_ctrl.reset();
        m_target_rel_alt_m = -state.pos_ned[2];
        return out;
    }

    // Throttle stick deflection outside deadband [0.42, 0.58] modifies
    // target altitude - speeds from PILOT_SPEED_UP/DN.
    if (state.rc_throttle > 0.58f) {
        float climb_speed = ((state.rc_throttle - 0.58f) / 0.42f) * m_climb_speed_ms;
        m_target_rel_alt_m += climb_speed * state.dt_s;
    } else if (state.rc_throttle < 0.42f) {
        float desc_speed = ((0.42f - state.rc_throttle) / 0.42f) * m_desc_speed_ms;
        m_target_rel_alt_m -= desc_speed * state.dt_s;
        if (m_target_rel_alt_m < 0.0f) m_target_rel_alt_m = 0.0f;
    }

    // Yaw stick rotates held heading - rate capped at m_att_ctrl's actual
    // max yaw rate (ATC_RATE_Y_MAX), same "target outruns what the
    // vehicle can physically achieve" fix as mode_loiter.hpp/
    // mode_stabilize.cpp.
    if (std::abs(state.rc_yaw) > 0.05f) {
        m_hold_yaw += state.rc_yaw * m_max_yaw_rate_rad * state.dt_s;
        while (m_hold_yaw > 3.14159265f)  m_hold_yaw -= 6.2831853f;
        while (m_hold_yaw < -3.14159265f) m_hold_yaw += 6.2831853f;
    }

    float current_rel_alt = -state.pos_ned[2];
    float climb_rate = -state.vel_ned[2];
    m_pos_ctrl.update_altitude(m_target_rel_alt_m, current_rel_alt, 0.0f, climb_rate, state.dt_s, out.thrust);

    // Roll and pitch sticks command lean angles, capped at ANGLE_MAX.
    float desired_roll = state.rc_roll * m_max_tilt_rad;
    float desired_pitch = state.rc_pitch * m_max_tilt_rad;

    float q_desired[4];
    euler_to_quat(desired_roll, desired_pitch, m_hold_yaw, q_desired);
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
