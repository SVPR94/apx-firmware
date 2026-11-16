// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "navigation_cascade.hpp"
#include "param_manager.hpp"
#include "radar_manager.hpp"
#include <cmath>
#include <cstdio>

namespace apx::modes {

namespace {
constexpr float kGravityMs2 = 9.81f;
constexpr float kDegToRad = 0.017453293f;

float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
} // namespace

void euler_to_quat(float roll, float pitch, float yaw, float q[4]) {
    float cr = std::cos(roll * 0.5f), sr = std::sin(roll * 0.5f);
    float cp = std::cos(pitch * 0.5f), sp = std::sin(pitch * 0.5f);
    float cy = std::cos(yaw * 0.5f), sy = std::sin(yaw * 0.5f);
    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
}

NavigationCascade::NavigationCascade() {
    // All gains/limits below are read from ParamManager (registered in
    // param_manager.cpp's init_defaults(), which documents where each default
    // came from and, for ANGLE_MAX in particular, why it is bounded for stability).
    // Falls back to the same values if a param somehow isn't registered yet.
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
    // set_rate_limits() takes one roll/pitch ceiling, not separate ones -
    // ATC_RATE_P_MAX exists as its own param for configuration symmetry,
    // but only R_MAX is actually applied to both axes here.
    // Flag a mismatch rather than silently ignoring it.
    float rate_r = params.get_float_or("ATC_RATE_R_MAX", 90.0f);
    float rate_p = params.get_float_or("ATC_RATE_P_MAX", 90.0f);
    if (std::fabs(rate_r - rate_p) > 0.5f) {
        std::printf("[APX Params] WARNING: ATC_RATE_R_MAX (%.1f) != ATC_RATE_P_MAX (%.1f) - "
                   "only R_MAX is applied (AttitudeController shares one roll/pitch rate limit)\n",
                   (double)rate_r, (double)rate_p);
    }
    m_att_ctrl.set_accel_limits(
        params.get_float_or("ATC_ACCEL_R_MAX", 300.0f) * kDegToRad,
        params.get_float_or("ATC_ACCEL_Y_MAX", 150.0f) * kDegToRad);

    m_pos_ctrl.set_position_gain(params.get_float_or("PSC_POSXY_P", 1.0f));
    m_pos_ctrl.set_horizontal_velocity_gains(
        params.get_float_or("PSC_VELXY_P", 2.0f),
        params.get_float_or("PSC_VELXY_I", 1.0f),
        params.get_float_or("PSC_VELXY_D", 0.5f));
    m_pos_ctrl.set_altitude_gain(params.get_float_or("PSC_POSZ_P", 1.0f));
    m_pos_ctrl.set_climb_rate_gains(
        params.get_float_or("PSC_VELZ_P", 1.5f),
        params.get_float_or("PSC_VELZ_I", 0.2f),
        params.get_float_or("PSC_VELZ_D", 0.05f));
    // Same PILOT_SPEED_UP/DN params ModeLoiter/ModeAltHold already use to
    // cap their OWN target-altitude ramp - this caps the *velocity
    // controller's* demand too (see PositionController::update_altitude()'s
    // comment for why the cascade needs this independently of whatever
    // rate a caller ramps its target at).
    m_pos_ctrl.set_max_vertical_speed(
        params.get_float_or("PILOT_SPEED_UP", 250.0f) / 100.0f,
        params.get_float_or("PILOT_SPEED_DN", 150.0f) / 100.0f);
    m_pos_ctrl.set_hover_thrust(params.get_float_or("MOT_THST_HOVER", 0.5f));

    float wpnav_speed_cms = params.get_float_or("WPNAV_SPEED", 500.0f);
    m_pos_ctrl.set_max_horizontal_speed(wpnav_speed_cms / 100.0f);
    // Also the reference-trajectory slew rate below (see compute()) - the
    // same speed governs both "how fast the velocity PID is allowed to
    // ask to move" and "how fast the target it's chasing is allowed to
    // move", which is what keeps them from ever diverging from each
    // other (see mode_loiter.hpp's kMaxHorizSpeedMs comment for the bug
    // this exact mismatch caused when the two were independent values).
    m_target_slew_rate_ms = wpnav_speed_cms / 100.0f;

    m_max_accel_ms2 = params.get_float_or("WPNAV_ACCEL", 300.0f) / 100.0f;

    // ANGLE_MAX is centidegrees (standard unit for this param).
    // See param_manager.cpp's registration comment for why APX's default
    // (2500 = 25deg) is chosen: this airframe's rate-loop gains were verified
    // stable up to this ceiling (avoiding motor-saturation nonlinearity at large
    // simultaneous roll+pitch) - raise it only after re-verifying stability.
    m_max_tilt_rad = static_cast<float>(params.get_int_or("ANGLE_MAX", 2500)) * 0.01f * kDegToRad;
    // Fail-safe ceiling, distinctly above the normal operating limit above
    // (not itself a parameter - it's an internal backstop, not a tuning
    // knob): if actual tilt is already this far past ANGLE_MAX, stop
    // asking for more lean and let self-leveling recover instead of
    // compounding it, providing proactive angle-limit protection. Scales with
    // ANGLE_MAX so it stays meaningful if that's changed.
    m_safety_tilt_rad = m_max_tilt_rad + (15.0f * kDegToRad);
}

ControlOutput NavigationCascade::compute(const float target_pos_ned[3], bool has_yaw, float target_yaw,
                                         const VehicleState& state,
                                         bool use_forced_climb_rate, float forced_climb_rate_ms,
                                         const float vel_ff_ne[2], const float accel_ff_ne[2]) {
    ControlOutput out;

    if (!m_smoothed_target_init || vel_ff_ne) {
        m_smoothed_target_ne[0] = vel_ff_ne ? target_pos_ned[0] : state.pos_ned[0];
        m_smoothed_target_ne[1] = vel_ff_ne ? target_pos_ned[1] : state.pos_ned[1];
        m_smoothed_target_init = true;
    }
    float max_step = m_target_slew_rate_ms * state.dt_s;
    m_smoothed_target_ne[0] += clampf(target_pos_ned[0] - m_smoothed_target_ne[0], -max_step, max_step);
    m_smoothed_target_ne[1] += clampf(target_pos_ned[1] - m_smoothed_target_ne[1], -max_step, max_step);

    // --- Horizontal: position error -> north/east accel demand ---
    float pos_actual_ne[2] = {state.pos_ned[0], state.pos_ned[1]};
    // Real feedforward when the caller has a planned trajectory velocity
    // (ModeAuto via PathTracker) instead of always zero - see this
    // function's header comment and PathTrackerOutput::vel_ff_ne.
    float vel_target_ne[2] = {vel_ff_ne ? vel_ff_ne[0] : 0.0f, vel_ff_ne ? vel_ff_ne[1] : 0.0f};
    float vel_actual_ne[2] = {state.vel_ned[0], state.vel_ned[1]};
    float accel_ne[2] = {0.0f, 0.0f};
    m_pos_ctrl.update_horizontal(m_smoothed_target_ne, pos_actual_ne, vel_target_ne, vel_actual_ne, state.dt_s, accel_ne);
    if (accel_ff_ne) {
        accel_ne[0] += accel_ff_ne[0];
        accel_ne[1] += accel_ff_ne[1];
    }
    // Limit the accel vector's magnitude, not each axis separately: per-axis
    // clamping rotates the demand toward 45deg, which bends diagonal legs.
    float accel_mag = std::sqrt(accel_ne[0] * accel_ne[0] + accel_ne[1] * accel_ne[1]);
    if (accel_mag > m_max_accel_ms2) {
        float scale = m_max_accel_ms2 / accel_mag;
        accel_ne[0] *= scale;
        accel_ne[1] *= scale;
    }

    float yaw_target = has_yaw ? target_yaw : state.yaw;

    // Rotate the world-frame N/E accel demand into forward/right in the
    // frame of yaw_target - the same yaw q_desired is built with below.
    // AttitudeController corrects tilt (swing) and heading (twist)
    // separately and drives the thrust axis onto q_desired's thrust axis
    // even while heading still lags, so the world thrust direction is
    // Rz(yaw_target) * lean. Using the measured yaw here rotated the
    // thrust vector by the yaw tracking error: accelerating while the
    // heading slewed pushed the vehicle sideways, and the yaw overshoot
    // that follows a heading change showed up as a sine-like cross-track
    // weave on long AUTO legs in Gazebo. (Measured yaw was chosen when the
    // attitude controller still mixed world/body error frames - see
    // attitude_controller.cpp - which is no longer the case.)
    // Sign convention: FRD body, thrust along body -Z.
    float accel_fwd = accel_ne[0] * std::cos(yaw_target) + accel_ne[1] * std::sin(yaw_target);
    float accel_right = -accel_ne[0] * std::sin(yaw_target) + accel_ne[1] * std::cos(yaw_target);
    float pitch_target = -accel_fwd / kGravityMs2;
    float roll_target = accel_right / kGravityMs2;
    // Limit the total lean, not each axis: per-axis clamping allowed
    // ~1.4x ANGLE_MAX on a diagonal demand (32deg seen with ANGLE_MAX 25).
    float lean = std::sqrt(pitch_target * pitch_target + roll_target * roll_target);
    if (lean > m_max_tilt_rad) {
        float scale = m_max_tilt_rad / lean;
        pitch_target *= scale;
        roll_target *= scale;
    }

    // Fail-safe: already tilted more than the safety limit (whatever the
    // cause) - stop commanding more lean and let self-leveling recover
    // instead of compounding it. See m_safety_tilt_rad's setup above.
    if (std::fabs(state.roll) > m_safety_tilt_rad || std::fabs(state.pitch) > m_safety_tilt_rad) {
        pitch_target = 0.0f;
        roll_target = 0.0f;
    }

    // --- Vertical: altitude/velocity error -> thrust ---
    float current_rel_alt = -state.pos_ned[2];
    float climb_rate = -state.vel_ned[2];

    // Terrain-following: adjust vertical target setpoint if terrain following active
    const float eff_target_ned_z = apx::radar::RadarManager::get_instance().adjust_target_altitude(
        target_pos_ned[2], state.pos_ned[2]);
    const float target_alt = -eff_target_ned_z;

    if (use_forced_climb_rate) {
        m_pos_ctrl.update_altitude(current_rel_alt, current_rel_alt, forced_climb_rate_ms, climb_rate,
                                   state.dt_s, out.thrust);
    } else {
        m_pos_ctrl.update_altitude(target_alt, current_rel_alt, 0.0f, climb_rate, state.dt_s, out.thrust);
    }

    // --- Attitude: roll/pitch/yaw target -> desired body rate -> mixer cmd ---
    float q_desired[4];
    euler_to_quat(roll_target, pitch_target, yaw_target, q_desired);
    float desired_rate[3];
    m_att_ctrl.update(q_desired, state.quat, state.dt_s, desired_rate);
    float rate_out[3];
    m_rate_ctrl.update(desired_rate, state.gyro_body, state.dt_s, rate_out);

    out.roll_cmd = rate_out[0];
    out.pitch_cmd = rate_out[1];
    out.yaw_cmd = rate_out[2];
    return out;
}

void NavigationCascade::reset() {
    m_pos_ctrl.reset();
    m_att_ctrl.reset();
    m_rate_ctrl.reset();
    m_smoothed_target_init = false;
}

} // namespace apx::modes
