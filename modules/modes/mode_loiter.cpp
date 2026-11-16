// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "mode_loiter.hpp"
#include "param_manager.hpp"
#include <cmath>
#include <cstdio>
#include <algorithm>

namespace apx::modes {

namespace {
constexpr float kDegToRad = 0.017453293f;
}

ModeLoiter::ModeLoiter() {
    auto& params = apx::gcs::ParamManager::get_instance();

    // Clamped to WPNAV_SPEED (the same param NavigationCascade's own
    // internal target-slew limiter reads - see navigation_cascade.cpp) so
    // this can never outrun what the cascade can actually track,
    // regardless of how LOIT_SPEED itself is tuned - see the comment on
    // m_max_horiz_speed_ms in mode_loiter.hpp for why that invariant
    // matters.
    float loit_speed_ms = params.get_float_or("LOIT_SPEED", 180.0f) / 100.0f;
    float wpnav_speed_ms = params.get_float_or("WPNAV_SPEED", 500.0f) / 100.0f;
    m_max_horiz_speed_ms = std::min(loit_speed_ms, wpnav_speed_ms);

    m_max_climb_speed_ms = params.get_float_or("PILOT_SPEED_UP", 250.0f) / 100.0f;
    m_max_desc_speed_ms = params.get_float_or("PILOT_SPEED_DN", 150.0f) / 100.0f;
    m_max_yaw_rate_rads = params.get_float_or("ATC_RATE_Y_MAX", 45.0f) * kDegToRad;
    m_disarm_delay_s = params.get_float_or("DISARM_DELAY", 30.0f);
}

void ModeLoiter::on_enter(const VehicleState& state) {
    m_target_pos_ned[0] = state.pos_ned[0];
    m_target_pos_ned[1] = state.pos_ned[1];
    m_target_pos_ned[2] = state.pos_ned[2];
    m_target_yaw = state.yaw;
    m_cascade.reset();
    m_land_detector.reset();
    // Entered while disarmed: on the ground. Entered while armed (mode
    // switch in flight): assume flying until LandDetector says otherwise.
    m_on_ground = !state.armed;
    m_disarm_timer_s = 0.0f;
    std::printf("[APX Mode] Entering LOITER (lock at N=%.2f, E=%.2f, Alt=%.2fm, Yaw=%.1fdeg)\n",
                (double)m_target_pos_ned[0], (double)m_target_pos_ned[1],
                -(double)m_target_pos_ned[2], (double)(m_target_yaw * 57.2958f));
}

void ModeLoiter::on_exit() {
    m_cascade.reset();
}

ControlOutput ModeLoiter::update(const VehicleState& state) {
    if (!state.armed) {
        m_cascade.reset();
        m_land_detector.reset();
        m_on_ground = true;
        m_disarm_timer_s = 0.0f;
        m_target_pos_ned[0] = state.pos_ned[0];
        m_target_pos_ned[1] = state.pos_ned[1];
        m_target_pos_ned[2] = state.pos_ned[2];
        m_target_yaw = state.yaw;
        return ControlOutput{};
    }

    float dt = std::clamp(state.dt_s, 0.001f, 0.05f);

    if (m_on_ground) {
        if (state.rc_valid && state.rc_throttle > kThrottleClimb) {
            // Pilot commands a climb: take off from wherever it sits.
            m_on_ground = false;
            m_land_detector.reset();
            m_disarm_timer_s = 0.0f;
            std::printf("[APX Mode] LOITER: takeoff\n");
        } else {
            // Idle on the ground (apx_main.cpp applies MOT_SPIN_ARM), with
            // the controllers held reset so nothing winds up meanwhile.
            m_cascade.reset();
            m_target_pos_ned[0] = state.pos_ned[0];
            m_target_pos_ned[1] = state.pos_ned[1];
            m_target_pos_ned[2] = state.pos_ned[2];
            m_target_yaw = state.yaw;
            ControlOutput idle;
            const bool throttle_zero = !state.rc_valid || state.rc_throttle < kThrottleZero;
            m_disarm_timer_s = throttle_zero ? m_disarm_timer_s + dt : 0.0f;
            if (m_disarm_delay_s > 0.0f && m_disarm_timer_s >= m_disarm_delay_s) {
                std::printf("[APX Mode] LOITER: landed with throttle at zero for %.0fs - disarming\n",
                           (double)m_disarm_delay_s);
                idle.request_disarm = true;
            }
            return idle;
        }
    }

    // 1. Horizontal Stick Deflection (Roll/Pitch) -> Slewed Target Position
    float stick_roll = state.rc_roll;
    float stick_pitch = state.rc_pitch;

    float v_fwd = 0.0f;
    float v_right = 0.0f;

    if (std::abs(stick_pitch) > kStickDeadband) {
        v_fwd = stick_pitch * m_max_horiz_speed_ms;
    }
    if (std::abs(stick_roll) > kStickDeadband) {
        v_right = stick_roll * m_max_horiz_speed_ms;
    }

    if (v_fwd != 0.0f || v_right != 0.0f) {
        // Rotate body-relative velocity demand to world NED using current held yaw
        float cy = std::cos(m_target_yaw);
        float sy = std::sin(m_target_yaw);
        float vel_n = v_fwd * cy - v_right * sy;
        float vel_e = v_fwd * sy + v_right * cy;

        m_target_pos_ned[0] += vel_n * dt;
        m_target_pos_ned[1] += vel_e * dt;
    }

    // 2. Throttle Stick Deflection -> Vertical Climb / Descent
    float stick_thr = state.rc_throttle;
    if (stick_thr > 0.58f) {
        // Climb command (0.58 .. 1.0 -> 0 .. m_max_climb_speed_ms)
        float climb_rate = ((stick_thr - 0.58f) / 0.42f) * m_max_climb_speed_ms;
        m_target_pos_ned[2] -= climb_rate * dt; // NED Down is negative altitude
    } else if (stick_thr < 0.42f) {
        // Descent command (0.42 .. 0.0 -> 0 .. m_max_desc_speed_ms)
        float desc_rate = ((0.42f - stick_thr) / 0.42f) * m_max_desc_speed_ms;
        m_target_pos_ned[2] += desc_rate * dt;
    }

    // 3. Yaw Stick Deflection -> Heading Rotation
    float stick_yaw = state.rc_yaw;
    if (std::abs(stick_yaw) > kStickDeadband) {
        m_target_yaw += stick_yaw * m_max_yaw_rate_rads * dt;
        while (m_target_yaw > 3.14159265f)  m_target_yaw -= 6.2831853f;
        while (m_target_yaw < -3.14159265f) m_target_yaw += 6.2831853f;
    }

    ControlOutput out = m_cascade.compute(m_target_pos_ned, true, m_target_yaw, state);

    // Pilot settling onto the ground with the throttle stick down: once
    // LandDetector confirms touchdown (near-zero climb rate, resting
    // altitude, thrust well below hover), drop to ground idle - see
    // m_on_ground. Disarming is left to DISARM_DELAY or the stick gesture.
    float current_rel_alt = -state.pos_ned[2];
    float climb_rate = -state.vel_ned[2];
    if (state.rc_throttle <= kThrottleClimb &&
        m_land_detector.update(current_rel_alt, climb_rate, out.thrust, state.dt_s)) {
        std::printf("[APX Mode] LOITER: touchdown confirmed (alt=%.2fm) - motors idle, "
                   "auto-disarm in %.0fs at zero throttle\n",
                   (double)current_rel_alt, (double)m_disarm_delay_s);
        m_on_ground = true;
        m_disarm_timer_s = 0.0f;
        m_cascade.reset();
        return ControlOutput{};
    }

    return out;
}

} // namespace apx::modes
