// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "mode_land.hpp"
#include "param_manager.hpp"
#include <cmath>
#include <cstdio>
#include <algorithm>

namespace apx::modes {

namespace {
constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegToRad = 0.017453293f;
constexpr float kStickDeadband = 0.05f;
} // namespace

ModeLand::ModeLand() {
    auto& params = apx::gcs::ParamManager::get_instance();
    m_land_speed_ms = params.get_float_or("LAND_SPEED", 50.0f) / 100.0f;
    m_land_speed_high_ms = params.get_float_or("LAND_SPEED_HIGH", 0.0f) / 100.0f;
    m_land_alt_low_m = params.get_float_or("LAND_ALT_LOW", 1000.0f) / 100.0f;
    m_speed_dn_ms = params.get_float_or("WPNAV_SPEED_DN", 150.0f) / 100.0f;
    m_max_reposition_speed_ms = params.get_float_or("LAND_REPOS_SPEED", 150.0f) / 100.0f;
    m_max_yaw_rate_rads = params.get_float_or("ATC_RATE_Y_MAX", 45.0f) * kDegToRad;
}

float ModeLand::descent_speed_ms(float alt_m) const {
    if (alt_m > m_land_alt_low_m) {
        return (m_land_speed_high_ms > 0.0f) ? m_land_speed_high_ms : m_speed_dn_ms;
    }
    return m_land_speed_ms;
}

void ModeLand::on_enter(const VehicleState& state) {
    m_hold_yaw = state.yaw;
    m_detector.reset();
    m_cascade.reset();

    float horiz_speed = std::sqrt(state.vel_ned[0] * state.vel_ned[0] + state.vel_ned[1] * state.vel_ned[1]);
    if (horiz_speed > 0.35f) {
        // Vehicle is moving at speed when LAND was commanded!
        // First brake forward safely along current travel vector to zero speed,
        // then land at the exact location where vehicle stopped (not where command was clicked).
        float d_stop = (horiz_speed * horiz_speed) / (2.0f * m_brake_accel_ms2);
        float un = state.vel_ned[0] / horiz_speed;
        float ue = state.vel_ned[1] / horiz_speed;
        m_hold_pos_ned[0] = state.pos_ned[0] + un * d_stop;
        m_hold_pos_ned[1] = state.pos_ned[1] + ue * d_stop;
        m_braking_vel_ne[0] = state.vel_ned[0];
        m_braking_vel_ne[1] = state.vel_ned[1];
        m_phase = LandPhase::Braking;
        std::printf("[APX Mode] Entering LAND: forward braking from %.2f m/s to projected stop at N=%.2f, E=%.2f (Alt=%.2fm, Yaw=%.1fdeg)\n",
                    (double)horiz_speed, (double)m_hold_pos_ned[0], (double)m_hold_pos_ned[1],
                    -(double)state.pos_ned[2], (double)(m_hold_yaw * 57.2958f));
    } else {
        // Already hovering or stationary: land immediately in place
        m_hold_pos_ned[0] = state.pos_ned[0];
        m_hold_pos_ned[1] = state.pos_ned[1];
        m_braking_vel_ne[0] = 0.0f;
        m_braking_vel_ne[1] = 0.0f;
        m_phase = LandPhase::Descend;
        std::printf("[APX Mode] Entering LAND (lock N=%.2f, E=%.2f, Alt=%.2fm, Yaw=%.1fdeg)\n",
                    (double)m_hold_pos_ned[0], (double)m_hold_pos_ned[1],
                    -(double)state.pos_ned[2], (double)(m_hold_yaw * 57.2958f));
    }
}

void ModeLand::on_exit() {
    m_cascade.reset();
}

ControlOutput ModeLand::update(const VehicleState& state) {
    if (!state.armed) {
        m_cascade.reset();
        m_detector.reset();
        return ControlOutput{};
    }

    if (m_detector.landed()) {
        ControlOutput out;
        out.request_disarm = true;
        return out;
    }

    const float dt = std::clamp(state.dt_s, 0.001f, 0.05f);

    // Pilot Manual Repositioning via Joystick / RC:
    // Roll/Pitch sticks nudge horizontal landing position in world NED
    if (state.rc_valid) {
        float v_fwd = 0.0f;
        float v_right = 0.0f;

        if (std::abs(state.rc_pitch) > kStickDeadband) {
            v_fwd = state.rc_pitch * m_max_reposition_speed_ms;
        }
        if (std::abs(state.rc_roll) > kStickDeadband) {
            v_right = state.rc_roll * m_max_reposition_speed_ms;
        }

        if (v_fwd != 0.0f || v_right != 0.0f) {
            // Rotate body-relative stick commands to world NED using held yaw
            float cy = std::cos(m_hold_yaw);
            float sy = std::sin(m_hold_yaw);
            float vel_n = v_fwd * cy - v_right * sy;
            float vel_e = v_fwd * sy + v_right * cy;

            m_hold_pos_ned[0] += vel_n * dt;
            m_hold_pos_ned[1] += vel_e * dt;
        }

        // Yaw stick adjusts / slews held heading
        if (std::abs(state.rc_yaw) > kStickDeadband) {
            m_hold_yaw += state.rc_yaw * m_max_yaw_rate_rads * dt;
            // Wrap to [-pi, pi]
            while (m_hold_yaw > kPi) m_hold_yaw -= 2.0f * kPi;
            while (m_hold_yaw < -kPi) m_hold_yaw += 2.0f * kPi;
        }
    }

    float current_rel_alt = -state.pos_ned[2];
    float current_descent_rate = 0.0f;
    const float* vel_ff_ptr = nullptr;

    if (m_phase == LandPhase::Braking) {
        // Decelerate feedforward velocity towards 0 at m_brake_accel_ms2
        float cur_ff_speed = std::sqrt(m_braking_vel_ne[0] * m_braking_vel_ne[0] +
                                       m_braking_vel_ne[1] * m_braking_vel_ne[1]);
        float dv = m_brake_accel_ms2 * dt;
        if (cur_ff_speed > dv) {
            m_braking_vel_ne[0] -= (m_braking_vel_ne[0] / cur_ff_speed) * dv;
            m_braking_vel_ne[1] -= (m_braking_vel_ne[1] / cur_ff_speed) * dv;
        } else {
            m_braking_vel_ne[0] = 0.0f;
            m_braking_vel_ne[1] = 0.0f;
        }
        vel_ff_ptr = m_braking_vel_ne;

        // Check if vehicle has safely come to rest
        float current_speed = std::sqrt(state.vel_ned[0] * state.vel_ned[0] +
                                        state.vel_ned[1] * state.vel_ned[1]);
        if (current_speed <= 0.25f && cur_ff_speed <= 0.1f) {
            // Stopped! Lock exact touchdown coordinates to current stopped location
            m_hold_pos_ned[0] = state.pos_ned[0];
            m_hold_pos_ned[1] = state.pos_ned[1];
            m_phase = LandPhase::Descend;
            std::printf("[APX Mode] LAND: braking complete, stopped at N=%.2f, E=%.2f - beginning vertical descent\n",
                        (double)m_hold_pos_ned[0], (double)m_hold_pos_ned[1]);
        }
        // While braking horizontally at speed, keep vertical altitude level
        current_descent_rate = 0.0f;
    } else {
        // LandPhase::Descend:
        current_descent_rate = descent_speed_ms(current_rel_alt);
    }

    float target_pos_ned[3] = {m_hold_pos_ned[0], m_hold_pos_ned[1], state.pos_ned[2]};
    ControlOutput out = m_cascade.compute(target_pos_ned, /*has_yaw=*/true, m_hold_yaw, state,
                                          /*use_forced_climb_rate=*/true, -current_descent_rate,
                                          vel_ff_ptr);

    if (m_phase == LandPhase::Descend) {
        float climb_rate = -state.vel_ned[2];
        if (m_detector.update(current_rel_alt, climb_rate, out.thrust, state.dt_s)) {
            std::printf("[APX Mode] LAND: touchdown confirmed (alt=%.2fm climb=%.2fm/s) - stopping motors\n",
                       (double)current_rel_alt, (double)climb_rate);
            ControlOutput stopped;
            stopped.request_disarm = true;
            return stopped;
        }
    }

    return out;
}

} // namespace apx::modes
