// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "mode_brake.hpp"
#include "param_manager.hpp"
#include <cmath>
#include <cstdio>
#include <algorithm>

namespace apx::modes {

ModeBrake::ModeBrake() {
    auto& params = apx::gcs::ParamManager::get_instance();
    // Deceleration in m/s^2 (BRAKE_ACCEL parameter is in cm/s^2, default 250 cm/s^2)
    m_brake_accel_ms2 = params.get_float_or("BRAKE_ACCEL", 250.0f) / 100.0f;
    if (m_brake_accel_ms2 < 0.5f) {
        m_brake_accel_ms2 = 2.5f;
    }
}

void ModeBrake::on_enter(const VehicleState& state) {
    m_cascade.reset();
    m_target_pos_ned[2] = state.pos_ned[2];
    m_target_yaw = state.yaw;

    float speed_xy = std::sqrt(state.vel_ned[0] * state.vel_ned[0] +
                               state.vel_ned[1] * state.vel_ned[1]);

    if (state.armed && speed_xy > 0.2f) {
        // Calculate projected stopping point along velocity vector
        float stop_dist = (speed_xy * speed_xy) / (2.0f * m_brake_accel_ms2);
        stop_dist = std::min(stop_dist, 10.0f); // safety clamp
        m_target_pos_ned[0] = state.pos_ned[0] + (state.vel_ned[0] / speed_xy) * stop_dist;
        m_target_pos_ned[1] = state.pos_ned[1] + (state.vel_ned[1] / speed_xy) * stop_dist;
        m_stopped = false;
        std::printf("[APX Mode] Entering BRAKE (braking from %.2fm/s -> target N=%.2f, E=%.2f, Alt=%.2fm, Yaw=%.1fdeg)\n",
                    (double)speed_xy, (double)m_target_pos_ned[0], (double)m_target_pos_ned[1],
                    -(double)m_target_pos_ned[2], (double)(m_target_yaw * 57.2958f));
    } else {
        m_target_pos_ned[0] = state.pos_ned[0];
        m_target_pos_ned[1] = state.pos_ned[1];
        m_stopped = true;
        std::printf("[APX Mode] Entering BRAKE (holding position at N=%.2f, E=%.2f, Alt=%.2fm, Yaw=%.1fdeg)\n",
                    (double)m_target_pos_ned[0], (double)m_target_pos_ned[1],
                    -(double)m_target_pos_ned[2], (double)(m_target_yaw * 57.2958f));
    }
}

void ModeBrake::on_exit() {
    m_cascade.reset();
}

ControlOutput ModeBrake::update(const VehicleState& state) {
    if (!state.armed) {
        m_cascade.reset();
        m_stopped = true;
        m_target_pos_ned[0] = state.pos_ned[0];
        m_target_pos_ned[1] = state.pos_ned[1];
        m_target_pos_ned[2] = state.pos_ned[2];
        m_target_yaw = state.yaw;
        return ControlOutput{};
    }

    if (!m_stopped) {
        float speed_xy = std::sqrt(state.vel_ned[0] * state.vel_ned[0] +
                                   state.vel_ned[1] * state.vel_ned[1]);
        if (speed_xy < 0.15f) {
            m_stopped = true;
            m_target_pos_ned[0] = state.pos_ned[0];
            m_target_pos_ned[1] = state.pos_ned[1];
            m_target_pos_ned[2] = state.pos_ned[2];
            std::printf("[APX Mode] BRAKE: vehicle stopped, holding position at N=%.2f, E=%.2f, Alt=%.2fm\n",
                        (double)m_target_pos_ned[0], (double)m_target_pos_ned[1],
                        -(double)m_target_pos_ned[2]);
        }
    }

    // Pilot stick inputs are ignored in BRAKE mode to pause the drone firmly in place
    // until the pilot or GCS explicitly commands a flight mode change.
    return m_cascade.compute(m_target_pos_ned, true, m_target_yaw, state, false, 0.0f, nullptr, nullptr);
}

} // namespace apx::modes
