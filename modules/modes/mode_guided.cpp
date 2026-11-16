// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "mode_guided.hpp"
#include <cstdio>
#include <cmath>
#include <algorithm>

namespace apx::modes {

namespace {
float wrap_pi(float angle) {
    while (angle > 3.14159265f)  angle -= 6.2831853f;
    while (angle < -3.14159265f) angle += 6.2831853f;
    return angle;
}
} // namespace

ModeGuided::ModeGuided() {}

void ModeGuided::on_enter(const VehicleState& state) {
    m_target_pos_ned[0] = state.pos_ned[0];
    m_target_pos_ned[1] = state.pos_ned[1];
    m_target_pos_ned[2] = state.pos_ned[2];
    m_target_yaw = state.yaw;
    m_has_yaw_target = true; // hold current heading until a GCS target says otherwise
    m_has_explicit_yaw = false;
    m_cascade.reset();
    std::printf("[APX Mode] Entering GUIDED (hold %.2f,%.2f,%.2f)\n",
               (double)m_target_pos_ned[0], (double)m_target_pos_ned[1], (double)m_target_pos_ned[2]);
}

void ModeGuided::on_exit() {
    m_cascade.reset();
}

ControlOutput ModeGuided::update(const VehicleState& state) {
    if (!state.armed) {
        m_cascade.reset();
        m_target_pos_ned[0] = state.pos_ned[0];
        m_target_pos_ned[1] = state.pos_ned[1];
        m_target_pos_ned[2] = state.pos_ned[2];
        m_target_yaw = state.yaw;
        m_has_explicit_yaw = false;
        return ControlOutput{};
    }

    // Auto-heading towards target destination if no explicit yaw override was commanded
    if (!m_has_explicit_yaw) {
        float dx = m_target_pos_ned[0] - state.pos_ned[0];
        float dy = m_target_pos_ned[1] - state.pos_ned[1];
        float dist = std::sqrt(dx * dx + dy * dy);
        if (dist > 1.0f) {
            float desired_yaw = std::atan2(dy, dx);
            float delta_yaw = wrap_pi(desired_yaw - m_target_yaw);
            float dt = std::clamp(state.dt_s, 0.001f, 0.05f);
            float max_yaw_step = 1.05f * dt; // ~60 deg/s
            m_target_yaw += std::clamp(delta_yaw, -max_yaw_step, max_yaw_step);
            m_target_yaw = wrap_pi(m_target_yaw);
            m_has_yaw_target = true;
        }
    }

    return m_cascade.compute(m_target_pos_ned, m_has_yaw_target, m_target_yaw, state);
}

void ModeGuided::set_guided_target(const GuidedTarget& target) {
    m_target_pos_ned[0] = target.pos_ned[0];
    m_target_pos_ned[1] = target.pos_ned[1];
    m_target_pos_ned[2] = target.pos_ned[2];
    if (target.has_yaw) {
        m_target_yaw = target.yaw;
        m_has_yaw_target = true;
        m_has_explicit_yaw = true;
    } else {
        m_has_explicit_yaw = false;
    }
    std::printf("[APX Mode] GUIDED target -> %.2f,%.2f,%.2f (explicit_yaw=%d)\n",
               (double)m_target_pos_ned[0], (double)m_target_pos_ned[1], (double)m_target_pos_ned[2],
               m_has_explicit_yaw ? 1 : 0);
}

void ModeGuided::request_takeoff(float target_rel_alt_m) {
    m_target_pos_ned[2] = -target_rel_alt_m; // NED: altitude is negative-down
    std::printf("[APX Mode] GUIDED takeoff -> %.2fm\n", (double)target_rel_alt_m);
}

} // namespace apx::modes
