// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "mode_agri_spray.hpp"
#include <cstdio>

namespace apx::modes {

ModeAgriSpray::ModeAgriSpray() {}

void ModeAgriSpray::on_enter(const VehicleState& state) {
    m_spray_state = SprayState::IDLE;
    m_hold_pos_ned[0] = state.pos_ned[0];
    m_hold_pos_ned[1] = state.pos_ned[1];
    m_hold_pos_ned[2] = state.pos_ned[2];
    m_hold_yaw = state.yaw;
    m_cascade.reset();
    std::printf("[APX Mode] Entering AGRI_SPRAY (pump/pattern logic not implemented - holding position)\n");
}

void ModeAgriSpray::on_exit() {
    m_spray_state = SprayState::IDLE; // pump relay off - see drivers/pump once wired up
    m_cascade.reset();
}

ControlOutput ModeAgriSpray::update(const VehicleState& state) {
    // TODO: pump relay + spray-area path logic (m_spray_state/
    // m_target_spray_rate are placeholders) - see class comment.
    if (!state.armed) {
        m_cascade.reset();
        m_hold_pos_ned[0] = state.pos_ned[0];
        m_hold_pos_ned[1] = state.pos_ned[1];
        m_hold_pos_ned[2] = state.pos_ned[2];
        return ControlOutput{};
    }
    return m_cascade.compute(m_hold_pos_ned, true, m_hold_yaw, state);
}

} // namespace apx::modes
