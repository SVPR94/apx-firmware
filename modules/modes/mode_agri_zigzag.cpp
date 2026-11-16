// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "mode_agri_zigzag.hpp"
#include <cstdio>

namespace apx::modes {

ModeAgriZigZag::ModeAgriZigZag() {}

void ModeAgriZigZag::on_enter(const VehicleState& state) {
    m_state = ZigZagState::SET_A;
    m_hold_pos_ned[0] = state.pos_ned[0];
    m_hold_pos_ned[1] = state.pos_ned[1];
    m_hold_pos_ned[2] = state.pos_ned[2];
    m_hold_yaw = state.yaw;
    m_cascade.reset();
    std::printf("[APX Mode] Entering AGRI_ZIGZAG (row pattern logic not implemented - holding position)\n");
}

void ModeAgriZigZag::on_exit() {
    m_cascade.reset();
}

ControlOutput ModeAgriZigZag::update(const VehicleState& state) {
    // TODO: A/B row-following state machine (m_state/m_pt_a_ned/
    // m_pt_b_ned/m_row_spacing are placeholders) - see class comment.
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
