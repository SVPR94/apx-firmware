// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "mode_poshold.hpp"
#include <cstdio>

namespace apx::modes {

ModePosHold::ModePosHold(FlightModeId represented_as) : m_id(represented_as) {}

void ModePosHold::on_enter(const VehicleState& state) {
    m_hold_pos_ned[0] = state.pos_ned[0];
    m_hold_pos_ned[1] = state.pos_ned[1];
    m_hold_pos_ned[2] = state.pos_ned[2];
    m_hold_yaw = state.yaw;
    m_cascade.reset();
    std::printf("[APX Mode] Entering %s\n", get_name().data());
}

void ModePosHold::on_exit() {
    m_cascade.reset();
}

ControlOutput ModePosHold::update(const VehicleState& state) {
    if (!state.armed) {
        m_cascade.reset();
        m_hold_pos_ned[0] = state.pos_ned[0];
        m_hold_pos_ned[1] = state.pos_ned[1];
        m_hold_pos_ned[2] = state.pos_ned[2];
        return ControlOutput{};
    }
    return m_cascade.compute(m_hold_pos_ned, true, m_hold_yaw, state);
}

std::string_view ModePosHold::get_name() const {
    return m_id == FlightModeId::LOITER ? "LOITER" : "POS_HOLD";
}

} // namespace apx::modes
