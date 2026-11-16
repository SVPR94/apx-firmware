// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "mode_rtl.hpp"
#include "param_manager.hpp"
#include <cmath>
#include <cstdio>

namespace apx::modes {

ModeRTL::ModeRTL() {
    // RTL_ALT is in cm (standard units); it was registered but never read
    // here, so RTL always returned at the hard-coded 15 m.
    m_rtl_altitude_m = apx::gcs::ParamManager::get_instance().get_float_or("RTL_ALT", 1500.0f) / 100.0f;
}

void ModeRTL::on_enter(const VehicleState& state) {
    m_climb_pos_ned[0] = state.pos_ned[0];
    m_climb_pos_ned[1] = state.pos_ned[1];
    // If already above the RTL altitude, don't descend to reach it first -
    // go straight to RETURNING at the current altitude.
    float current_rel_alt = -state.pos_ned[2];
    m_state = (current_rel_alt >= m_rtl_altitude_m - kAltToleranceM)
                  ? RtlState::RETURNING
                  : RtlState::CLIMBING;
    m_detector.reset();
    m_cascade.reset();
    std::printf("[APX Mode] Entering RTL (alt target %.1fm)\n", (double)m_rtl_altitude_m);
}

void ModeRTL::on_exit() {
    m_cascade.reset();
}

ControlOutput ModeRTL::update(const VehicleState& state) {
    if (!state.armed) {
        m_cascade.reset();
        m_detector.reset();
        return ControlOutput{};
    }

    float current_rel_alt = -state.pos_ned[2];
    float climb_rate = -state.vel_ned[2];

    switch (m_state) {
        case RtlState::CLIMBING: {
            float target[3] = {m_climb_pos_ned[0], m_climb_pos_ned[1], -m_rtl_altitude_m};
            ControlOutput out = m_cascade.compute(target, false, 0.0f, state);
            if (current_rel_alt >= m_rtl_altitude_m - kAltToleranceM) {
                m_state = RtlState::RETURNING;
                std::printf("[APX Mode] RTL: reached %.1fm, returning to home\n", (double)current_rel_alt);
            }
            return out;
        }
        case RtlState::RETURNING: {
            float target[3] = {state.home_ned[0], state.home_ned[1], -m_rtl_altitude_m};
            ControlOutput out = m_cascade.compute(target, false, 0.0f, state);
            float dx = state.pos_ned[0] - state.home_ned[0];
            float dy = state.pos_ned[1] - state.home_ned[1];
            float horiz_dist = std::sqrt(dx * dx + dy * dy);
            if (horiz_dist <= kHomeRadiusM) {
                m_state = RtlState::DESCENDING;
                std::printf("[APX Mode] RTL: over home, descending\n");
            }
            return out;
        }
        case RtlState::DESCENDING:
        default: {
            if (m_detector.landed()) {
                ControlOutput out;
                out.request_disarm = true;
                return out;
            }
            float target[3] = {state.home_ned[0], state.home_ned[1], state.pos_ned[2]};
            ControlOutput out = m_cascade.compute(target, false, 0.0f, state,
                                                  /*use_forced_climb_rate=*/true, -kDescentRateMs);
            if (m_detector.update(current_rel_alt, climb_rate, out.thrust, state.dt_s)) {
                std::printf("[APX Mode] RTL: touchdown confirmed - stopping motors\n");
                ControlOutput stopped;
                stopped.request_disarm = true;
                return stopped;
            }
            return out;
        }
    }
}

} // namespace apx::modes
