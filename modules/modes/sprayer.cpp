// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "sprayer.hpp"
#include "param_manager.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace apx::modes {

Sprayer& Sprayer::instance() {
    static Sprayer sprayer;
    return sprayer;
}

void Sprayer::load_params() {
    auto& params = apx::gcs::ParamManager::get_instance();
    m_enabled = params.get_int_or("SPRAY_ENABLE", 1) != 0;
    m_pump_rate_pct = params.get_float_or("SPRAY_PUMP_RATE", 10.0f);
    m_pump_min_pct = params.get_float_or("SPRAY_PUMP_MIN", 0.0f);
    m_speed_min_ms = params.get_float_or("SPRAY_SPEED_MIN", 100.0f) / 100.0f;
    m_spinner_pwm = static_cast<uint16_t>(params.get_int_or("SPRAY_SPINNER", 1300));
    m_pump_ch = std::clamp<int>(params.get_int_or("SPRAY_PUMP_CH", 9), 1, 16);
    m_spin_ch = std::clamp<int>(params.get_int_or("SPRAY_SPIN_CH", 10), 1, 16);
    m_pwm_min = static_cast<uint16_t>(params.get_int_or("SPRAY_PWM_MIN", 1100));
    m_pwm_max = static_cast<uint16_t>(params.get_int_or("SPRAY_PWM_MAX", 1900));
}

bool Sprayer::run(bool on) {
    if (on && !m_enabled) {
        return false;
    }
    if (on != m_requested) {
        std::printf("[APX Sprayer] spraying %s\n", on ? "requested ON" : "OFF");
    }
    m_requested = on;
    // Clearing manual override returns control to the automatic speed-proportional scheduler
    m_manual_control = false;
    m_manual_spinner_control = false;
    if (!on) {
        m_manual_pump_pwm = m_pwm_min;
        m_manual_spinner_pwm = m_pwm_min;
        if (m_spraying) {
            m_spraying = false;
            m_state_changed = true;
        }
        m_pump_fraction = 0.0f;
    }
    return true;
}

void Sprayer::set_pump_pwm_manual(uint16_t pwm) {
    m_manual_control = true;
    m_manual_pump_pwm = std::clamp<uint16_t>(pwm, m_pwm_min, m_pwm_max);
    if (m_manual_pump_pwm <= m_pwm_min) {
        if (m_spraying) {
            m_spraying = false;
            m_state_changed = true;
        }
        m_pump_fraction = 0.0f;
    } else {
        if (!m_spraying) {
            m_spraying = true;
            m_state_changed = true;
        }
        m_pump_fraction = static_cast<float>(m_manual_pump_pwm - m_pwm_min) /
                          static_cast<float>(m_pwm_max - m_pwm_min);
    }
    std::printf("[APX Sprayer] Manual spray pump PWM set to %u (fraction=%.2f, spraying=%s)\n",
                m_manual_pump_pwm, (double)m_pump_fraction, m_spraying ? "ON" : "OFF");
}

void Sprayer::set_spinner_pwm_manual(uint16_t pwm) {
    m_manual_spinner_control = true;
    m_manual_spinner_pwm = std::clamp<uint16_t>(pwm, m_pwm_min, m_pwm_max);
    std::printf("[APX Sprayer] Manual spinner PWM set to %u\n", m_manual_spinner_pwm);
}

void Sprayer::clear_manual_control() {
    m_manual_control = false;
    m_manual_spinner_control = false;
    m_manual_pump_pwm = m_pwm_min;
    m_manual_spinner_pwm = m_pwm_min;
}

void Sprayer::set_flow_rate(float rate_0_to_1) {
    float rate = std::clamp(rate_0_to_1, 0.0f, 1.0f);
    uint16_t pwm = static_cast<uint16_t>(m_pwm_min + rate * (m_pwm_max - m_pwm_min));
    set_pump_pwm_manual(pwm);
}

void Sprayer::update(const VehicleState& state) {
    if (m_params_dirty.exchange(false)) {
        load_params();
    }
    const float dt = state.dt_s;

    // Detect falling edge of armed state: transition from ARMED to DISARMED
    const bool was_armed = m_last_armed;
    m_last_armed = state.armed;

    if (was_armed && !state.armed) {
        if (m_requested || m_manual_control || m_manual_spinner_control || m_spraying) {
            std::printf("[APX Sprayer] disarmed - spraying OFF\n");
        }
        m_requested = false;
        m_manual_control = false;
        m_manual_pump_pwm = m_pwm_min;
        m_manual_spinner_control = false;
        m_manual_spinner_pwm = m_pwm_min;
        if (m_spraying) {
            m_spraying = false;
            m_state_changed = true;
        }
        m_pump_fraction = 0.0f;
        m_above_min_s = m_below_min_s = 0.0f;
        return;
    }

    if (m_manual_control) {
        // Direct manual servo command / DO_SET_SERVO has authority.
        // It persists until disarmed, overridden, or turned off by DO_SET_SERVO / DO_SPRAYER 0.
        return;
    }

    const float ground_speed = std::sqrt(state.vel_ned[0] * state.vel_ned[0] +
                                         state.vel_ned[1] * state.vel_ned[1]);
    bool want = false;
    if (m_enabled && m_requested && state.armed) {
        if (m_speed_min_ms <= 0.01f || ground_speed >= m_speed_min_ms) {
            m_above_min_s += dt;
            m_below_min_s = 0.0f;
        } else {
            m_below_min_s += dt;
            m_above_min_s = 0.0f;
        }
        want = m_spraying ? (m_below_min_s < kShutOffDelayS) : (m_above_min_s >= kTurnOnDelayS);
    } else {
        m_above_min_s = m_below_min_s = 0.0f;
    }

    if (want != m_spraying) {
        m_spraying = want;
        m_state_changed = true;
    }
    if (m_spraying) {
        const float effective_speed = (m_speed_min_ms <= 0.01f && ground_speed < 0.1f) ? 1.0f : ground_speed;
        const float pct = std::max(effective_speed * m_pump_rate_pct, m_pump_min_pct);
        m_pump_fraction = std::clamp(pct / 100.0f, 0.0f, 1.0f);
    } else {
        m_pump_fraction = 0.0f;
    }
}

uint16_t Sprayer::pump_pwm() const {
    if (m_manual_control) {
        return m_manual_pump_pwm;
    }
    return m_spraying ? static_cast<uint16_t>(m_pwm_min + m_pump_fraction * (m_pwm_max - m_pwm_min))
                      : m_pwm_min;
}

uint16_t Sprayer::spinner_pwm() const {
    if (m_manual_spinner_control) {
        return m_manual_spinner_pwm;
    }
    return m_spraying ? m_spinner_pwm : m_pwm_min;
}

} // namespace apx::modes
