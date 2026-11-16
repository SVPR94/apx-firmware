// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "pump_driver.hpp"
#include <algorithm>

namespace apx::drivers {

PumpDriver::PumpDriver(apx::hal::IHalPwm& pwm, uint8_t channel, uint32_t freq_hz)
    : DriverBase("pump"), m_pwm(pwm), m_channel(channel), m_freq_hz(freq_hz) {}

bool PumpDriver::init() {
    m_pwm.init(m_channel, m_freq_hz);
    m_pwm.set_duty(m_channel, kMinPwmUs);
    set_status(DriverStatus::RUNNING);
    return true;
}

void PumpDriver::set_flow_rate(float rate_0_to_1) {
    m_flow_rate = std::max(0.0f, std::min(1.0f, rate_0_to_1));
}

void PumpDriver::enable() {
    m_enabled = true;
    m_pwm.enable(m_channel);
    uint32_t duty = kMinPwmUs + static_cast<uint32_t>(m_flow_rate * (kMaxPwmUs - kMinPwmUs));
    m_pwm.set_duty(m_channel, duty);
    mark_updated();
}

void PumpDriver::disable() {
    m_enabled = false;
    m_pwm.set_duty(m_channel, kMinPwmUs);
    m_pwm.disable(m_channel);
    mark_updated();
}

void PumpDriver::update() {
    if (m_enabled) {
        uint32_t duty = kMinPwmUs + static_cast<uint32_t>(m_flow_rate * (kMaxPwmUs - kMinPwmUs));
        m_pwm.set_duty(m_channel, duty);
    }
}

void PumpDriver::update_volume(float dt_s, float area_m2, float speed_ms) {
    if (!m_enabled) return;
    // Simple volume model: flow_rate * speed * area * dt
    constexpr float kLitersPerMeterSquare = 0.001f; // 1 L/m^2 application rate
    m_volume_ml += m_flow_rate * speed_ms * area_m2 * kLitersPerMeterSquare * dt_s * 1000.0f;
}

} // namespace apx::drivers
