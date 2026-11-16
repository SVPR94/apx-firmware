// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "relay_driver.hpp"

namespace apx::drivers {

RelayDriver::RelayDriver(apx::hal::IHalGpio& gpio, uint8_t pin, bool active_high)
    : DriverBase("relay"), m_gpio(gpio), m_pin(pin), m_active_high(active_high) {}

bool RelayDriver::init() {
    m_gpio.configure(m_pin, apx::hal::Direction::OUTPUT, apx::hal::Pull::NONE);
    set_state(false);
    set_status(DriverStatus::RUNNING);
    return true;
}

void RelayDriver::set_state(bool on) {
    m_state = on;
    m_gpio.write(m_pin, m_active_high ? on : !on);
    mark_updated();
}

void RelayDriver::toggle() {
    set_state(!m_state);
}

} // namespace apx::drivers
