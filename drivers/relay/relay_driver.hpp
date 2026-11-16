// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once
#include "apx/drivers/driver_base.hpp"
#include "apx/hal/hal_gpio.hpp"
#include <cstdint>
#include <array>

namespace apx::drivers {

class RelayDriver : public DriverBase {
public:
    RelayDriver(apx::hal::IHalGpio& gpio, uint8_t pin, bool active_high = true);
    bool init() override;
    void set_state(bool on);
    void toggle();
    bool get_state() const { return m_state; }
private:
    apx::hal::IHalGpio& m_gpio;
    uint8_t m_pin;
    bool    m_active_high;
    bool    m_state{false};
};

} // namespace apx::drivers
