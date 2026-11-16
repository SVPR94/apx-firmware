// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include <cstdint>

namespace apx::hal {

class IHalPwm {
public:
    virtual ~IHalPwm() = default;

    virtual void init(uint32_t channel, uint32_t freq_hz) = 0;
    virtual void set_duty(uint32_t channel, uint32_t duty_us) = 0;
    virtual void enable(uint32_t channel) = 0;
    virtual void disable(uint32_t channel) = 0;
};

} // namespace apx::hal
