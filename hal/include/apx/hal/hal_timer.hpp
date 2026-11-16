// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include <cstdint>

namespace apx::hal {

class IHalTimer {
public:
    virtual ~IHalTimer() = default;

    virtual uint64_t get_microseconds() const = 0;
    virtual uint64_t get_milliseconds() const = 0;
    virtual void delay_us(uint32_t us) = 0;
    virtual void delay_ms(uint32_t ms) = 0;
};

} // namespace apx::hal
