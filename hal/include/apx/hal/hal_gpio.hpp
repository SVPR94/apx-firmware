// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include <cstdint>
#include <functional>

namespace apx::hal {

enum class Direction {
    INPUT,
    OUTPUT
};

enum class Pull {
    NONE,
    UP,
    DOWN
};

enum class Edge {
    RISING,
    FALLING,
    BOTH
};

class IHalGpio {
public:
    virtual ~IHalGpio() = default;

    virtual void configure(uint32_t pin, Direction direction, Pull pull = Pull::NONE) = 0;
    virtual void write(uint32_t pin, bool value) = 0;
    virtual bool read(uint32_t pin) = 0;

    using InterruptCallback = std::function<void(uint32_t pin)>;
    virtual void set_interrupt(uint32_t pin, Edge edge, InterruptCallback callback) = 0;
};

} // namespace apx::hal
