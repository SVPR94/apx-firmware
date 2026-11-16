// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include <cstdint>
#include <cstddef>

namespace apx::hal {

struct CanFrame {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
    bool extended_id;
};

class IHalCan {
public:
    virtual ~IHalCan() = default;

    virtual void init(uint32_t baud_rate) = 0;
    virtual bool send(const CanFrame& frame) = 0;
    virtual bool recv(CanFrame& frame, uint32_t timeout_ms) = 0;
};

} // namespace apx::hal
