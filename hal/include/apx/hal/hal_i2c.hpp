// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include <cstdint>
#include <cstddef>

namespace apx::hal {

class IHalI2c {
public:
    virtual ~IHalI2c() = default;

    virtual void init(uint32_t clock_hz) = 0;
    virtual bool write_reg(uint8_t addr, uint8_t reg, const uint8_t* data, size_t len) = 0;
    virtual bool read_reg(uint8_t addr, uint8_t reg, uint8_t* buf, size_t len) = 0;
};

} // namespace apx::hal
