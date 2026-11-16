// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include <cstdint>
#include <cstddef>

namespace apx::hal {

enum class SpiMode {
    MODE_0, // CPOL=0, CPHA=0
    MODE_1, // CPOL=0, CPHA=1
    MODE_2, // CPOL=1, CPHA=0
    MODE_3  // CPOL=1, CPHA=1
};

class IHalSpi {
public:
    virtual ~IHalSpi() = default;

    virtual void init(uint32_t clock_hz, SpiMode mode, uint8_t bits = 8) = 0;
    virtual bool transfer(const uint8_t* tx, uint8_t* rx, size_t len) = 0;
    virtual void cs_select(uint32_t id) = 0;
    virtual void cs_deselect(uint32_t id) = 0;
};

} // namespace apx::hal
