// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include <cstdint>
#include <cstddef>

namespace apx::hal {

class IHalStorage {
public:
    virtual ~IHalStorage() = default;

    virtual bool read(uint32_t offset, uint8_t* buf, size_t len) = 0;
    virtual bool write(uint32_t offset, const uint8_t* buf, size_t len) = 0;
    virtual bool erase(uint32_t offset, size_t len) = 0;
    virtual void sync() = 0;
};

} // namespace apx::hal
