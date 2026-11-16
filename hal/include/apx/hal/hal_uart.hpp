// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include <cstdint>
#include <cstddef>
#include <functional>

namespace apx::hal {

class IHalUart {
public:
    virtual ~IHalUart() = default;

    virtual bool init(uint32_t baud_rate) = 0;
    virtual void deinit() = 0;
    virtual int write(const uint8_t* buf, size_t len) = 0;
    virtual int read(uint8_t* buf, size_t max_len) = 0;
    virtual int available() const = 0;
    virtual void flush() = 0;

    virtual void set_baudrate(uint32_t baud_rate) { init(baud_rate); }

    using RxCallback = std::function<void(const uint8_t* buf, size_t len)>;
    virtual void set_rx_callback(RxCallback fn) = 0;
};

} // namespace apx::hal
