// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once
#include "apx/drivers/driver_base.hpp"
#include "apx/hal/hal_uart.hpp"
#include <cstdint>
#include <array>

namespace apx::drivers {

struct RcData {
    float    channels[18];   // normalized -1.0 to 1.0
    bool     failsafe;
    bool     frame_lost;
    uint64_t timestamp_us;
};

class SbusDriver : public DriverBase {
public:
    static constexpr int SBUS_FRAME_SIZE   = 25;
    static constexpr int SBUS_CHANNEL_COUNT = 16;

    explicit SbusDriver(apx::hal::IHalUart& uart);

    bool init() override;
    void update() override;

    const RcData& get_data() const { return m_data; }

private:
    bool parse_frame(const uint8_t buf[SBUS_FRAME_SIZE]);
    static float normalize(uint16_t raw);

    apx::hal::IHalUart& m_uart;
    uint8_t  m_buf[SBUS_FRAME_SIZE]{};
    int      m_buf_idx{0};
    RcData   m_data{};
};

} // namespace apx::drivers
