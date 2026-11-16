// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include "apx/drivers/driver_base.hpp"
#include "apx/hal/hal_uart.hpp"
#include <cstdint>

namespace apx::drivers {

// Standard UART radar / altimeter driver (supports standard 9-byte radar packets, Benewake TF02/TF03/TFmini)
class UartRadarDriver : public DriverBase {
public:
    UartRadarDriver(apx::hal::IHalUart& uart, uint8_t sensor_idx = 0, uint32_t baud = 115200);
    bool init() override;
    void update() override;

    float get_distance_m() const { return m_distance_m; }
    uint8_t get_signal_quality() const { return m_signal_quality; }
    bool is_valid() const { return m_valid; }

private:
    apx::hal::IHalUart& m_uart;
    uint8_t m_sensor_idx{0};
    uint32_t m_baud{115200};

    float m_distance_m{0.0f};
    uint8_t m_signal_quality{0};
    bool m_valid{false};

    enum class ParserState : uint8_t {
        WAIT_HEADER1,
        WAIT_HEADER2,
        READ_DATA
    };
    ParserState m_state{ParserState::WAIT_HEADER1};
    uint8_t m_buf[7]{};
    uint8_t m_buf_idx{0};
};

} // namespace apx::drivers
