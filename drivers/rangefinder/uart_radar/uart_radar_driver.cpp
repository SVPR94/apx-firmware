// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "uart_radar_driver.hpp"
#include "../../../modules/radar/radar_manager.hpp"

namespace apx::drivers {

UartRadarDriver::UartRadarDriver(apx::hal::IHalUart& uart, uint8_t sensor_idx, uint32_t baud)
    : m_uart(uart), m_sensor_idx(sensor_idx), m_baud(baud) {
}

bool UartRadarDriver::init() {
    return true;
}

void UartRadarDriver::update() {
    uint8_t byte = 0;
    while (m_uart.read(&byte, 1) > 0) {
        switch (m_state) {
            case ParserState::WAIT_HEADER1:
                if (byte == 0x59) {
                    m_state = ParserState::WAIT_HEADER2;
                }
                break;

            case ParserState::WAIT_HEADER2:
                if (byte == 0x59) {
                    m_state = ParserState::READ_DATA;
                    m_buf_idx = 0;
                } else if (byte != 0x59) {
                    m_state = ParserState::WAIT_HEADER1;
                }
                break;

            case ParserState::READ_DATA:
                m_buf[m_buf_idx++] = byte;
                if (m_buf_idx >= 7) {
                    // Checksum: low 8 bits of sum(0x59 + 0x59 + buf[0..5])
                    uint8_t csum = 0x59 + 0x59;
                    for (int i = 0; i < 6; ++i) {
                        csum += m_buf[i];
                    }

                    if (csum == m_buf[6]) {
                        uint16_t dist_cm = static_cast<uint16_t>(m_buf[0]) | (static_cast<uint16_t>(m_buf[1]) << 8);
                        uint16_t strength = static_cast<uint16_t>(m_buf[2]) | (static_cast<uint16_t>(m_buf[3]) << 8);

                        m_distance_m = dist_cm * 0.01f;
                        m_signal_quality = static_cast<uint8_t>(std::clamp(strength / 10, 0, 100));
                        m_valid = (strength > 100 && dist_cm > 0);

                        apx::radar::RadarManager::get_instance().feed_sensor_reading(
                            m_sensor_idx, m_distance_m, m_signal_quality, m_valid, 0);
                    }
                    m_state = ParserState::WAIT_HEADER1;
                }
                break;
        }
    }
}

} // namespace apx::drivers
