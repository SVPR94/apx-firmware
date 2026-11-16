// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "sbus_driver.hpp"
#include "apx/kernel/timer.hpp"

namespace apx::drivers {

SbusDriver::SbusDriver(apx::hal::IHalUart& uart)
    : DriverBase("sbus"), m_uart(uart) {}

bool SbusDriver::init() {
    // S.BUS: 100000 baud, 8E2, inverted
    if (!m_uart.init(100000)) {
        set_status(DriverStatus::FAILED);
        return false;
    }
    set_status(DriverStatus::RUNNING);
    return true;
}

float SbusDriver::normalize(uint16_t raw) {
    // S.BUS range: 172 (min) to 1811 (max), 992 = center
    constexpr float kMin = 172.0f, kMax = 1811.0f;
    return (static_cast<float>(raw) - kMin) / (kMax - kMin) * 2.0f - 1.0f;
}

bool SbusDriver::parse_frame(const uint8_t buf[SBUS_FRAME_SIZE]) {
    if (buf[0] != 0x0F || buf[24] != 0x00) return false;

    uint16_t ch[16] = {};
    ch[0]  = (buf[1]      | buf[2]  << 8) & 0x07FF;
    ch[1]  = (buf[2]  >> 3 | buf[3]  << 5) & 0x07FF;
    ch[2]  = (buf[3]  >> 6 | buf[4]  << 2 | buf[5] << 10) & 0x07FF;
    ch[3]  = (buf[5]  >> 1 | buf[6]  << 7) & 0x07FF;
    ch[4]  = (buf[6]  >> 4 | buf[7]  << 4) & 0x07FF;
    ch[5]  = (buf[7]  >> 7 | buf[8]  << 1 | buf[9] << 9) & 0x07FF;
    ch[6]  = (buf[9]  >> 2 | buf[10] << 6) & 0x07FF;
    ch[7]  = (buf[10] >> 5 | buf[11] << 3) & 0x07FF;
    ch[8]  = (buf[12]     | buf[13] << 8) & 0x07FF;
    ch[9]  = (buf[13] >> 3 | buf[14] << 5) & 0x07FF;
    ch[10] = (buf[14] >> 6 | buf[15] << 2 | buf[16] << 10) & 0x07FF;
    ch[11] = (buf[16] >> 1 | buf[17] << 7) & 0x07FF;
    ch[12] = (buf[17] >> 4 | buf[18] << 4) & 0x07FF;
    ch[13] = (buf[18] >> 7 | buf[19] << 1 | buf[20] << 9) & 0x07FF;
    ch[14] = (buf[20] >> 2 | buf[21] << 6) & 0x07FF;
    ch[15] = (buf[21] >> 5 | buf[22] << 3) & 0x07FF;

    for (int i = 0; i < 16; i++) {
        m_data.channels[i] = normalize(ch[i]);
    }
    m_data.failsafe   = (buf[23] & 0x08) != 0;
    m_data.frame_lost = (buf[23] & 0x04) != 0;
    m_data.timestamp_us = apx::kernel::now_us();
    return true;
}

void SbusDriver::update() {
    uint8_t byte = 0;
    while (m_uart.available() > 0) {
        m_uart.read(&byte, 1);
        if (m_buf_idx == 0 && byte != 0x0F) continue;
        m_buf[m_buf_idx++] = byte;
        if (m_buf_idx == SBUS_FRAME_SIZE) {
            if (parse_frame(m_buf)) {
                mark_updated();
            }
            m_buf_idx = 0;
        }
    }
}

} // namespace apx::drivers
