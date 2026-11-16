// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "crsf_driver.hpp"
#include "apx/kernel/timer.hpp"

namespace apx::drivers {

CrsfDriver::CrsfDriver(apx::hal::IHalUart& uart)
    : DriverBase("crsf"), m_uart(uart) {}

bool CrsfDriver::init() {
    if (!m_uart.init(420000)) {
        set_status(DriverStatus::FAILED);
        return false;
    }
    set_status(DriverStatus::RUNNING);
    return true;
}

uint8_t CrsfDriver::crc8_dvb_s2(uint8_t crc, uint8_t a) const {
    crc ^= a;
    for (int i = 0; i < 8; i++) {
        crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : crc << 1;
    }
    return crc;
}

bool CrsfDriver::parse_frame(const uint8_t* buf, uint8_t len) {
    if (buf[2] != CRSF_FRAMETYPE_RC_CHANNELS) return false;
    if (len < 26) return false;

    // Verify CRC
    uint8_t crc = 0;
    for (int i = 2; i < len - 1; i++) crc = crc8_dvb_s2(crc, buf[i]);
    if (crc != buf[len - 1]) return false;

    const uint8_t* p = &buf[3];
    // 16 channels x 11 bits packed
    uint16_t ch[16] = {};
    ch[0]  = ((p[0])       | (p[1]  << 8)) & 0x7FF;
    ch[1]  = ((p[1]  >> 3) | (p[2]  << 5)) & 0x7FF;
    ch[2]  = ((p[2]  >> 6) | (p[3]  << 2) | (p[4] << 10)) & 0x7FF;
    ch[3]  = ((p[4]  >> 1) | (p[5]  << 7)) & 0x7FF;
    ch[4]  = ((p[5]  >> 4) | (p[6]  << 4)) & 0x7FF;
    ch[5]  = ((p[6]  >> 7) | (p[7]  << 1) | (p[8] << 9)) & 0x7FF;
    ch[6]  = ((p[8]  >> 2) | (p[9]  << 6)) & 0x7FF;
    ch[7]  = ((p[9]  >> 5) | (p[10] << 3)) & 0x7FF;
    ch[8]  = ((p[11])      | (p[12] << 8)) & 0x7FF;
    ch[9]  = ((p[12] >> 3) | (p[13] << 5)) & 0x7FF;
    ch[10] = ((p[13] >> 6) | (p[14] << 2) | (p[15] << 10)) & 0x7FF;
    ch[11] = ((p[15] >> 1) | (p[16] << 7)) & 0x7FF;
    ch[12] = ((p[16] >> 4) | (p[17] << 4)) & 0x7FF;
    ch[13] = ((p[17] >> 7) | (p[18] << 1) | (p[19] << 9)) & 0x7FF;
    ch[14] = ((p[19] >> 2) | (p[20] << 6)) & 0x7FF;
    ch[15] = ((p[20] >> 5) | (p[21] << 3)) & 0x7FF;

    // CRSF raw range: 172-1811 (same as SBUS)
    for (int i = 0; i < 16; i++) {
        m_data.channels[i] = (static_cast<float>(ch[i]) - 992.0f) / 820.0f;
    }
    m_data.timestamp_us = apx::kernel::now_us();
    return true;
}

void CrsfDriver::update() {
    uint8_t byte = 0;
    while (m_uart.available() > 0) {
        m_uart.read(&byte, 1);
        if (m_buf_idx == 0 && byte != 0xC8) continue; // CRSF address
        m_buf[m_buf_idx++] = byte;
        if (m_buf_idx >= 3) {
            uint8_t frame_len = m_buf[1] + 2; // length field + address + length bytes
            if (m_buf_idx >= frame_len) {
                if (parse_frame(m_buf, frame_len)) mark_updated();
                m_buf_idx = 0;
            }
        }
        if (m_buf_idx >= CRSF_FRAME_SIZE_MAX) m_buf_idx = 0;
    }
}

} // namespace apx::drivers
