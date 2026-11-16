// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "dshot_driver.hpp"
#include "apx/kernel/timer.hpp"
#include <cstring>
#include <time.h>

namespace apx::drivers {

DshotDriver::DshotDriver(DshotSpeed speed)
    : DriverBase("dshot_esc"), m_speed(speed)
{
    switch (speed) {
        case DshotSpeed::DSHOT150:  m_bit_time_ns = 6667; break;
        case DshotSpeed::DSHOT300:  m_bit_time_ns = 3333; break;
        case DshotSpeed::DSHOT600:  m_bit_time_ns = 1667; break;
        case DshotSpeed::DSHOT1200: m_bit_time_ns = 833;  break;
    }
}

uint16_t DshotDriver::encode_packet(uint16_t value, bool telemetry) const {
    uint16_t packet = (value << 1) | (telemetry ? 1 : 0);
    // CRC: XOR of three nibbles
    uint16_t crc = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
    return (packet << 4) | crc;
}

void DshotDriver::transmit_frame(uint8_t /*channel*/, uint16_t /*packet*/) {
    // In production: PWM DMA or timer-based bit-bang via HAL PWM
    // Placeholder for HAL integration
}

bool DshotDriver::init() {
    set_status(DriverStatus::RUNNING);
    m_throttle.fill(0);
    return true;
}

void DshotDriver::arm_all() {
    for (int i = 0; i < 300; i++) {
        for (uint8_t ch = 0; ch < MAX_CHANNELS; ch++) {
            transmit_frame(ch, encode_packet(0, false));
        }
        apx::kernel::delay_ms(1);
    }
    m_armed = true;
}

void DshotDriver::set_throttle(uint8_t channel, uint16_t value_0_2047) {
    if (channel >= MAX_CHANNELS || !m_armed) return;
    // Throttle values 48-2047, 0 = disarm signal
    m_throttle[channel] = (value_0_2047 == 0) ? 0 : (48 + value_0_2047);
}

void DshotDriver::send_command(uint8_t channel, DshotCmd cmd, bool telemetry_req) {
    if (channel >= MAX_CHANNELS) return;
    transmit_frame(channel, encode_packet(static_cast<uint16_t>(cmd), telemetry_req));
}

void DshotDriver::update() {
    for (uint8_t ch = 0; ch < MAX_CHANNELS; ch++) {
        transmit_frame(ch, encode_packet(m_throttle[ch], false));
    }
    mark_updated();
}

} // namespace apx::drivers
