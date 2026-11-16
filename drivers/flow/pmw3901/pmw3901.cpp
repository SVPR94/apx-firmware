// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "pmw3901.hpp"

namespace apx::drivers {

Pmw3901Driver::Pmw3901Driver(apx::hal::IHalSpi& spi) : m_spi(spi) {
}

bool Pmw3901Driver::init() {
    uint8_t prod_id = read_reg(REG_PRODUCT_ID);
    if (prod_id != 0x49) {
        return false;
    }
    
    // PMW3901 startup sequence magic registers
    write_reg(0x7F, 0x00);
    write_reg(0x61, 0xAD);
    write_reg(0x7F, 0x03);
    write_reg(0x40, 0x00);
    write_reg(0x7F, 0x05);
    write_reg(0x41, 0xB3);
    write_reg(0x43, 0xF1);
    write_reg(0x45, 0x14);
    write_reg(0x5B, 0x32);
    write_reg(0x5F, 0x34);
    write_reg(0x7B, 0x08);
    write_reg(0x7F, 0x06);
    write_reg(0x44, 0x1B);
    write_reg(0x40, 0xBF);
    write_reg(0x4E, 0x3F);
    write_reg(0x7F, 0x08);
    write_reg(0x65, 0x20);
    write_reg(0x6A, 0x18);
    write_reg(0x7F, 0x09);
    write_reg(0x4F, 0xAF);
    write_reg(0x5F, 0x40);
    write_reg(0x48, 0x80);
    write_reg(0x49, 0x80);
    write_reg(0x57, 0x77);
    write_reg(0x60, 0x78);
    write_reg(0x61, 0x78);
    write_reg(0x62, 0x08);
    write_reg(0x63, 0x50);
    write_reg(0x7F, 0x0A);
    write_reg(0x45, 0x60);
    write_reg(0x7F, 0x00);
    write_reg(0x4D, 0x11);
    write_reg(0x55, 0x80);
    write_reg(0x74, 0x1F);
    write_reg(0x75, 0x1F);
    write_reg(0x4A, 0x78);
    write_reg(0x4B, 0x78);
    write_reg(0x44, 0x08);
    write_reg(0x45, 0x50);
    write_reg(0x64, 0xFF);
    write_reg(0x65, 0x4F);
    write_reg(0x7F, 0x14);
    write_reg(0x65, 0x60);
    write_reg(0x66, 0x08);
    write_reg(0x63, 0x78);
    write_reg(0x7F, 0x15);
    write_reg(0x48, 0x58);
    write_reg(0x7F, 0x07);
    write_reg(0x41, 0x0D);
    write_reg(0x43, 0x14);
    write_reg(0x4B, 0x0E);
    write_reg(0x45, 0x0F);
    write_reg(0x44, 0x42);
    write_reg(0x4C, 0x80);
    write_reg(0x7F, 0x10);
    write_reg(0x5B, 0x02);
    write_reg(0x7F, 0x07);
    write_reg(0x40, 0x41);
    write_reg(0x70, 0x00);
    
    return true;
}

void Pmw3901Driver::update() {
    uint8_t motion = read_reg(REG_MOTION);
    (void)motion; // Ignore motion bit, just read delta directly
    
    int16_t dx = read_reg(REG_DELTA_X_L) | (read_reg(REG_DELTA_X_H) << 8);
    int16_t dy = read_reg(REG_DELTA_Y_L) | (read_reg(REG_DELTA_Y_H) << 8);
    m_data.quality = read_reg(REG_SQUAL);
    
    // Pixel to radians mapping
    // PMW3901 pixel angular resolution is ~1.7 mrad/pixel? Wait, standard is FOV / pixels.
    // Given description: assume height 1m, pixel angle 1.7deg/pixel -> 1.7 * pi / 180 = 0.0296706 rad
    constexpr float DEG_TO_RAD = 3.14159265f / 180.0f;
    constexpr float PIXEL_TO_RAD = 1.7f * DEG_TO_RAD;
    
    if (m_data.quality > 15) {
        m_data.flow_x_rad = dx * PIXEL_TO_RAD;
        m_data.flow_y_rad = dy * PIXEL_TO_RAD;
    } else {
        m_data.flow_x_rad = 0.0f;
        m_data.flow_y_rad = 0.0f;
    }
}

FlowData Pmw3901Driver::get_data() const {
    return m_data;
}

uint8_t Pmw3901Driver::read_reg(uint8_t reg) {
    m_spi.select();
    m_spi.transfer(reg & 0x7F);
    uint8_t val = m_spi.transfer(0x00);
    m_spi.deselect();
    return val;
}

void Pmw3901Driver::write_reg(uint8_t reg, uint8_t val) {
    m_spi.select();
    m_spi.transfer(reg | 0x80);
    m_spi.transfer(val);
    m_spi.deselect();
}

} // namespace apx::drivers
