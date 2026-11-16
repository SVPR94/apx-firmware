// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "rm3100.hpp"
#include "apx/kernel/timer.hpp"

namespace apx::drivers {

Rm3100::Rm3100(apx::hal::IHalSpi& spi, uint8_t cs_id, uint16_t cycle_count)
    : DriverBase("rm3100"), m_spi(spi), m_cs_id(cs_id), m_cycle_count(cycle_count) {
    float s = 0.3671f * (m_cycle_count / 100.0f);
    m_scale = (s > 0.0f) ? (1.0f / s) : 1.0f;
}

bool Rm3100::write_reg(uint8_t reg, uint8_t val) {
    uint8_t tx[2] = {static_cast<uint8_t>(reg & 0x7F), val};
    uint8_t rx[2] = {};
    m_spi.cs_select(m_cs_id);
    bool ok = m_spi.transfer(tx, rx, 2);
    m_spi.cs_deselect(m_cs_id);
    return ok;
}

uint8_t Rm3100::read_reg(uint8_t reg) {
    uint8_t tx[2] = {static_cast<uint8_t>(reg | Rm3100Reg::READ_FLAG), 0};
    uint8_t rx[2] = {};
    m_spi.cs_select(m_cs_id);
    m_spi.transfer(tx, rx, 2);
    m_spi.cs_deselect(m_cs_id);
    return rx[1];
}

bool Rm3100::init() {
    set_status(DriverStatus::INITIALIZING);
    uint8_t whoami = read_reg(Rm3100Reg::REVID);
    if (whoami != Rm3100Reg::REVID_VAL) {
        set_status(DriverStatus::FAILED);
        return false;
    }

    write_reg(Rm3100Reg::CCPX, m_cycle_count >> 8);
    write_reg(Rm3100Reg::CCPX + 1, m_cycle_count & 0xFF);
    write_reg(Rm3100Reg::CCPY, m_cycle_count >> 8);
    write_reg(Rm3100Reg::CCPY + 1, m_cycle_count & 0xFF);
    write_reg(Rm3100Reg::CCPZ, m_cycle_count >> 8);
    write_reg(Rm3100Reg::CCPZ + 1, m_cycle_count & 0xFF);

    write_reg(Rm3100Reg::CMM, 0x71);
    set_status(DriverStatus::RUNNING);
    return true;
}

void Rm3100::update() {
    uint8_t status = read_reg(Rm3100Reg::STATUS);
    if (status & 0x80) {
        uint8_t tx[10] = {static_cast<uint8_t>(Rm3100Reg::MX | Rm3100Reg::READ_FLAG)};
        uint8_t rx[10] = {};
        m_spi.cs_select(m_cs_id);
        m_spi.transfer(tx, rx, 10);
        m_spi.cs_deselect(m_cs_id);

        int32_t x_raw = (rx[1] << 16) | (rx[2] << 8) | rx[3];
        if (x_raw & 0x800000) x_raw |= 0xFF000000;
        int32_t y_raw = (rx[4] << 16) | (rx[5] << 8) | rx[6];
        if (y_raw & 0x800000) y_raw |= 0xFF000000;
        int32_t z_raw = (rx[7] << 16) | (rx[8] << 8) | rx[9];
        if (z_raw & 0x800000) z_raw |= 0xFF000000;

        m_data.mag_xyz[0] = x_raw * m_scale;
        m_data.mag_xyz[1] = y_raw * m_scale;
        m_data.mag_xyz[2] = z_raw * m_scale;
        m_data.timestamp_us = apx::kernel::now_us();
        mark_updated();
        if (m_topic) m_topic->publish(m_data);
    }
}

} // namespace apx::drivers
