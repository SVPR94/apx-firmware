// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "ina226.hpp"
#include "apx/kernel/timer.hpp"
#include <algorithm>

namespace apx::drivers {

Ina226::Ina226(apx::hal::IHalI2c& i2c, uint8_t addr, float shunt_ohm, uint8_t cell_count)
    : DriverBase("ina226"), m_i2c(i2c), m_addr(addr),
      m_shunt_ohm(shunt_ohm), m_cell_count(cell_count) {}

bool Ina226::write_reg(uint8_t reg, uint16_t val) {
    uint8_t buf[2] = { static_cast<uint8_t>(val >> 8), static_cast<uint8_t>(val & 0xFF) };
    return m_i2c.write_reg(m_addr, reg, buf, 2);
}

bool Ina226::read_reg(uint8_t reg, uint16_t& val) {
    uint8_t buf[2] = {};
    if (!m_i2c.read_reg(m_addr, reg, buf, 2)) return false;
    val = (static_cast<uint16_t>(buf[0]) << 8) | buf[1];
    return true;
}

bool Ina226::init() {
    set_status(DriverStatus::INITIALIZING);

    uint16_t die_id = 0;
    if (!read_reg(Ina226Reg::DIE_ID, die_id) || die_id != Ina226Reg::DIE_ID_VAL) {
        set_status(DriverStatus::FAILED);
        return false;
    }

    // Config: 16 averages, 1.1ms conversion time for both bus and shunt
    write_reg(Ina226Reg::CONFIG, 0x4527);

    // Current LSB = max_current / 32768 — assume 200A max
    m_current_lsb = 200.0f / 32768.0f;
    uint16_t cal = static_cast<uint16_t>(0.00512f / (m_shunt_ohm * m_current_lsb));
    write_reg(Ina226Reg::CALIBRATION, cal);

    set_status(DriverStatus::RUNNING);
    return true;
}

float Ina226::voltage_to_remaining_pct(float voltage_per_cell) const {
    // Simple linear interpolation: 3.3V = 0%, 4.2V = 100%
    constexpr float kMinV = 3.3f, kMaxV = 4.2f;
    float pct = (voltage_per_cell - kMinV) / (kMaxV - kMinV) * 100.0f;
    return std::max(0.0f, std::min(100.0f, pct));
}

void Ina226::update() {
    uint16_t bus_v_raw = 0, current_raw = 0;
    if (!read_reg(Ina226Reg::BUS_V, bus_v_raw)) return;
    if (!read_reg(Ina226Reg::CURRENT, current_raw)) return;

    m_data.voltage_v = static_cast<float>(bus_v_raw) * 0.00125f; // 1.25mV/LSB
    m_data.current_a = static_cast<float>(static_cast<int16_t>(current_raw)) * m_current_lsb;
    m_data.power_w   = m_data.voltage_v * m_data.current_a;
    m_data.remaining_pct = voltage_to_remaining_pct(m_data.voltage_v / m_cell_count);
    m_data.timestamp_us  = apx::kernel::now_us();

    mark_updated();
    if (m_topic) m_topic->publish(m_data);
}

bool Ina226::is_healthy() const {
    return get_status() == DriverStatus::RUNNING && DriverBase::is_healthy();
}

} // namespace apx::drivers
