// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once
#include "apx/drivers/driver_base.hpp"
#include "apx/hal/hal_i2c.hpp"
#include "apx/kernel/ubus.hpp"
#include <cstdint>

namespace apx::drivers {

struct BatteryData {
    float voltage_v;
    float current_a;
    float power_w;
    float remaining_pct;
    uint64_t timestamp_us;
};

namespace Ina226Reg {
    constexpr uint8_t CONFIG      = 0x00;
    constexpr uint8_t SHUNT_V     = 0x01;
    constexpr uint8_t BUS_V       = 0x02;
    constexpr uint8_t POWER       = 0x03;
    constexpr uint8_t CURRENT     = 0x04;
    constexpr uint8_t CALIBRATION = 0x05;
    constexpr uint8_t DIE_ID      = 0xFF;
    constexpr uint16_t DIE_ID_VAL = 0x2260;
}

class Ina226 : public DriverBase {
public:
    Ina226(apx::hal::IHalI2c& i2c, uint8_t addr,
           float shunt_ohm = 0.001f, uint8_t cell_count = 6);
    bool init() override;
    void update() override;
    bool is_healthy() const override;
    const BatteryData& get_data() const { return m_data; }
    void publish_to(apx::kernel::Topic<BatteryData>& topic) { m_topic = &topic; }
private:
    bool write_reg(uint8_t reg, uint16_t val);
    bool read_reg(uint8_t reg, uint16_t& val);
    float voltage_to_remaining_pct(float voltage_per_cell) const;
    apx::hal::IHalI2c& m_i2c;
    uint8_t  m_addr;
    float    m_shunt_ohm;
    uint8_t  m_cell_count;
    float    m_current_lsb{0.0f};
    BatteryData m_data{};
    apx::kernel::Topic<BatteryData>* m_topic{nullptr};
};
} // namespace apx::drivers
