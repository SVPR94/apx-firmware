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
struct MagData { float mag_xyz[3]; uint64_t timestamp_us; }; // Gauss

namespace Ist8310Reg {
    constexpr uint8_t WHO_AM_I = 0x00; // 0x10
    constexpr uint8_t STAT1    = 0x02; // bit0=data ready
    constexpr uint8_t DATAXL   = 0x03;
    constexpr uint8_t DATAXH   = 0x04;
    constexpr uint8_t DATAYL   = 0x05;
    constexpr uint8_t DATAYH   = 0x06;
    constexpr uint8_t DATAZL   = 0x07;
    constexpr uint8_t DATAZH   = 0x08;
    constexpr uint8_t CNTL1    = 0x0A; // 0x09=100Hz continuous
    constexpr uint8_t CNTL2    = 0x0B; // 0x01=soft reset
    constexpr uint8_t AVGCNTL  = 0x41; // 0xC0=16 times average
    constexpr uint8_t DEFAULT_ADDR = 0x0E;
    constexpr float SENSITIVITY = 0.3e-4f; // 0.3 uT/LSB in Gauss
}

class Ist8310 : public DriverBase {
public:
    Ist8310(apx::hal::IHalI2c& i2c, uint8_t addr = Ist8310Reg::DEFAULT_ADDR);
    bool init() override;
    void update() override;
    bool is_healthy() const override;
    const MagData& get_data() const { return m_data; }
    void publish_to(apx::kernel::Topic<MagData>& t) { m_topic=&t; }
private:
    bool write_reg(uint8_t reg, uint8_t val);
    bool read_regs(uint8_t reg, uint8_t* buf, int len);
    apx::hal::IHalI2c& m_i2c;
    uint8_t m_addr;
    MagData m_data{};
    apx::kernel::Topic<MagData>* m_topic{nullptr};
};
} // namespace apx::drivers
