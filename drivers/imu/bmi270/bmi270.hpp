// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once
#include "apx/drivers/driver_base.hpp"
#include "imu/icm42688/icm42688.hpp"
#include "apx/hal/hal_spi.hpp"
#include "apx/kernel/ubus.hpp"
#include <cstdint>

namespace apx::drivers {

namespace Bmi270Reg {
    constexpr uint8_t CHIP_ID     = 0x00; // 0x24
    constexpr uint8_t ERR_REG     = 0x02;
    constexpr uint8_t STATUS      = 0x03;
    constexpr uint8_t DATA_8      = 0x0C; // ACC_X_LSB, 6 bytes
    constexpr uint8_t DATA_14     = 0x12; // GYR_X_LSB, 6 bytes
    constexpr uint8_t ACC_CONF    = 0x40;
    constexpr uint8_t ACC_RANGE   = 0x41;
    constexpr uint8_t GYR_CONF    = 0x42;
    constexpr uint8_t GYR_RANGE   = 0x43;
    constexpr uint8_t PWR_CTRL    = 0x7D;
    constexpr uint8_t CMD         = 0x7E;
    constexpr uint8_t INIT_CTRL   = 0x59;
    constexpr uint8_t INIT_DATA   = 0x5E;
    constexpr uint8_t INIT_ADDR_0 = 0x5B;
    constexpr uint8_t INIT_ADDR_1 = 0x5C;
    constexpr uint8_t READ_FLAG   = 0x80;
}

class Bmi270 : public DriverBase {
public:
    Bmi270(apx::hal::IHalSpi& spi, uint8_t cs_id);
    bool init() override;
    void update() override;
    bool is_healthy() const override;
    const ImuData& get_data() const { return m_data; }
    void publish_to(apx::kernel::Topic<ImuData>& t) { m_topic=&t; }
private:
    bool write_reg(uint8_t r, uint8_t v);
    uint8_t read_reg(uint8_t r);
    bool burst_read(uint8_t r, uint8_t* buf, int n);
    bool upload_config();
    apx::hal::IHalSpi& m_spi;
    uint8_t m_cs_id;
    ImuData m_data{};
    apx::kernel::Topic<ImuData>* m_topic{nullptr};
    float m_accel_scale{16.0f*9.80665f/32768.0f};
    float m_gyro_scale{2000.0f*3.14159265f/180.0f/32768.0f};
};
} // namespace apx::drivers
