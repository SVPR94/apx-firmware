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

namespace Bmi088AccReg {
    constexpr uint8_t CHIP_ID   = 0x00; // 0x1E
    constexpr uint8_t DATA      = 0x12; // ACC_X_LSB..ACC_Z_MSB (6 bytes)
    constexpr uint8_t CONF      = 0x40;
    constexpr uint8_t RANGE     = 0x41;
    constexpr uint8_t PWR_CTRL  = 0x7D;
    constexpr uint8_t PWR_CONF  = 0x7C;
    constexpr uint8_t SOFTRESET = 0x7E;
    constexpr uint8_t READ_FLAG = 0x80;
}
namespace Bmi088GyrReg {
    constexpr uint8_t CHIP_ID   = 0x00; // 0x0F
    constexpr uint8_t DATA      = 0x02; // RATE_X_LSB..RATE_Z_MSB (6 bytes)
    constexpr uint8_t RANGE     = 0x0F;
    constexpr uint8_t BANDWIDTH = 0x10;
    constexpr uint8_t LPM1      = 0x11;
    constexpr uint8_t SOFTRESET = 0x14;
    constexpr uint8_t READ_FLAG = 0x80;
}

class Bmi088 : public DriverBase {
public:
    // Two separate SPI CS lines: one for accel, one for gyro
    Bmi088(apx::hal::IHalSpi& spi, uint8_t cs_acc, uint8_t cs_gyr);
    bool init() override;
    void update() override;
    bool is_healthy() const override;
    const ImuData& get_data() const { return m_data; }
    void publish_to(apx::kernel::Topic<ImuData>& t) { m_topic=&t; }
private:
    bool write_acc(uint8_t r, uint8_t v);
    bool write_gyr(uint8_t r, uint8_t v);
    uint8_t read_acc(uint8_t r);
    uint8_t read_gyr(uint8_t r);
    bool burst_acc(uint8_t r, uint8_t* buf, int n);
    bool burst_gyr(uint8_t r, uint8_t* buf, int n);
    apx::hal::IHalSpi& m_spi;
    uint8_t m_cs_acc, m_cs_gyr;
    ImuData m_data{};
    apx::kernel::Topic<ImuData>* m_topic{nullptr};
    // 24g range => scale = 24*9.80665/32768
    float m_accel_scale{24.0f*9.80665f/32768.0f};
    // 2000dps => scale = 2000*pi/180/32768
    float m_gyro_scale{2000.0f*3.14159265f/180.0f/32768.0f};
};
} // namespace apx::drivers
