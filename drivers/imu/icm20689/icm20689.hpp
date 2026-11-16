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

namespace Icm20689Reg {
    constexpr uint8_t SMPLRT_DIV   = 0x19;
    constexpr uint8_t CONFIG        = 0x1A;
    constexpr uint8_t GYRO_CONFIG   = 0x1B;
    constexpr uint8_t ACCEL_CONFIG  = 0x1C;
    constexpr uint8_t ACCEL_CONFIG2 = 0x1D;
    constexpr uint8_t ACCEL_XOUT_H  = 0x3B;
    constexpr uint8_t TEMP_OUT_H    = 0x41;
    constexpr uint8_t GYRO_XOUT_H   = 0x43;
    constexpr uint8_t PWR_MGMT_1    = 0x6B;
    constexpr uint8_t WHO_AM_I      = 0x75;
    constexpr uint8_t WHO_AM_I_VAL  = 0x98;
    constexpr uint8_t READ_FLAG     = 0x80;
}

class Icm20689 : public DriverBase {
public:
    Icm20689(apx::hal::IHalSpi& spi, uint8_t cs_id);
    bool init() override;
    void update() override;
    bool is_healthy() const override;
    const ImuData& get_data() const { return m_data; }
    void publish_to(apx::kernel::Topic<ImuData>& t) { m_topic=&t; }
private:
    bool write_reg(uint8_t reg, uint8_t val);
    uint8_t read_reg(uint8_t reg);
    bool burst_read(uint8_t reg, uint8_t* buf, int len);
    apx::hal::IHalSpi& m_spi;
    uint8_t m_cs_id;
    ImuData m_data{};
    apx::kernel::Topic<ImuData>* m_topic{nullptr};
    // ±16g, ±2000dps defaults
    float m_accel_scale{16.0f*9.80665f/32768.0f};
    float m_gyro_scale{2000.0f*3.14159265f/180.0f/32768.0f};
};
} // namespace apx::drivers
