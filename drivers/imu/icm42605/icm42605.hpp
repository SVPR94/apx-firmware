// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once
#include "apx/drivers/driver_base.hpp"
#include "imu/icm42688/icm42688.hpp"  // reuse ImuData
#include "apx/hal/hal_spi.hpp"
#include "apx/kernel/ubus.hpp"
#include <cstdint>

namespace apx::drivers {

namespace Icm42605Reg {
    constexpr uint8_t DEVICE_CONFIG = 0x11;
    constexpr uint8_t INT_STATUS    = 0x2D;
    constexpr uint8_t TEMP_DATA1    = 0x1D;
    constexpr uint8_t ACCEL_DATA_X1 = 0x1F;
    constexpr uint8_t GYRO_DATA_X1  = 0x25;
    constexpr uint8_t PWR_MGMT0     = 0x4E;
    constexpr uint8_t GYRO_CONFIG0  = 0x4F;
    constexpr uint8_t ACCEL_CONFIG0 = 0x50;
    constexpr uint8_t FIFO_CONFIG   = 0x16;
    constexpr uint8_t FIFO_CONFIG1  = 0x5F;
    constexpr uint8_t FIFO_COUNT_H  = 0x2E;
    constexpr uint8_t FIFO_DATA     = 0x30;
    constexpr uint8_t WHO_AM_I      = 0x75;
    constexpr uint8_t WHO_AM_I_VAL  = 0x42;
    constexpr uint8_t READ_FLAG     = 0x80;
}

class Icm42605 : public DriverBase {
public:
    Icm42605(apx::hal::IHalSpi& spi, uint8_t cs_id);
    bool init() override;
    void update() override;
    bool is_healthy() const override;
    const ImuData& get_data() const { return m_data; }
    void publish_to(apx::kernel::Topic<ImuData>& t) { m_topic = &t; }
private:
    bool write_reg(uint8_t reg, uint8_t val);
    uint8_t read_reg(uint8_t reg);
    apx::hal::IHalSpi& m_spi;
    uint8_t m_cs_id;
    ImuData m_data{};
    apx::kernel::Topic<ImuData>* m_topic{nullptr};
    static constexpr float kG = 9.80665f;
    static constexpr float kDegToRad = 3.14159265f / 180.0f;
    float m_accel_scale{16.0f * kG / 32768.0f};
    float m_gyro_scale{2000.0f * kDegToRad / 32768.0f};
};
} // namespace apx::drivers
