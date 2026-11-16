// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once
#include "apx/drivers/driver_base.hpp"
#include "apx/hal/hal_spi.hpp"
#include "apx/kernel/ubus.hpp"
#include <cstdint>
#include <array>

namespace apx::drivers {

struct ImuData {
    float accel_xyz[3]; // m/s^2
    float gyro_xyz[3];  // rad/s
    float temp_c;
    uint64_t timestamp_us;
};

namespace Icm42688Reg {
    constexpr uint8_t DEVICE_CONFIG = 0x11;
    constexpr uint8_t TEMP_DATA1   = 0x1D;
    constexpr uint8_t ACCEL_DATA_X1= 0x1F;
    constexpr uint8_t GYRO_DATA_X1 = 0x25;
    constexpr uint8_t PWR_MGMT0    = 0x4E;
    constexpr uint8_t GYRO_CONFIG0 = 0x4F;
    constexpr uint8_t ACCEL_CONFIG0= 0x50;
    constexpr uint8_t WHO_AM_I     = 0x75;
    constexpr uint8_t WHO_AM_I_VAL = 0x47;
    constexpr uint8_t READ_FLAG    = 0x80;
}

enum class ImuOdr    { HZ_1000, HZ_500, HZ_200 };
enum class AccelRange{ G2, G4, G8, G16 };
enum class GyroRange { DPS250, DPS500, DPS1000, DPS2000 };

class Icm42688 : public DriverBase {
public:
    explicit Icm42688(apx::hal::IHalSpi& spi, uint8_t cs_id,
                      ImuOdr odr = ImuOdr::HZ_1000,
                      AccelRange accel_range = AccelRange::G16,
                      GyroRange  gyro_range  = GyroRange::DPS2000);
    bool init() override;
    void deinit() override;
    void update() override;
    bool is_healthy() const override;
    const ImuData& get_data() const { return m_data; }
    bool self_test();
    void publish_to(apx::kernel::Topic<ImuData>& topic) { m_topic = &topic; }
private:
    bool write_reg(uint8_t reg, uint8_t val);
    uint8_t read_reg(uint8_t reg);
    bool burst_read(uint8_t start_reg, uint8_t* buf, uint8_t len);
    apx::hal::IHalSpi& m_spi;
    uint8_t    m_cs_id;
    ImuOdr     m_odr;
    AccelRange m_accel_range;
    GyroRange  m_gyro_range;
    float      m_accel_scale{0.0f};
    float      m_gyro_scale{0.0f};
    ImuData    m_data{};
    apx::kernel::Topic<ImuData>* m_topic{nullptr};
};
} // namespace apx::drivers
