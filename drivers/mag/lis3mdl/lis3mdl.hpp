// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once
#include "apx/drivers/driver_base.hpp"
#include "apx/hal/hal_i2c.hpp"
#include "mag/ist8310/ist8310.hpp"  // reuse MagData
#include "apx/kernel/ubus.hpp"
#include <cstdint>

namespace apx::drivers {
namespace Lis3mdlReg {
    constexpr uint8_t WHO_AM_I = 0x0F; // 0x3D
    constexpr uint8_t CTRL1    = 0x20;
    constexpr uint8_t CTRL2    = 0x21;
    constexpr uint8_t CTRL3    = 0x22;
    constexpr uint8_t CTRL4    = 0x23;
    constexpr uint8_t OUT_X_L  = 0x28;
    constexpr uint8_t DEFAULT_ADDR = 0x1E;
    constexpr float SENSITIVITY_4G = 1.0f / 6842.0f; // +-4 Gauss, Gauss/LSB
}
class Lis3mdl : public DriverBase {
public:
    Lis3mdl(apx::hal::IHalI2c& i2c, uint8_t addr = Lis3mdlReg::DEFAULT_ADDR);
    bool init() override;
    void update() override;
    bool is_healthy() const override;
    const MagData& get_data() const { return m_data; }
    void publish_to(apx::kernel::Topic<MagData>& t) { m_topic=&t; }
private:
    apx::hal::IHalI2c& m_i2c;
    uint8_t m_addr;
    MagData m_data{};
    apx::kernel::Topic<MagData>* m_topic{nullptr};
};
} // namespace apx::drivers
