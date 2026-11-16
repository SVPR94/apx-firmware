// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include "../../include/apx/drivers/driver_base.hpp"
#include <cstdint>

namespace apx::hal {
    class IHalSpi {
    public:
        virtual ~IHalSpi() = default;
        virtual uint8_t transfer(uint8_t data) = 0;
        virtual void select() = 0;
        virtual void deselect() = 0;
    };
}

namespace apx::drivers {

struct FlowData {
    float flow_x_rad;
    float flow_y_rad;
    uint8_t quality;
    uint64_t timestamp_us;
};

class Pmw3901Driver : public DriverBase {
public:
    explicit Pmw3901Driver(apx::hal::IHalSpi& spi);
    bool init() override;
    void update() override;
    FlowData get_data() const;

private:
    apx::hal::IHalSpi& m_spi;
    FlowData m_data{};
    
    static constexpr uint8_t REG_PRODUCT_ID = 0x00;
    static constexpr uint8_t REG_REVISION_ID = 0x01;
    static constexpr uint8_t REG_MOTION = 0x02;
    static constexpr uint8_t REG_DELTA_X_L = 0x03;
    static constexpr uint8_t REG_DELTA_X_H = 0x04;
    static constexpr uint8_t REG_DELTA_Y_L = 0x05;
    static constexpr uint8_t REG_DELTA_Y_H = 0x06;
    static constexpr uint8_t REG_SQUAL = 0x07;
    
    uint8_t read_reg(uint8_t reg);
    void write_reg(uint8_t reg, uint8_t val);
};

} // namespace apx::drivers
