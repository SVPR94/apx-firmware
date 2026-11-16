// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once
#include "apx/drivers/driver_base.hpp"
#include "apx/hal/hal_spi.hpp"
#include "apx/kernel/ubus.hpp"
#include <cstdint>

namespace apx::drivers {

struct MagData {
    float    mag_xyz[3]; // Gauss
    uint64_t timestamp_us;
};

namespace Rm3100Reg {
    constexpr uint8_t POLL   = 0x00;
    constexpr uint8_t CMM    = 0x01;
    constexpr uint8_t CCPX   = 0x04;
    constexpr uint8_t CCPY   = 0x06;
    constexpr uint8_t CCPZ   = 0x08;
    constexpr uint8_t TMRC   = 0x0B;
    constexpr uint8_t MX     = 0x24;
    constexpr uint8_t STATUS = 0x34;
    constexpr uint8_t HSHAKE = 0x35;
    constexpr uint8_t REVID  = 0x36;
    constexpr uint8_t REVID_VAL = 0x22;
    constexpr uint8_t READ_FLAG = 0x80;
}

class Rm3100 : public DriverBase {
public:
    explicit Rm3100(apx::hal::IHalSpi& spi, uint8_t cs_id, uint16_t cycle_count = 200);
    bool init() override;
    void update() override;
    const MagData& get_data() const { return m_data; }
    void publish_to(apx::kernel::Topic<MagData>& topic) { m_topic = &topic; }
private:
    bool write_reg(uint8_t reg, uint8_t val);
    uint8_t read_reg(uint8_t reg);
    apx::hal::IHalSpi& m_spi;
    uint8_t  m_cs_id;
    uint16_t m_cycle_count;
    float    m_scale{0.0f};
    MagData  m_data{};
    apx::kernel::Topic<MagData>* m_topic{nullptr};
};

} // namespace apx::drivers
