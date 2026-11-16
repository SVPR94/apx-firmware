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

struct BaroData {
    float pressure_pa;
    float temperature_c;
    float altitude_m;
    uint64_t timestamp_us;
};

namespace Ms5611Cmd {
    constexpr uint8_t RESET      = 0x1E;
    constexpr uint8_t CONVERT_D1 = 0x48; // OSR=1024
    constexpr uint8_t CONVERT_D2 = 0x58; // OSR=1024
    constexpr uint8_t ADC_READ   = 0x00;
    constexpr uint8_t PROM_READ  = 0xA0;
}

class Ms5611 : public DriverBase {
public:
    explicit Ms5611(apx::hal::IHalSpi& spi, uint8_t cs_id);
    bool init() override;
    void update() override;
    bool is_healthy() const override;
    const BaroData& get_data() const { return m_data; }
    void publish_to(apx::kernel::Topic<BaroData>& topic) { m_topic = &topic; }
private:
    enum class State { IDLE, WAIT_TEMP, WAIT_PRESS };
    bool read_prom();
    void start_conversion(uint8_t cmd);
    uint32_t read_adc();
    void compute(uint32_t d1_raw, uint32_t d2_raw);
    static float pressure_to_altitude(float pressure_pa);
    apx::hal::IHalSpi& m_spi;
    uint8_t  m_cs_id;
    uint16_t m_prom[8]{};
    State    m_state{State::IDLE};
    uint64_t m_conversion_start_us{0};
    uint32_t m_d1_raw{0};
    BaroData m_data{};
    apx::kernel::Topic<BaroData>* m_topic{nullptr};
};
} // namespace apx::drivers
