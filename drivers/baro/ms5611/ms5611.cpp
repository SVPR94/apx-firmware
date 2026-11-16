// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include <time.h>
// SPDX-License-Identifier: CC-BY-NC-SA-4.0
// APX Flight Controller — MS5611 Barometer Driver
#include "ms5611.hpp"
#include "apx/kernel/timer.hpp"
#include <cmath>
#include <cstring>

namespace apx::drivers {

Ms5611::Ms5611(apx::hal::IHalSpi& spi, uint8_t cs_id)
    : DriverBase("ms5611"), m_spi(spi), m_cs_id(cs_id) {}

bool Ms5611::read_prom() {
    for (int i = 0; i < 8; i++) {
        uint8_t tx[3] = { static_cast<uint8_t>(Ms5611Cmd::PROM_READ + i * 2), 0, 0 };
        uint8_t rx[3] = {};
        m_spi.cs_select(m_cs_id);
        bool ok = m_spi.transfer(tx, rx, 3);
        m_spi.cs_deselect(m_cs_id);
        if (!ok) return false;
        m_prom[i] = (static_cast<uint16_t>(rx[1]) << 8) | rx[2];
    }
    return true;
}

void Ms5611::start_conversion(uint8_t cmd) {
    uint8_t tx[1] = { cmd };
    uint8_t rx[1] = {};
    m_spi.cs_select(m_cs_id);
    m_spi.transfer(tx, rx, 1);
    m_spi.cs_deselect(m_cs_id);
    m_conversion_start_us = apx::kernel::now_us();
}

uint32_t Ms5611::read_adc() {
    uint8_t tx[4] = { Ms5611Cmd::ADC_READ, 0, 0, 0 };
    uint8_t rx[4] = {};
    m_spi.cs_select(m_cs_id);
    m_spi.transfer(tx, rx, 4);
    m_spi.cs_deselect(m_cs_id);
    return (static_cast<uint32_t>(rx[1]) << 16)
         | (static_cast<uint32_t>(rx[2]) << 8)
         |  static_cast<uint32_t>(rx[3]);
}

void Ms5611::compute(uint32_t d1_raw, uint32_t d2_raw) {
    // Compensation per MS5611 datasheet
    int32_t dT    = static_cast<int32_t>(d2_raw) - (static_cast<int32_t>(m_prom[5]) << 8);
    int32_t TEMP  = 2000 + (static_cast<int64_t>(dT) * m_prom[6] >> 23);

    int64_t OFF   = (static_cast<int64_t>(m_prom[2]) << 16) + (static_cast<int64_t>(m_prom[4]) * dT >> 7);
    int64_t SENS  = (static_cast<int64_t>(m_prom[1]) << 15) + (static_cast<int64_t>(m_prom[3]) * dT >> 8);

    // Second order corrections
    int32_t T2 = 0;
    int64_t OFF2 = 0, SENS2 = 0;
    if (TEMP < 2000) {
        T2    = static_cast<int32_t>((static_cast<int64_t>(dT) * dT) >> 31);
        OFF2  = 5LL * (TEMP - 2000) * (TEMP - 2000) / 2;
        SENS2 = 5LL * (TEMP - 2000) * (TEMP - 2000) / 4;
        if (TEMP < -1500) {
            OFF2  += 7LL * (TEMP + 1500) * (TEMP + 1500);
            SENS2 += 11LL * (TEMP + 1500) * (TEMP + 1500) / 2;
        }
    }
    TEMP -= T2;
    OFF  -= OFF2;
    SENS -= SENS2;

    int32_t P = static_cast<int32_t>((static_cast<int64_t>(d1_raw) * SENS / 2097152LL - OFF) / 32768LL);

    m_data.temperature_c = static_cast<float>(TEMP) / 100.0f;
    m_data.pressure_pa   = static_cast<float>(P);
    m_data.altitude_m    = pressure_to_altitude(m_data.pressure_pa);
    m_data.timestamp_us  = apx::kernel::now_us();
}

float Ms5611::pressure_to_altitude(float pressure_pa) {
    return 44307.69396f * (1.0f - std::pow(pressure_pa / 101325.0f, 0.190284f));
}

bool Ms5611::init() {
    set_status(DriverStatus::INITIALIZING);
    uint8_t tx[1] = { Ms5611Cmd::RESET };
    uint8_t rx[1] = {};
    m_spi.cs_select(m_cs_id);
    m_spi.transfer(tx, rx, 1);
    m_spi.cs_deselect(m_cs_id);

    apx::kernel::delay_ms(4);

    if (!read_prom()) {
        set_status(DriverStatus::FAILED);
        return false;
    }

    start_conversion(Ms5611Cmd::CONVERT_D2); // start temp first
    m_state = State::WAIT_TEMP;
    set_status(DriverStatus::RUNNING);
    return true;
}

void Ms5611::update() {
    constexpr uint64_t kConversionTimeUs = 4100; // 4.1ms for OSR=1024

    uint64_t now = apx::kernel::now_us();
    if (now - m_conversion_start_us < kConversionTimeUs) {
        return; // conversion not ready
    }

    if (m_state == State::WAIT_TEMP) {
        uint32_t d2 = read_adc();
        m_d1_raw = d2; // store temporarily
        // We actually need D2 (temp raw) first
        // Restart for pressure
        start_conversion(Ms5611Cmd::CONVERT_D1);
        m_state = State::WAIT_PRESS;
    } else if (m_state == State::WAIT_PRESS) {
        uint32_t d1 = read_adc();
        uint32_t d2 = m_d1_raw; // was stored from previous cycle
        compute(d1, d2);
        mark_updated();
        if (m_topic) m_topic->publish(m_data);
        start_conversion(Ms5611Cmd::CONVERT_D2);
        m_state = State::WAIT_TEMP;
    }
}

bool Ms5611::is_healthy() const {
    return get_status() == DriverStatus::RUNNING && DriverBase::is_healthy();
}

} // namespace apx::drivers
