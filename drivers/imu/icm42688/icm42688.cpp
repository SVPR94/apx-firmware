// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include <time.h>
// SPDX-License-Identifier: CC-BY-NC-SA-4.0
// APX Flight Controller — ICM-42688-P IMU Driver Implementation
#include "icm42688.hpp"
#include "apx/kernel/timer.hpp"
#include <cstring>
#include <cmath>

namespace apx::drivers {

static constexpr float kG = 9.80665f;
static constexpr float kPi = 3.14159265358979f;
static constexpr float kDegToRad = kPi / 180.0f;

Icm42688::Icm42688(apx::hal::IHalSpi& spi, uint8_t cs_id, ImuOdr odr,
                   AccelRange accel_range, GyroRange gyro_range)
    : DriverBase("icm42688"), m_spi(spi), m_cs_id(cs_id),
      m_odr(odr), m_accel_range(accel_range), m_gyro_range(gyro_range)
{
    switch (accel_range) {
        case AccelRange::G2:  m_accel_scale = 2.0f  * kG / 32768.0f; break;
        case AccelRange::G4:  m_accel_scale = 4.0f  * kG / 32768.0f; break;
        case AccelRange::G8:  m_accel_scale = 8.0f  * kG / 32768.0f; break;
        case AccelRange::G16: m_accel_scale = 16.0f * kG / 32768.0f; break;
    }
    switch (gyro_range) {
        case GyroRange::DPS250:  m_gyro_scale = 250.0f  * kDegToRad / 32768.0f; break;
        case GyroRange::DPS500:  m_gyro_scale = 500.0f  * kDegToRad / 32768.0f; break;
        case GyroRange::DPS1000: m_gyro_scale = 1000.0f * kDegToRad / 32768.0f; break;
        case GyroRange::DPS2000: m_gyro_scale = 2000.0f * kDegToRad / 32768.0f; break;
    }
}

bool Icm42688::write_reg(uint8_t reg, uint8_t val) {
    uint8_t tx[2] = { static_cast<uint8_t>(reg & 0x7F), val };
    uint8_t rx[2] = {};
    m_spi.cs_select(m_cs_id);
    bool ok = m_spi.transfer(tx, rx, 2);
    m_spi.cs_deselect(m_cs_id);
    return ok;
}

uint8_t Icm42688::read_reg(uint8_t reg) {
    uint8_t tx[2] = { static_cast<uint8_t>(reg | Icm42688Reg::READ_FLAG), 0x00 };
    uint8_t rx[2] = {};
    m_spi.cs_select(m_cs_id);
    m_spi.transfer(tx, rx, 2);
    m_spi.cs_deselect(m_cs_id);
    return rx[1];
}

bool Icm42688::burst_read(uint8_t start_reg, uint8_t* buf, uint8_t len) {
    uint8_t tx[17] = {};
    uint8_t rx[17] = {};
    tx[0] = static_cast<uint8_t>(start_reg | Icm42688Reg::READ_FLAG);
    m_spi.cs_select(m_cs_id);
    bool ok = m_spi.transfer(tx, rx, len + 1);
    m_spi.cs_deselect(m_cs_id);
    if (ok) {
        memcpy(buf, &rx[1], len);
    }
    return ok;
}

bool Icm42688::init() {
    set_status(DriverStatus::INITIALIZING);

    if (read_reg(Icm42688Reg::WHO_AM_I) != Icm42688Reg::WHO_AM_I_VAL) {
        set_status(DriverStatus::FAILED);
        return false;
    }

    // Soft reset
    write_reg(Icm42688Reg::DEVICE_CONFIG, 0x01);
    // Wait 1ms for reset
    apx::kernel::delay_ms(1);

    // Set accel ODR and range: bits[6:5]=range, bits[3:0]=ODR
    uint8_t accel_cfg = 0;
    switch (m_accel_range) {
        case AccelRange::G2:  accel_cfg |= (0b11 << 5); break;
        case AccelRange::G4:  accel_cfg |= (0b10 << 5); break;
        case AccelRange::G8:  accel_cfg |= (0b01 << 5); break;
        case AccelRange::G16: accel_cfg |= (0b00 << 5); break;
    }
    accel_cfg |= (m_odr == ImuOdr::HZ_1000) ? 0x06 : (m_odr == ImuOdr::HZ_500) ? 0x07 : 0x09;
    write_reg(Icm42688Reg::ACCEL_CONFIG0, accel_cfg);

    // Set gyro ODR and range
    uint8_t gyro_cfg = 0;
    switch (m_gyro_range) {
        case GyroRange::DPS250:  gyro_cfg |= (0b11 << 5); break;
        case GyroRange::DPS500:  gyro_cfg |= (0b10 << 5); break;
        case GyroRange::DPS1000: gyro_cfg |= (0b01 << 5); break;
        case GyroRange::DPS2000: gyro_cfg |= (0b00 << 5); break;
    }
    gyro_cfg |= (m_odr == ImuOdr::HZ_1000) ? 0x06 : (m_odr == ImuOdr::HZ_500) ? 0x07 : 0x09;
    write_reg(Icm42688Reg::GYRO_CONFIG0, gyro_cfg);

    // Enable accel + gyro in low-noise mode
    write_reg(Icm42688Reg::PWR_MGMT0, 0x0F);

    // Wait for startup
    apx::kernel::delay_ms(50);

    set_status(DriverStatus::RUNNING);
    return true;
}

void Icm42688::deinit() {
    write_reg(Icm42688Reg::PWR_MGMT0, 0x00); // power down
    set_status(DriverStatus::DISABLED);
}

void Icm42688::update() {
    uint8_t buf[14] = {};
    // Read: TEMP_DATA1, TEMP_DATA0, ACCEL_X1..Z0, GYRO_X1..Z0
    if (!burst_read(Icm42688Reg::TEMP_DATA1, buf, 14)) {
        return;
    }

    auto to_int16 = [](uint8_t hi, uint8_t lo) -> int16_t {
        return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | lo);
    };

    m_data.temp_c = static_cast<float>(to_int16(buf[0], buf[1])) / 132.48f + 25.0f;
    m_data.accel_xyz[0] = static_cast<float>(to_int16(buf[2],  buf[3]))  * m_accel_scale;
    m_data.accel_xyz[1] = static_cast<float>(to_int16(buf[4],  buf[5]))  * m_accel_scale;
    m_data.accel_xyz[2] = static_cast<float>(to_int16(buf[6],  buf[7]))  * m_accel_scale;
    m_data.gyro_xyz[0]  = static_cast<float>(to_int16(buf[8],  buf[9]))  * m_gyro_scale;
    m_data.gyro_xyz[1]  = static_cast<float>(to_int16(buf[10], buf[11])) * m_gyro_scale;
    m_data.gyro_xyz[2]  = static_cast<float>(to_int16(buf[12], buf[13])) * m_gyro_scale;
    m_data.timestamp_us = apx::kernel::now_us();

    mark_updated();

    if (m_topic) {
        m_topic->publish(m_data);
    }
}

bool Icm42688::is_healthy() const {
    return get_status() == DriverStatus::RUNNING && DriverBase::is_healthy();
}

bool Icm42688::self_test() {
    // Simplified self-test: read WHO_AM_I
    return read_reg(Icm42688Reg::WHO_AM_I) == Icm42688Reg::WHO_AM_I_VAL;
}

} // namespace apx::drivers
