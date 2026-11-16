// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "icm42605.hpp"
#include "apx/kernel/timer.hpp"
#include <cstring>
#include <time.h>
namespace apx::drivers {
Icm42605::Icm42605(apx::hal::IHalSpi& spi, uint8_t cs_id)
    : DriverBase("icm42605"), m_spi(spi), m_cs_id(cs_id) {}
bool Icm42605::write_reg(uint8_t reg, uint8_t val) {
    uint8_t tx[2]={static_cast<uint8_t>(reg&0x7F),val}, rx[2]={};
    m_spi.cs_select(m_cs_id); bool ok=m_spi.transfer(tx,rx,2); m_spi.cs_deselect(m_cs_id);
    return ok;
}
uint8_t Icm42605::read_reg(uint8_t reg) {
    uint8_t tx[2]={static_cast<uint8_t>(reg|Icm42605Reg::READ_FLAG),0}, rx[2]={};
    m_spi.cs_select(m_cs_id); m_spi.transfer(tx,rx,2); m_spi.cs_deselect(m_cs_id);
    return rx[1];
}
bool Icm42605::init() {
    set_status(DriverStatus::INITIALIZING);
    if (read_reg(Icm42605Reg::WHO_AM_I) != Icm42605Reg::WHO_AM_I_VAL) {
        set_status(DriverStatus::FAILED); return false;
    }
    write_reg(Icm42605Reg::DEVICE_CONFIG, 0x01); // reset
    apx::kernel::delay_ms(2);
    write_reg(Icm42605Reg::GYRO_CONFIG0,  0x06); // ±2000dps, 1kHz ODR
    write_reg(Icm42605Reg::ACCEL_CONFIG0, 0x06); // ±16g, 1kHz ODR
    write_reg(Icm42605Reg::PWR_MGMT0, 0x0F);     // accel+gyro low-noise
    apx::kernel::delay_ms(50);
    set_status(DriverStatus::RUNNING);
    return true;
}
void Icm42605::update() {
    uint8_t tx[15]={static_cast<uint8_t>(Icm42605Reg::TEMP_DATA1|Icm42605Reg::READ_FLAG)};
    uint8_t rx[15]={};
    m_spi.cs_select(m_cs_id); m_spi.transfer(tx,rx,15); m_spi.cs_deselect(m_cs_id);
    auto to16=[](uint8_t h,uint8_t l)->int16_t{return (int16_t)((uint16_t(h)<<8)|l);};
    m_data.temp_c = float(to16(rx[1],rx[2]))/132.48f+25.0f;
    for(int i=0;i<3;i++) m_data.accel_xyz[i]=float(to16(rx[3+2*i],rx[4+2*i]))*m_accel_scale;
    for(int i=0;i<3;i++) m_data.gyro_xyz[i] =float(to16(rx[9+2*i],rx[10+2*i]))*m_gyro_scale;
    m_data.timestamp_us = apx::kernel::now_us();
    mark_updated();
    if(m_topic) m_topic->publish(m_data);
}
bool Icm42605::is_healthy() const {
    return get_status()==DriverStatus::RUNNING && DriverBase::is_healthy();
}
} // namespace apx::drivers
