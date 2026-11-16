// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "icm20689.hpp"
#include "apx/kernel/timer.hpp"
#include <cstring>
#include <time.h>
namespace apx::drivers {
Icm20689::Icm20689(apx::hal::IHalSpi& spi, uint8_t cs_id)
    : DriverBase("icm20689"), m_spi(spi), m_cs_id(cs_id) {}
bool Icm20689::write_reg(uint8_t r, uint8_t v) {
    uint8_t tx[2]={uint8_t(r&0x7F),v}, rx[2]={};
    m_spi.cs_select(m_cs_id); bool ok=m_spi.transfer(tx,rx,2); m_spi.cs_deselect(m_cs_id);
    return ok;
}
uint8_t Icm20689::read_reg(uint8_t r) {
    uint8_t tx[2]={uint8_t(r|Icm20689Reg::READ_FLAG),0}, rx[2]={};
    m_spi.cs_select(m_cs_id); m_spi.transfer(tx,rx,2); m_spi.cs_deselect(m_cs_id);
    return rx[1];
}
bool Icm20689::burst_read(uint8_t reg, uint8_t* buf, int len) {
    uint8_t tx[15]={uint8_t(reg|Icm20689Reg::READ_FLAG)};
    uint8_t rx[15]={};
    m_spi.cs_select(m_cs_id); bool ok=m_spi.transfer(tx,rx,len+1); m_spi.cs_deselect(m_cs_id);
    if(ok) memcpy(buf,rx+1,len);
    return ok;
}
bool Icm20689::init() {
    set_status(DriverStatus::INITIALIZING);
    if(read_reg(Icm20689Reg::WHO_AM_I)!=Icm20689Reg::WHO_AM_I_VAL){
        set_status(DriverStatus::FAILED); return false;
    }
    write_reg(Icm20689Reg::PWR_MGMT_1, 0x80); // reset
    apx::kernel::delay_ms(100);
    write_reg(Icm20689Reg::PWR_MGMT_1, 0x01); // clock auto
    write_reg(Icm20689Reg::SMPLRT_DIV,  0x00); // 1kHz
    write_reg(Icm20689Reg::GYRO_CONFIG,  0x18); // ±2000 dps
    write_reg(Icm20689Reg::ACCEL_CONFIG, 0x18); // ±16 g
    write_reg(Icm20689Reg::ACCEL_CONFIG2,0x08); // LPF 460Hz
    apx::kernel::delay_ms(10);
    set_status(DriverStatus::RUNNING);
    return true;
}
void Icm20689::update() {
    uint8_t buf[14]={};
    // Read 14 bytes: ACCEL_XOUT_H..GYRO_ZOUT_L (skips TEMP for burst efficiency)
    if(!burst_read(Icm20689Reg::ACCEL_XOUT_H,buf,14)) return;
    auto to16=[](uint8_t h,uint8_t l)->int16_t{return (int16_t)((uint16_t(h)<<8)|l);};
    for(int i=0;i<3;i++) m_data.accel_xyz[i]=float(to16(buf[2*i],buf[2*i+1]))*m_accel_scale;
    m_data.temp_c = float(to16(buf[6],buf[7]))/340.0f+36.53f;
    for(int i=0;i<3;i++) m_data.gyro_xyz[i]=float(to16(buf[8+2*i],buf[9+2*i]))*m_gyro_scale;
    m_data.timestamp_us=apx::kernel::now_us();
    mark_updated();
    if(m_topic) m_topic->publish(m_data);
}
bool Icm20689::is_healthy() const {
    return get_status()==DriverStatus::RUNNING && DriverBase::is_healthy();
}
} // namespace apx::drivers
