// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "bmi088.hpp"
#include "apx/kernel/timer.hpp"
#include <cstring>
#include <time.h>
namespace apx::drivers {
Bmi088::Bmi088(apx::hal::IHalSpi& spi, uint8_t cs_acc, uint8_t cs_gyr)
    : DriverBase("bmi088"), m_spi(spi), m_cs_acc(cs_acc), m_cs_gyr(cs_gyr) {}
bool Bmi088::write_acc(uint8_t r,uint8_t v){
    uint8_t tx[2]={uint8_t(r&0x7F),v},rx[2]={};
    m_spi.cs_select(m_cs_acc); bool ok=m_spi.transfer(tx,rx,2); m_spi.cs_deselect(m_cs_acc);
    return ok;
}
bool Bmi088::write_gyr(uint8_t r,uint8_t v){
    uint8_t tx[2]={uint8_t(r&0x7F),v},rx[2]={};
    m_spi.cs_select(m_cs_gyr); bool ok=m_spi.transfer(tx,rx,2); m_spi.cs_deselect(m_cs_gyr);
    return ok;
}
uint8_t Bmi088::read_acc(uint8_t r){
    // BMI088 accel SPI needs dummy byte after address
    uint8_t tx[3]={uint8_t(r|Bmi088AccReg::READ_FLAG),0,0},rx[3]={};
    m_spi.cs_select(m_cs_acc); m_spi.transfer(tx,rx,3); m_spi.cs_deselect(m_cs_acc);
    return rx[2]; // skip dummy
}
uint8_t Bmi088::read_gyr(uint8_t r){
    uint8_t tx[2]={uint8_t(r|Bmi088GyrReg::READ_FLAG),0},rx[2]={};
    m_spi.cs_select(m_cs_gyr); m_spi.transfer(tx,rx,2); m_spi.cs_deselect(m_cs_gyr);
    return rx[1];
}
bool Bmi088::burst_acc(uint8_t r,uint8_t* buf,int n){
    uint8_t tx[16]={uint8_t(r|Bmi088AccReg::READ_FLAG),0},rx[16]={};
    m_spi.cs_select(m_cs_acc); bool ok=m_spi.transfer(tx,rx,n+2); m_spi.cs_deselect(m_cs_acc);
    if (ok) {
        memcpy(buf, rx + 2, n);
    }
    return ok; // 2-byte skip (addr+dummy)
}
bool Bmi088::burst_gyr(uint8_t r,uint8_t* buf,int n){
    uint8_t tx[10]={uint8_t(r|Bmi088GyrReg::READ_FLAG)},rx[10]={};
    m_spi.cs_select(m_cs_gyr); bool ok=m_spi.transfer(tx,rx,n+1); m_spi.cs_deselect(m_cs_gyr);
    if (ok) {
        memcpy(buf, rx + 1, n);
    }
    return ok;
}
bool Bmi088::init(){
    set_status(DriverStatus::INITIALIZING);
    // Check accel WHO_AM_I
    if(read_acc(Bmi088AccReg::CHIP_ID)!=0x1E){ set_status(DriverStatus::FAILED); return false; }
    // Check gyro WHO_AM_I
    if(read_gyr(Bmi088GyrReg::CHIP_ID)!=0x0F){ set_status(DriverStatus::FAILED); return false; }
    // Accel: soft reset
    write_acc(Bmi088AccReg::SOFTRESET, 0xB6);
    apx::kernel::delay_ms(50);
    // Accel: active mode, range ±24g, ODR 1600Hz
    write_acc(Bmi088AccReg::PWR_CTRL, 0x04); // enable accel
    write_acc(Bmi088AccReg::PWR_CONF, 0x00); // active
    write_acc(Bmi088AccReg::CONF,     0xAC); // ODR=1600Hz, BWP=Normal
    write_acc(Bmi088AccReg::RANGE,    0x03); // ±24g
    // Gyro: soft reset
    write_gyr(Bmi088GyrReg::SOFTRESET, 0xB6);
    apx::kernel::delay_ms(30);
    // Gyro: ±2000dps, 1000Hz, normal mode
    write_gyr(Bmi088GyrReg::RANGE,     0x00);
    write_gyr(Bmi088GyrReg::BANDWIDTH, 0x02); // 1000Hz ODR, 116Hz filter
    write_gyr(Bmi088GyrReg::LPM1,      0x00); // normal mode
    apx::kernel::delay_ms(10);
    set_status(DriverStatus::RUNNING);
    return true;
}
void Bmi088::update(){
    uint8_t abuf[6]={}, gbuf[6]={};
    if(!burst_acc(Bmi088AccReg::DATA,abuf,6)) return;
    if(!burst_gyr(Bmi088GyrReg::DATA,gbuf,6)) return;
    auto to16=[](uint8_t l,uint8_t h)->int16_t{return (int16_t)((uint16_t(h)<<8)|l);};
    for(int i=0;i<3;i++){
        m_data.accel_xyz[i]=float(to16(abuf[2*i],abuf[2*i+1]))*m_accel_scale;
        m_data.gyro_xyz[i] =float(to16(gbuf[2*i],gbuf[2*i+1]))*m_gyro_scale;
    }
    m_data.temp_c=25.0f; // BMI088 has temp reg at 0x22 accel bank
    m_data.timestamp_us=apx::kernel::now_us();
    mark_updated();
    if(m_topic) m_topic->publish(m_data);
}
bool Bmi088::is_healthy() const {
    return get_status()==DriverStatus::RUNNING && DriverBase::is_healthy();
}
} // namespace apx::drivers
