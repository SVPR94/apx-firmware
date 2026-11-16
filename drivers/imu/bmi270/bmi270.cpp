// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "bmi270.hpp"
#include "apx/kernel/timer.hpp"
#include <cstring>
#include <time.h>
// Note: BMI270 requires a config blob upload to internal memory.
// The config blob is available from Bosch GitHub (open-source, BSD license).
// For APX, we use the minimal config that enables accel+gyro.
// The full config blob (8k bytes) would be loaded from flash/ROM in production.
static const uint8_t kBmi270ConfigMinimal[] = {0}; // placeholder
namespace apx::drivers {
Bmi270::Bmi270(apx::hal::IHalSpi& spi, uint8_t cs_id)
    : DriverBase("bmi270"), m_spi(spi), m_cs_id(cs_id) {}
bool Bmi270::write_reg(uint8_t r,uint8_t v){
    uint8_t tx[2]={uint8_t(r&0x7F),v},rx[2]={};
    m_spi.cs_select(m_cs_id); bool ok=m_spi.transfer(tx,rx,2); m_spi.cs_deselect(m_cs_id);
    return ok;
}
uint8_t Bmi270::read_reg(uint8_t r){
    uint8_t tx[3]={uint8_t(r|Bmi270Reg::READ_FLAG),0,0},rx[3]={};
    m_spi.cs_select(m_cs_id); m_spi.transfer(tx,rx,3); m_spi.cs_deselect(m_cs_id);
    return rx[2]; // dummy byte after address in SPI read
}
bool Bmi270::burst_read(uint8_t r,uint8_t* buf,int n){
    uint8_t tx[16]={uint8_t(r|Bmi270Reg::READ_FLAG),0},rx[16]={};
    m_spi.cs_select(m_cs_id); bool ok=m_spi.transfer(tx,rx,n+2); m_spi.cs_deselect(m_cs_id);
    if(ok) memcpy(buf,rx+2,n); return ok;
}
bool Bmi270::upload_config(){
    // Disable sensor before config load
    write_reg(Bmi270Reg::PWR_CTRL, 0x00);
    write_reg(Bmi270Reg::INIT_CTRL, 0x00);
    // In production: write config blob in 256-byte chunks to INIT_DATA
    // For now: skip (placeholder)
    write_reg(Bmi270Reg::INIT_CTRL, 0x01); // enable init
    apx::kernel::delay_ms(20);
    uint8_t status = read_reg(Bmi270Reg::INIT_CTRL);
    return (status & 0x01) != 0; // bit 0 set = init complete
}
bool Bmi270::init(){
    set_status(DriverStatus::INITIALIZING);
    if(read_reg(Bmi270Reg::CHIP_ID)!=0x24){ set_status(DriverStatus::FAILED); return false; }
    // Soft reset
    write_reg(Bmi270Reg::CMD, 0xB6);
    apx::kernel::delay_ms(10);
    // Upload config (required for feature engine)
    upload_config();
    // Enable accel + gyro
    write_reg(Bmi270Reg::PWR_CTRL, 0x0E); // bit1=acc, bit2=gyr, bit3=temp
    apx::kernel::delay_ms(50);
    // Configure: accel ±16g 1600Hz, gyro ±2000dps 1600Hz
    write_reg(Bmi270Reg::ACC_CONF,  0xA8); // BWP=osr4, ODR=1600Hz
    write_reg(Bmi270Reg::ACC_RANGE, 0x03); // ±16g
    write_reg(Bmi270Reg::GYR_CONF,  0xE8); // BWP=normal, perf=noise_opt, ODR=1600Hz
    write_reg(Bmi270Reg::GYR_RANGE, 0x00); // ±2000dps
    set_status(DriverStatus::RUNNING);
    return true;
}
void Bmi270::update(){
    uint8_t buf[12]={};
    if(!burst_read(Bmi270Reg::DATA_8, buf, 12)) return;
    auto to16=[](uint8_t l,uint8_t h)->int16_t{return (int16_t)((uint16_t(h)<<8)|l);};
    for(int i=0;i<3;i++) m_data.accel_xyz[i]=float(to16(buf[2*i],buf[2*i+1]))*m_accel_scale;
    for(int i=0;i<3;i++) m_data.gyro_xyz[i] =float(to16(buf[6+2*i],buf[7+2*i]))*m_gyro_scale;
    m_data.timestamp_us=apx::kernel::now_us();
    mark_updated();
    if(m_topic) m_topic->publish(m_data);
}
bool Bmi270::is_healthy() const {
    return get_status()==DriverStatus::RUNNING && DriverBase::is_healthy();
}
} // namespace apx::drivers
