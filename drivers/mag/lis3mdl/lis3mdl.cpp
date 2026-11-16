// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "lis3mdl.hpp"
#include "apx/kernel/timer.hpp"
namespace apx::drivers {
Lis3mdl::Lis3mdl(apx::hal::IHalI2c& i2c, uint8_t addr) : DriverBase("lis3mdl"), m_i2c(i2c), m_addr(addr) {}
bool Lis3mdl::init() {
    set_status(DriverStatus::INITIALIZING);
    uint8_t id=0;
    if(!m_i2c.read_reg(m_addr,Lis3mdlReg::WHO_AM_I,&id,1)||id!=0x3D){set_status(DriverStatus::FAILED);return false;}
    // Ultra-high performance XY, ODR 80Hz, enable temp sensor
    uint8_t v=0x7C; m_i2c.write_reg(m_addr,Lis3mdlReg::CTRL1,&v,1);
    v=0x00; m_i2c.write_reg(m_addr,Lis3mdlReg::CTRL2,&v,1); // +-4 Gauss
    v=0x00; m_i2c.write_reg(m_addr,Lis3mdlReg::CTRL3,&v,1); // continuous
    v=0x0C; m_i2c.write_reg(m_addr,Lis3mdlReg::CTRL4,&v,1); // ultra-high Z
    set_status(DriverStatus::RUNNING);
    return true;
}
void Lis3mdl::update() {
    uint8_t buf[6]={};
    if(!m_i2c.read_reg(m_addr,Lis3mdlReg::OUT_X_L|0x80,buf,6)) return;
    auto to16=[](uint8_t l,uint8_t h)->int16_t{return (int16_t)((uint16_t(h)<<8)|l);};
    for(int i=0;i<3;i++) m_data.mag_xyz[i]=float(to16(buf[2*i],buf[2*i+1]))*Lis3mdlReg::SENSITIVITY_4G;
    m_data.timestamp_us=apx::kernel::now_us();
    mark_updated();
    if(m_topic) m_topic->publish(m_data);
}
bool Lis3mdl::is_healthy() const { return get_status()==DriverStatus::RUNNING&&DriverBase::is_healthy(); }
} // namespace apx::drivers
