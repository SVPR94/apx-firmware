// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "ist8310.hpp"
#include "apx/kernel/timer.hpp"
#include <time.h>

namespace apx::drivers {
Ist8310::Ist8310(apx::hal::IHalI2c& i2c, uint8_t addr) : DriverBase("ist8310"), m_i2c(i2c), m_addr(addr) {}
bool Ist8310::write_reg(uint8_t r, uint8_t v) { return m_i2c.write_reg(m_addr, r, &v, 1); }
bool Ist8310::read_regs(uint8_t r, uint8_t* buf, int n) { return m_i2c.read_reg(m_addr, r, buf, n); }
bool Ist8310::init() {
    set_status(DriverStatus::INITIALIZING);
    uint8_t id=0;
    if(!m_i2c.read_reg(m_addr, Ist8310Reg::WHO_AM_I, &id, 1) || id!=0x10) {
        set_status(DriverStatus::FAILED); return false;
    }
    write_reg(Ist8310Reg::CNTL2, 0x01); // soft reset
    apx::kernel::delay_ms(20);
    write_reg(Ist8310Reg::AVGCNTL, 0xC0); // 16-sample average
    write_reg(Ist8310Reg::CNTL1,   0x09); // continuous 100Hz
    set_status(DriverStatus::RUNNING);
    return true;
}
void Ist8310::update() {
    uint8_t stat=0;
    if(!m_i2c.read_reg(m_addr, Ist8310Reg::STAT1, &stat, 1)) return;
    if(!(stat&0x01)) return; // data not ready
    uint8_t buf[6]={};
    if(!m_i2c.read_reg(m_addr, Ist8310Reg::DATAXL, buf, 6)) return;
    auto to16=[](uint8_t l,uint8_t h)->int16_t{return (int16_t)((uint16_t(h)<<8)|l);};
    for(int i=0;i<3;i++) m_data.mag_xyz[i]=float(to16(buf[2*i],buf[2*i+1]))*Ist8310Reg::SENSITIVITY;
    m_data.timestamp_us=apx::kernel::now_us();
    mark_updated();
    if(m_topic) m_topic->publish(m_data);
}
bool Ist8310::is_healthy() const { return get_status()==DriverStatus::RUNNING && DriverBase::is_healthy(); }
} // namespace apx::drivers
