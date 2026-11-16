// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "ublox_f9p.hpp"
#include "apx/kernel/timer.hpp"
#include <cstring>
#include <cmath>
#include <time.h>

namespace apx::drivers {

static constexpr uint8_t UBX_SYNC1 = 0xB5, UBX_SYNC2 = 0x62;
static constexpr uint8_t UBX_CLS_CFG = 0x06, UBX_CLS_NAV = 0x01, UBX_CLS_ACK = 0x05;
static constexpr uint8_t UBX_ID_VALSET = 0x8A, UBX_ID_NAVPVT = 0x07, UBX_ID_RELPOSNED = 0x3C;

UbloxF9P::UbloxF9P(apx::hal::IHalUart& uart, uint32_t baud)
    : DriverBase("ublox_f9p"), m_uart(uart), m_baud(baud) {}

bool UbloxF9P::send_ubx(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len) {
    uint8_t ck_a=0, ck_b=0;
    auto chk=[&](uint8_t b){ ck_a+=b; ck_b+=ck_a; };
    chk(cls); chk(id); chk(uint8_t(len)); chk(uint8_t(len>>8));
    for(int i=0;i<len;i++) chk(payload[i]);

    uint8_t hdr[6]={UBX_SYNC1,UBX_SYNC2,cls,id,uint8_t(len),uint8_t(len>>8)};
    m_uart.write(hdr,6);
    if(len) m_uart.write(payload,len);
    uint8_t trailer[2]={ck_a,ck_b};
    m_uart.write(trailer,2);
    return true;
}

bool UbloxF9P::send_valset_u4(uint32_t key, uint32_t value) {
    uint8_t payload[12] = {
        0x01, 0x01, 0x00, 0x00, // version=1, layers=RAM+BBR
        uint8_t(key), uint8_t(key>>8), uint8_t(key>>16), uint8_t(key>>24),
        uint8_t(value), uint8_t(value>>8), uint8_t(value>>16), uint8_t(value>>24)
    };
    return send_ubx(UBX_CLS_CFG, UBX_ID_VALSET, payload, 12);
}

bool UbloxF9P::send_valset_u1(uint32_t key, uint8_t value) {
    uint8_t payload[9] = {
        0x01, 0x01, 0x00, 0x00,
        uint8_t(key), uint8_t(key>>8), uint8_t(key>>16), uint8_t(key>>24),
        value
    };
    return send_ubx(UBX_CLS_CFG, UBX_ID_VALSET, payload, 9);
}

bool UbloxF9P::init() {
    set_status(DriverStatus::INITIALIZING);
    if(!m_uart.init(m_baud)) { set_status(DriverStatus::FAILED); return false; }

    apx::kernel::delay_ms(100);

    // Configure UART baud
    send_valset_u4(F9PKey::UART1_BAUDRATE, m_baud);
    // Set measurement rate to 10Hz (100ms)
    send_valset_u4(F9PKey::RATE_MEAS, 100);
    // Enable RTK DGNSS mode
    send_valset_u1(F9PKey::NAVHPG_DGNSSMODE, 3);
    // Enable NAV-PVT and NAV-RELPOSNED output
    send_valset_u1(F9PKey::MSGOUT_PVT_UART1, 1);
    send_valset_u1(F9PKey::MSGOUT_RELPOS_UART1, 1);

    set_status(DriverStatus::RUNNING);
    return true;
}

bool UbloxF9P::parse_byte(uint8_t b) {
    switch(m_parse_state) {
        case ParseState::SYNC1:   if(b==UBX_SYNC1) m_parse_state=ParseState::SYNC2; break;
        case ParseState::SYNC2:   if(b==UBX_SYNC2){m_parse_state=ParseState::CLASS;} else m_parse_state=ParseState::SYNC1; break;
        case ParseState::CLASS:   m_msg_class=b; m_ck_a=b; m_ck_b=b; m_parse_state=ParseState::ID; break;
        case ParseState::ID:      m_msg_id=b; m_ck_a+=b; m_ck_b+=m_ck_a; m_parse_state=ParseState::LEN1; break;
        case ParseState::LEN1:    m_payload_len=b; m_ck_a+=b; m_ck_b+=m_ck_a; m_parse_state=ParseState::LEN2; break;
        case ParseState::LEN2:    m_payload_len|=uint16_t(b)<<8; m_ck_a+=b; m_ck_b+=m_ck_a; m_payload_idx=0; m_parse_state=ParseState::PAYLOAD; break;
        case ParseState::PAYLOAD:
            if(m_payload_idx<MAX_PAYLOAD) m_payload_buf[m_payload_idx]=b;
            m_ck_a+=b; m_ck_b+=m_ck_a;
            if(++m_payload_idx>=m_payload_len) m_parse_state=ParseState::CK_A;
            break;
        case ParseState::CK_A:    if(b!=m_ck_a){m_parse_state=ParseState::SYNC1;} else m_parse_state=ParseState::CK_B; break;
        case ParseState::CK_B:
            m_parse_state=ParseState::SYNC1;
            if(b==m_ck_b) {
                if(m_msg_class==UBX_CLS_NAV && m_msg_id==UBX_ID_NAVPVT)    { parse_nav_pvt(m_payload_buf); return true; }
                if(m_msg_class==UBX_CLS_NAV && m_msg_id==UBX_ID_RELPOSNED) { parse_nav_relposned(m_payload_buf); return true; }
            }
            break;
    }
    return false;
}

void UbloxF9P::parse_nav_pvt(const uint8_t* p) {
    // NAV-PVT is 92 bytes (same as M9)
    m_rtk_data.fix_type     = static_cast<GpsFixType>(p[20]);
    m_rtk_data.fix_type_raw = p[20];
    m_rtk_data.satellites   = p[23];
    m_rtk_data.lat_deg  = (int32_t)(p[28]|(p[29]<<8)|(p[30]<<16)|(p[31]<<24)) * 1e-7;
    m_rtk_data.lon_deg  = (int32_t)(p[24]|(p[25]<<8)|(p[26]<<16)|(p[27]<<24)) * 1e-7;
    m_rtk_data.alt_msl_m= (int32_t)(p[36]|(p[37]<<8)|(p[38]<<16)|(p[39]<<24)) * 1e-3f;
    m_rtk_data.vel_n_ms = (int32_t)(p[48]|(p[49]<<8)|(p[50]<<16)|(p[51]<<24)) * 1e-3f;
    m_rtk_data.vel_e_ms = (int32_t)(p[52]|(p[53]<<8)|(p[54]<<16)|(p[55]<<24)) * 1e-3f;
    m_rtk_data.vel_d_ms = (int32_t)(p[56]|(p[57]<<8)|(p[58]<<16)|(p[59]<<24)) * 1e-3f;
    m_rtk_data.hdop     = (p[76]|(p[77]<<8)) * 0.01f;
    m_rtk_data.rtk_fixed = (p[20] == 6); // fix type 6 = fixed RTK
    m_rtk_data.valid    = (p[20] >= 3);
    m_rtk_data.timestamp_us = apx::kernel::now_us();
}

void UbloxF9P::parse_nav_relposned(const uint8_t* p) {
    // NAV-RELPOSNED: relative position NED from base in cm
    float rn = (int32_t)(p[8]|(p[9]<<8)|(p[10]<<16)|(p[11]<<24)) * 0.01f;
    float re = (int32_t)(p[12]|(p[13]<<8)|(p[14]<<16)|(p[15]<<24)) * 0.01f;
    float rd = (int32_t)(p[16]|(p[17]<<8)|(p[18]<<16)|(p[19]<<24)) * 0.01f;
    m_rtk_data.rel_pos_ned[0] = rn;
    m_rtk_data.rel_pos_ned[1] = re;
    m_rtk_data.rel_pos_ned[2] = rd;
    m_rtk_data.rel_pos_accuracy_m = (p[36]|(p[37]<<8)|(p[38]<<16)|(p[39]<<24)) * 1e-4f;
}

void UbloxF9P::inject_rtcm(const uint8_t* data, size_t len) {
    m_uart.write(data, (int)len);
}

void UbloxF9P::update() {
    uint8_t b=0;
    while(m_uart.available()>0) {
        m_uart.read(&b,1);
        if(parse_byte(b)) mark_updated();
    }
}

bool UbloxF9P::is_healthy() const {
    return get_status()==DriverStatus::RUNNING && DriverBase::is_healthy();
}

} // namespace apx::drivers
