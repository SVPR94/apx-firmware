// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "ublox_driver.hpp"
#include <cstring>
#include <chrono>

namespace apx::drivers {

UbloxDriver::UbloxDriver(apx::hal::IHalUart& uart) : DriverBase("ublox"), m_uart(uart) {
}

bool UbloxDriver::init() {
    // We expect the UART to be correctly set by system initialization, but we re-init
    m_uart.set_baudrate(38400);

    // CFG-PRT (0x06 0x00) - Set UART1 to 38400, 8N1
    uint8_t cfg_prt[20] = {0};
    cfg_prt[0] = 1; // UART1
    cfg_prt[4] = 0xC0; // 8N1
    cfg_prt[5] = 0x08;
    cfg_prt[8] = 0x00; // 38400 baud
    cfg_prt[9] = 0x96;
    cfg_prt[12] = 0x07; // inProtoMask (UBX+NMEA+RTCM)
    cfg_prt[14] = 0x01; // outProtoMask (UBX)
    send_ubx(UBX_CLASS_CFG, UBX_CFG_PRT, cfg_prt, sizeof(cfg_prt));

    // CFG-RATE (0x06 0x08) - 100ms meas rate (10Hz), 1 nav cycle
    uint8_t cfg_rate[6] = {0};
    cfg_rate[0] = 100; // measRate (ms)
    cfg_rate[1] = 0;
    cfg_rate[2] = 1;   // navRate (cycles)
    cfg_rate[3] = 0;
    cfg_rate[4] = 1;   // timeRef (GPS)
    cfg_rate[5] = 0;
    send_ubx(UBX_CLASS_CFG, UBX_CFG_RATE, cfg_rate, sizeof(cfg_rate));

    // CFG-MSG (0x06 0x01) - Enable NAV-PVT at rate 1
    uint8_t cfg_msg[3] = {UBX_CLASS_NAV, UBX_NAV_PVT, 1};
    send_ubx(UBX_CLASS_CFG, UBX_CFG_MSG, cfg_msg, sizeof(cfg_msg));

    return true;
}

void UbloxDriver::update() {
    uint8_t buf[128];
    int len = m_uart.read(buf, sizeof(buf));
    for (int i = 0; i < len; ++i) {
        parse_byte(buf[i]);
    }
}

// get_data() implemented in header

bool UbloxDriver::send_ubx(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len) {
    uint8_t header[6] = {
        UBX_SYNC1, UBX_SYNC2,
        cls, id,
        static_cast<uint8_t>(len & 0xFF),
        static_cast<uint8_t>(len >> 8)
    };
    
    uint8_t ck_a = 0, ck_b = 0;
    for (int i = 2; i < 6; ++i) {
        ck_a += header[i];
        ck_b += ck_a;
    }
    for (uint16_t i = 0; i < len; ++i) {
        ck_a += payload[i];
        ck_b += ck_a;
    }
    
    m_uart.write(header, 6);
    if (len > 0) {
        m_uart.write(payload, len);
    }
    uint8_t checksum[2] = {ck_a, ck_b};
    m_uart.write(checksum, 2);
    return true;
}

bool UbloxDriver::parse_byte(uint8_t byte) {
    switch (m_parse_state) {
        case ParseState::SYNC1:
            if (byte == UBX_SYNC1) m_parse_state = ParseState::SYNC2;
            break;
        case ParseState::SYNC2:
            if (byte == UBX_SYNC2) {
                m_parse_state = ParseState::CLASS;
                m_ck_a = 0;
                m_ck_b = 0;
            } else {
                m_parse_state = ParseState::SYNC1;
            }
            break;
        case ParseState::CLASS:
            m_msg_class = byte;
            m_ck_a += byte; m_ck_b += m_ck_a;
            m_parse_state = ParseState::ID;
            break;
        case ParseState::ID:
            m_msg_id = byte;
            m_ck_a += byte; m_ck_b += m_ck_a;
            m_parse_state = ParseState::LEN1;
            break;
        case ParseState::LEN1:
            m_payload_len = byte;
            m_ck_a += byte; m_ck_b += m_ck_a;
            m_parse_state = ParseState::LEN2;
            break;
        case ParseState::LEN2:
            m_payload_len |= (static_cast<uint16_t>(byte) << 8);
            m_ck_a += byte; m_ck_b += m_ck_a;
            if (m_payload_len > MAX_PAYLOAD) {
                m_parse_state = ParseState::SYNC1;
            } else if (m_payload_len == 0) {
                m_parse_state = ParseState::CK_A;
            } else {
                m_payload_idx = 0;
                m_parse_state = ParseState::PAYLOAD;
            }
            break;
        case ParseState::PAYLOAD:
            m_payload_buf[m_payload_idx++] = byte;
            m_ck_a += byte; m_ck_b += m_ck_a;
            if (m_payload_idx >= m_payload_len) {
                m_parse_state = ParseState::CK_A;
            }
            break;
        case ParseState::CK_A:
            if (byte == m_ck_a) {
                m_parse_state = ParseState::CK_B;
            } else {
                m_parse_state = ParseState::SYNC1;
            }
            break;
        case ParseState::CK_B:
            m_parse_state = ParseState::SYNC1;
            if (byte == m_ck_b) {
                if (m_msg_class == UBX_CLASS_NAV && m_msg_id == UBX_NAV_PVT && m_payload_len >= 92) {
                    parse_nav_pvt(m_payload_buf);
                    return true;
                }
            }
            break;
    }
    return false;
}

void UbloxDriver::parse_nav_pvt(const uint8_t* payload) {
    uint8_t fixType = payload[20];
    uint8_t flags = payload[21];
    
    // lat/lon (deg * 1e7)
    int32_t lon = 0;
    std::memcpy(&lon, &payload[24], 4);
    int32_t lat = 0;
    std::memcpy(&lat, &payload[28], 4);
    
    m_data.lat_deg = lon * 1e-7;
    m_data.lon_deg = lat * 1e-7;
    
    // height (mm)
    int32_t height = 0;
    std::memcpy(&height, &payload[32], 4);
    int32_t hMSL = 0;
    std::memcpy(&hMSL, &payload[36], 4);
    
    m_data.alt_ellipsoid_m = height * 1e-3f;
    m_data.alt_msl_m = hMSL * 1e-3f;
    
    // velocity (mm/s)
    int32_t velN = 0, velE = 0, velD = 0, gSpeed = 0;
    std::memcpy(&velN, &payload[48], 4);
    std::memcpy(&velE, &payload[52], 4);
    std::memcpy(&velD, &payload[56], 4);
    std::memcpy(&gSpeed, &payload[60], 4);
    
    m_data.vel_n_ms = velN * 1e-3f;
    m_data.vel_e_ms = velE * 1e-3f;
    m_data.vel_d_ms = velD * 1e-3f;
    m_data.speed_ms = gSpeed * 1e-3f;
    
    // DOPs (0.01)
    uint16_t pDOP = 0;
    std::memcpy(&pDOP, &payload[76], 2);
    m_data.hdop = pDOP * 0.01f;
    m_data.vdop = pDOP * 0.01f; // approximation since NAV-PVT only gives pDOP
    
    m_data.satellites = payload[23];
    
    if (fixType == 0) m_data.fix_type = GpsFixType::NO_FIX;
    else if (fixType == 1) m_data.fix_type = GpsFixType::DEAD_RECK;
    else if (fixType == 2) m_data.fix_type = GpsFixType::FIX_2D;
    else if (fixType == 3) m_data.fix_type = GpsFixType::FIX_3D;
    else if (fixType == 4) m_data.fix_type = GpsFixType::GNSS_DEAD_RECK;
    else m_data.fix_type = GpsFixType::NO_FIX;
    
    m_data.valid = (flags & 0x01) != 0;
    // timestamp_us ideally from system clock, but here we just increment or use std::chrono
    // Not directly implemented, assuming upper layer assigns timestamp_us, or use a system function.
}

} // namespace apx::drivers
