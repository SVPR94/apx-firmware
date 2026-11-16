// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include "apx/drivers/driver_base.hpp"
#include "apx/hal/hal_uart.hpp"
#include <cstdint>
#include <cmath>

namespace apx::drivers {

enum class GpsFixType { NO_FIX=0, DEAD_RECK=1, FIX_2D=2, FIX_3D=3, GNSS_DEAD_RECK=4 };

struct GpsData {
    double lat_deg;
    double lon_deg;
    float alt_msl_m;
    float alt_ellipsoid_m;
    float vel_n_ms, vel_e_ms, vel_d_ms;
    float speed_ms;
    float hdop;
    float vdop;
    uint8_t satellites;
    GpsFixType fix_type;
    bool valid;
    uint64_t timestamp_us;
};

class UbloxDriver : public DriverBase {
public:
    explicit UbloxDriver(apx::hal::IHalUart& uart);
    bool init() override;
    void update() override;
    GpsData get_data() const { return m_data; }

private:
    static constexpr uint8_t UBX_SYNC1 = 0xB5;
    static constexpr uint8_t UBX_SYNC2 = 0x62;
    static constexpr uint8_t UBX_CLASS_NAV = 0x01;
    static constexpr uint8_t UBX_CLASS_CFG = 0x06;
    static constexpr uint8_t UBX_CLASS_ACK = 0x05;
    static constexpr uint8_t UBX_NAV_PVT = 0x07;
    static constexpr uint8_t UBX_CFG_PRT = 0x00;
    static constexpr uint8_t UBX_CFG_RATE = 0x08;
    static constexpr uint8_t UBX_CFG_MSG = 0x01;

    bool send_ubx(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len);
    bool parse_byte(uint8_t byte);
    void parse_nav_pvt(const uint8_t* payload);

    enum class ParseState { SYNC1, SYNC2, CLASS, ID, LEN1, LEN2, PAYLOAD, CK_A, CK_B };
    ParseState m_parse_state{ParseState::SYNC1};
    uint8_t m_msg_class{0}, m_msg_id{0};
    uint16_t m_payload_len{0}, m_payload_idx{0};
    uint8_t m_ck_a{0}, m_ck_b{0};
    static constexpr uint16_t MAX_PAYLOAD = 512;
    uint8_t m_payload_buf[MAX_PAYLOAD]{};

    apx::hal::IHalUart& m_uart;
    GpsData m_data{};
};

} // namespace apx::drivers
