// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once
#include "apx/drivers/driver_base.hpp"
#include "gps/ublox/ublox_driver.hpp"
#include "apx/hal/hal_uart.hpp"
#include <cstdint>
#include <cstddef>

namespace apx::drivers {

struct GpsDataRTK : public GpsData {
    float rel_pos_ned[3]{};
    bool  rtk_fixed{false};
    float rel_pos_accuracy_m{0.0f};
    int   fix_type_raw{0}; // 5=float_rtk, 6=fixed_rtk
};

// UBX CFG-VALSET key IDs (F9P uses key-value config)
namespace F9PKey {
    constexpr uint32_t UART1_BAUDRATE    = 0x40520001;
    constexpr uint32_t NAVHPG_DGNSSMODE  = 0x20140011; // 3=RTK
    constexpr uint32_t MSGOUT_PVT_UART1  = 0x20910007; // 1 per epoch
    constexpr uint32_t MSGOUT_RELPOS_UART1=0x20910090; // 1 per epoch
    constexpr uint32_t RATE_MEAS         = 0x30210001; // measurement period ms
}

class UbloxF9P : public DriverBase {
public:
    explicit UbloxF9P(apx::hal::IHalUart& uart, uint32_t baud = 460800);
    bool init() override;
    void update() override;
    bool is_healthy() const override;

    const GpsDataRTK& get_data_rtk() const { return m_rtk_data; }
    const GpsData&    get_data()     const { return m_rtk_data; }

    // Forward RTCM corrections from base station
    void inject_rtcm(const uint8_t* data, size_t len);

private:
    bool send_valset_u4(uint32_t key, uint32_t value);
    bool send_valset_u1(uint32_t key, uint8_t value);
    bool send_ubx(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len);
    bool parse_byte(uint8_t byte);
    void parse_nav_pvt(const uint8_t* payload);
    void parse_nav_relposned(const uint8_t* payload);

    enum class ParseState { SYNC1, SYNC2, CLASS, ID, LEN1, LEN2, PAYLOAD, CK_A, CK_B };
    ParseState m_parse_state{ParseState::SYNC1};
    uint8_t  m_msg_class{}, m_msg_id{};
    uint16_t m_payload_len{}, m_payload_idx{};
    uint8_t  m_ck_a{}, m_ck_b{};
    static constexpr uint16_t MAX_PAYLOAD = 512;
    uint8_t  m_payload_buf[MAX_PAYLOAD]{};

    apx::hal::IHalUart& m_uart;
    uint32_t  m_baud;
    GpsDataRTK m_rtk_data{};
};

} // namespace apx::drivers
