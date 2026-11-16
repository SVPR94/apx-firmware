// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once
#include "apx/drivers/driver_base.hpp"
#include "apx/hal/hal_uart.hpp"
#include "rc_input/sbus/sbus_driver.hpp"
#include <cstdint>
#include <array>

namespace apx::drivers {

class CrsfDriver : public DriverBase {
public:
    static constexpr uint8_t CRSF_FRAMETYPE_RC_CHANNELS = 0x16;
    static constexpr uint8_t CRSF_FRAME_SIZE_MAX = 64;

    explicit CrsfDriver(apx::hal::IHalUart& uart);
    bool init() override;
    void update() override;
    const RcData& get_data() const { return m_data; }

private:
    uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a) const;
    bool parse_frame(const uint8_t* buf, uint8_t len);

    apx::hal::IHalUart& m_uart;
    uint8_t m_buf[CRSF_FRAME_SIZE_MAX]{};
    int     m_buf_idx{0};
    RcData  m_data{};
};

} // namespace apx::drivers
