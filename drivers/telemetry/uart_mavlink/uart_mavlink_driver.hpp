// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include "../../include/apx/drivers/driver_base.hpp"
#include <cstdint>
#include <vector>

// Note: Re-using the abstract UART from ublox driver for brevity here
// Normally it would be centrally located.
namespace apx::hal {
    class IHalUart; // Declared in ublox or a central hal_uart.hpp
}

namespace apx::drivers {

class UartMavlinkDriver : public DriverBase {
public:
    UartMavlinkDriver(apx::hal::IHalUart& uart, uint32_t baud = 57600);
    bool init() override;
    void update() override;
    
    int read_bytes(uint8_t* buf, int max_len);
    int write_bytes(const uint8_t* buf, int len);

private:
    apx::hal::IHalUart& m_uart;
    uint32_t m_baud;
    
    static constexpr int TX_BUF_SIZE = 256;
    uint8_t m_tx_ring_buf[TX_BUF_SIZE]{};
    int m_tx_head{0};
    int m_tx_tail{0};
};

} // namespace apx::drivers
