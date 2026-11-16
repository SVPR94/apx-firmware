// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "uart_mavlink_driver.hpp"

// Assuming IHalUart is defined elsewhere with set_baudrate, read, write
#include "../gps/ublox/ublox_driver.hpp" // Has IHalUart definition for now

namespace apx::drivers {

UartMavlinkDriver::UartMavlinkDriver(apx::hal::IHalUart& uart, uint32_t baud)
    : m_uart(uart), m_baud(baud) {
}

bool UartMavlinkDriver::init() {
    m_uart.set_baudrate(m_baud);
    return true;
}

void UartMavlinkDriver::update() {
    // Flush TX ring buffer
    while (m_tx_tail != m_tx_head) {
        int tail = m_tx_tail;
        int head = m_tx_head;
        int to_write = 0;
        
        if (head > tail) {
            to_write = head - tail;
        } else {
            to_write = TX_BUF_SIZE - tail;
        }
        
        int written = m_uart.write(&m_tx_ring_buf[tail], to_write);
        if (written > 0) {
            m_tx_tail = (m_tx_tail + written) % TX_BUF_SIZE;
        } else {
            break; // UART full or error
        }
    }
}

int UartMavlinkDriver::read_bytes(uint8_t* buf, int max_len) {
    return m_uart.read(buf, max_len);
}

int UartMavlinkDriver::write_bytes(const uint8_t* buf, int len) {
    int written = 0;
    for (int i = 0; i < len; ++i) {
        int next_head = (m_tx_head + 1) % TX_BUF_SIZE;
        if (next_head == m_tx_tail) {
            // Buffer full, drop remaining
            break;
        }
        m_tx_ring_buf[m_tx_head] = buf[i];
        m_tx_head = next_head;
        written++;
    }
    return written;
}

} // namespace apx::drivers
