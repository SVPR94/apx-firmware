#pragma once
// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
// APX HAL port: Zephyr serial port (IHalUart) - any Zephyr UART device,
// including USB CDC-ACM. Interrupt-driven with RX/TX ring buffers; needs
// CONFIG_SERIAL, CONFIG_UART_INTERRUPT_DRIVEN and CONFIG_RING_BUFFER.

#include <zephyr/device.h>
#include <zephyr/sys/ring_buffer.h>

#include "apx/hal/hal_uart.hpp"

namespace apx::hal::zephyr {

class ZephyrUart final : public apx::hal::IHalUart {
public:
    static constexpr size_t kTxBufferSize = 8192;
    static constexpr size_t kRxBufferSize = 2048;

    explicit ZephyrUart(const struct device* dev);

    // baud_rate 0 keeps the devicetree setting (current-speed); ignored by
    // USB CDC-ACM. Returns false if the device is not ready.
    bool init(uint32_t baud_rate) override;
    void deinit() override;
    // Queues the whole buffer or nothing (a MAVLink frame is never
    // truncated when the host stops reading). Returns bytes queued.
    int write(const uint8_t* buf, size_t len) override;
    int read(uint8_t* buf, size_t max_len) override;
    int available() const override;
    void flush() override;
    // Called from interrupt context with each received chunk; keep it short.
    void set_rx_callback(RxCallback fn) override { m_rx_callback = fn; }

    bool ready() const { return m_ready; }

private:
    static void isr(const struct device* dev, void* user_data);

    const struct device* m_dev;
    bool m_ready{false};
    struct ring_buf m_tx;
    struct ring_buf m_rx;
    uint8_t m_tx_storage[kTxBufferSize];
    uint8_t m_rx_storage[kRxBufferSize];
    RxCallback m_rx_callback;
};

} // namespace apx::hal::zephyr
