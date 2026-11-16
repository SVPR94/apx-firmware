// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
// APX HAL port: Zephyr serial port (IHalUart).

#include "uart_zephyr.hpp"

#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

namespace apx::hal::zephyr {

ZephyrUart::ZephyrUart(const struct device* dev) : m_dev(dev) {
    ring_buf_init(&m_tx, sizeof(m_tx_storage), m_tx_storage);
    ring_buf_init(&m_rx, sizeof(m_rx_storage), m_rx_storage);
}

bool ZephyrUart::init(uint32_t baud_rate) {
    if (m_dev == nullptr || !device_is_ready(m_dev)) {
        return false;
    }
    if (baud_rate != 0) {
        struct uart_config cfg;
        if (uart_config_get(m_dev, &cfg) == 0) {
            cfg.baudrate = baud_rate;
            (void)uart_configure(m_dev, &cfg); // CDC-ACM: -ENOSYS, harmless
        }
    }
    uart_irq_callback_user_data_set(m_dev, isr, this);
    uart_irq_rx_enable(m_dev);
    m_ready = true;
    return true;
}

void ZephyrUart::deinit() {
    if (!m_ready) return;
    uart_irq_rx_disable(m_dev);
    uart_irq_tx_disable(m_dev);
    m_ready = false;
}

void ZephyrUart::isr(const struct device* dev, void* user_data) {
    auto* self = static_cast<ZephyrUart*>(user_data);
    while (uart_irq_update(dev) > 0 && uart_irq_is_pending(dev) > 0) {
        if (uart_irq_rx_ready(dev) > 0) {
            uint8_t buf[64];
            const int n = uart_fifo_read(dev, buf, sizeof(buf));
            if (n > 0) {
                ring_buf_put(&self->m_rx, buf, static_cast<uint32_t>(n));
                if (self->m_rx_callback) {
                    self->m_rx_callback(buf, static_cast<size_t>(n));
                }
            }
        }
        if (uart_irq_tx_ready(dev) > 0) {
            uint8_t* data = nullptr;
            const uint32_t n = ring_buf_get_claim(&self->m_tx, &data, 64);
            if (n == 0) {
                uart_irq_tx_disable(dev);
            } else {
                const int sent = uart_fifo_fill(dev, data, static_cast<int>(n));
                ring_buf_get_finish(&self->m_tx, sent > 0 ? static_cast<uint32_t>(sent) : 0U);
            }
        }
    }
}

int ZephyrUart::write(const uint8_t* buf, size_t len) {
    if (!m_ready) return -1;
    int queued = 0;
    const unsigned int key = irq_lock();
    if (ring_buf_space_get(&m_tx) >= len) {
        queued = static_cast<int>(ring_buf_put(&m_tx, buf, static_cast<uint32_t>(len)));
    }
    irq_unlock(key);
    if (queued > 0) {
        uart_irq_tx_enable(m_dev);
    }
    return queued;
}

int ZephyrUart::read(uint8_t* buf, size_t max_len) {
    if (!m_ready) return -1;
    const unsigned int key = irq_lock();
    const uint32_t n = ring_buf_get(&m_rx, buf, static_cast<uint32_t>(max_len));
    irq_unlock(key);
    return static_cast<int>(n);
}

int ZephyrUart::available() const {
    return static_cast<int>(ring_buf_size_get(const_cast<struct ring_buf*>(&m_rx)));
}

void ZephyrUart::flush() {
    for (int i = 0; i < 100 && !ring_buf_is_empty(&m_tx); ++i) {
        k_msleep(1);
    }
}

} // namespace apx::hal::zephyr
