#pragma once
// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
// APX bootloader (Zephyr port): Port implementation on Zephyr's flash,
// UART/USB CDC and GPIO APIs.

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#include "apx/bootloader/port.hpp"
#include "uart_zephyr.hpp"

namespace apx::bootloader::zephyr {

class ZephyrPort final : public Port {
public:
    ZephyrPort(apx::hal::zephyr::ZephyrUart& link, bool link_is_uart);

    size_t read(uint8_t* buf, size_t max_len) override;
    void write(const uint8_t* buf, size_t len) override;
    void flush() override;
    bool set_baud(uint32_t baud) override;
    uint32_t millis() override;
    void idle() override;
    bool erase_app() override;
    bool program(uint32_t offset, const uint8_t* data, size_t len) override;
    void read_app(uint32_t offset, uint8_t* data, size_t len) override;
    size_t program_unit() const override;
    bool serial_word(uint32_t offset, uint32_t& word) override;

private:
    apx::hal::zephyr::ZephyrUart& m_link;
    bool m_link_is_uart;
    const struct device* m_flash;
    uint32_t m_last_blink{0};
};

} // namespace apx::bootloader::zephyr
