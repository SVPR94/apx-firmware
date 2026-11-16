// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
// APX bootloader (Zephyr port): Port implementation.

#include "port_zephyr.hpp"

#include <cstring>

#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/kernel.h>

#include "boot_jump.hpp"

namespace apx::bootloader::zephyr {
namespace {

#if DT_NODE_EXISTS(DT_ALIAS(led0))
const gpio_dt_spec kLed = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
#endif

struct EraseCtx {
    const struct device* flash;
    off_t begin;
    off_t end;
    bool ok;
};

bool page_blank(const struct device* flash, off_t start, size_t size) {
    uint8_t buf[256];
    for (size_t off = 0; off < size; off += sizeof(buf)) {
        const size_t n = (size - off) < sizeof(buf) ? (size - off) : sizeof(buf);
        if (flash_read(flash, start + static_cast<off_t>(off), buf, n) != 0) {
            return false;
        }
        for (size_t i = 0; i < n; ++i) {
            if (buf[i] != 0xFF) {
                return false;
            }
        }
    }
    return true;
}

// Erase the pages (sectors) of the application area; already blank pages are
// skipped, which makes re-flashing a small image much faster.
bool erase_page(const struct flash_pages_info* info, void* data) {
    auto* ctx = static_cast<EraseCtx*>(data);
    if (info->start_offset + static_cast<off_t>(info->size) <= ctx->begin || info->start_offset >= ctx->end) {
        return true; // outside the application area
    }
    if (!page_blank(ctx->flash, info->start_offset, info->size)) {
        if (flash_erase(ctx->flash, info->start_offset, info->size) != 0) {
            ctx->ok = false;
            return false;
        }
    }
    return true;
}

} // namespace

ZephyrPort::ZephyrPort(apx::hal::zephyr::ZephyrUart& link, bool link_is_uart)
    : m_link(link), m_link_is_uart(link_is_uart),
      m_flash(DEVICE_DT_GET(DT_CHOSEN(zephyr_flash_controller))) {
#if DT_NODE_EXISTS(DT_ALIAS(led0))
    gpio_pin_configure_dt(&kLed, GPIO_OUTPUT_INACTIVE);
#endif
}

size_t ZephyrPort::read(uint8_t* buf, size_t max_len) {
    const int n = m_link.read(buf, max_len);
    return n > 0 ? static_cast<size_t>(n) : 0;
}

void ZephyrPort::write(const uint8_t* buf, size_t len) {
    // ZephyrUart queues whole buffers only; wait for room if the host is slow.
    for (int tries = 0; tries < 200 && m_link.write(buf, len) <= 0; ++tries) {
        k_msleep(1);
    }
}

void ZephyrPort::flush() { m_link.flush(); }

bool ZephyrPort::set_baud(uint32_t baud) {
    if (!m_link_is_uart) {
        return false; // USB CDC: baud rate has no meaning
    }
    m_link.set_baudrate(baud);
    return true;
}

uint32_t ZephyrPort::millis() { return k_uptime_get_32(); }

void ZephyrPort::idle() {
    k_msleep(1);
#if DT_NODE_EXISTS(DT_ALIAS(led0))
    const uint32_t now = k_uptime_get_32();
    if (now - m_last_blink >= 100) { // fast 5 Hz blink: "in bootloader"
        m_last_blink = now;
        gpio_pin_toggle_dt(&kLed);
    }
#endif
}

bool ZephyrPort::erase_app() {
    if (!device_is_ready(m_flash)) {
        return false;
    }
    EraseCtx ctx{m_flash, static_cast<off_t>(app_offset()), static_cast<off_t>(app_offset() + app_size()), true};
    flash_page_foreach(m_flash, erase_page, &ctx);
    return ctx.ok;
}

bool ZephyrPort::program(uint32_t offset, const uint8_t* data, size_t len) {
    if (offset + len > app_size()) {
        return false;
    }
    return flash_write(m_flash, static_cast<off_t>(app_offset() + offset), data, len) == 0;
}

void ZephyrPort::read_app(uint32_t offset, uint8_t* data, size_t len) {
    if (flash_read(m_flash, static_cast<off_t>(app_offset() + offset), data, len) != 0) {
        std::memset(data, 0xFF, len);
    }
}

size_t ZephyrPort::program_unit() const { return flash_get_write_block_size(m_flash); }

bool ZephyrPort::serial_word(uint32_t offset, uint32_t& word) {
    uint8_t id[16];
    const ssize_t n = hwinfo_get_device_id(id, sizeof(id));
    if (n <= 0 || offset + 4 > static_cast<uint32_t>(n)) {
        return false;
    }
    std::memcpy(&word, &id[offset], 4);
    return true;
}

} // namespace apx::bootloader::zephyr
