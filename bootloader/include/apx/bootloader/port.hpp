#pragma once
// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
// APX bootloader - what the protocol core needs from the OS / MCU.
//
// The core (protocol.cpp) is portable C++ with no OS headers. Each OS port
// implements this interface once: platforms/zephyr/bootloader/src/port_zephyr.cpp
// for Zephyr, tests/bootloader/sim_port.cpp for the host simulator.

#include <cstddef>
#include <cstdint>

namespace apx::bootloader {

class Port {
public:
    virtual ~Port() = default;

    // ---- link to the ground station (USB CDC or UART) ----
    // Non-blocking: copies up to max_len received bytes, returns the count.
    virtual size_t read(uint8_t* buf, size_t max_len) = 0;
    virtual void write(const uint8_t* buf, size_t len) = 0;
    // Wait until everything written has been sent (or a short timeout).
    virtual void flush() {}
    // Change the link baud rate (UART only). Return false if unsupported.
    virtual bool set_baud(uint32_t baud) { (void)baud; return false; }

    // ---- time ----
    virtual uint32_t millis() = 0;
    // Called while waiting for input: sleep ~1 ms, blink a LED, ...
    virtual void idle() = 0;

    // ---- application flash area; offsets are relative to its start ----
    // Erase the whole application area (may take seconds).
    virtual bool erase_app() = 0;
    // Program len bytes at offset. offset and len are multiples of
    // program_unit(); the area was erased before.
    virtual bool program(uint32_t offset, const uint8_t* data, size_t len) = 0;
    virtual void read_app(uint32_t offset, uint8_t* data, size_t len) = 0;
    // Smallest programmable unit in bytes (power of two, 4..64): 32 on STM32H7.
    virtual size_t program_unit() const = 0;

    // ---- identity ----
    // 32-bit word of the chip's unique serial number at byte offset
    // (0, 4, 8, ...). Return false past the end.
    virtual bool serial_word(uint32_t offset, uint32_t& word) = 0;
};

// Fixed board facts reported to the uploader.
struct Identity {
    uint32_t board_id;          // must match the .apj board_id
    uint32_t board_revision;
    uint32_t app_size;          // bytes available for the application
    uint32_t chip_id;           // MCU id code (0 if unknown)
    const char* chip_description;  // "family,revision", e.g. "STM32H757xx,V"
    const char* software;          // bootloader version string
};

} // namespace apx::bootloader
