#pragma once
// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
// APX board interface: what the portable firmware needs from a flight
// controller board, independent of OS and manufacturer.
//
// Each OS port provides one implementation:
//   Zephyr     platforms/zephyr/app/src/board_zephyr.cpp - generic, reads
//              everything from the board's devicetree, so a new board
//              needs only board files, no code
//   <new OS>   e.g. platforms/freertos/board_<board>.cpp
//
// Keep this interface small and hardware-neutral: pins, buses and drivers
// stay behind it.

#include <cstdint>

#include "apx/hal/hal_uart.hpp"

namespace apx::board {

struct BoardInfo {
    const char* name;    // board id, e.g. "cube_orange_plus"
    const char* target;  // full build target, e.g. "cube_orange_plus/stm32h757xx/m7"
    const char* os;      // OS port name, e.g. "zephyr"
};

// Power rails, status LED and communication links. Called once, first.
// Returns false if something the firmware needs is missing (it still runs
// and blinks, so the board can be diagnosed).
bool init();

const BoardInfo& info();

// Status LED (no-op on boards without one).
void set_status_led(bool on);
void toggle_status_led();

// Primary ground-station (MAVLink) link: USB CDC or a telemetry UART,
// chosen by the board. nullptr if the board has none.
apx::hal::IHalUart* gcs_link();

// Link to the board's IO coprocessor (IO MCU handling main outputs, RC input
// and the safety switch), or nullptr on boards without one.
apx::hal::IHalUart* iomcu_link();

// Reset the board. to_bootloader: ask the bootloader to stay active for a
// firmware upload instead of starting the firmware again.
[[noreturn]] void reboot(bool to_bootloader);

} // namespace apx::board
