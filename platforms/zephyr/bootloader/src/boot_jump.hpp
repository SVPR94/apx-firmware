#pragma once
// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
// APX bootloader (Zephyr port): application image checks, boot control
// access and the hand-over to the application.

#include <cstddef>
#include <cstdint>

#include "apx/bootloader/boot_control.hpp"

namespace apx::bootloader::zephyr {

uint32_t app_base_address();   // absolute address of the application vector table
uint32_t app_offset();         // offset of the application area in flash
uint32_t app_size();

// True if the application area starts with a plausible vector table
// (stack pointer in RAM, Thumb reset handler inside the area). The first
// program unit is written last during an upload, so a partial upload fails
// this check.
bool app_valid();

// Shared RAM block (devicetree node apx_boot_control).
volatile BootControl* boot_control();

// Pending request recorded by the early boot hook (it reads and clears the
// shared block before the kernel starts).
BootRequest early_request();

// Request a clean start of the application: record it in the boot control
// block and reset; the early boot hook jumps before any driver starts.
[[noreturn]] void start_application();

} // namespace apx::bootloader::zephyr
