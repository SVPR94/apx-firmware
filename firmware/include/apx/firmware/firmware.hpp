#pragma once
// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
// APX flight firmware - portable application entry.
//
// The OS port's startup code (Zephyr main(), a FreeRTOS task, ...) calls
// run() once the OS is up. run() only uses the APX OS abstraction
// (apx/kernel), the HAL interfaces (apx/hal) and the board interface
// (apx/firmware/board.hpp), so it is identical on every OS and board.

#include <cstdint>

namespace apx::firmware {

constexpr uint32_t kLoopHz = 400;              // flight loop rate
constexpr uint32_t kLoopPeriodUs = 1000000 / kLoopHz;
constexpr uint32_t kBringupNoticePeriodS = 60; // repeat of the "cannot fly" notice

// Never returns in normal operation.
int run();

} // namespace apx::firmware
