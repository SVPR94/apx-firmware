// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include <cstdint>

namespace apx::kernel {

uint64_t now_us();
uint64_t now_ms();
uint64_t elapsed_us(uint64_t start_us);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

} // namespace apx::kernel
