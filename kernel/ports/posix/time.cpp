// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
// APX OS port: POSIX / hosted (SITL, Linux companion) - time.

#include <chrono>
#include <thread>

#include "apx/kernel/timer.hpp"

namespace apx::kernel {

uint64_t now_us() {
    using namespace std::chrono;
    return static_cast<uint64_t>(duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count());
}

uint64_t now_ms() { return now_us() / 1000ULL; }

uint64_t elapsed_us(uint64_t start_us) { return now_us() - start_us; }

void delay_us(uint32_t us) { std::this_thread::sleep_for(std::chrono::microseconds(us)); }

void delay_ms(uint32_t ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

} // namespace apx::kernel
