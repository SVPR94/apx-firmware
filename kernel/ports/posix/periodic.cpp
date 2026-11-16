// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
// APX OS port: POSIX / hosted - periodic timer and OS identity.

#include <chrono>
#include <thread>

#include "apx/kernel/os.hpp"
#include "apx/kernel/periodic.hpp"

namespace apx::kernel {

using Clock = std::chrono::steady_clock;

struct PeriodicTimer::Impl {
    Clock::time_point next{};
    bool running{false};
};

PeriodicTimer::PeriodicTimer(uint32_t period_us) : m_period_us(period_us), m_impl(new Impl()) {}
PeriodicTimer::~PeriodicTimer() = default;

void PeriodicTimer::start() {
    m_impl->next = Clock::now() + std::chrono::microseconds(m_period_us);
    m_impl->running = true;
}

void PeriodicTimer::stop() { m_impl->running = false; }

uint32_t PeriodicTimer::wait() {
    if (!m_impl->running) start();
    const auto period = std::chrono::microseconds(m_period_us);
    std::this_thread::sleep_until(m_impl->next);
    uint32_t elapsed = 1;
    m_impl->next += period;
    const auto now = Clock::now();
    while (m_impl->next <= now) { // overran: skip missed ticks
        m_impl->next += period;
        ++elapsed;
    }
    return elapsed;
}

const char* os_name() { return "posix"; }

void yield() { std::this_thread::yield(); }

} // namespace apx::kernel
