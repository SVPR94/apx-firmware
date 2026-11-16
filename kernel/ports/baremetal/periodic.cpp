// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
// APX OS port: bare-metal Cortex-M7 (no RTOS) - periodic timer and OS identity.
//
// No scheduler to block in: wait() spins on now_us() until the deadline.

#include "apx/kernel/os.hpp"
#include "apx/kernel/periodic.hpp"
#include "apx/kernel/timer.hpp"

namespace apx::kernel {

struct PeriodicTimer::Impl {
    uint64_t next_us{0};
    bool running{false};
};

PeriodicTimer::PeriodicTimer(uint32_t period_us) : m_period_us(period_us), m_impl(new Impl()) {}
PeriodicTimer::~PeriodicTimer() = default;

void PeriodicTimer::start() {
    m_impl->next_us = now_us() + m_period_us;
    m_impl->running = true;
}

void PeriodicTimer::stop() { m_impl->running = false; }

uint32_t PeriodicTimer::wait() {
    if (!m_impl->running) start();
    while (now_us() < m_impl->next_us) {
        __asm__ volatile("nop");
    }
    uint32_t elapsed = 1;
    m_impl->next_us += m_period_us;
    const uint64_t now = now_us();
    while (m_impl->next_us <= now) {
        m_impl->next_us += m_period_us;
        ++elapsed;
    }
    return elapsed;
}

const char* os_name() { return "baremetal"; }

void yield() {}

} // namespace apx::kernel
