// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
// APX OS port: Zephyr - periodic timer and OS identity.

#include <zephyr/kernel.h>

#include "apx/kernel/os.hpp"
#include "apx/kernel/periodic.hpp"

namespace apx::kernel {

struct PeriodicTimer::Impl {
    struct k_timer timer;
};

PeriodicTimer::PeriodicTimer(uint32_t period_us) : m_period_us(period_us), m_impl(new Impl()) {
    k_timer_init(&m_impl->timer, nullptr, nullptr);
}

PeriodicTimer::~PeriodicTimer() { k_timer_stop(&m_impl->timer); }

void PeriodicTimer::start() {
    k_timer_start(&m_impl->timer, K_USEC(m_period_us), K_USEC(m_period_us));
}

void PeriodicTimer::stop() { k_timer_stop(&m_impl->timer); }

uint32_t PeriodicTimer::wait() { return k_timer_status_sync(&m_impl->timer); }

const char* os_name() { return "zephyr"; }

void yield() { k_yield(); }

} // namespace apx::kernel
