// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
// APX OS port: bare-metal Cortex-M7 (no RTOS) - mutex and critical section.
//
// Single core, no scheduler: the only possible contention is an ISR vs. the
// main loop, so both Mutex and CriticalSection mask interrupts (PRIMASK).
// That is correct and sufficient here; it is not a blocking mutex. (The
// arm-none-eabi libstdc++ is a single-threaded build without std::mutex.)

#include <cstdint>

#include "apx/kernel/critical.hpp"
#include "apx/kernel/mutex.hpp"

namespace apx::kernel {
namespace {
inline uint32_t irq_save() {
    uint32_t primask;
    __asm volatile("mrs %0, primask" : "=r"(primask));
    __asm volatile("cpsid i" ::: "memory");
    return primask;
}

inline void irq_restore(uint32_t primask) {
    if ((primask & 1u) == 0) {
        __asm volatile("cpsie i" ::: "memory");
    }
}
} // namespace

struct Mutex::Impl {
    uint32_t saved_primask{0};
    int depth{0}; // recursive lock() from the same context
};

Mutex::Mutex() : m_impl(new Impl()) {}
Mutex::~Mutex() = default;

void Mutex::lock() {
    const uint32_t primask = irq_save();
    if (m_impl->depth++ == 0) m_impl->saved_primask = primask;
}

void Mutex::unlock() {
    if (m_impl->depth > 0 && --m_impl->depth == 0) irq_restore(m_impl->saved_primask);
}

CriticalSection::CriticalSection() : m_state(irq_save()) {}
CriticalSection::~CriticalSection() { irq_restore(m_state); }

} // namespace apx::kernel
