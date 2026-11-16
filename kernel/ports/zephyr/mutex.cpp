// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "apx/kernel/mutex.hpp"

// apx::kernel::Mutex was declared (pimpl'd Impl, no definition anywhere)
// but never implemented - harmless while nothing constructed one, but
// modules/params/param_manager.cpp needs a real mutex once it's linked
// into anything (see platforms/stm32h7/boards/apx_h743_v1/CMakeLists.txt)
// where std::mutex itself doesn't exist: this arm-none-eabi toolchain's
// libstdc++ is a single-threaded (no gthreads) build, so <mutex> compiles
// but doesn't actually define std::mutex at all ("'mutex' is not a
// member of 'std'") - confirmed by an actual build attempt, not assumed.

#if defined(APX_PLATFORM_ZEPHYR)

// Zephyr: a real kernel mutex (recursive for the owning thread, priority
// inheritance), not interrupt masking.
#include <zephyr/kernel.h>

namespace apx::kernel {

struct Mutex::Impl {
    struct k_mutex m;
};

Mutex::Mutex() : m_impl(new Impl()) {
    k_mutex_init(&m_impl->m);
}

Mutex::~Mutex() = default;

void Mutex::lock() {
    k_mutex_lock(&m_impl->m, K_FOREVER);
}

void Mutex::unlock() {
    k_mutex_unlock(&m_impl->m);
}

} // namespace apx::kernel

#elif defined(APX_PLATFORM_STM32H7) && (defined(__arm__) || defined(__thumb__))

// Bare-metal, single core, no RTOS/scheduler wired in yet (see
// platforms/stm32h7/README.md) - there is no preemptive thread that
// could contend for this lock today, only (eventually) an ISR
// vs. main-loop race. A real RTOS mutex (with priority inheritance,
// blocking/wake) is out of scope until an RTOS exists; what a bare-metal
// single-core target actually needs right now is IRQ-masking around the
// critical section, which is what this does - correct and sufficient
// for "no ISR sees a half-updated ParamManager map", not a general-
// purpose blocking mutex.
#include <cstdint>

namespace apx::kernel {

struct Mutex::Impl {
    uint32_t saved_primask{0};
    int depth{0}; // supports recursive lock() from the same context
};

Mutex::Mutex() : m_impl(new Impl()) {}
Mutex::~Mutex() = default;

void Mutex::lock() {
    if (m_impl->depth == 0) {
        uint32_t primask;
        __asm volatile("mrs %0, primask" : "=r"(primask));
        __asm volatile("cpsid i");
        m_impl->saved_primask = primask;
    }
    ++m_impl->depth;
}

void Mutex::unlock() {
    if (m_impl->depth > 0) {
        --m_impl->depth;
        if (m_impl->depth == 0 && (m_impl->saved_primask & 1u) == 0) {
            __asm volatile("cpsie i");
        }
    }
}

} // namespace apx::kernel

#else // hosted (SITL/Linux)

#include <mutex>

namespace apx::kernel {

struct Mutex::Impl {
    std::mutex m;
};

Mutex::Mutex() : m_impl(new Impl()) {}
Mutex::~Mutex() = default;

void Mutex::lock() { m_impl->m.lock(); }
void Mutex::unlock() { m_impl->m.unlock(); }

} // namespace apx::kernel

#endif
