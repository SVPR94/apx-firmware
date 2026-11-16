// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
// APX OS port TEMPLATE - copy to kernel/ports/<your_os>/ and fill in.
//
// Not compiled. Every function below must be implemented for a new OS
// (FreeRTOS, ChibiOS, NuttX, ...); the linker reports anything missing.
// Split it into time.cpp / mutex.cpp / periodic.cpp like the other ports
// if you prefer - the file names do not matter, kernel/sources.cmake globs
// kernel/ports/<os>/*.cpp. See kernel/ports/README.md for the mapping
// of each function to FreeRTOS, ChibiOS and NuttX APIs.

#error "kernel/ports/template is a template, not a port"

#include "apx/kernel/critical.hpp"
#include "apx/kernel/mutex.hpp"
#include "apx/kernel/os.hpp"
#include "apx/kernel/periodic.hpp"
#include "apx/kernel/timer.hpp"

namespace apx::kernel {

// ---- time (timer.hpp) -----------------------------------------------------
// Monotonic microseconds since boot, 64-bit, never wraps in practice.
// Prefer a hardware cycle counter (DWT CYCCNT / 32-bit timer) over the
// scheduler tick: the flight loop measures dt with this.
uint64_t now_us() { return 0; /* TODO */ }
uint64_t now_ms() { return now_us() / 1000ULL; }
uint64_t elapsed_us(uint64_t start_us) { return now_us() - start_us; }
void delay_us(uint32_t us) { (void)us; /* TODO: sleep (busy-wait below 1 tick) */ }
void delay_ms(uint32_t ms) { (void)ms; /* TODO: sleep the calling thread */ }

// ---- Mutex (mutex.hpp) ----------------------------------------------------
// Recursive, blocking, priority-inheriting where the OS offers it.
struct Mutex::Impl {
    // TODO: OS mutex handle
};
Mutex::Mutex() : m_impl(new Impl()) { /* TODO: create */ }
Mutex::~Mutex() = default; /* TODO: delete the OS object in ~Impl if needed */
void Mutex::lock() { /* TODO */ }
void Mutex::unlock() { /* TODO */ }

// ---- CriticalSection (critical.hpp) ---------------------------------------
// Masks interrupts / the scheduler; nestable. m_state may hold saved state.
CriticalSection::CriticalSection() { /* TODO */ }
CriticalSection::~CriticalSection() { /* TODO */ }

// ---- PeriodicTimer (periodic.hpp) -----------------------------------------
// Drift-free fixed-rate wait. wait() returns elapsed periods (>= 1).
struct PeriodicTimer::Impl {
    uint64_t next_us{0};
};
PeriodicTimer::PeriodicTimer(uint32_t period_us) : m_period_us(period_us), m_impl(new Impl()) {}
PeriodicTimer::~PeriodicTimer() = default;
void PeriodicTimer::start() { /* TODO */ }
void PeriodicTimer::stop() { /* TODO */ }
uint32_t PeriodicTimer::wait() { return 1; /* TODO: sleep until next deadline */ }

// ---- OS identity (os.hpp) -------------------------------------------------
const char* os_name() { return "your_os"; }
void yield() { /* TODO */ }

} // namespace apx::kernel
