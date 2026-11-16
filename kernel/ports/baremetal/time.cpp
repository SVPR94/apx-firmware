// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
// APX OS port: bare-metal Cortex-M7 (no RTOS) - time.
//
// No OS clock to call into. Uses the Cortex-M7 DWT cycle counter (present
// on every M7, no extra peripheral needed) rather than a hardware timer
// peripheral, so this works before any timer driver exists. Accuracy
// depends on the actual core clock: until SystemInit() configures the PLL
// (see platforms/stm32h7/README.md), the core runs on the ~64 MHz HSI reset
// clock, which is what kAssumedCoreClockHz reflects. Update it together
// with SystemInit().

#include <cstdint>

#include "apx/kernel/timer.hpp"

namespace apx::kernel {
namespace {
// ARMv7-M architecture-defined addresses (not board-specific).
constexpr uint32_t kDwtCtrlAddr = 0xE0001000;
constexpr uint32_t kDwtCyccntAddr = 0xE0001004;
constexpr uint32_t kDemcrAddr = 0xE000EDFC;
constexpr uint32_t kDwtCycCntEnaBit = 1u << 0;
constexpr uint32_t kDemcrTraceEnaBit = 1u << 24;

constexpr uint32_t kAssumedCoreClockHz = 64000000;
static_assert(kAssumedCoreClockHz % 1000000U == 0, "core clock must be whole MHz");

volatile uint32_t& reg(uint32_t addr) { return *reinterpret_cast<volatile uint32_t*>(addr); }

bool g_dwt_started = false;

void ensure_dwt_started() {
    if (g_dwt_started) return;
    reg(kDemcrAddr) |= kDemcrTraceEnaBit;
    reg(kDwtCyccntAddr) = 0;
    reg(kDwtCtrlAddr) |= kDwtCycCntEnaBit;
    g_dwt_started = true;
}
} // namespace

uint64_t now_us() {
    ensure_dwt_started();
    // CYCCNT is 32 bits (wraps every ~67 s at 64 MHz). Extended to 64 bits
    // by counting wraps; monotonic as long as it is called at least once per
    // wrap period (the control loop calls it every tick). Not interrupt-safe:
    // call it from one execution context.
    static uint32_t s_last_cycles = 0;
    static uint64_t s_high = 0;
    const uint32_t cycles = reg(kDwtCyccntAddr);
    if (cycles < s_last_cycles) s_high += (1ULL << 32);
    s_last_cycles = cycles;
    return (s_high | cycles) / (kAssumedCoreClockHz / 1000000U);
}

uint64_t now_ms() { return now_us() / 1000ULL; }

uint64_t elapsed_us(uint64_t start_us) { return now_us() - start_us; }

void delay_us(uint32_t us) {
    ensure_dwt_started();
    const uint32_t start = reg(kDwtCyccntAddr);
    const uint32_t cycles = static_cast<uint32_t>((static_cast<uint64_t>(us) * kAssumedCoreClockHz) / 1000000ULL);
    while ((reg(kDwtCyccntAddr) - start) < cycles) {
        __asm__ volatile("nop");
    }
}

void delay_ms(uint32_t ms) { delay_us(ms * 1000U); }

} // namespace apx::kernel
