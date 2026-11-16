// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "apx/kernel/timer.hpp"

// Was an empty stub - now_us()/now_ms()/elapsed_us() were declared in
// timer.hpp and called from several drivers/modules/estimator/ekf3.cpp
// (see their apx::kernel::now_us() calls) but never defined anywhere,
// which only stayed invisible because none of those translation units
// are part of the SITL link (SITL's VehicleState timing comes straight
// from apx_main.cpp's own loop, not this). Any target that actually
// pulls drivers/ in - starting with platforms/stm32h7 - needs a real
// definition to link at all.

#if defined(APX_PLATFORM_ZEPHYR)

// Zephyr: time from the kernel's 64-bit hardware cycle counter (SysTick at
// the core clock), so dt in the 400 Hz loop is not quantised to the
// scheduler tick.
#include <zephyr/kernel.h>

namespace apx::kernel {

uint64_t now_us() {
#if defined(CONFIG_TIMER_HAS_64BIT_CYCLE_COUNTER)
    return k_cyc_to_us_floor64(k_cycle_get_64());
#else
    return k_ticks_to_us_floor64(k_uptime_ticks());
#endif
}

uint64_t now_ms() {
    return now_us() / 1000ULL;
}

uint64_t elapsed_us(uint64_t start_us) {
    return now_us() - start_us;
}

void delay_us(uint32_t us) {
    k_usleep(static_cast<int32_t>(us));
}

void delay_ms(uint32_t ms) {
    k_msleep(static_cast<int32_t>(ms));
}

} // namespace apx::kernel

#elif defined(APX_PLATFORM_STM32H7)

// Bare-metal: no OS clock to call into. Uses the Cortex-M7 DWT cycle
// counter (present on every M7, no extra peripheral needed) rather than
// a hardware timer peripheral, so this works before any timer driver
// exists. Accuracy depends on the actual core clock - see
// board_config.hpp's clock constants and platforms/stm32h7/README.md's
// "No SystemInit()" note: until that lands, the core runs on the ~64MHz
// HSI reset clock, not the 400MHz this assumes, so now_us()/now_ms() will
// under-report elapsed time by roughly that ratio until SystemInit() sets
// the real clock. Tracked there, not hidden here.
#include <cstdint>

namespace apx::kernel {
namespace {
// Cortex-M7 Data Watchpoint and Trace unit registers (ARMv7-M
// architecture-defined addresses, not board-specific).
constexpr uint32_t kDwtCtrlAddr = 0xE0001000;
constexpr uint32_t kDwtCyccntAddr = 0xE0001004;
constexpr uint32_t kDemcrAddr = 0xE000EDFC;
constexpr uint32_t kDwtCycCntEnaBit = 1u << 0;
constexpr uint32_t kDemcrTraceEnaBit = 1u << 24;

volatile uint32_t& reg(uint32_t addr) {
    return *reinterpret_cast<volatile uint32_t*>(addr);
}

bool g_dwt_started = false;

// board_config.hpp documents a 400MHz target core clock; SystemInit()
// doesn't yet configure the PLL to reach it (see this file's header
// comment), so this reflects what's actually running today, not the
// eventual target - update alongside SystemInit() when that lands.
constexpr uint32_t kAssumedCoreClockHz = 64000000;

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
    // CYCCNT is only 32 bits (wraps every ~67 s at 64 MHz, ~10.7 s at
    // 400 MHz). Extend it to 64 bits by counting wraps, so now_us() stays
    // monotonic as long as it is called at least once per wrap period
    // (the control loop calls it every tick). Not interrupt-safe: call it
    // from one execution context until a hardware-timer source replaces it.
    static uint32_t s_last_cycles = 0;
    static uint64_t s_high = 0;
    const uint32_t cycles = reg(kDwtCyccntAddr);
    if (cycles < s_last_cycles) {
        s_high += (1ULL << 32);
    }
    s_last_cycles = cycles;
    const uint64_t total = s_high | cycles;
    // Divide by cycles-per-microsecond (core clock is a whole number of MHz)
    // rather than multiplying by 1e6 first, which overflows after ~13 h.
    static_assert(kAssumedCoreClockHz % 1000000U == 0, "core clock must be whole MHz");
    return total / (kAssumedCoreClockHz / 1000000U);
}

uint64_t now_ms() {
    return now_us() / 1000ULL;
}

uint64_t elapsed_us(uint64_t start_us) {
    return now_us() - start_us;
}

void delay_us(uint32_t us) {
    ensure_dwt_started();
    uint32_t start = reg(kDwtCyccntAddr);
    uint32_t cycles = static_cast<uint32_t>((static_cast<uint64_t>(us) * kAssumedCoreClockHz) / 1000000ULL);
    while ((reg(kDwtCyccntAddr) - start) < cycles) {
        __asm__ volatile("nop");
    }
}

void delay_ms(uint32_t ms) {
    delay_us(ms * 1000U);
}

} // namespace apx::kernel

#else // hosted (SITL/Linux)

#include <chrono>
#include <thread>

namespace apx::kernel {

uint64_t now_us() {
    using namespace std::chrono;
    return static_cast<uint64_t>(duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count());
}

uint64_t now_ms() {
    return now_us() / 1000ULL;
}

uint64_t elapsed_us(uint64_t start_us) {
    return now_us() - start_us;
}

void delay_us(uint32_t us) {
    std::this_thread::sleep_for(std::chrono::microseconds(us));
}

void delay_ms(uint32_t ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

} // namespace apx::kernel

#endif
