#pragma once
// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
// APX OS abstraction: fixed-rate loop timer.

#include <cstdint>
#include <memory>

namespace apx::kernel {

// Drives a fixed-rate loop (e.g. the 400 Hz flight loop) without drift:
//
//     PeriodicTimer tick(2500);   // 400 Hz
//     tick.start();
//     while (true) { tick.wait(); ...one iteration... }
//
// wait() blocks the calling thread until the next period boundary and
// returns the number of periods that elapsed since the previous wait():
// 1 normally, more when an iteration overran (those ticks are skipped,
// not queued).
class PeriodicTimer {
public:
    explicit PeriodicTimer(uint32_t period_us);
    ~PeriodicTimer();

    PeriodicTimer(const PeriodicTimer&) = delete;
    PeriodicTimer& operator=(const PeriodicTimer&) = delete;

    void start();
    void stop();
    uint32_t wait();
    uint32_t period_us() const { return m_period_us; }

private:
    struct Impl;
    uint32_t m_period_us;
    std::unique_ptr<Impl> m_impl;
};

} // namespace apx::kernel
