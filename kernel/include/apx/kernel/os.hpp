#pragma once
// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
// APX OS abstraction: identity and scheduling helpers.
//
// The OS abstraction is the apx::kernel namespace:
//   timer.hpp     now_us/now_ms/elapsed_us/delay_us/delay_ms
//   mutex.hpp     Mutex, LockGuard
//   critical.hpp  CriticalSection
//   periodic.hpp  PeriodicTimer
//   os.hpp        os_name, yield
// Each OS implements it once under kernel/ports/<os>/ (see
// kernel/ports/README.md). Code outside kernel/ports/ and platforms/ must not
// include OS headers directly.

namespace apx::kernel {

// Short OS identifier: "zephyr", "posix", "baremetal", ...
const char* os_name();

// Give up the CPU to another ready thread of the same priority (no-op
// without a scheduler).
void yield();

} // namespace apx::kernel
