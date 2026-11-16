#pragma once
// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
// APX OS abstraction: interrupt-safe critical section.

#include <cstdint>

namespace apx::kernel {

// Scoped critical section: while an instance lives, no interrupt handler or
// other thread can run code that also takes a CriticalSection. Nestable.
// Keep the protected region short (a few register/buffer updates).
//
// Port mapping: Zephyr irq_lock/irq_unlock, bare-metal PRIMASK,
// FreeRTOS taskENTER_CRITICAL/taskEXIT_CRITICAL, ChibiOS chSysLock/Unlock,
// NuttX enter_critical_section/leave_critical_section, POSIX a global
// recursive mutex.
class CriticalSection {
public:
    CriticalSection();
    ~CriticalSection();

    CriticalSection(const CriticalSection&) = delete;
    CriticalSection& operator=(const CriticalSection&) = delete;

private:
    uint32_t m_state{0}; // port-specific saved interrupt state
};

} // namespace apx::kernel
