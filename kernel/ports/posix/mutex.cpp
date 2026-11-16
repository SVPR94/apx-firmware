// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
// APX OS port: POSIX / hosted - mutex and critical section.

#include <mutex>

#include "apx/kernel/critical.hpp"
#include "apx/kernel/mutex.hpp"

namespace apx::kernel {

struct Mutex::Impl {
    std::mutex m;
};

Mutex::Mutex() : m_impl(new Impl()) {}
Mutex::~Mutex() = default;
void Mutex::lock() { m_impl->m.lock(); }
void Mutex::unlock() { m_impl->m.unlock(); }

// No interrupts on a hosted OS: a process-wide recursive lock gives the
// same "nothing else runs this section concurrently" guarantee.
namespace {
std::recursive_mutex& critical_lock() {
    static std::recursive_mutex m;
    return m;
}
} // namespace

CriticalSection::CriticalSection() { critical_lock().lock(); }
CriticalSection::~CriticalSection() { critical_lock().unlock(); }

} // namespace apx::kernel
