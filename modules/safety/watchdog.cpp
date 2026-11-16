// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "watchdog.hpp"
#include <iostream>

namespace apx::safety {

Watchdog::Watchdog() {
    // Init hardware watchdog if on Zephyr
}

void Watchdog::register_task(const std::string& name, float timeout_s) {
    m_tasks[name] = {timeout_s, 0.0f};
}

void Watchdog::kick(const std::string& name) {
    if (m_tasks.find(name) != m_tasks.end()) {
        m_tasks[name].time_since_last_kick = 0.0f;
    }
}

void Watchdog::update(float dt_s) {
    bool all_ok = true;
    for (auto& [name, info] : m_tasks) {
        info.time_since_last_kick += dt_s;
        if (info.time_since_last_kick > info.timeout_s) {
            std::cerr << "Watchdog timeout on task: " << name << "\n";
            all_ok = false;
            // Trigger failsafe or reboot
        }
    }

    if (all_ok) {
        hardware_watchdog_kick();
    }
}

void Watchdog::hardware_watchdog_kick() {
    // e.g. task_wdt_feed() in Zephyr
}

} // namespace apx::safety
