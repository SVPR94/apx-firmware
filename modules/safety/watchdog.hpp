// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include <string>
#include <unordered_map>
#include <cstdint>

namespace apx::safety {

class Watchdog {
public:
    Watchdog();
    ~Watchdog() = default;

    void register_task(const std::string& name, float timeout_s);
    void kick(const std::string& name);
    void update(float dt_s);

private:
    struct TaskInfo {
        float timeout_s;
        float time_since_last_kick;
    };

    std::unordered_map<std::string, TaskInfo> m_tasks;
    void hardware_watchdog_kick();
};

} // namespace apx::safety
