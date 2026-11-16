// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include <string>
#include <functional>
#include <cstdint>
#include <memory>

namespace apx::kernel {

class Task {
public:
    Task(const std::string& name, size_t stack_size, int priority, std::function<void()> fn);
    ~Task();

    void start();
    void stop();
    void join();
    
    void set_period_us(uint32_t us);
    void sleep_until_next();

    static std::shared_ptr<Task> create_periodic(
        const std::string& name, 
        size_t stack_size, 
        int priority, 
        uint32_t period_us, 
        std::function<void()> fn);

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace apx::kernel
