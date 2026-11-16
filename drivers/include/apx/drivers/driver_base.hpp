// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once
#include <cstdint>
#include <string_view>

namespace apx::drivers {

enum class DriverStatus { UNINITIALIZED, INITIALIZING, RUNNING, DEGRADED, FAILED, DISABLED };

class DriverBase {
public:
    explicit DriverBase(std::string_view name = "driver") : m_name(name) {}
    virtual ~DriverBase() = default;

    virtual bool init() = 0;
    virtual void deinit() {}
    virtual void update() {}
    virtual bool is_healthy() const;

    DriverStatus     get_status() const { return m_status; }
    std::string_view get_name()   const { return m_name; }

protected:
    void set_status(DriverStatus s) { m_status = s; }
    void mark_updated();

    uint64_t m_last_update_us{0};

private:
    std::string_view m_name;
    DriverStatus     m_status{DriverStatus::UNINITIALIZED};
    static constexpr uint64_t kHealthTimeoutUs = 1000000; // 1 second
};

} // namespace apx::drivers
