// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once
#include <cstdint>
#include <cmath>

namespace apx::sensor {

enum class SensorHealthStatus { UNKNOWN, HEALTHY, DEGRADED, FAILED };

class SensorHealth {
public:
    void mark_updated(uint64_t now_us);
    void mark_failed();
    float compute_score(uint64_t now_us) const;
    bool is_usable(uint64_t now_us) const;
    SensorHealthStatus get_status(uint64_t now_us) const;
    void set_timeout_us(uint64_t us) { m_timeout_us = us; }
    uint32_t get_consecutive_failures() const { return m_consecutive_failures; }
    void reset();

private:
    uint64_t m_last_update_us{0};
    uint64_t m_timeout_us{200000}; // 200ms
    uint32_t m_consecutive_failures{0};
    static constexpr float kDegradeThreshold = 0.5f;
    static constexpr float kFailThreshold    = 0.2f;
    static constexpr uint32_t kMaxFailures   = 10;
};

} // namespace apx::sensor
