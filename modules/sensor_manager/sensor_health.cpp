// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "sensor_health.hpp"

namespace apx::sensor {

void SensorHealth::mark_updated(uint64_t now_us) {
    m_last_update_us = now_us;
    if (m_consecutive_failures > 0) m_consecutive_failures--;
}

void SensorHealth::mark_failed() {
    if (m_consecutive_failures < kMaxFailures) m_consecutive_failures++;
}

float SensorHealth::compute_score(uint64_t now_us) const {
    if (m_last_update_us == 0) return 0.0f;
    uint64_t age = now_us - m_last_update_us;
    // Score decays with age and failures
    float age_score = (age < m_timeout_us) ? 1.0f - static_cast<float>(age) / m_timeout_us : 0.0f;
    float fail_score = 1.0f - static_cast<float>(m_consecutive_failures) / kMaxFailures;
    return age_score * fail_score;
}

bool SensorHealth::is_usable(uint64_t now_us) const {
    return compute_score(now_us) >= kDegradeThreshold;
}

SensorHealthStatus SensorHealth::get_status(uint64_t now_us) const {
    float s = compute_score(now_us);
    if (s >= kDegradeThreshold) return SensorHealthStatus::HEALTHY;
    if (s >= kFailThreshold)    return SensorHealthStatus::DEGRADED;
    if (m_last_update_us == 0)  return SensorHealthStatus::UNKNOWN;
    return SensorHealthStatus::FAILED;
}

void SensorHealth::reset() {
    m_last_update_us = 0;
    m_consecutive_failures = 0;
}

} // namespace apx::sensor
