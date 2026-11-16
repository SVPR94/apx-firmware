// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "crash_detector.hpp"
#include <cmath>

namespace apx::modes {

bool CrashDetector::update(float roll_rad, float pitch_rad, float rel_alt_m, float climb_rate_ms,
                           bool armed, float dt_s) {
    // Scoped to "while armed", not sticky forever: once the crash forces
    // a disarm (see mode_manager.cpp), the *next* tick sees armed=false
    // here and this clears itself, so a later re-arm gets a clean slate
    // instead of being permanently blocked by one past incident.
    if (!armed) {
        reset();
        return false;
    }

    if (m_crashed) return true;

    bool excessive_angle = std::fabs(roll_rad) > kCrashAngleRad || std::fabs(pitch_rad) > kCrashAngleRad;
    m_angle_confirm_time_s = excessive_angle ? (m_angle_confirm_time_s + dt_s) : 0.0f;

    bool hard_impact = (rel_alt_m < kImpactAltThreshold) && (climb_rate_ms < -kImpactClimbRateThreshold);
    m_impact_confirm_time_s = hard_impact ? (m_impact_confirm_time_s + dt_s) : 0.0f;

    m_crashed = (m_angle_confirm_time_s >= kConfirmSeconds) || (m_impact_confirm_time_s >= kConfirmSeconds);
    return m_crashed;
}

void CrashDetector::reset() {
    m_angle_confirm_time_s = 0.0f;
    m_impact_confirm_time_s = 0.0f;
    m_crashed = false;
}

} // namespace apx::modes
