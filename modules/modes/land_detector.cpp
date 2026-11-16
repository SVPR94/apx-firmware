// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "land_detector.hpp"
#include <cmath>

namespace apx::modes {

bool LandDetector::update(float rel_alt_m, float climb_rate_ms, float thrust, float dt_s) {
    if (m_landed) return true;

    bool likely_landed = (std::fabs(climb_rate_ms) < kClimbRateThreshold) &&
                         (rel_alt_m < kAltThreshold) &&
                         (thrust < kHoverThrottleEstimate * kThrottleFraction);

    m_confirm_time_s = likely_landed ? (m_confirm_time_s + dt_s) : 0.0f;
    m_landed = m_confirm_time_s >= kConfirmSeconds;
    return m_landed;
}

void LandDetector::reset() {
    m_confirm_time_s = 0.0f;
    m_landed = false;
}

} // namespace apx::modes
