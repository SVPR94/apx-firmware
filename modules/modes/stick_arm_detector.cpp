// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "stick_arm_detector.hpp"
#include <cmath>

namespace apx::modes {

StickArmDetector::Result StickArmDetector::update(bool armed, bool rc_valid,
                                                   float rc_throttle, float rc_yaw, float dt_s) {
    if (!rc_valid) {
        reset();
        return Result::NONE;
    }

    bool yaw_centered = std::fabs(rc_yaw) < kYawCenterBand;
    if (!m_gesture_available) {
        if (yaw_centered) m_gesture_available = true;
        m_arm_hold_s = 0.0f;
        m_disarm_hold_s = 0.0f;
        return Result::NONE;
    }

    bool throttle_low = rc_throttle < kThrottleMinThreshold;
    bool yaw_right = rc_yaw > kYawGestureThreshold;
    bool yaw_left = rc_yaw < -kYawGestureThreshold;

    if (throttle_low && yaw_right && !armed) {
        m_arm_hold_s += dt_s;
        m_disarm_hold_s = 0.0f;
    } else if (throttle_low && yaw_left && armed) {
        m_disarm_hold_s += dt_s;
        m_arm_hold_s = 0.0f;
    } else {
        m_arm_hold_s = 0.0f;
        m_disarm_hold_s = 0.0f;
    }

    if (m_arm_hold_s >= kHoldSeconds) {
        m_gesture_available = false;
        m_arm_hold_s = m_disarm_hold_s = 0.0f;
        return Result::ARM;
    }
    if (m_disarm_hold_s >= kHoldSeconds) {
        m_gesture_available = false;
        m_arm_hold_s = m_disarm_hold_s = 0.0f;
        return Result::DISARM;
    }
    return Result::NONE;
}

void StickArmDetector::reset() {
    m_gesture_available = true;
    m_arm_hold_s = 0.0f;
    m_disarm_hold_s = 0.0f;
}

} // namespace apx::modes
