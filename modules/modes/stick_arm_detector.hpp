// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

namespace apx::modes {

// RC-stick arm/disarm gesture recognition:
// throttle held at minimum + yaw stick held full right
// for a sustained hold arms; throttle at minimum + yaw full left disarms.
// Ticked every frame from apx_main.cpp (see that file) alongside
// ModeManager::update() - this class only *recognizes* the gesture, it
// doesn't own the armed flag itself (that's MavlinkHandler::set_armed(),
// same authority MAV_CMD_COMPONENT_ARM_DISARM already uses) or decide
// whether arming should actually be allowed right now (that's
// ModeManager::can_arm(), which the caller should also check before
// acting on an ARM result).
class StickArmDetector {
public:
    enum class Result { NONE, ARM, DISARM };

    // rc_throttle in [0,1] (0=stick down), rc_yaw in [-1,1] (signed,
    // centered at 0) - same conventions VehicleState::rc_throttle/rc_yaw
    // already use everywhere else (see mode_loiter.cpp). Returns ARM/
    // DISARM the one tick the hold duration completes; NONE otherwise.
    Result update(bool armed, bool rc_valid, float rc_throttle, float rc_yaw, float dt_s);
    void reset();

private:
    // Real sticks (or their MAVLink equivalent) must be at minimum
    // throttle for either gesture - this is what stops an in-flight
    // yaw-heavy maneuver from ever being mistaken for a disarm request:
    // throttle is never near 0 while actually flying under normal
    // control, so kThrottleMinThreshold alone is most of the real-world
    // safety margin here, avoiding unintentional in-flight motor cutoff.
    static constexpr float kThrottleMinThreshold = 0.05f;
    static constexpr float kYawGestureThreshold = 0.9f;  // "held full over"
    static constexpr float kYawCenterBand = 0.2f;        // "returned to center"
    static constexpr float kHoldSeconds = 1.5f;

    // Requires the yaw stick to pass back through center after a trigger
    // (or after the RC link was invalid) before a new gesture can be
    // recognized - without this, simply continuing to hold the stick in
    // the arm/disarm corner would re-trigger every tick once the hold
    // duration is first reached.
    bool m_gesture_available{true};
    float m_arm_hold_s{0.0f};
    float m_disarm_hold_s{0.0f};
};

} // namespace apx::modes
