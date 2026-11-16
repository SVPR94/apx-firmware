// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

namespace apx::modes {

// Mode-independent safety net, checked every tick by ModeManager
// regardless of which FlightMode is active (see mode_manager.cpp) - unlike
// LandDetector, which only ModeLand/ModeRTL's final leg care about, a
// crash can happen in any mode (a diverging guided move, a physics
// anomaly, anything that gets the vehicle badly inverted while still
// armed and trying to fly). Two independent signals, each sustained for
// kConfirmSeconds to reject a single noisy sample:
//   - attitude far enough past any legitimate flight envelope (well
//     beyond NavigationCascade's own safety tilt clamp - see
//     navigation_cascade.cpp's kSafetyTiltRad) that self-leveling isn't
//     recovering it
//   - a real, sustained ground impact while armed: near-zero altitude
//     reached at a high descent rate outside of LAND's own controlled,
//     detected touchdown (ModeLand/ModeRTL already have LandDetector for
//     the normal case - this is for hitting the ground any *other* way)
class CrashDetector {
public:
    // Call every tick with the current state; returns true once a crash
    // has been confirmed (sticky - keeps returning true afterward until
    // reset()).
    bool update(float roll_rad, float pitch_rad, float rel_alt_m, float climb_rate_ms, bool armed, float dt_s);
    void reset();
    bool crashed() const { return m_crashed; }

private:
    // Comfortably beyond navigation_cascade.cpp's kSafetyTiltRad (40deg) -
    // that clamp is meant to catch and recover from an aggressive-but-
    // recoverable excursion; this threshold is for "not coming back from
    // this without help".
    static constexpr float kCrashAngleRad = 1.396f; // 80deg
    // A real, uncontrolled impact: high sink rate still present at
    // essentially zero altitude - LAND's own controlled touchdown always
    // arrives with climb rate near zero (see land_detector.hpp), so this
    // only fires for the abnormal case.
    static constexpr float kImpactAltThreshold = 0.30f; // m
    static constexpr float kImpactClimbRateThreshold = 1.5f; // m/s (descending)
    static constexpr float kConfirmSeconds = 0.3f;

    float m_angle_confirm_time_s{0.0f};
    float m_impact_confirm_time_s{0.0f};
    bool m_crashed{false};
};

} // namespace apx::modes
