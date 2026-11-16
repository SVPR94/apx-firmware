// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

namespace apx::modes {

// Shared by ModeLand and ModeRTL's final descent leg. Three independent,
// physically-grounded signals (not a timeout): resting altitude, near-zero
// climb rate, and thrust well below what hover would need (the ground -
// not the rotors - is supporting the vehicle's weight). All three must
// hold continuously for kConfirmSeconds before touchdown is declared, the
// same way a real landing detector debounces against a single noisy
// sample instead of triggering on one.
class LandDetector {
public:
    // Call every tick with the current state; returns true once touchdown
    // has been confirmed (sticky - keeps returning true afterward until
    // reset()).
    bool update(float rel_alt_m, float climb_rate_ms, float thrust, float dt_s);
    void reset();
    bool landed() const { return m_landed; }

private:
    // Same hover-throttle estimate the thrust model in
    // apx_gazebo_plugin.cpp implies for this airframe (11N/motor max,
    // throttle^2 curve, ~2.2kg): hover ~= sqrt(weight / (4*11N)) ~= 0.70.
    // TODO: hardcoded estimate, not learned - adapt via
    // MOT_THST_HOVER online.
    static constexpr float kHoverThrottleEstimate = 0.70f;
    static constexpr float kThrottleFraction = 0.6f;   // "well below hover"
    static constexpr float kClimbRateThreshold = 0.05f; // m/s
    // Above the resting height of any airframe this detector runs on -
    // not just the ~0.20m-tall apx_quad landing gear this was originally
    // tuned for. Confirmed live: apx_hex's landing gear rests at ~0.6m
    // relative altitude (a genuinely taller real-world undercarriage, not
    // a bug - see sitl/models/apx_hex/model.sdf), which the previous
    // 0.30m threshold sat BELOW - LAND would descend and hold correctly
    // but "landed" could never actually become true, so it never
    // auto-disarmed (the exact same class of bug this threshold's own
    // history already records once before, when it was 0.15m and sat
    // below apx_quad's own resting height). 1.5m comfortably clears any
    // reasonable landing gear height while staying far below any normal
    // cruise altitude, so it isn't a meaningfully weaker anti-false-
    // positive gate than a tighter, airframe-specific value would be -
    // the climb-rate and thrust conditions above do the real work of
    // rejecting "still flying" ticks.
    static constexpr float kAltThreshold = 1.5f; // m
    static constexpr float kConfirmSeconds = 0.5f;

    float m_confirm_time_s{0.0f};
    bool m_landed{false};
};

} // namespace apx::modes
