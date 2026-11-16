// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include "mode_base.hpp"
#include "navigation_cascade.hpp"
#include "land_detector.hpp"

namespace apx::modes {

class ModeLoiter : public FlightMode {
public:
    ModeLoiter();
    ~ModeLoiter() override = default;

    void on_enter(const VehicleState& state) override;
    void on_exit() override;
    ControlOutput update(const VehicleState& state) override;

    FlightModeId id() const override { return FlightModeId::LOITER; }
    std::string_view get_name() const override { return "LOITER"; }

    bool is_position_required() const override { return true; }
    bool is_gps_required() const override { return true; }

private:
    NavigationCascade m_cascade;
    // Landing: once LandDetector confirms touchdown the motors drop to idle
    // (MOT_SPIN_ARM) and the vehicle stays armed - it no longer disarms the
    // moment it touches down. It disarms after DISARM_DELAY seconds on the
    // ground with throttle at zero, or by the throttle-down + yaw-left stick
    // gesture (apx_main.cpp). Throttle above mid-stick takes off again.
    LandDetector m_land_detector;
    // True while sitting on the ground: after arming in this mode, or after
    // touchdown. "Landed" also matches a freshly-armed vehicle that hasn't
    // lifted off yet, so the ground state starts true when the mode sees the
    // vehicle disarmed and is only cleared by a climb command.
    bool m_on_ground{false};
    float m_disarm_timer_s{0.0f};
    float m_disarm_delay_s{30.0f}; // DISARM_DELAY, 0 = never auto-disarm
    static constexpr float kThrottleZero = 0.05f;   // same as StickArmDetector's minimum
    static constexpr float kThrottleClimb = 0.58f;  // top of the mid-stick deadband
    float m_target_pos_ned[3]{0.0f, 0.0f, 0.0f};
    float m_target_yaw{0.0f};

    // Read from ParamManager at construction time - see ModeLoiter's
    // constructor. m_max_horiz_speed_ms is LOIT_SPEED clamped to never
    // exceed WPNAV_SPEED - the same param NavigationCascade's own
    // internal target-slew limiter uses (see navigation_cascade.cpp) - so
    // this raw stick-driven target can never outrun what the cascade can
    // actually track. This used to be a separate, faster constant (3.5
    // m/s vs the cascade's then-hardcoded 2.0 m/s slew rate), so sustained
    // stick deflection made the smoothed target the cascade actually
    // flies toward fall further and further behind - an unbounded,
    // ever-growing position error for as long as the stick was held, not
    // the bounded tracking lag a single stepped target gets. Confirmed
    // live: combined with active yaw that grew into a full flip. The
    // std::min() clamp at construction makes that mismatch structurally
    // impossible regardless of how LOIT_SPEED is tuned.
    float m_max_horiz_speed_ms{1.8f};
    float m_max_climb_speed_ms{2.5f};
    float m_max_desc_speed_ms{1.5f};
    // AttitudeController's own real max yaw rate (ATC_RATE_Y_MAX) - was a
    // separate, faster hardcoded constant (90deg/s vs the controller's
    // 45deg/s default), so at full stick deflection this target ramp
    // permanently outran what the vehicle could ever physically catch up
    // to: yaw error grew without bound for as long as the stick was held,
    // not just a brief tracking lag.
    float m_max_yaw_rate_rads{0.7f};

    static constexpr float kStickDeadband   = 0.05f;
};

} // namespace apx::modes
