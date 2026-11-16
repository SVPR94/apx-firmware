// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include "mode_base.hpp"
#include "navigation_cascade.hpp"
#include "land_detector.hpp"

namespace apx::modes {

// RTL: climb to m_rtl_altitude (if not already above it) at the current
// position, navigate to home (NED origin) at that altitude, then hand off
// to the same descent+LandDetector logic ModeLand uses. Position/velocity
// in VehicleState are already NED-relative-to-home (see
// apx_main.cpp/apx_gazebo_plugin.cpp), so "home" is simply (0,0).
class ModeRTL : public FlightMode {
public:
    ModeRTL();
    ~ModeRTL() override = default;

    void on_enter(const VehicleState& state) override;
    void on_exit() override;
    ControlOutput update(const VehicleState& state) override;

    FlightModeId id() const override { return FlightModeId::RTL; }
    std::string_view get_name() const override { return "RTL"; }

    bool is_position_required() const override { return true; }
    bool is_gps_required() const override { return true; }

    void set_rtl_altitude(float alt) { m_rtl_altitude_m = alt; }

private:
    enum class RtlState { CLIMBING, RETURNING, DESCENDING };

    static constexpr float kAltToleranceM = 0.3f;
    static constexpr float kHomeRadiusM = 1.0f;
    static constexpr float kDescentRateMs = 0.5f;

    RtlState m_state{RtlState::CLIMBING};
    float m_rtl_altitude_m{15.0f}; // RTL_ALT, read in the constructor
    float m_climb_pos_ned[2]{0, 0}; // horizontal position held during CLIMBING

    NavigationCascade m_cascade;
    LandDetector m_detector;
};

} // namespace apx::modes
