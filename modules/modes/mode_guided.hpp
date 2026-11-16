// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include "mode_base.hpp"
#include "navigation_cascade.hpp"

namespace apx::modes {

// GUIDED: the mode MAV_CMD_NAV_TAKEOFF, SET_POSITION_TARGET_LOCAL_NED/
// GLOBAL_INT and MAV_CMD_DO_REPOSITION all act on - there is no
// separate "TAKEOFF" custom_mode; takeoff is a sub-behavior here
// (request_takeoff() sets the altitude target and lets the navigation cascade climb to it).
//
// Behavior: always holds m_target_pos_ned via NavigationCascade. On entry
// the target is "stay right here" (current position at time of entry, or
// arm-time position if entered while disarmed on the ground). A GCS
// target (set_guided_target) or a takeoff request (request_takeoff) just
// changes what "here" means - the control loop doesn't otherwise care
// which one moved it.
class ModeGuided : public FlightMode {
public:
    ModeGuided();
    ~ModeGuided() override = default;

    void on_enter(const VehicleState& state) override;
    void on_exit() override;
    ControlOutput update(const VehicleState& state) override;

    FlightModeId id() const override { return FlightModeId::GUIDED; }
    std::string_view get_name() const override { return "GUIDED"; }

    bool is_position_required() const override { return true; }
    bool is_gps_required() const override { return true; }

    void set_guided_target(const GuidedTarget& target) override;
    void request_takeoff(float target_rel_alt_m) override;

private:
    NavigationCascade m_cascade;
    float m_target_pos_ned[3]{0, 0, 0};
    bool m_has_yaw_target{false};
    bool m_has_explicit_yaw{false};
    float m_target_yaw{0};
};

} // namespace apx::modes
