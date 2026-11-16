// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include <string_view>
#include "vehicle_state.hpp"

namespace apx::modes {

// Custom mode enum for APX flight modes (see
// mode_manager.cpp's to_custom_mode_number()/from_custom_mode_number()) so
// standard GCS mode selection and MAV_CMD_DO_SET_MODE
// work reliably on the GCS side. GUIDED is the mode
// MAV_CMD_NAV_TAKEOFF/SET_POSITION_TARGET_*/MAV_CMD_DO_REPOSITION act on;
// takeoff is a sub-behavior inside ModeGuided (see mode_guided.hpp).
enum class FlightModeId {
    STABILIZE,
    ALT_HOLD,
    AUTO,
    GUIDED,
    LOITER,   // treated as POS_HOLD-equivalent (hold current position)
    RTL,
    LAND,
    POS_HOLD,
    BRAKE,    // custom mode 17: immediate pause & 3D position lock
    AGRI_SPRAY,
    AGRI_ZIGZAG,
};

class FlightMode {
public:
    virtual ~FlightMode() = default;

    // state is the vehicle's condition at the moment of entry - e.g. so a
    // hold-position mode can capture "here" without a separate call.
    virtual void on_enter(const VehicleState& state) = 0;
    virtual void on_exit() = 0;
    virtual ControlOutput update(const VehicleState& state) = 0;

    virtual FlightModeId id() const = 0;
    virtual std::string_view get_name() const = 0;

    virtual bool can_arm() const { return true; }
    virtual bool is_position_required() const { return false; }
    virtual bool is_gps_required() const { return false; }

    // GUIDED (and anything else that wants to) can accept a "go here" from
    // the GCS. Modes that don't care about this just don't override it.
    virtual void set_guided_target(const GuidedTarget&) {}

    // GUIDED-only: MAV_CMD_NAV_TAKEOFF. Same default-no-op pattern.
    virtual void request_takeoff(float /*target_rel_alt_m*/) {}

    // MAV_CMD_DO_CHANGE_SPEED (horizontal). speed_ms <= 0 restores the
    // mode's default. Returns false if the mode doesn't support it.
    virtual bool set_speed_xy(float /*speed_ms*/) { return false; }
};

} // namespace apx::modes
