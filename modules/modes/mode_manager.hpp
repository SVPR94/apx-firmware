// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include <atomic>

#include <memory>
#include <optional>
#include <cstdint>
#include <vector>
#include "mode_base.hpp"
#include "crash_detector.hpp"
#include "mode_auto.hpp"

namespace apx::modes {

// Single place mode switching/transition logic lives: adding a new mode
// means (1) a FlightMode subclass, (2) one line in create_mode()'s switch,
// (3) if it should be GCS-selectable, one line in the custom-mode tables below.
// Nothing else in the codebase needs to change.
class ModeManager {
public:
    ModeManager();

    // Switches mode if preconditions are met (arming state, GPS/position
    // availability vs. what the target mode requires); returns false and
    // leaves the current mode active otherwise. Safe to call with the mode
    // already active (no-op, returns true).
    bool request_mode(FlightModeId id, const VehicleState& state);

    // MAV_CMD_NAV_TAKEOFF: if not already in GUIDED this switches to it first
    // (matching GCS's own "Takeoff" action, which forces GUIDED), then
    // asks the active mode to climb.
    bool request_takeoff(float target_rel_alt_m, const VehicleState& state);

    // MAV_CMD_NAV_LAND: switches to LAND mode.
    bool request_land(const VehicleState& state);

    // SET_POSITION_TARGET_LOCAL_NED/GLOBAL_INT, MAV_CMD_DO_REPOSITION:
    // only takes effect while GUIDED is active (ignores external setpoints
    // in any other mode) - returns false if ignored for that reason.
    bool set_guided_target(const GuidedTarget& target);

    // MISSION_COUNT upload completing (see mavlink_handler.cpp): stored
    // here, not inside a ModeAuto instance, because create_mode() makes a
    // fresh ModeAuto every time AUTO is (re-)entered (same as every other
    // mode) - this is what a freshly-entered ModeAuto gets loaded with.
    // Uploading a new mission while AUTO is already active does NOT
    // retroactively change what the running ModeAuto is flying (a mission
    // upload takes effect on next AUTO entry).
    void load_mission(const std::vector<Waypoint>& mission) { m_mission = mission; }

    // MAV_CMD_DO_CHANGE_SPEED passthrough to the active mode (AUTO only
    // today); false if the active mode doesn't support it.
    bool set_speed_xy(float speed_ms) { return m_current->set_speed_xy(speed_ms); }

    // -1 when not in AUTO or AUTO has no mission progress to report - see
    // ModeAuto::current_seq(). Polled by mavlink_handler.cpp to send
    // MISSION_CURRENT/MISSION_ITEM_REACHED.
    int auto_mission_current_seq() const;

    ControlOutput update(const VehicleState& state);

    FlightModeId current_id() const { return m_current->id(); }
    std::string_view current_name() const { return m_current->get_name(); }
    // FlightMode::can_arm() passthrough - was declared on every mode
    // (default true) but never actually called from anywhere; both the
    // MAVLink ARM command handler and the new RC stick-arm gesture use
    // this so a mode that overrides it (e.g. to refuse arming without a
    // GPS fix) is actually enforced regardless of which path requested
    // arming.
    bool can_arm() const { return m_current->can_arm(); }

    // A non-forced disarm (RC stick gesture or MAVLink) is only honoured on
    // the ground - in the air it would cut the motors - except in
    // STABILIZE, where throttle is manual.
    bool disarm_allowed() const;

    // MAV_CMD_DO_SET_MODE / heartbeat custom_mode <-> FlightModeId.
    // Standard custom mode numbering for seamless GCS interoperability.
    static uint32_t to_custom_mode_number(FlightModeId id);
    static std::optional<FlightModeId> from_custom_mode_number(uint32_t custom_mode);

private:
    std::unique_ptr<FlightMode> create_mode(FlightModeId id);

    std::unique_ptr<FlightMode> m_current;
    // Checked every update() regardless of m_current - see
    // crash_detector.hpp for why this lives here and not inside a mode.
    CrashDetector m_crash_detector;
    bool m_crash_logged{false}; // so the [APX SAFETY] line prints once per incident, not every tick
    // Mode-independent "on the ground" check for disarm_allowed(): seconds
    // the vehicle has continuously looked landed (resting altitude, no climb
    // rate, mode thrust well below hover). Atomic: written by the control
    // loop, read by the MAVLink thread.
    std::atomic<float> m_ground_time_s{0.0f};
    std::vector<Waypoint> m_mission;
};

} // namespace apx::modes
