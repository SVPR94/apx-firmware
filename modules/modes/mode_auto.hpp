// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include "mode_base.hpp"
#include "navigation_cascade.hpp"
#include "land_detector.hpp"
#include "path_tracker.hpp"
#include <vector>
#include <cstdint>

namespace apx::modes {

// One mission item, already resolved to NED-relative-to-home by the
// caller (MavlinkHandler - see its handle_mission_item_int()) so this
// module never needs to know about MAVLink frames/lat-lon, the same
// pattern GuidedTarget already uses for SET_POSITION_TARGET_*.
struct Waypoint {
    // Original MAVLink mission seq (post seq-0-skip - see
    // MavlinkHandler::convert_mission_to_waypoints()) - kept so
    // current_seq() reports the number the GCS actually uploaded that
    // item as, not this vector's own (seq-0-shifted) index.
    uint16_t seq{0};
    float pos_ned[3]{0, 0, 0};
    bool has_location{true};
    // Raw MAV_CMD_NAV_* value (see the constants in mode_auto.cpp) - only
    // WAYPOINT/TAKEOFF/LAND/RETURN_TO_LAUNCH are actually sequenced
    // specially; anything else is treated as an instant pass-through
    // (arrives immediately, mission moves on) rather than rejected, so an
    // unsupported DO_* item in an otherwise-normal mission doesn't stall
    // the whole thing.
    uint16_t command{16}; // MAV_CMD_NAV_WAYPOINT
    // Mission item's own acceptance radius (its param2 for NAV_WAYPOINT) -
    // falls back to WPNAV_RADIUS (see mode_auto.cpp) when the GCS
    // sent 0 (standard "use default" convention).
    float accept_radius_m{0.0f};
    // Mission item's param1 (DO_SPRAYER on/off; NAV_WAYPOINT hold time; DO_SET_SERVO channel).
    float param1{0.0f};
    // Mission item's param2 (DO_SET_SERVO PWM value; DO_CHANGE_SPEED speed).
    float param2{0.0f};
    float param3{0.0f};
    float param4{0.0f};
};

// AUTO: real waypoint sequencing over whatever mission MavlinkHandler most
// recently uploaded (see mode_manager.hpp's load_mission()/m_mission -
// ModeAuto instances are recreated on every mode entry, same as every
// other mode, so the mission itself has to live above that, in
// ModeManager, and get handed to a fresh ModeAuto each time AUTO is
// entered). Falls back to holding the entry position/altitude - same as
// before - only when there's no mission loaded, so selecting AUTO with an
// empty mission is still the safe no-op it always was rather than
// dropping the vehicle.
class ModeAuto : public FlightMode {
public:
    ModeAuto();
    ~ModeAuto() override = default;

    void on_enter(const VehicleState& state) override;
    void on_exit() override;
    ControlOutput update(const VehicleState& state) override;

    FlightModeId id() const override { return FlightModeId::AUTO; }
    std::string_view get_name() const override { return "AUTO"; }

    bool is_position_required() const override { return true; }
    bool is_gps_required() const override { return true; }

    void load_mission(const std::vector<Waypoint>& mission);

    // MAV_CMD_DO_CHANGE_SPEED received as a command while AUTO is active.
    bool set_speed_xy(float speed_ms) override;

    // -1 while there's no mission progress to report (no mission loaded,
    // or the mission's already finished and the mode has fallen back to
    // holding) - see mavlink_handler.cpp's MISSION_CURRENT/
    // MISSION_ITEM_REACHED telemetry, the only consumer of this.
    int current_seq() const {
        return (m_mission_complete || m_current_wp_index >= m_mission.size())
                   ? -1
                   : static_cast<int>(m_mission[m_current_wp_index].seq);
    }

    // Public so mode_auto.cpp's file-local is_recognized() helper (and any
    // future caller building a Waypoint list, e.g. MavlinkHandler) can use
    // them without reaching into ModeAuto's internals.
    static constexpr uint16_t kCmdNavWaypoint = 16;
    static constexpr uint16_t kCmdNavReturnToLaunch = 20;
    static constexpr uint16_t kCmdNavLand = 21;
    static constexpr uint16_t kCmdNavTakeoff = 22;
    static constexpr uint16_t kCmdConditionChangeAlt = 113;
    static constexpr uint16_t kCmdConditionYaw = 115;
    static constexpr uint16_t kCmdDoChangeSpeed = 178;
    static constexpr uint16_t kCmdDoSetServo = 183;
    static constexpr uint16_t kCmdDoChangeAltitude = 186;
    static constexpr uint16_t kCmdDoSprayer = 216;

    bool is_yaw_locked() const { return m_yaw_locked; }
    float locked_yaw() const { return m_locked_yaw; }
    // Heading sent to the controller on the last update() (slewed).
    float commanded_yaw() const { return m_last_cmd_yaw; }

private:
    // LAND / RTL items are flown in stages: (RTL only) climb to RTL_ALT in
    // place, fly level to the landing point as a stopping leg, then descend.
    enum class LandStage { Climb, Approach, Descend };

    void advance_to_next_item(const VehicleState& state);
    void setup_current_leg(const VehicleState& state);
    size_t next_nav_index(size_t from) const;
    float descent_speed_ms(float alt_m) const;
    float cruise_speed_ms() const;
    void apply_speed_request(float speed_ms);
    // Runs every zero-time item (DO_*, and CONDITION_YAW when a nav item
    // follows it) from the current index up to the next nav / blocking item.
    void process_instant_items(const VehicleState& state);
    // CONDITION_YAW: compute the target from the item; base_yaw is the
    // heading relative angles are measured from.
    void start_condition_yaw(const Waypoint& item, float base_yaw);
    // Moves m_cond_current_yaw toward m_cond_target_yaw (rate-limited with
    // a deceleration profile, honouring the requested turn direction).
    void slew_condition_yaw(float dt);

    std::vector<Waypoint> m_mission;
    size_t m_current_wp_index{0};
    bool m_mission_complete{false};

    PathTracker m_path_tracker;
    float m_leg_start_ned[3]{0, 0, 0};
    bool m_leg_initialized{false};
    // Set by a MAV_CMD_DO_CHANGE_SPEED mission item or command; <=0 means
    // "none requested, use WPNAV_SPEED". It replaces
    // WPNAV_SPEED (may be faster), bounded only by kMaxSpeedMs.
    float m_requested_speed_ms{-1.0f};

    NavigationCascade m_cascade;
    LandDetector m_land_detector; // only used while descending on a LAND/RTL item
    LandStage m_land_stage{LandStage::Climb};
    float m_hold_pos_ned[3]{0, 0, 0};
    float m_hold_yaw{0};

    // Condition state (for CONDITION_YAW and CONDITION_CHANGE_ALT)
    bool m_condition_active{false};
    float m_cond_target_yaw{0.0f};
    float m_cond_current_yaw{0.0f};
    float m_cond_yaw_speed_rads{0.0f};
    int m_cond_yaw_dir{0};
    bool m_yaw_locked{false};
    float m_locked_yaw{0.0f};
    float m_last_cmd_yaw{0.0f};

    float m_max_reposition_speed_ms{1.5f};
    float m_max_yaw_rate_rads{0.785f};

    // Parameters, read once per mode entry (ModeAuto is recreated on every
    // entry - see mode_manager.cpp's create_mode()).
    float m_wpnav_speed_ms{5.0f};      // WPNAV_SPEED
    float m_wpnav_radius_m{2.0f};      // WPNAV_RADIUS
    int32_t m_wp_yaw_behavior{2};      // WP_YAW_BEHAVIOR
    float m_rtl_alt_m{15.0f};          // RTL_ALT
    float m_speed_up_ms{2.5f};         // WPNAV_SPEED_UP
    float m_speed_dn_ms{1.5f};         // WPNAV_SPEED_DN
    float m_land_speed_ms{0.5f};       // LAND_SPEED
    float m_land_speed_high_ms{0.0f};  // LAND_SPEED_HIGH (0 = WPNAV_SPEED_DN)
    float m_land_alt_low_m{10.0f};     // LAND_ALT_LOW
};

} // namespace apx::modes
