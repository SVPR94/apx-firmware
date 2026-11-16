// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include "mode_base.hpp"
#include "position_controller/position_controller.hpp"
#include "attitude_controller/attitude_controller.hpp"
#include "rate_controller/rate_controller.hpp"

namespace apx::modes {

// ALT_HOLD: unlike POS_HOLD/LOITER, roll/pitch/yaw are controlled
// by the pilot's sticks while altitude is maintained automatically - it does NOT
// actively correct horizontal drift. Pilot roll/pitch/yaw/throttle come
// from VehicleState.rc_* (see mavlink_handler.cpp's MANUAL_CONTROL/
// RC_CHANNELS_OVERRIDE handling for how those get populated).
class ModeAltHold : public FlightMode {
public:
    ModeAltHold();
    ~ModeAltHold() override = default;

    void on_enter(const VehicleState& state) override;
    void on_exit() override;
    ControlOutput update(const VehicleState& state) override;

    FlightModeId id() const override { return FlightModeId::ALT_HOLD; }
    std::string_view get_name() const override { return "ALT_HOLD"; }

private:
    control::PositionController m_pos_ctrl;
    control::AttitudeController m_att_ctrl;
    control::RateController m_rate_ctrl;
    float m_target_rel_alt_m{0};
    float m_hold_yaw{0};

    // Read from ParamManager (ANGLE_MAX, ATC_RATE_Y_MAX, PILOT_SPEED_UP/
    // DN) at construction time - see ModeAltHold's constructor.
    float m_max_tilt_rad{0.436f};
    float m_max_yaw_rate_rad{0.785f};
    float m_climb_speed_ms{2.5f};
    float m_desc_speed_ms{1.5f};
};

} // namespace apx::modes
