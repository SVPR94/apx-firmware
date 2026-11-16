// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include "mode_base.hpp"
#include "attitude_controller/attitude_controller.hpp"
#include "rate_controller/rate_controller.hpp"

namespace apx::modes {

// STABILIZE: pilot controls attitude (roll/pitch tilt), yaw rate, and collective thrust.
class ModeStabilize : public FlightMode {
public:
    ModeStabilize();
    ~ModeStabilize() override = default;

    void on_enter(const VehicleState& state) override;
    void on_exit() override;
    ControlOutput update(const VehicleState& state) override;

    FlightModeId id() const override { return FlightModeId::STABILIZE; }
    std::string_view get_name() const override { return "STABILIZE"; }

    bool can_arm() const override { return true; }

private:
    control::AttitudeController m_att_ctrl;
    control::RateController m_rate_ctrl;
    float m_target_yaw{0.0f};

    // Read from ParamManager (ANGLE_MAX, ATC_RATE_Y_MAX) at construction
    // time - see ModeStabilize's constructor.
    float m_max_tilt_rad{0.436f};
    float m_max_yaw_rate_rad{0.785f};
};

} // namespace apx::modes
