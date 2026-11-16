// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include "mode_base.hpp"
#include "navigation_cascade.hpp"

namespace apx::modes {

// POS_HOLD: holds the 3D position/heading captured at entry
// using the same NavigationCascade GUIDED uses, but does not accept an external
// GCS target - maintains stationary lock at current coordinates.
class ModePosHold : public FlightMode {
public:
    explicit ModePosHold(FlightModeId represented_as = FlightModeId::POS_HOLD);
    ~ModePosHold() override = default;

    void on_enter(const VehicleState& state) override;
    void on_exit() override;
    ControlOutput update(const VehicleState& state) override;

    FlightModeId id() const override { return m_id; }
    std::string_view get_name() const override;

    bool is_position_required() const override { return true; }
    bool is_gps_required() const override { return true; }

private:
    FlightModeId m_id;
    NavigationCascade m_cascade;
    float m_hold_pos_ned[3]{0, 0, 0};
    float m_hold_yaw{0};
};

} // namespace apx::modes
