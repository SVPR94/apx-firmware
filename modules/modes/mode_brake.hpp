// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include "mode_base.hpp"
#include "navigation_cascade.hpp"

namespace apx::modes {

class ModeBrake : public FlightMode {
public:
    ModeBrake();
    ~ModeBrake() override = default;

    void on_enter(const VehicleState& state) override;
    void on_exit() override;
    ControlOutput update(const VehicleState& state) override;

    FlightModeId id() const override { return FlightModeId::BRAKE; }
    std::string_view get_name() const override { return "BRAKE"; }

    bool is_position_required() const override { return true; }
    bool is_gps_required() const override { return true; }

private:
    NavigationCascade m_cascade;
    float m_target_pos_ned[3]{0.0f, 0.0f, 0.0f};
    float m_target_yaw{0.0f};
    float m_brake_accel_ms2{2.5f};
    bool m_stopped{false};
};

} // namespace apx::modes
