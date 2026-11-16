// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include "mode_base.hpp"
#include "navigation_cascade.hpp"

namespace apx::modes {

enum class SprayState {
    IDLE,
    SPRAYING,
    PAUSED,
    DONE
};

// AGRI_SPRAY: pump/relay control and spray-area path logic aren't
// implemented yet (needs the flow sensor/pump driver + a defined spray
// pattern). Until then this is a safe placeholder: hold the position it
// was entered at, same as ModeAuto, rather than doing nothing.
class ModeAgriSpray : public FlightMode {
public:
    ModeAgriSpray();
    ~ModeAgriSpray() override = default;

    void on_enter(const VehicleState& state) override;
    void on_exit() override;
    ControlOutput update(const VehicleState& state) override;

    FlightModeId id() const override { return FlightModeId::AGRI_SPRAY; }
    std::string_view get_name() const override { return "AGRI_SPRAY"; }

    bool is_position_required() const override { return true; }
    bool is_gps_required() const override { return true; }

private:
    SprayState m_spray_state{SprayState::IDLE};
    float m_target_spray_rate{0.0f};

    NavigationCascade m_cascade;
    float m_hold_pos_ned[3]{0, 0, 0};
    float m_hold_yaw{0};
};

} // namespace apx::modes
