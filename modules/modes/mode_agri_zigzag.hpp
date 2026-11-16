// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include "mode_base.hpp"
#include "navigation_cascade.hpp"

namespace apx::modes {

enum class ZigZagState {
    SET_A,
    SET_B,
    FLYING_ROW,
    TURNING,
    COMPLETE
};

// AGRI_ZIGZAG: the point-A/point-B row-following pattern isn't implemented
// yet (needs a way for the GCS/RC to mark A and B, then row-spacing
// geometry on top of the same NavigationCascade GUIDED uses). Until then
// this is a safe placeholder: hold the position it was entered at.
class ModeAgriZigZag : public FlightMode {
public:
    ModeAgriZigZag();
    ~ModeAgriZigZag() override = default;

    void on_enter(const VehicleState& state) override;
    void on_exit() override;
    ControlOutput update(const VehicleState& state) override;

    FlightModeId id() const override { return FlightModeId::AGRI_ZIGZAG; }
    std::string_view get_name() const override { return "AGRI_ZIGZAG"; }

    bool is_position_required() const override { return true; }
    bool is_gps_required() const override { return true; }

private:
    ZigZagState m_state{ZigZagState::SET_A};
    float m_row_spacing{5.0f};
    float m_pt_a_ned[2]{0, 0};
    float m_pt_b_ned[2]{0, 0};

    NavigationCascade m_cascade;
    float m_hold_pos_ned[3]{0, 0, 0};
    float m_hold_yaw{0};
};

} // namespace apx::modes
