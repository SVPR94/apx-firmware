// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include "mode_base.hpp"
#include "navigation_cascade.hpp"
#include "land_detector.hpp"

namespace apx::modes {

enum class LandPhase {
    Braking,    // Decelerating forward horizontal motion safely to a stop
    Descend     // Holding stopped position and descending vertically
};

// LAND: safely brakes horizontal speed along line of travel first, then descends
// vertically while holding the stopped position, using LandDetector to disarm on touchdown.
class ModeLand : public FlightMode {
public:
    ModeLand();
    ~ModeLand() override = default;

    void on_enter(const VehicleState& state) override;
    void on_exit() override;
    ControlOutput update(const VehicleState& state) override;

    FlightModeId id() const override { return FlightModeId::LAND; }
    std::string_view get_name() const override { return "LAND"; }

    bool is_position_required() const override { return true; }
    // Deliberately false, unlike GUIDED/POS_HOLD: allows
    // LAND without GPS (falls back to baro/optical descent) since
    // refusing to land is far more dangerous than landing imprecisely.
    bool is_gps_required() const override { return false; }

    bool has_landed() const { return m_detector.landed(); }
    LandPhase phase() const { return m_phase; }

    float target_yaw() const { return m_hold_yaw; }
    const float* target_pos_ned() const { return m_hold_pos_ned; }

private:
    float descent_speed_ms(float alt_m) const;

    NavigationCascade m_cascade;
    LandDetector m_detector;
    LandPhase m_phase{LandPhase::Descend};
    float m_hold_pos_ned[2]{0.0f, 0.0f};
    float m_braking_vel_ne[2]{0.0f, 0.0f};
    float m_hold_yaw{0.0f};

    float m_brake_accel_ms2{1.5f};

    float m_max_reposition_speed_ms{1.5f}; // LAND_REPOS_SPEED cm/s -> m/s
    float m_max_yaw_rate_rads{0.785f};     // ATC_RATE_Y_MAX deg/s -> rad/s
    float m_land_speed_ms{0.5f};          // LAND_SPEED cm/s -> m/s
    float m_land_speed_high_ms{0.0f};     // LAND_SPEED_HIGH cm/s -> m/s
    float m_land_alt_low_m{10.0f};        // LAND_ALT_LOW cm -> m
    float m_speed_dn_ms{1.5f};            // WPNAV_SPEED_DN cm/s -> m/s
};

} // namespace apx::modes
