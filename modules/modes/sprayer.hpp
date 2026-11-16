// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
//    floored at SPRAY_PUMP_MIN %), so the dose per square metre stays
//    roughly constant as speed changes
//  - pump runs only while armed and moving faster than SPRAY_SPEED_MIN,
//    with a short turn-on delay and a longer shut-off delay so brief speed
//    dips (corners) don't chatter the valve
//  - spinner (rotary atomiser / agitator) runs at SPRAY_SPINNER PWM while
//    spraying
// Outputs are PWM values for the servo channels SPRAY_PUMP_CH /
// SPRAY_SPIN_CH; SITL forwards them to Gazebo (spray visual), hardware
// drives them through the PWM HAL.

#pragma once

#include "vehicle_state.hpp"
#include <atomic>
#include <cstdint>

namespace apx::modes {

class Sprayer {
public:
    static Sprayer& instance();

    Sprayer(const Sprayer&) = delete;
    Sprayer& operator=(const Sprayer&) = delete;

    // Reads SPRAY_* parameters (call after ParamManager::init_defaults()).
    void load_params();
    // Safe from a ParamManager change callback (those run with its mutex
    // held, so they must not read parameters): reload on the next update().
    void mark_params_dirty() { m_params_dirty = true; }

    // MAV_CMD_DO_SPRAYER: request spraying on/off. Returns false when the
    // sprayer is disabled (SPRAY_ENABLE=0).
    bool run(bool on);
    bool requested() const { return m_requested; }

    // Direct PWM control via MAV_CMD_DO_SET_SERVO (1100 = full off, 1900 = full on)
    void set_pump_pwm_manual(uint16_t pwm);
    void set_spinner_pwm_manual(uint16_t pwm);
    void clear_manual_control();
    bool manual_control() const { return m_manual_control; }
    // Flow rate fraction (0.0 = full off, 1.0 = full on)
    void set_flow_rate(float rate_0_to_1);

    // Call every control tick.
    void update(const VehicleState& state);

    bool spraying() const { return m_spraying; }
    float pump_fraction() const { return m_pump_fraction; }
    uint16_t pump_pwm() const;
    uint16_t spinner_pwm() const;
    // 0-based servo output indices (SPRAY_PUMP_CH / SPRAY_SPIN_CH are 1-based, e.g. ch 9 -> index 8)
    int pump_channel() const { return m_pump_ch - 1; }
    int spinner_channel() const { return m_spin_ch - 1; }

    // Edge flags for user notification; cleared when read.
    bool take_state_changed() {
        bool changed = m_state_changed;
        m_state_changed = false;
        return changed;
    }

private:
    Sprayer() = default;

    bool m_enabled{true};
    float m_pump_rate_pct{10.0f};   // % pump per 1 m/s ground speed
    float m_pump_min_pct{0.0f};
    float m_speed_min_ms{1.0f};
    uint16_t m_spinner_pwm{1300};
    int m_pump_ch{9};               // Servo 9 default for spray pump
    int m_spin_ch{10};              // Servo 10 default for spinner
    uint16_t m_pwm_min{1100};       // 1100us = full off (0%)
    uint16_t m_pwm_max{1900};       // 1900us = full on (100%)

    std::atomic<bool> m_params_dirty{false};
    bool m_last_armed{false};
    bool m_requested{false};
    bool m_spraying{false};
    bool m_manual_control{false};
    uint16_t m_manual_pump_pwm{1100};
    bool m_manual_spinner_control{false};
    uint16_t m_manual_spinner_pwm{1100};
    bool m_state_changed{false};
    float m_pump_fraction{0.0f};
    float m_above_min_s{0.0f};
    float m_below_min_s{0.0f};

    static constexpr float kTurnOnDelayS = 0.1f;
    static constexpr float kShutOffDelayS = 1.0f;
    static constexpr uint16_t kPwmMin = 1100;
    static constexpr uint16_t kPwmMax = 1900;
};

} // namespace apx::modes
