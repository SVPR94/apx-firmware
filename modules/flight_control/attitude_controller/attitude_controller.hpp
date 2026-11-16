// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

namespace apx::control {

class AttitudeController {
public:
    AttitudeController();
    void update(const float q_desired[4], const float q_actual[4], float dt_s, float desired_rate_rads[3]);
    void set_proportional_gain(float p_roll, float p_pitch, float p_yaw);
    void set_rate_limits(float max_roll_pitch, float max_yaw);
    // Angular acceleration limits (rad/s^2): caps how fast the
    // *commanded rate itself* is allowed to change, separate
    // from set_rate_limits()'s cap on its magnitude. See update()'s
    // comment for why this matters - without it, a sudden angle-target
    // change (e.g. a waypoint corner needing a new heading/lean
    // direction) turns into an instant step in desired_rate, which is
    // exactly the kind of transient that excited runaway oscillation in 
    // live SITL testing. 0 (the default) means "no limit", matching the
    // pre-existing unslewed behavior.
    void set_accel_limits(float max_accel_rp_rads2, float max_accel_y_rads2);
    // Clears the rate-slew state (m_prev_desired_rate) - call whenever
    // the rate/position controllers downstream are also reset (mode
    // entry, disarm), or the first tick after a reset would slew away
    // from a stale previous-rate value instead of starting fresh.
    void reset();
private:
    float m_p_gain[3]{4.5f, 4.5f, 4.5f};
    float m_max_rate_rp{1.57f};
    float m_max_rate_y{0.78f};
    float m_max_accel_rp{0.0f}; // rad/s^2, 0 = unlimited
    float m_max_accel_y{0.0f};
    float m_prev_desired_rate[3]{0.0f, 0.0f, 0.0f};
    bool m_have_prev_rate{false};
    float clamp(float val, float min, float max) const;
};

} // namespace apx::control
