// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once
#include "../rate_controller/pid_controller.hpp"

namespace apx::control {

class PositionController {
public:
    PositionController();
    void update_horizontal(const float pos_target[2], const float pos_actual[2], 
                           const float vel_target[2], const float vel_actual[2],
                           float dt, float accel_target[2]);
    void update_altitude(float alt_target, float alt_actual,
                         float vel_target, float vel_actual,
                         float dt, float& thrust_target);
    void reset();

    // Previously fixed at construction time with no way to tune them from
    // outside - added so callers (see modules/modes/navigation_cascade.cpp)
    // can actually tune this instead of only being able to clamp its
    // output after the fact.
    void set_position_gain(float pos_p) { m_pos_p = pos_p; }
    void set_altitude_gain(float alt_p) { m_alt_p = alt_p; }
    void set_horizontal_velocity_gains(float kp, float ki, float kd);
    void set_climb_rate_gains(float kp, float ki, float kd);
    // Ceiling on the P-term's horizontal velocity demand, applied in
    // update_horizontal() before it reaches the velocity PID. Without this,
    // a large position error (pos_p=1.0 x an 8m error = an 8m/s velocity demand)
    // had nothing capping it before hitting the velocity loop - see
    // update_horizontal()'s comment for why that mattered.
    void set_max_horizontal_speed(float speed_ms) { m_max_speed_ms = speed_ms; }
    // Same idea as set_max_horizontal_speed(), but update_altitude() never
    // had an equivalent cap at all (a real asymmetry, not just an unset
    // default) - see update_altitude()'s comment for the "jerky takeoff"
    // this caused: while the vehicle is still on the ground (weight on the
    // landing gear physically holds alt_actual at 0), any caller ramping
    // alt_target upward - even gently, e.g. LOITER's own climb - keeps
    // growing the (alt_target - alt_actual) error every tick with nothing
    // bounding the resulting vel_sp_z, so the velocity PID's P-term (and,
    // via that same unbounded error, its I-term) built up far past what's
    // needed for a smooth climb by the time the vehicle actually broke
    // ground and started responding - then dumped all of it at once as a
    // kick right at liftoff.
    void set_max_vertical_speed(float up_ms, float down_ms) {
        m_max_climb_speed_ms = up_ms;
        m_max_descent_speed_ms = down_ms;
    }
    // MOT_THST_HOVER: a learned baseline thrust that update_altitude()'s
    // climb-rate PID output is a CORRECTION around, not the whole thrust command.
    // Without this, the PID had to build 100% of hover-equivalent thrust out of
    // P+I+D from a cold/zero start every single time - the conditional-
    // integration anti-windup fixed the symptom (the I-term winding up while
    // grounded and dumping it all at liftoff), but not this underlying reason:
    // the controller needs to start near the physically-correct steady-state
    // thrust value. Starts at a configurable default and slowly adapts toward
    // whatever thrust actually sustains a near-zero climb rate (~10s time
    // constant) rather than a one-shot guess.
    void set_hover_thrust(float hover_thrust) { m_hover_thrust = hover_thrust; }
    float get_hover_thrust() const { return m_hover_thrust; }

private:
    PidController m_vel_x_pid;
    PidController m_vel_y_pid;
    PidController m_vel_z_pid;
    float m_pos_p{1.0f};
    float m_alt_p{1.0f};
    float m_max_speed_ms{5.0f};         // Default horizontal speed limit (5 m/s)
    float m_max_climb_speed_ms{2.5f};   // Default climb speed limit (2.5 m/s)
    float m_max_descent_speed_ms{1.5f}; // Default descent speed limit (1.5 m/s)
    float m_hover_thrust{0.5f};         // MOT_THST_HOVER baseline thrust
    float m_hover_persist_timer_s{0.0f}; // throttles how often the learned value is written back to ParamManager
};

} // namespace apx::control
