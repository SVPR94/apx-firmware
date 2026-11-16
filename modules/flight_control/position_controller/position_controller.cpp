// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "position_controller.hpp"
#include "param_manager.hpp"
#include <cmath>

namespace apx::control {

PositionController::PositionController() {
    m_vel_x_pid.set_gains(1.0f, 0.1f, 0.0f);
    m_vel_y_pid.set_gains(1.0f, 0.1f, 0.0f);
    m_vel_z_pid.set_gains(1.5f, 0.2f, 0.05f);
    m_vel_x_pid.set_output_limits(-10.0f, 10.0f);
    m_vel_y_pid.set_output_limits(-10.0f, 10.0f);
    // The Z-vel PID's output is now a CORRECTION around m_hover_thrust
    // (see update_altitude()), not the whole [0,1] thrust command, so its
    // own range needs to span enough to pull thrust all the way to 0 or 1
    // regardless of where the hover baseline sits - +-1.0, not [0,1].
    m_vel_z_pid.set_output_limits(-1.0f, 1.0f);
    // The I-term alone reaching full authority mattered a lot more when
    // it had to build 100% of hover thrust from scratch; it's still
    // tightened to stay a supporting contributor around the baseline
    // rather than dominate it, same reasoning as before. PidController's
    // conditional-integration anti-windup (see pid_controller.cpp) is
    // still the fix for the ground-windup symptom; the hover baseline
    // below is what addresses the underlying "starts from zero every
    // time" reason that symptom existed at all.
    m_vel_z_pid.set_integral_limit(0.3f);
}

void PositionController::update_horizontal(const float pos_target[2], const float pos_actual[2],
                                           const float vel_target[2], const float vel_actual[2],
                                           float dt, float accel_target[2]) {
    float vel_sp_x = (pos_target[0] - pos_actual[0]) * m_pos_p + vel_target[0];
    float vel_sp_y = (pos_target[1] - pos_actual[1]) * m_pos_p + vel_target[1];

    // Cap the *magnitude* of the combined velocity demand (not each axis
    // independently, which would distort direction) - previously nothing
    // bounded this, so any sizeable position error translated straight
    // into a proportionally large velocity setpoint (pos_p=1.0 x an 8m
    // error = 8m/s) with no ceiling before it hit the velocity PID.
    // WPNAV_SPEED (default 5m/s) plays this role.
    float speed_sp = std::sqrt(vel_sp_x * vel_sp_x + vel_sp_y * vel_sp_y);
    if (speed_sp > m_max_speed_ms && speed_sp > 1e-6f) {
        float scale = m_max_speed_ms / speed_sp;
        vel_sp_x *= scale;
        vel_sp_y *= scale;
    }

    accel_target[0] = m_vel_x_pid.compute(vel_sp_x - vel_actual[0], dt);
    accel_target[1] = m_vel_y_pid.compute(vel_sp_y - vel_actual[1], dt);
}

void PositionController::update_altitude(float alt_target, float alt_actual,
                                         float vel_target, float vel_actual,
                                         float dt, float& thrust_target) {
    float vel_sp_z = (alt_target - alt_actual) * m_alt_p + vel_target;
    // See set_max_vertical_speed()'s comment: this cap is what actually
    // stops a growing-while-grounded position error from turning into an
    // unbounded climb-rate demand.
    if (vel_sp_z > m_max_climb_speed_ms) {
        vel_sp_z = m_max_climb_speed_ms;
    } else if (vel_sp_z < -m_max_descent_speed_ms) {
        vel_sp_z = -m_max_descent_speed_ms;
    }
    float correction = m_vel_z_pid.compute(vel_sp_z - vel_actual, dt);
    thrust_target = m_hover_thrust + correction;
    if (thrust_target < 0.0f) thrust_target = 0.0f;
    if (thrust_target > 1.0f) thrust_target = 1.0f;

    // Slow learning toward whatever thrust actually sustains a near-zero
    // climb rate (~10s time constant), not a one-shot guess. Only adapts while
    // genuinely near-hover (climb rate small): learning during an active
    // climb/descent would bake transient, non-hover thrust into the
    // baseline. A 10s time constant is slow enough that any one noisy
    // tick can't meaningfully perturb it, but fast enough to track a
    // real, sustained mass/battery-voltage change over a flight.
    if (dt > 0.0f && std::fabs(vel_actual) < 0.2f) {
        float alpha = dt / (10.0f + dt);
        m_hover_thrust += (thrust_target - m_hover_thrust) * alpha;
        if (m_hover_thrust < 0.1f) m_hover_thrust = 0.1f;
        if (m_hover_thrust > 0.9f) m_hover_thrust = 0.9f;

        // Persist back to ParamManager (not just this instance's member)
        // every ~1s - each FlightMode owns its OWN PositionController,
        // recreated fresh on every mode (re-)entry (see ModeManager::
        // create_mode()), so without this, learning would reset to the
        // MOT_THST_HOVER default every time the pilot switches modes and
        // back, even mid-flight. Writing the learned value back to the
        // shared ParamManager means the NEXT mode's constructor (which
        // reads MOT_THST_HOVER the same way this one did) picks up
        // wherever learning actually got to, ensuring consistent
        // persistence across mode switches.
        m_hover_persist_timer_s += dt;
        if (m_hover_persist_timer_s >= 1.0f) {
            apx::gcs::ParamManager::get_instance().set_float("MOT_THST_HOVER", m_hover_thrust);
            m_hover_persist_timer_s = 0.0f;
        }
    }
}

void PositionController::reset() {
    m_vel_x_pid.reset();
    m_vel_y_pid.reset();
    m_vel_z_pid.reset();
}

void PositionController::set_horizontal_velocity_gains(float kp, float ki, float kd) {
    m_vel_x_pid.set_gains(kp, ki, kd);
    m_vel_y_pid.set_gains(kp, ki, kd);
}

void PositionController::set_climb_rate_gains(float kp, float ki, float kd) {
    m_vel_z_pid.set_gains(kp, ki, kd);
}

} // namespace apx::control
