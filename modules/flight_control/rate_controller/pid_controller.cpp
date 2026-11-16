// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "pid_controller.hpp"
#include <cmath>

namespace apx::control {

PidController::PidController(float kp, float ki, float kd, float output_limit) 
    : m_kp(kp), m_ki(ki), m_kd(kd), m_output_min(-output_limit), m_output_max(output_limit) {}

float PidController::compute(float error, float dt_s) {
    if (dt_s <= 0.0f) return 0.0f;
    // No previous sample right after construction/reset(): differentiating
    // against the zero m_prev_error would produce D = kd * error / dt, a
    // full-scale kick on the first tick of every mode entry. Start at 0.
    float derivative = m_have_prev_error ? (error - m_prev_error) / dt_s : 0.0f;
    m_prev_error = error;
    m_have_prev_error = true;
    return compute_with_derivative(error, -derivative, dt_s); // Note: typical practice uses negative measured rate for D
}

float PidController::compute_with_derivative(float error, float measured_derivative, float dt_s) {
    if (dt_s <= 0.0f) return 0.0f;

    if (!m_have_deriv_filtered) {
        // Start the filter AT the first real sample instead of blending
        // from a stale zero - avoids a one-tick startup transient that
        // would otherwise look like a brief, spurious D-term kick.
        m_deriv_filtered = measured_derivative;
        m_have_deriv_filtered = true;
    } else if (m_deriv_cutoff_hz > 0.0f) {
        // Standard single-pole RC low-pass, alpha recomputed from the
        // ACTUAL dt this call - not a fixed coefficient - so the filter's
        // real cutoff always matches m_deriv_cutoff_hz regardless of loop
        // rate. alpha here is the weight on the NEW sample (standard
        // convention); RC = 1/(2*pi*fc).
        float rc = 1.0f / (2.0f * 3.14159265f * m_deriv_cutoff_hz);
        float alpha = dt_s / (rc + dt_s);
        m_deriv_filtered = m_deriv_filtered + alpha * (measured_derivative - m_deriv_filtered);
    } else {
        // cutoff <= 0 means "no filtering" - pass the raw derivative straight through.
        m_deriv_filtered = measured_derivative;
    }

    // Conditional-integration anti-windup: only accumulate the integral
    // if the *unclamped* output isn't already saturated in the direction
    // that error is pushing it. Without this, a sustained error with the
    // output pinned at its limit (e.g. the altitude PID demanding more
    // climb than available while the vehicle is still on the ground,
    // physically unable to move) keeps accumulating m_integral the whole
    // time regardless - m_integral_limit alone doesn't prevent this, it
    // only caps how far the runaway accumulation can go, so the output
    // still saturates on I alone and then has to unwind before it can
    // ever produce a sane value once the real error shrinks. That stored
    // energy paying out all at once the moment the error condition
    // clears is exactly what produced a thrust "kick" at liftoff.
    float unclamped_output = (error * m_kp) + m_integral - (m_deriv_filtered * m_kd);
    bool saturated_high = unclamped_output >= m_output_max && error > 0.0f;
    bool saturated_low = unclamped_output <= m_output_min && error < 0.0f;
    if (!saturated_high && !saturated_low) {
        m_integral += error * m_ki * dt_s;
        m_integral = clamp(m_integral, -m_integral_limit, m_integral_limit);
    }

    float output = (error * m_kp) + m_integral - (m_deriv_filtered * m_kd); // D term acts on measurement to avoid kick
    return clamp(output, m_output_min, m_output_max);
}

void PidController::reset() {
    m_integral = 0.0f;
    m_prev_error = 0.0f;
    m_have_prev_error = false;
    m_deriv_filtered = 0.0f;
    m_have_deriv_filtered = false;
}

void PidController::set_gains(float kp, float ki, float kd) {
    m_kp = kp; m_ki = ki; m_kd = kd;
}

void PidController::set_output_limits(float min, float max) {
    m_output_min = min;
    m_output_max = max;
}

void PidController::set_integral_limit(float limit) {
    m_integral_limit = limit;
}

void PidController::set_derivative_lpf_cutoff(float hz) {
    m_deriv_cutoff_hz = hz;
}

float PidController::clamp(float val, float min, float max) const {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

} // namespace apx::control
