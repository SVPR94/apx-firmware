// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "rate_controller.hpp"

namespace apx::control {

RateController::RateController() {
    for (int i=0; i<3; ++i) {
        m_pid[i].set_output_limits(-1.0f, 1.0f);
        // 20Hz D-term filter - matches the previous fixed-alpha
        // implementation's effective cutoff at this loop's 400Hz rate,
        // but now actually computed from dt (see pid_controller.cpp)
        // instead of silently ignoring whatever was requested.
        m_pid[i].set_derivative_lpf_cutoff(20.0f);
    }
}

void RateController::update(const float desired_rate_rads[3], const float measured_rate_rads[3], float dt_s, float output[3]) {
    for (int i=0; i<3; ++i) {
        float error = desired_rate_rads[i] - measured_rate_rads[i];
        // D-term acts on d(measured_rate)/dt (angular *acceleration*), not
        // on the raw rate itself - passing the rate straight into
        // compute_with_derivative() fed it a value with the wrong units entirely:
        // a ~1-2rad/s rate scaled by D gains is a tiny contribution, whereas
        // a real d(rate)/dt during a maneuver is tens of rad/s^2. Fixed
        // by actually differentiating the measurement, using the "on
        // measurement, not on error" pattern to avoid derivative kick.
        float rate_deriv = 0.0f;
        if (dt_s > 0.0f && m_have_prev_rate[i]) {
            rate_deriv = (measured_rate_rads[i] - m_prev_measured_rate[i]) / dt_s;
        }
        m_prev_measured_rate[i] = measured_rate_rads[i];
        m_have_prev_rate[i] = true;
        float pid_out = m_pid[i].compute_with_derivative(error, rate_deriv, dt_s);
        float ff_out = m_ff_gain[i] * desired_rate_rads[i];

        output[i] = pid_out + ff_out;
        if (output[i] > 1.0f) output[i] = 1.0f;
        if (output[i] < -1.0f) output[i] = -1.0f;
    }
}

void RateController::set_gains(int axis, float kp, float ki, float kd, float ff) {
    if (axis >= 0 && axis < 3) {
        m_pid[axis].set_gains(kp, ki, kd);
        m_ff_gain[axis] = ff;
    }
}

void RateController::reset() {
    for(int i=0; i<3; ++i) {
        m_pid[i].reset();
        m_prev_measured_rate[i] = 0.0f;
        m_have_prev_rate[i] = false;
    }
}

} // namespace apx::control
