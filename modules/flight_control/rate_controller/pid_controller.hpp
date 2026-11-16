// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once
namespace apx::control {
class PidController {
public:
    PidController() = default;
    PidController(float kp, float ki, float kd, float output_limit = 1.0f);
    float compute(float error, float dt_s);
    float compute_with_derivative(float error, float measured_derivative, float dt_s);
    void reset();
    void set_gains(float kp, float ki, float kd);
    void set_output_limits(float min, float max);
    void set_integral_limit(float limit);
    // Low-pass filter cutoff (Hz) on the derivative term.
    // The alpha this implies is recomputed from the actual dt passed to
    // compute()/compute_with_derivative() every call (see pid_controller.cpp),
    // not stored as a fixed coefficient. This ensures the filter bandwidth
    // tracks accurately regardless of the update rate.
    void set_derivative_lpf_cutoff(float hz);
    float get_integral() const { return m_integral; }
private:
    float m_kp{0}, m_ki{0}, m_kd{0};
    float m_integral{0};
    float m_prev_error{0};
    bool m_have_prev_error{false};
    float m_deriv_filtered{0};
    bool m_have_deriv_filtered{false};
    float m_deriv_cutoff_hz{20.0f}; // matches the previous fixed alpha=0.8 at 400Hz
    float m_output_min{-1.0f}, m_output_max{1.0f};
    float m_integral_limit{1.0f};
    float clamp(float val, float min, float max) const;
};
} // namespace apx::control
