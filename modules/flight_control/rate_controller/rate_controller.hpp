// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once
#include "pid_controller.hpp"
namespace apx::control {

class RateController {
public:
    RateController();
    void update(const float desired_rate_rads[3], const float measured_rate_rads[3], float dt_s, float output[3]);
    void set_gains(int axis, float kp, float ki, float kd, float ff);
    void reset();
private:
    PidController m_pid[3];
    float m_ff_gain[3]{0,0,0};
    float m_prev_measured_rate[3]{0,0,0};
    bool m_have_prev_rate[3]{false,false,false};
};

} // namespace apx::control
