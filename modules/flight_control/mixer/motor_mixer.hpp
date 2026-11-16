// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once
#include "motor_geometry.hpp"

namespace apx::control {

class MotorMixer {
public:
    MotorMixer();
    void mix(float thrust, float roll, float pitch, float yaw, FrameType frame, float outputs[8]);
private:
    int get_motor_count(FrameType frame) const;
    const MotorConfig* get_motor_configs(FrameType frame) const;
};

} // namespace apx::control
