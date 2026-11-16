// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "motor_mixer.hpp"
#include <algorithm>
#include <cmath>

namespace apx::control {

MotorMixer::MotorMixer() {}

void MotorMixer::mix(float thrust, float roll, float pitch, float yaw, FrameType frame, float outputs[8]) {
    int count = get_motor_count(frame);
    const MotorConfig* configs = get_motor_configs(frame);

    if (count == 0 || configs == nullptr) return;

    // 1. Prioritize Roll and Pitch attitude stabilization:
    // Compute roll and pitch contributions first.
    float rp[8];
    float min_rp = 0.0f, max_rp = 0.0f;
    for (int i = 0; i < count; ++i) {
        rp[i] = roll * configs[i].roll_factor + pitch * configs[i].pitch_factor;
        if (i == 0 || rp[i] < min_rp) min_rp = rp[i];
        if (i == 0 || rp[i] > max_rp) max_rp = rp[i];
    }

    float rp_spread = max_rp - min_rp;
    if (rp_spread > 1.0f) {
        float scale = 1.0f / rp_spread;
        roll *= scale;
        pitch *= scale;
        for (int i = 0; i < count; ++i) rp[i] *= scale;
        rp_spread = 1.0f;
    }

    // 2. Allocate remaining headroom to Yaw.
    // Unlike before (where large yaw demand scaled down roll and pitch equally,
    // compromising attitude stabilization and causing wobbles during aggressive turns),
    // the multirotor mixer prioritizes attitude stability and bounds yaw authority
    // so yaw can never steal roll/pitch control headroom.
    float yaw_headroom = std::max(0.15f, 1.0f - rp_spread);
    float y[8];
    float min_y = 0.0f, max_y = 0.0f;
    for (int i = 0; i < count; ++i) {
        y[i] = yaw * configs[i].yaw_factor;
        if (i == 0 || y[i] < min_y) min_y = y[i];
        if (i == 0 || y[i] > max_y) max_y = y[i];
    }
    float y_spread = max_y - min_y;
    float yaw_scale = 1.0f;
    if (y_spread > yaw_headroom && y_spread > 1e-4f) {
        yaw_scale = yaw_headroom / y_spread;
    }
    float effective_yaw = yaw * yaw_scale;

    float rpy[8];
    float min_rpy = 0.0f, max_rpy = 0.0f;
    for (int i = 0; i < count; ++i) {
        rpy[i] = rp[i] + effective_yaw * configs[i].yaw_factor;
        if (i == 0 || rpy[i] < min_rpy) min_rpy = rpy[i];
        if (i == 0 || rpy[i] > max_rpy) max_rpy = rpy[i];
    }

    // Final combined-spread check: steps 1 and 2 above each bound their
    // OWN spread (rp_spread<=1.0, y_spread<=yaw_headroom<=1.0)
    // independently, but never checked the COMBINED per-motor rpy[i]
    // spread against the full available [0,1] output band - roll+pitch's
    // contribution and yaw's contribution don't necessarily cancel on
    // every motor, so even with both individually in-budget, their SUM
    // could still exceed 1.0 (worst case, up to 2.0). When that happened,
    // the thrust-shift desaturation below couldn't actually fit
    // everything and silently fell through to the per-motor clamp,
    // disproportionately robbing whichever single motor hit 0 first
    // rather than reducing the whole commanded rotation. If the combined
    // spread still doesn't fit, scale roll+pitch+yaw down TOGETHER so the
    // direction of the combined rotational demand is preserved, just shrunk
    // - not sacrificing whichever axis happens to clip first.
    float rpy_spread = max_rpy - min_rpy;
    if (rpy_spread > 1.0f) {
        float scale = 1.0f / rpy_spread;
        roll *= scale;
        pitch *= scale;
        effective_yaw *= scale;
        for (int i = 0; i < count; ++i) rpy[i] *= scale;
    }

    float outputs_pre[8];
    float min_out = 1.0f;
    float max_out = 0.0f;
    for (int i = 0; i < count; ++i) {
        outputs_pre[i] = thrust + rpy[i];
        if (i == 0 || outputs_pre[i] < min_out) min_out = outputs_pre[i];
        if (i == 0 || outputs_pre[i] > max_out) max_out = outputs_pre[i];
    }

    // Desaturation: uniform thrust shift only (preserves the now-fitting
    // roll/pitch/yaw differences between motors exactly).
    float boost = 0.0f;
    if (min_out < 0.0f) {
        boost = -min_out;
    }
    if (max_out + boost > 1.0f) {
        boost -= (max_out + boost - 1.0f);
    }

    for (int i = 0; i < count; ++i) {
        outputs[i] = outputs_pre[i] + boost;
        // Last-resort safety net, not the normal path any more - the
        // scaling above already keeps the full [0,1] band satisfiable.
        if (outputs[i] < 0.0f) outputs[i] = 0.0f;
        if (outputs[i] > 1.0f) outputs[i] = 1.0f;
    }
}

// Only frames with a mixing table are supported. QUAD_PLUS, HEX_FLAT,
// OCTO_COAX and OCTO_PLUS have no table yet, so they report 0 motors and
// mix() leaves the outputs untouched (it used to return 4/6/8 for them and
// then index a null table).
int MotorMixer::get_motor_count(FrameType frame) const {
    switch (frame) {
        case FrameType::QUAD_X: return 4;
        case FrameType::HEX_X:  return 6;
        case FrameType::OCTO_X: return 8;
        default: return 0;
    }
}

const MotorConfig* MotorMixer::get_motor_configs(FrameType frame) const {
    switch (frame) {
        case FrameType::QUAD_X: return QUAD_X_MOTORS;
        case FrameType::HEX_X: return HEX_X_MOTORS;
        case FrameType::OCTO_X: return OCTO_X_MOTORS;
        default: return nullptr;
    }
}

} // namespace apx::control
