// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "board_config.hpp"
#include "attitude_controller/attitude_controller.hpp"
#include "rate_controller/rate_controller.hpp"
#include "position_controller/position_controller.hpp"
#include "mixer/motor_mixer.hpp"

namespace {

using namespace apx::platform::stm32h7::cube_orange_plus;
static_assert(NUM_SENSOR_LANES == 3, "Cube Orange+ must be configured with exactly 3 sensor lanes");

constexpr float kFixedDtS = 0.0025f; // 400 Hz control loop step
volatile float g_sink;

void run_one_control_iteration() {
    static apx::control::AttitudeController att_ctrl;
    static apx::control::RateController rate_ctrl;
    static apx::control::PositionController pos_ctrl;
    static apx::control::MotorMixer mixer;

    float q_desired[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float q_actual[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float desired_rate[3] = {0.0f, 0.0f, 0.0f};
    att_ctrl.update(q_desired, q_actual, kFixedDtS, desired_rate);

    float measured_rate[3] = {0.0f, 0.0f, 0.0f};
    float rate_out[3] = {0.0f, 0.0f, 0.0f};
    rate_ctrl.update(desired_rate, measured_rate, kFixedDtS, rate_out);

    float outputs[8] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    mixer.mix(0.5f, rate_out[0], rate_out[1], rate_out[2],
              apx::control::FrameType::HEX_X, outputs);

    g_sink = outputs[0] + outputs[1] + outputs[2] + outputs[3] + outputs[4] + outputs[5];
    (void)pos_ctrl;
}

} // namespace

extern "C" int main() {
    while (true) {
        run_one_control_iteration();
    }
}
