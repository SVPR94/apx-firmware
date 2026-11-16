// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
// built only for APX_PLATFORM_STM32H7 (see this board's CMakeLists.txt).
//
// Scope, honestly: there is no STM32H7 HAL backend yet (hal/stm32h7/ is
// still empty - see README.md), so this cannot read a real IMU, drive a
// real motor, or send real MAVLink. What it *does* do is construct and
// exercise APX's actual flight-control math (RateController,
// AttitudeController, PositionController, MotorMixer - the same classes
// modules/modes/navigation_cascade.cpp drives on SITL) with synthetic
// input, so that a successful build+link here is real evidence that
// APX's control-loop code - not just this board's startup boilerplate -
// cross-compiles and links correctly for Cortex-M7, not just a "hello
// world" that proves nothing about the actual firmware logic.
//
// Next real step beyond this file: an hal/stm32h7/ backend so this can
// read real sensors and drive real PWM outputs instead of synthetic
// values in a loop that never sees hardware.

#include "attitude_controller/attitude_controller.hpp"
#include "rate_controller/rate_controller.hpp"
#include "position_controller/position_controller.hpp"
#include "mixer/motor_mixer.hpp"

namespace {

// No HAL timer/GPIO backend yet (see file header) - this just proves the
// control-loop math runs without a working notion of wall-clock dt; a
// fixed 400Hz-equivalent step matches main/apx_main.cpp's SITL loop
// period so the same gains behave the same way once real timing exists.
constexpr float kFixedDtS = 0.0025f;

volatile float g_sink; // prevents the optimizer from discarding the "work"

void run_one_control_iteration() {
    static apx::control::AttitudeController att_ctrl;
    static apx::control::RateController rate_ctrl;
    static apx::control::PositionController pos_ctrl;
    static apx::control::MotorMixer mixer;

    // Synthetic inputs (no real sensors behind this yet) - just enough
    // to exercise every stage of the cascade once per loop.
    float q_desired[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float q_actual[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float desired_rate[3] = {0.0f, 0.0f, 0.0f};
    att_ctrl.update(q_desired, q_actual, kFixedDtS, desired_rate);

    float measured_rate[3] = {0.0f, 0.0f, 0.0f};
    float rate_out[3] = {0.0f, 0.0f, 0.0f};
    rate_ctrl.update(desired_rate, measured_rate, kFixedDtS, rate_out);

    float outputs[8] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    mixer.mix(0.5f, rate_out[0], rate_out[1], rate_out[2],
              apx::control::FrameType::QUAD_X, outputs);

    g_sink = outputs[0] + outputs[1] + outputs[2] + outputs[3];
    (void)pos_ctrl; // constructed to prove it links; not exercised yet (no position source)
}

} // namespace

extern "C" int main() {
    // No SysTick/RTOS scheduler wired up yet (see file header) - this is
    // a free-running loop, not a real 400Hz task, until hal/stm32h7/
    // exists to drive it from a real timer.
    while (true) {
        run_one_control_iteration();
    }
}
