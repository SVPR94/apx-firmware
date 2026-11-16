// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include <gtest/gtest.h>
#include "rate_controller/pid_controller.hpp"
#include <cmath>

using namespace apx::control;

class PidControllerTest : public ::testing::Test {
protected:
    PidController pid;

    void SetUp() override {
        pid.set_gains(1.0f, 0.5f, 0.1f);
        pid.set_output_limits(-10.0f, 10.0f);
        pid.reset();
    }
};

TEST_F(PidControllerTest, StepResponse) {
    float output = pid.compute(1.0f, 0.01f);
    EXPECT_GT(output, 0.0f);
}

TEST_F(PidControllerTest, IntegralWindup) {
    pid.set_gains(0.0f, 1.0f, 0.0f);
    pid.set_integral_limit(2.0f);
    for (int i = 0; i < 1000; ++i) {
        pid.compute(1.0f, 0.1f);
    }
    EXPECT_LE(pid.get_integral(), 2.01f);
    EXPECT_GE(pid.get_integral(), 1.99f);
}

// A step in error after a settled tick gives a derivative response that is
// low-pass filtered (smaller than the raw kd*de/dt) and decays afterwards.
TEST_F(PidControllerTest, DerivativeFilter) {
    pid.set_gains(1.0f, 0.0f, 0.1f);
    pid.set_derivative_lpf_cutoff(5.0f);
    pid.compute(0.0f, 0.01f);                  // settled, establishes the previous error
    float out_step = pid.compute(1.0f, 0.01f); // step: P = 1 plus filtered D
    float out_next = pid.compute(1.0f, 0.01f); // no further change: D decays
    const float raw_d = 0.1f * 1.0f / 0.01f;
    EXPECT_GT(out_step, 1.0f);
    EXPECT_LT(out_step - 1.0f, raw_d);
    EXPECT_LT(out_next, out_step);
}

TEST_F(PidControllerTest, OutputLimits) {
    pid.set_gains(100.0f, 0.0f, 0.0f);
    pid.set_output_limits(-5.0f, 5.0f);
    float output = pid.compute(100.0f, 0.01f);
    EXPECT_FLOAT_EQ(output, 5.0f);

    float output_neg = pid.compute(-100.0f, 0.01f);
    EXPECT_FLOAT_EQ(output_neg, -5.0f);
}

// After reset() there is no previous error to differentiate against; the
// first tick must not produce a D kick of kd * error / dt.
TEST_F(PidControllerTest, NoDerivativeKickOnFirstTickAfterReset) {
    pid.set_gains(0.0f, 0.0f, 1.0f);
    pid.reset();
    EXPECT_NEAR(pid.compute(2.0f, 0.0025f), 0.0f, 1e-6f);
    // A real change on the next tick still produces derivative action.
    EXPECT_GT(pid.compute(2.1f, 0.0025f), 0.0f);
}
