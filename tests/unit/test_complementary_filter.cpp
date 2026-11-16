// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include <gtest/gtest.h>
#include "complementary_filter.hpp"
#include <cmath>

using namespace apx::estimator;

class ComplementaryFilterTest : public ::testing::Test {
protected:
    ComplementaryFilter filter;

    void SetUp() override {
        filter.reset();
        filter.set_gains(2.0f, 0.05f);
    }
};

TEST_F(ComplementaryFilterTest, LevelConvergence) {
    float gyro[3] = {0.0f, 0.0f, 0.0f};
    float accel[3] = {0.0f, 0.0f, 9.81f};
    for (int i = 0; i < 100; ++i) {
        filter.update(gyro, accel, nullptr, 0.01f);
    }
    const auto& state = filter.get_state();
    EXPECT_NEAR(state.euler_rpy[0], 0.0f, 0.05f);
    EXPECT_NEAR(state.euler_rpy[1], 0.0f, 0.05f);
}

TEST_F(ComplementaryFilterTest, PitchConvergence) {
    float gyro[3] = {0.0f, 0.0f, 0.0f};
    // 45 degrees pitch: ax = -g * sin(45), az = g * cos(45)
    float g = 9.81f;
    float sin45 = 0.7071f;
    float accel[3] = {-sin45 * g, 0.0f, sin45 * g};
    for (int i = 0; i < 500; ++i) {
        filter.update(gyro, accel, nullptr, 0.01f);
    }
    const auto& state = filter.get_state();
    EXPECT_NEAR(state.euler_rpy[1], 0.785f, 0.1f); // ~pi/4 rad
}
