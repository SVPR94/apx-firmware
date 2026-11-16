// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include <gtest/gtest.h>
#include "mixer/motor_mixer.hpp"

using namespace apx::control;

class MotorMixerTest : public ::testing::Test {
protected:
    MotorMixer mixer;
};

TEST_F(MotorMixerTest, ZeroThrust) {
    float outputs[8]{};
    mixer.mix(0.0f, 0.0f, 0.0f, 0.0f, FrameType::QUAD_X, outputs);
    for (int i = 0; i < 4; ++i) {
        EXPECT_FLOAT_EQ(outputs[i], 0.0f);
    }
}

TEST_F(MotorMixerTest, MaxThrust) {
    float outputs[8]{};
    mixer.mix(1.0f, 0.0f, 0.0f, 0.0f, FrameType::QUAD_X, outputs);
    for (int i = 0; i < 4; ++i) {
        EXPECT_FLOAT_EQ(outputs[i], 1.0f);
    }
}

TEST_F(MotorMixerTest, RollDifferential) {
    float outputs[8]{};
    mixer.mix(0.5f, 0.2f, 0.0f, 0.0f, FrameType::QUAD_X, outputs);
    EXPECT_NE(outputs[0], outputs[1]);
    for (int i = 0; i < 4; ++i) {
        EXPECT_GE(outputs[i], 0.0f);
        EXPECT_LE(outputs[i], 1.0f);
    }
}

TEST_F(MotorMixerTest, HexXMix) {
    float outputs[8]{};
    mixer.mix(0.6f, 0.0f, 0.0f, 0.0f, FrameType::HEX_X, outputs);
    for (int i = 0; i < 6; ++i) {
        EXPECT_NEAR(outputs[i], 0.6f, 1e-4f);
    }
    EXPECT_FLOAT_EQ(outputs[6], 0.0f);
    EXPECT_FLOAT_EQ(outputs[7], 0.0f);
}

// Frames without a mixing table must not touch the outputs (previously
// they reported a motor count and dereferenced a null table).
TEST_F(MotorMixerTest, FramesWithoutTableLeaveOutputsUntouched) {
    for (FrameType frame : {FrameType::QUAD_PLUS, FrameType::HEX_FLAT,
                            FrameType::OCTO_COAX, FrameType::OCTO_PLUS}) {
        float outputs[8];
        for (float& o : outputs) o = -1.0f;
        mixer.mix(0.5f, 0.1f, 0.1f, 0.1f, frame, outputs);
        for (float o : outputs) {
            EXPECT_FLOAT_EQ(o, -1.0f);
        }
    }
}
