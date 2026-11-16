// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include <gtest/gtest.h>
#include "sensor_voter.hpp"

using apx::sensor::SensorVoter;

class SensorVoterTest : public ::testing::Test {
protected:
    SensorVoter<float, 3> voter;

    void SetUp() override {
        voter.set_max_deviation(0.5f);
    }
};

TEST_F(SensorVoterTest, AllAgree) {
    voter.add_measurement(0, 10.0f, true);
    voter.add_measurement(1, 10.1f, true);
    voter.add_measurement(2, 9.9f, true);
    voter.vote();
    EXPECT_TRUE(voter.has_winner());
    EXPECT_NEAR(voter.get_best(), 10.0f, 0.2f);
}

TEST_F(SensorVoterTest, OneOutlier) {
    voter.add_measurement(0, 10.0f, true);
    voter.add_measurement(1, 10.1f, true);
    voter.add_measurement(2, 50.0f, true);
    voter.vote();
    EXPECT_TRUE(voter.has_winner());
    EXPECT_NE(voter.get_winner_index(), 2);
    EXPECT_NEAR(voter.get_best(), 10.05f, 0.1f);
}

TEST_F(SensorVoterTest, InvalidSensorExcluded) {
    voter.add_measurement(0, 10.0f, true);
    voter.add_measurement(1, 10.1f, true);
    voter.add_measurement(2, 10.0f, false);
    voter.vote();
    EXPECT_TRUE(voter.has_winner());
    EXPECT_NE(voter.get_winner_index(), 2);
}
