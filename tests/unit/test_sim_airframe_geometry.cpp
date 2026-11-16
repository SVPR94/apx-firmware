// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include <gtest/gtest.h>
#include "mixer/motor_geometry.hpp"
#include "physics/multirotor_dynamics.hpp"

#include <algorithm>
#include <cmath>

using apx::control::MotorConfig;
using apx::sitl::MultirotorParams;

namespace {

// The built-in physics airframe and the firmware mixer must agree motor by
// motor: a motor at body (x, y) with spin d is driven with
// pitch = x/arm, roll = -y/arm, yaw = d (FRD body, see multirotor_dynamics.cpp).
// A mismatch flies while symmetric and diverges on the first asymmetry.
void expect_matches(const MultirotorParams& p, const MotorConfig* table, int n, float arm) {
    ASSERT_EQ(p.num_motors, n);
    for (int i = 0; i < n; ++i) {
        SCOPED_TRACE("motor " + std::to_string(i + 1));
        EXPECT_NEAR(p.motor_pos[i][0] / arm, table[i].pitch_factor, 1e-3f);
        EXPECT_NEAR(-p.motor_pos[i][1] / arm, table[i].roll_factor, 1e-3f);
        EXPECT_FLOAT_EQ(p.motor_dir[i], table[i].yaw_factor);
    }
}

// Mixer factors for X frames are the motor's unit direction (quad uses +-1
// per axis, i.e. arm/sqrt(2) scaled), so compare against normalized position.
void expect_matches_quad(const MultirotorParams& p) {
    ASSERT_EQ(p.num_motors, 4);
    for (int i = 0; i < 4; ++i) {
        SCOPED_TRACE("motor " + std::to_string(i + 1));
        EXPECT_EQ(std::signbit(p.motor_pos[i][0]), std::signbit(apx::control::QUAD_X_MOTORS[i].pitch_factor));
        EXPECT_EQ(std::signbit(-p.motor_pos[i][1]), std::signbit(apx::control::QUAD_X_MOTORS[i].roll_factor));
        EXPECT_FLOAT_EQ(p.motor_dir[i], apx::control::QUAD_X_MOTORS[i].yaw_factor);
    }
}

} // namespace

TEST(SimAirframeGeometryTest, QuadXMatchesMixer) {
    expect_matches_quad(MultirotorParams::make_quad_x(2.0f, 0.225f));
}

TEST(SimAirframeGeometryTest, HexXMatchesMixer) {
    expect_matches(MultirotorParams::make_hex_x(2.8f, 0.28f), apx::control::HEX_X_MOTORS, 6, 0.28f);
}

TEST(SimAirframeGeometryTest, OctoXMatchesMixer) {
    expect_matches(MultirotorParams::make_octo_x(3.8f, 0.33f), apx::control::OCTO_X_MOTORS, 8, 0.33f);
}

// Spin directions must alternate around the ring so yaw reaction torques
// cancel between neighbours (a CW-CW pair couples yaw into roll/pitch).
TEST(SimAirframeGeometryTest, SpinAlternatesAroundRing) {
    for (const auto& p : {MultirotorParams::make_hex_x(2.8f, 0.28f), MultirotorParams::make_octo_x(3.8f, 0.33f)}) {
        const int n = p.num_motors;
        int order[8];
        for (int i = 0; i < n; ++i) order[i] = i;
        auto angle = [&p](int i) { return std::atan2(p.motor_pos[i][1], p.motor_pos[i][0]); };
        std::sort(order, order + n, [&](int a, int b) { return angle(a) < angle(b); });
        for (int k = 0; k < n; ++k) {
            EXPECT_NE(p.motor_dir[order[k]], p.motor_dir[order[(k + 1) % n]]) << n << "-rotor, position " << k;
        }
    }
}
