// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include <gtest/gtest.h>
#include "fence.hpp"
#include "param_manager.hpp"

using apx::safety::FenceManager;
using apx::safety::FencePoint;
using apx::safety::FenceType;
using apx::safety::FenceBreachType;
using apx::safety::FenceAction;
using apx::gcs::ParamManager;

class FenceTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto& pm = ParamManager::get_instance();
        pm.init_defaults();
        pm.set_int("FENCE_ENABLE", 1);
        pm.set_int("FENCE_TYPE", 5); // Alt + Polygon
        pm.set_int("FENCE_ACTION", 1); // RTL
        pm.set_float("FENCE_ALT_MAX", 50.0f);
        pm.set_float("FENCE_MARGIN", 0.0f);

        auto& fm = FenceManager::instance();
        fm.reset();
        fm.clear_polygon();
    }

    void TearDown() override {
        auto& pm = ParamManager::get_instance();
        pm.set_int("FENCE_ENABLE", 0);
        auto& fm = FenceManager::instance();
        fm.reset();
        fm.clear_polygon();
    }
};

TEST_F(FenceTest, PointInPolygonGeometricTest) {
    // Square polygon: 100x100 centered at origin (-50,-50) to (50,50)
    std::vector<FencePoint> square = {
        {-50.0f, -50.0f},
        { 50.0f, -50.0f},
        { 50.0f,  50.0f},
        {-50.0f,  50.0f}
    };

    // Inside points
    EXPECT_TRUE(FenceManager::point_in_polygon(0.0f, 0.0f, square));
    EXPECT_TRUE(FenceManager::point_in_polygon(20.0f, -30.0f, square));
    EXPECT_TRUE(FenceManager::point_in_polygon(-40.0f, 40.0f, square));

    // Outside points
    EXPECT_FALSE(FenceManager::point_in_polygon(60.0f, 0.0f, square));
    EXPECT_FALSE(FenceManager::point_in_polygon(0.0f, -70.0f, square));
    EXPECT_FALSE(FenceManager::point_in_polygon(100.0f, 100.0f, square));

    // Concave polygon (L-shape)
    std::vector<FencePoint> l_shape = {
        {0.0f, 0.0f},
        {40.0f, 0.0f},
        {40.0f, 20.0f},
        {20.0f, 20.0f},
        {20.0f, 40.0f},
        {0.0f, 40.0f}
    };
    EXPECT_TRUE(FenceManager::point_in_polygon(10.0f, 10.0f, l_shape));
    EXPECT_TRUE(FenceManager::point_in_polygon(10.0f, 30.0f, l_shape));
    EXPECT_TRUE(FenceManager::point_in_polygon(30.0f, 10.0f, l_shape));
    // The cut-out corner of the L: (30, 30) should be outside
    EXPECT_FALSE(FenceManager::point_in_polygon(30.0f, 30.0f, l_shape));

    // Polygon with fewer than 3 points is not a valid enclosure
    std::vector<FencePoint> invalid = {{0, 0}, {10, 10}};
    EXPECT_TRUE(FenceManager::point_in_polygon(50.0f, 50.0f, invalid));
}

TEST_F(FenceTest, AltitudeFenceDetection) {
    auto& fm = FenceManager::instance();
    auto& pm = ParamManager::get_instance();
    pm.set_int("FENCE_TYPE", 1); // Alt max only
    pm.set_float("FENCE_ALT_MAX", 50.0f);
    pm.set_float("FENCE_MARGIN", 2.0f);

    // Altitude well below ceiling
    FenceBreachType b = fm.check(25.0f, 0.0f, 0.0f, 0.1f);
    EXPECT_EQ(b, FenceBreachType::NONE);

    // Altitude right at ceiling + 1m (within margin 2m)
    b = fm.check(51.0f, 0.0f, 0.0f, 0.1f);
    EXPECT_EQ(b, FenceBreachType::NONE);

    // Altitude exceeds ceiling + margin (52m)
    // Run for > 0.2s debounce
    fm.check(55.0f, 0.0f, 0.0f, 0.1f);
    b = fm.check(55.0f, 0.0f, 0.0f, 0.15f);
    EXPECT_EQ(b, FenceBreachType::ALTITUDE);
    EXPECT_TRUE(fm.is_breached());
    EXPECT_EQ(fm.breach_count(), 1);
}

TEST_F(FenceTest, ZeroAltitudeMaxMeansNoHeightRestriction) {
    auto& fm = FenceManager::instance();
    auto& pm = ParamManager::get_instance();
    pm.set_int("FENCE_TYPE", 1); // Alt max only
    // User requirement: if 0 then consider no height restrictions
    pm.set_float("FENCE_ALT_MAX", 0.0f);

    // Check with extreme altitudes: should NEVER breach
    for (float alt = 10.0f; alt <= 5000.0f; alt += 500.0f) {
        FenceBreachType b = fm.check(alt, 0.0f, 0.0f, 0.5f);
        EXPECT_EQ(b, FenceBreachType::NONE);
        EXPECT_FALSE(fm.is_breached());
    }
}

TEST_F(FenceTest, PolygonFenceDetection) {
    auto& fm = FenceManager::instance();
    auto& pm = ParamManager::get_instance();
    pm.set_int("FENCE_TYPE", 4); // Polygon only
    std::vector<FencePoint> poly = {
        {-20.0f, -20.0f},
        { 20.0f, -20.0f},
        { 20.0f,  20.0f},
        {-20.0f,  20.0f}
    };
    fm.set_polygon_points(poly);

    // Inside polygon
    FenceBreachType b = fm.check(10.0f, 0.0f, 0.0f, 0.1f);
    EXPECT_EQ(b, FenceBreachType::NONE);

    // Outside polygon: sustained for debounce (> 0.2s)
    fm.check(10.0f, 50.0f, 0.0f, 0.15f);
    b = fm.check(10.0f, 50.0f, 0.0f, 0.15f);
    EXPECT_EQ(b, FenceBreachType::POLYGON);
    EXPECT_TRUE(fm.is_breached());
    EXPECT_EQ(fm.breach_count(), 1);

    // Return inside polygon: clears after 1.0s clear debounce
    for (int i = 0; i < 11; ++i) {
        b = fm.check(10.0f, 0.0f, 0.0f, 0.1f);
    }
    EXPECT_EQ(b, FenceBreachType::NONE);
    EXPECT_FALSE(fm.is_breached());
}

TEST_F(FenceTest, PreArmCheckValidation) {
    auto& fm = FenceManager::instance();
    auto& pm = ParamManager::get_instance();
    pm.set_int("FENCE_ENABLE", 1);
    pm.set_int("FENCE_TYPE", 5);
    pm.set_float("FENCE_ALT_MAX", 30.0f);

    std::vector<FencePoint> poly = {
        {-10.0f, -10.0f},
        { 10.0f, -10.0f},
        { 10.0f,  10.0f},
        {-10.0f,  10.0f}
    };
    fm.set_polygon_points(poly);

    std::string msg;
    // Inside polygon, altitude 0 (on ground) -> safe to arm
    EXPECT_TRUE(fm.pre_arm_check(0.0f, 0.0f, 0.0f, msg));

    // Vehicle placed outside boundary -> arming denied
    EXPECT_FALSE(fm.pre_arm_check(0.0f, 25.0f, 0.0f, msg));
    EXPECT_FALSE(msg.empty());

    // Vehicle altitude exceeds maximum -> arming denied
    EXPECT_FALSE(fm.pre_arm_check(45.0f, 0.0f, 0.0f, msg));
    EXPECT_FALSE(msg.empty());

    // When fence is disabled, pre-arm check always passes
    pm.set_int("FENCE_ENABLE", 0);
    EXPECT_TRUE(fm.pre_arm_check(50.0f, 100.0f, 100.0f, msg));
}

TEST_F(FenceTest, ActionConfiguration) {
    auto& fm = FenceManager::instance();
    auto& pm = ParamManager::get_instance();

    pm.set_int("FENCE_ACTION", 0);
    EXPECT_EQ(fm.get_action(), FenceAction::REPORT_ONLY);

    pm.set_int("FENCE_ACTION", 1);
    EXPECT_EQ(fm.get_action(), FenceAction::RTL);

    pm.set_int("FENCE_ACTION", 2);
    EXPECT_EQ(fm.get_action(), FenceAction::LAND);

    pm.set_int("FENCE_ACTION", 4);
    EXPECT_EQ(fm.get_action(), FenceAction::BRAKE);
}
