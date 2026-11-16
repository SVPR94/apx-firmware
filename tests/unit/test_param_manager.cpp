// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include <gtest/gtest.h>
#include "param_manager.hpp"

#include <future>

using apx::gcs::ParamManager;
using apx::gcs::ParamValue;

// Change callbacks run after the ParamManager lock is released, so a
// callback may read parameters. This deadlocked before (hosted mutex is
// non-recursive); the test bounds the call with a timeout instead of hanging.
TEST(ParamManagerTest, CallbackMayReadParameters) {
    auto& params = ParamManager::get_instance();
    params.init_defaults();
    float seen = -1.0f;
    params.add_change_callback([&seen](const std::string& name, const ParamValue&) {
        if (name == "WPNAV_RADIUS") {
            seen = ParamManager::get_instance().get_float_or("WPNAV_RADIUS", -2.0f);
        }
    });
    auto result = std::async(std::launch::async, [&params] { return params.set_float("WPNAV_RADIUS", 350.0f); });
    ASSERT_EQ(result.wait_for(std::chrono::seconds(2)), std::future_status::ready) << "set_float deadlocked";
    EXPECT_TRUE(result.get());
    EXPECT_FLOAT_EQ(seen, 350.0f);
}
