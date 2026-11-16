// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include <gtest/gtest.h>

#include "altitude_estimator.hpp"

#include <cmath>
#include <random>
#include <vector>

using apx::estimator::AltitudeEstimator;

namespace {

constexpr float kG = 9.80665f;
constexpr uint64_t kImuUs = 2500;  // 400 Hz
constexpr int kBaroEvery = 8;      // 50 Hz
const float kLevel[4] = {1.0f, 0.0f, 0.0f, 0.0f};

struct Sim {
    AltitudeEstimator est;
    std::mt19937 rng{3};
    float accel_bias{0.0f};
    float accel_sigma{0.0f};
    float baro_sigma{0.0f};
    float baro_datum{540.0f};
    uint64_t t{1000};
    int k{0};

    // One IMU tick of true vertical motion (h up, a_up); baro every 8th.
    void step(float h, float a_up) {
        std::normal_distribution<float> n(0.0f, 1.0f);
        const float f[3] = {0.0f, 0.0f, -(a_up + kG) + accel_bias + accel_sigma * n(rng)};
        est.predict_imu(t, f, kLevel);
        if (k % kBaroEvery == 0) {
            est.update_baro(0, baro_datum + h + baro_sigma * n(rng));
        }
        t += kImuUs;
        ++k;
    }
};

} // namespace

TEST(AltitudeEstimatorTest, StartsAtZeroAfterReferencePeriod) {
    Sim s;
    for (int i = 0; i < 200; ++i) s.step(0.0f, 0.0f); // 0.5 s: still referencing
    EXPECT_FALSE(s.est.initialised());
    for (int i = 0; i < 300; ++i) s.step(0.0f, 0.0f); // 1.25 s
    ASSERT_TRUE(s.est.initialised());
    EXPECT_NEAR(s.est.height(), 0.0f, 0.01f);
    EXPECT_NEAR(s.est.climb_rate(), 0.0f, 0.01f);
}

TEST(AltitudeEstimatorTest, TracksClimbWithoutLag) {
    Sim s;
    for (int i = 0; i < 800; ++i) s.step(0.0f, 0.0f);
    // accelerate at 1 m/s^2 for 2 s, cruise 2 m/s for 3 s
    float h = 0.0f, v = 0.0f;
    const float dt = kImuUs * 1e-6f;
    for (int i = 0; i < 2000; ++i) {
        const float a = (i < 800) ? 1.0f : 0.0f;
        v += a * dt;
        h += v * dt;
        s.step(h, a);
    }
    EXPECT_NEAR(s.est.climb_rate(), v, 0.02f);
    EXPECT_NEAR(s.est.height(), h, 0.02f);
}

TEST(AltitudeEstimatorTest, LearnsAccelBias) {
    Sim s;
    s.accel_bias = 0.3f; // body z reads 0.3 m/s^2 high (down), i.e. a_up reads 0.3 low
    for (int i = 0; i < 400 * 60; ++i) s.step(0.0f, 0.0f);
    EXPECT_NEAR(s.est.accel_bias(), -0.3f, 0.03f);
    EXPECT_NEAR(s.est.climb_rate(), 0.0f, 0.02f);
}

TEST(AltitudeEstimatorTest, SmoothsBaroNoise) {
    Sim s;
    s.baro_sigma = 0.3f;
    s.accel_sigma = 1.0f;
    for (int i = 0; i < 800; ++i) s.step(0.0f, 0.0f);
    std::vector<float> h;
    for (int i = 0; i < 400 * 60; ++i) {
        s.step(0.0f, 0.0f);
        h.push_back(s.est.height());
    }
    double m = 0, v = 0;
    for (float x : h) m += x;
    m /= h.size();
    for (float x : h) v += (x - m) * (x - m);
    const double sd = std::sqrt(v / h.size());
    EXPECT_LT(sd, 0.3 / 2.0); // well below the raw baro noise
    EXPECT_GT(sd, 0.005);     // but not frozen: noise still shows through
}

TEST(AltitudeEstimatorTest, GatesBaroGlitchButRecoversFromStep) {
    Sim s;
    for (int i = 0; i < 800; ++i) s.step(0.0f, 0.0f);
    // single 30 m glitch is rejected
    s.est.update_baro(0, s.baro_datum + 30.0f);
    EXPECT_EQ(s.est.baro_rejects(), 1u);
    EXPECT_NEAR(s.est.height(), 0.0f, 0.05f);
    // a lasting 5 m shift is eventually accepted
    for (int i = 0; i < 400 * 10; ++i) s.step(5.0f, 0.0f);
    EXPECT_NEAR(s.est.height(), 5.0f, 0.1f);
}

TEST(AltitudeEstimatorTest, RezeroMakesCurrentHeightZero) {
    Sim s;
    s.baro_datum = 540.0f;
    for (int i = 0; i < 800; ++i) s.step(0.0f, 0.0f);
    // baro drifts 0.8 m over 20 s while parked (temperature, weather)
    for (int i = 0; i < 400 * 20; ++i) s.step(0.8f * static_cast<float>(i) / (400 * 20), 0.0f);
    for (int i = 0; i < 400 * 5; ++i) s.step(0.8f, 0.0f);
    EXPECT_NEAR(s.est.height(), 0.8f, 0.05f);
    s.est.rezero();
    EXPECT_NEAR(s.est.height(), 0.0f, 1e-6f);
    for (int i = 0; i < 400 * 2; ++i) s.step(0.8f, 0.0f);
    EXPECT_NEAR(s.est.height(), 0.0f, 0.05f);
}
