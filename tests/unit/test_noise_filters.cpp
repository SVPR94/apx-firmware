// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include <gtest/gtest.h>
#include "filters/noise_filters.hpp"

#include <cmath>

using namespace apx::filters;

namespace {

constexpr float kFs = 1000.0f;
constexpr float kPi = 3.14159265f;

// Steady-state amplitude gain of a filter for a sine at freq_hz.
template <typename Filter>
float sine_gain(Filter& f, float freq_hz) {
    const int settle = 4000;
    const int measure = 4000;
    float peak = 0.0f;
    for (int n = 0; n < settle + measure; ++n) {
        const float y = f.apply(std::sin(2.0f * kPi * freq_hz * n / kFs));
        if (n >= settle) {
            peak = std::fmax(peak, std::fabs(y));
        }
    }
    return peak;
}

} // namespace

TEST(ButterworthLPFTest, MinusThreeDbAtCutoff) {
    ButterworthLPF lpf;
    lpf.init(50.0f, kFs);
    EXPECT_NEAR(sine_gain(lpf, 50.0f), 0.7071f, 0.02f);
}

TEST(ButterworthLPFTest, NoResonantPeakBelowCutoff) {
    // Butterworth is maximally flat: gain never exceeds 1 in the passband.
    for (float f : {5.0f, 20.0f, 35.0f, 45.0f}) {
        ButterworthLPF lpf;
        lpf.init(50.0f, kFs);
        EXPECT_LE(sine_gain(lpf, f), 1.005f) << "at " << f << " Hz";
    }
}

TEST(ButterworthLPFTest, UnityDcGain) {
    ButterworthLPF lpf;
    lpf.init(50.0f, kFs);
    float y = 0.0f;
    for (int n = 0; n < 2000; ++n) {
        y = lpf.apply(1.0f);
    }
    EXPECT_NEAR(y, 1.0f, 1e-4f);
}

TEST(NotchFilterTest, RejectsCenterPassesDc) {
    NotchFilter notch;
    notch.init(120.0f, 20.0f, kFs);
    EXPECT_LT(sine_gain(notch, 120.0f), 0.03f);   // > 30 dB rejection
    EXPECT_GT(sine_gain(notch, 20.0f), 0.95f);    // passband well away from the notch
    float y = 0.0f;
    for (int n = 0; n < 4000; ++n) {
        y = notch.apply(1.0f);
    }
    EXPECT_NEAR(y, 1.0f, 1e-3f);
}

TEST(HarmonicNotchTest, RetuneKeepsFilterState) {
    // A DC signal through the notch must stay at 1.0 across a frequency
    // change; resetting the state on retune would drop it to 0 for a tick.
    HarmonicNotchFilter notch;
    notch.init(80.0f, 2, 20.0f, kFs);
    for (int n = 0; n < 4000; ++n) {
        notch.apply(1.0f);
    }
    notch.update_fundamental(90.0f);
    EXPECT_NEAR(notch.apply(1.0f), 1.0f, 0.05f);
}
