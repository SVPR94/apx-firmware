// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once
#include <functional>
#include <vector>
#include <array>
namespace apx::calibration {

struct MagCalResult {
    float hard_iron[3]{};
    float soft_iron[3]{1,1,1};
    bool  success{false};
    const char* error_msg{nullptr};
};

using ProgressCallback = std::function<void(int pct, const char* msg)>;

class CalMag {
public:
    // Simple: slowly rotate drone, collect samples
    // No 360 random spins - just guided slow rotation in 2 orientations
    MagCalResult calibrate(ProgressCallback cb = nullptr, int collect_seconds = 15);
private:
    static constexpr int kMinSamples = 100;
};
} // namespace apx::calibration
