// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once
#include <functional>
namespace apx::calibration {

struct GyroCalResult {
    float offset_xyz[3]{};
    bool  success{false};
    const char* error_msg{nullptr};
};

using ProgressCallback = std::function<void(int pct, const char* msg)>;

class CalGyro {
public:
    GyroCalResult calibrate(ProgressCallback cb = nullptr);
    void set_sample_count(int n) { m_sample_count = n; }
private:
    int m_sample_count{1000};
    static constexpr float kMotionThreshold = 0.05f; // rad/s
};
} // namespace apx::calibration
