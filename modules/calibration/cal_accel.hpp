// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once
#include <functional>
namespace apx::calibration {

struct AccelCalResult {
    float offset_xyz[3]{};
    float scale_xyz[3]{1,1,1};
    bool  success{false};
    const char* error_msg{nullptr};
};

using ProgressCallback = std::function<void(int pct, const char* msg)>;

class CalAccel {
public:
    AccelCalResult calibrate_level(ProgressCallback cb = nullptr);
    AccelCalResult calibrate_6point(ProgressCallback cb = nullptr);
    void set_sample_count(int n) { m_sample_count = n; }
private:
    int m_sample_count{500};
    bool collect_samples(float avg_xyz[3]);
    bool detect_motion(const float samples[][3], int count);
};
} // namespace apx::calibration
