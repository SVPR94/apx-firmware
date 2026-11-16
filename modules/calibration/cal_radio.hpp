// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once
#include <functional>
#include <cstdint>
namespace apx::calibration {
using ProgressCallback = std::function<void(int pct, const char* msg)>;
struct RadioCalResult {
    float min_val[18]{}, max_val[18]{}, center[18]{};
    bool reversed[18]{};
    bool success{false};
    const char* error_msg{nullptr};
};
class CalRadio {
public:
    // Move all sticks to extremes during collection_seconds
    RadioCalResult calibrate(int collection_seconds = 10, ProgressCallback cb = nullptr);
};
} // namespace apx::calibration
