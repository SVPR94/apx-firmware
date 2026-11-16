// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once
#include <functional>
namespace apx::calibration {
using ProgressCallback = std::function<void(int pct, const char* msg)>;
struct EscCalResult { bool success{false}; const char* error_msg{nullptr}; };
class CalEsc {
public:
    EscCalResult calibrate(bool armed, ProgressCallback cb = nullptr);
};
} // namespace apx::calibration
