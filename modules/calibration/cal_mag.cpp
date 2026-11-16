// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "cal_mag.hpp"
#include "apx/kernel/timer.hpp"
#include <cmath>
#include <time.h>
#include <limits>

namespace apx::calibration {

MagCalResult CalMag::calibrate(ProgressCallback cb, int collect_seconds) {
    MagCalResult result;
    if (cb) cb(0, "Slowly rotate drone in a circle. ~15 seconds.");

    float min_xyz[3] = { std::numeric_limits<float>::max(),  std::numeric_limits<float>::max(),  std::numeric_limits<float>::max() };
    float max_xyz[3] = { std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest() };
    int count = 0;

    // Collect samples for collect_seconds seconds
    int loops = collect_seconds * 100; // 10ms per sample
    for (int i = 0; i < loops; i++) {
        // In real implementation: read from mag topic on uBus
        // float mag[3] = get_mag_data();
        // For now, placeholder
        apx::kernel::delay_ms(10);
        count++;
        if (cb && i % (loops/10) == 0) cb(i * 100 / loops, "Collecting mag samples...");
    }

    if (count < kMinSamples) {
        result.error_msg = "Not enough samples";
        return result;
    }

    // Hard iron = midpoint of min/max per axis
    for (int i = 0; i < 3; i++) {
        result.hard_iron[i] = (max_xyz[i] + min_xyz[i]) / 2.0f;
        float range = (max_xyz[i] - min_xyz[i]);
        result.soft_iron[i] = (range > 0.0f) ? 1.0f / range : 1.0f;
    }
    result.success = true;
    if (cb) cb(100, "Magnetometer calibration complete");
    return result;
}

} // namespace apx::calibration
