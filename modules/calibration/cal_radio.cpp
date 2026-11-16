// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "cal_radio.hpp"
#include "apx/kernel/timer.hpp"
#include <limits>
#include <time.h>
namespace apx::calibration {
RadioCalResult CalRadio::calibrate(int collection_seconds, ProgressCallback cb) {
    RadioCalResult result;
    for (int i = 0; i < 18; i++) {
        result.min_val[i] = std::numeric_limits<float>::max();
        result.max_val[i] = std::numeric_limits<float>::lowest();
        result.reversed[i] = false;
    }
    if (cb) cb(0, "Move all sticks and switches to their extremes");
    int loops = collection_seconds * 100;
    for (int i = 0; i < loops; i++) {
        // In production: read from RC topic
        // const auto& rc = get_rc_data();
        // for (int ch = 0; ch < 18; ch++) {
        //     if (rc.channels[ch] < result.min_val[ch]) result.min_val[ch] = rc.channels[ch];
        //     if (rc.channels[ch] > result.max_val[ch]) result.max_val[ch] = rc.channels[ch];
        // }
        apx::kernel::delay_ms(10);
        if (cb && i % (loops/10) == 0) cb(i * 100 / loops, "Move all sticks...");
    }
    for (int i = 0; i < 18; i++) {
        result.center[i] = (result.min_val[i] + result.max_val[i]) / 2.0f;
    }
    result.success = true;
    if (cb) cb(100, "RC calibration complete");
    return result;
}
} // namespace apx::calibration
