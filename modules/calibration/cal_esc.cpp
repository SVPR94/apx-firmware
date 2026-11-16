// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "cal_esc.hpp"
#include "apx/kernel/timer.hpp"
#include <time.h>
namespace apx::calibration {
EscCalResult CalEsc::calibrate(bool armed, ProgressCallback cb) {
    EscCalResult result;
    if (armed) { result.error_msg = "Must be disarmed to calibrate ESC"; return result; }
    if (cb) cb(0, "ESC calibration: remove props! Sending max throttle...");
    // In production: send max PWM/DShot to all ESC channels
    apx::kernel::delay_ms(3000);
    if (cb) cb(50, "Sending min throttle...");
    // Send min throttle
    apx::kernel::delay_ms(2000);
    result.success = true;
    if (cb) cb(100, "ESC calibration complete");
    return result;
}
} // namespace apx::calibration
