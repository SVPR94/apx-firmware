// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "cal_accel.hpp"
#include "apx/kernel/timer.hpp"
#include "sensor_manager/sensor_manager.hpp"
#include <cmath>
#include <time.h>

namespace apx::calibration {

static constexpr float kG = 9.80665f;
static constexpr float kMotionThreshold = 0.5f; // m/s^2

bool CalAccel::collect_samples(float avg_xyz[3]) {
    avg_xyz[0] = avg_xyz[1] = avg_xyz[2] = 0.0f;
    for (int i = 0; i < m_sample_count; i++) {
        const auto& d = apx::sensor::SensorManager::get_instance().get_primary_imu();
        avg_xyz[0] += d.accel_xyz[0];
        avg_xyz[1] += d.accel_xyz[1];
        avg_xyz[2] += d.accel_xyz[2];
        apx::kernel::delay_ms(2);
    }
    avg_xyz[0] /= m_sample_count;
    avg_xyz[1] /= m_sample_count;
    avg_xyz[2] /= m_sample_count;
    return true;
}

AccelCalResult CalAccel::calibrate_level(ProgressCallback cb) {
    AccelCalResult result;
    if (cb) cb(0, "Place drone level and still");
    apx::kernel::delay_ms(3000);
    if (cb) cb(30, "Collecting samples...");
    float avg[3] = {};
    if (!collect_samples(avg)) {
        result.error_msg = "Sample collection failed";
        return result;
    }
    // Level: accel_z should be -g, x and y should be 0
    result.offset_xyz[0] = avg[0];
    result.offset_xyz[1] = avg[1];
    result.offset_xyz[2] = avg[2] + kG; // gravity compensation
    result.scale_xyz[0] = result.scale_xyz[1] = result.scale_xyz[2] = 1.0f;
    result.success = true;
    if (cb) cb(100, "Accelerometer calibration complete");
    return result;
}

AccelCalResult CalAccel::calibrate_6point(ProgressCallback cb) {
    // Simplified: just do level cal for now
    // Full 6-point would guide user through 6 orientations
    return calibrate_level(cb);
}

} // namespace apx::calibration
