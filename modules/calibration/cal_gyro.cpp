// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "cal_gyro.hpp"
#include "apx/kernel/timer.hpp"
#include "sensor_manager/sensor_manager.hpp"
#include <cmath>
#include <time.h>

namespace apx::calibration {

GyroCalResult CalGyro::calibrate(ProgressCallback cb) {
    GyroCalResult result;
    if (cb) cb(0, "Keep drone completely still");
    apx::kernel::delay_ms(2000);

    float sum[3] = {};
    float prev[3] = {};
    bool first = true;

    for (int i = 0; i < m_sample_count; i++) {
        const auto& d = apx::sensor::SensorManager::get_instance().get_primary_imu();
        if (!first) {
            for (int j = 0; j < 3; j++) {
                if (std::abs(d.gyro_xyz[j] - prev[j]) > kMotionThreshold) {
                    result.error_msg = "Motion detected during calibration";
                    return result;
                }
            }
        }
        for (int j = 0; j < 3; j++) {
            sum[j] += d.gyro_xyz[j];
            prev[j] = d.gyro_xyz[j];
        }
        first = false;
        if (cb && i % 100 == 0) cb(i * 100 / m_sample_count, "Calibrating...");
        apx::kernel::delay_ms(1);
    }

    for (int j = 0; j < 3; j++) result.offset_xyz[j] = sum[j] / m_sample_count;
    result.success = true;
    if (cb) cb(100, "Gyro calibration complete");
    return result;
}

} // namespace apx::calibration
