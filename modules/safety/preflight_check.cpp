// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "preflight_check.hpp"

namespace apx::safety {

PreflightCheck::PreflightCheck() {}

void PreflightCheck::run_checks() {
    m_errors.clear();

    if (!check_imu_calibrated()) {
        m_errors.push_back({"IMU not calibrated", true});
    }
    if (!check_mag_calibrated()) {
        m_errors.push_back({"Compass not calibrated", true});
    }
    if (!check_rc_calibrated()) {
        m_errors.push_back({"RC not calibrated", true});
    }
    if (!check_gps_fix()) {
        m_errors.push_back({"No GPS fix", false}); // might be non-critical for some modes
    }
    if (!check_baro_healthy()) {
        m_errors.push_back({"Barometer unhealthy", true});
    }
    if (!check_battery_voltage()) {
        m_errors.push_back({"Battery voltage low", true});
    }
}

bool PreflightCheck::can_arm() const {
    for (const auto& err : m_errors) {
        if (err.is_critical) {
            return false;
        }
    }
    return true;
}

std::vector<PreflightError> PreflightCheck::get_errors() const {
    return m_errors;
}

// Stubs for actual checks
bool PreflightCheck::check_imu_calibrated() { return true; }
bool PreflightCheck::check_mag_calibrated() { return true; }
bool PreflightCheck::check_rc_calibrated() { return true; }
bool PreflightCheck::check_gps_fix() { return true; }
bool PreflightCheck::check_baro_healthy() { return true; }
bool PreflightCheck::check_battery_voltage() { return true; }

} // namespace apx::safety
