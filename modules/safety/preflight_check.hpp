// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include <string>
#include <vector>

namespace apx::safety {

struct PreflightError {
    std::string message;
    bool is_critical;
};

class PreflightCheck {
public:
    PreflightCheck();
    ~PreflightCheck() = default;

    void run_checks();
    bool can_arm() const;
    std::vector<PreflightError> get_errors() const;

private:
    std::vector<PreflightError> m_errors;

    bool check_imu_calibrated();
    bool check_mag_calibrated();
    bool check_rc_calibrated();
    bool check_gps_fix();
    bool check_baro_healthy();
    bool check_battery_voltage();
};

} // namespace apx::safety
