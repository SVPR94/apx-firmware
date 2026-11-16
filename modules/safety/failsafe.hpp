// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include <cstdint>
#include <vector>

namespace apx::safety {

enum class FailsafeTrigger {
    RC_LOST,
    GPS_LOST,
    LOW_BATTERY,
    CRITICAL_BATTERY,
    GEOFENCE_BREACH,
    SENSOR_FAILURE,
    GCS_LOST
};

enum class FailsafeAction {
    WARN,
    ALT_HOLD,
    RTL,
    LAND,
    DISARM
};

class FailsafeManager {
public:
    FailsafeManager();
    ~FailsafeManager() = default;

    void update(float dt_s);
    void trigger(FailsafeTrigger trigger);
    void resolve(FailsafeTrigger trigger);
    void set_action(FailsafeTrigger trigger, FailsafeAction action);
    FailsafeAction get_current_action() const;

private:
    struct State {
        bool active = false;
        float active_time = 0.0f;
        FailsafeAction action = FailsafeAction::WARN;
        const float hysteresis = 1.0f;
    };

    State m_triggers[7];
    FailsafeAction m_current_action;

    void evaluate_priority();
};

} // namespace apx::safety
