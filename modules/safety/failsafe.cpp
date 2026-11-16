// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "failsafe.hpp"
#include <iostream>
#include <algorithm>

namespace apx::safety {

FailsafeManager::FailsafeManager() : m_current_action(FailsafeAction::WARN) {
    // Default actions
    m_triggers[static_cast<int>(FailsafeTrigger::RC_LOST)].action = FailsafeAction::RTL;
    m_triggers[static_cast<int>(FailsafeTrigger::GPS_LOST)].action = FailsafeAction::ALT_HOLD;
    m_triggers[static_cast<int>(FailsafeTrigger::LOW_BATTERY)].action = FailsafeAction::RTL;
    m_triggers[static_cast<int>(FailsafeTrigger::CRITICAL_BATTERY)].action = FailsafeAction::LAND;
    m_triggers[static_cast<int>(FailsafeTrigger::GEOFENCE_BREACH)].action = FailsafeAction::RTL;
    m_triggers[static_cast<int>(FailsafeTrigger::SENSOR_FAILURE)].action = FailsafeAction::LAND;
    m_triggers[static_cast<int>(FailsafeTrigger::GCS_LOST)].action = FailsafeAction::RTL;
}

void FailsafeManager::update(float dt_s) {
    bool state_changed = false;
    for (int i = 0; i < 7; ++i) {
        if (m_triggers[i].active) {
            m_triggers[i].active_time += dt_s;
            if (m_triggers[i].active_time >= m_triggers[i].hysteresis) {
                state_changed = true;
            }
        }
    }
    if (state_changed) {
        evaluate_priority();
    }
}

void FailsafeManager::trigger(FailsafeTrigger trigger) {
    int idx = static_cast<int>(trigger);
    if (!m_triggers[idx].active) {
        m_triggers[idx].active = true;
        m_triggers[idx].active_time = 0.0f;
    }
}

void FailsafeManager::resolve(FailsafeTrigger trigger) {
    int idx = static_cast<int>(trigger);
    m_triggers[idx].active = false;
    m_triggers[idx].active_time = 0.0f;
    evaluate_priority();
}

void FailsafeManager::set_action(FailsafeTrigger trigger, FailsafeAction action) {
    m_triggers[static_cast<int>(trigger)].action = action;
}

FailsafeAction FailsafeManager::get_current_action() const {
    return m_current_action;
}

void FailsafeManager::evaluate_priority() {
    FailsafeAction highest = FailsafeAction::WARN;
    for (int i = 0; i < 7; ++i) {
        if (m_triggers[i].active && m_triggers[i].active_time >= m_triggers[i].hysteresis) {
            if (static_cast<int>(m_triggers[i].action) > static_cast<int>(highest)) {
                highest = m_triggers[i].action;
            }
        }
    }
    
    if (m_current_action != highest) {
        m_current_action = highest;
        std::cout << "Failsafe action escalated to " << static_cast<int>(m_current_action) << "\n";
    }
}

} // namespace apx::safety
