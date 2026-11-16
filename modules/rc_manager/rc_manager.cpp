// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "rc_manager.hpp"
#include "rc_input/sbus/sbus_driver.hpp"
#include "modes/mode_base.hpp"
#include "apx/kernel/timer.hpp"
#include "param_manager.hpp"
#include <cmath>
#include <cstdio>
#include <algorithm>

namespace apx::rc {

void RcManager::register_source(apx::drivers::DriverBase* src, int priority) {
    if(m_source_count>=MAX_SOURCES) return;
    m_sources[m_source_count++]={src,priority};
    // Sort by priority (higher = better)
    std::sort(m_sources.begin(), m_sources.begin()+m_source_count,
              [](const Source& a, const Source& b){ return a.priority>b.priority; });
}

float RcManager::apply_deadband(float input, float db) const {
    if(std::abs(input)<db) return 0.0f;
    float sign = (input>0)?1.0f:-1.0f;
    return sign * (std::abs(input)-db)/(1.0f-db);
}

float RcManager::apply_expo(float input, float expo) const {
    // Cubic expo: out = input*(1-expo) + input^3*expo
    return input*(1.0f-expo) + input*input*input*expo;
}

void RcManager::update() {
    // Find best active source
    for(int s=0;s<m_source_count;s++) {
        if(!m_sources[s].drv) continue;
        // Try casting to SbusDriver
        auto* sbus = static_cast<apx::drivers::SbusDriver*>(m_sources[s].drv);
        if(sbus && sbus->is_healthy()) {
            const auto& d = sbus->get_data();
            m_failsafe = d.failsafe || d.frame_lost;
            for(int i=0;i<16;i++) {
                char rev_name[24];
                std::snprintf(rev_name, sizeof(rev_name), "RC%d_REV", i + 1);
                int32_t rev = apx::gcs::ParamManager::get_instance().get_int_or(rev_name, 1);

                float ch = d.channels[i];
                if (rev < 0) {
                    ch = -ch;
                }
                ch = apply_deadband(ch, m_deadband);
                m_channels[i] = apply_expo(ch, m_expo);
            }
            m_valid=true;
            m_last_us = apx::kernel::now_us();
            return;
        }
    }
    // No valid source
    uint64_t now=apx::kernel::now_us();
    if(now-m_last_us > 1000000) { // 1s timeout
        m_failsafe=true; m_valid=false;
    }
}

float RcManager::get_channel(int ch) const { return (ch>=0&&ch<18)?m_channels[ch]:0.0f; }
float RcManager::get_roll()     const { return get_channel(m_map.roll); }
float RcManager::get_pitch()    const { return get_channel(m_map.pitch); }
float RcManager::get_throttle() const {
    float raw = get_channel(m_map.throttle);
    float thr = (raw + 1.0f) / 2.0f; // -1..1 -> 0..1
    char rev_name[24];
    std::snprintf(rev_name, sizeof(rev_name), "RC%d_REV", m_map.throttle + 1);
    int32_t rev = apx::gcs::ParamManager::get_instance().get_int_or(rev_name, 1);
    if (rev < 0) {
        thr = 1.0f - thr;
    }
    return std::clamp(thr, 0.0f, 1.0f);
}
float RcManager::get_yaw()      const { return get_channel(m_map.yaw); }
bool  RcManager::get_aux(int n) const { return (n>=0&&n<8) ? (get_channel(m_map.aux[n])>0.5f) : false; }

apx::modes::FlightModeId RcManager::get_requested_mode() const {
    float v = get_channel(m_map.mode_ch);
    if(v < -0.5f) return apx::modes::FlightModeId::STABILIZE;
    if(v < 0.0f)  return apx::modes::FlightModeId::ALT_HOLD;
    if(v < 0.5f)  return apx::modes::FlightModeId::POS_HOLD;
    if(v < 0.8f)  return apx::modes::FlightModeId::AUTO;
    return apx::modes::FlightModeId::RTL;
}

} // namespace apx::rc
