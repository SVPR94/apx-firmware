// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "servo_driver.hpp"
#include <algorithm>
namespace apx::drivers {
ServoDriver::ServoDriver(apx::hal::IHalPwm& pwm, uint8_t first_ch, uint8_t count)
    : DriverBase("servo"), m_pwm(pwm), m_first_ch(first_ch), m_count(count) {}
bool ServoDriver::init() {
    m_pwm.init(m_first_ch, 50); // 50Hz for servo
    set_status(DriverStatus::RUNNING);
    disarm_all();
    return true;
}
const ServoDriver::ChannelConfig& ServoDriver::get_channel_config(uint8_t ch) const {
    return m_cfgs[ch < MAX_CHANNELS ? ch : 0];
}
void ServoDriver::set_channel_config(uint8_t ch, const ChannelConfig& cfg) {
    if(ch < MAX_CHANNELS) m_cfgs[ch] = cfg;
}
void ServoDriver::set_pwm_us(uint8_t ch, uint16_t us) {
    if(ch >= MAX_CHANNELS) return;
    auto& cfg = m_cfgs[ch];
    us = std::max(cfg.min_us, std::min(cfg.max_us, us));
    m_current[ch] = us;
    m_pwm.set_duty(m_first_ch + ch, us);
    mark_updated();
}
void ServoDriver::set_normalized(uint8_t ch, float val) {
    val = std::max(-1.0f, std::min(1.0f, val));
    auto& cfg = m_cfgs[ch];
    if(cfg.reversed) val = -val;
    val += cfg.trim;
    uint16_t us;
    if(val >= 0) us = cfg.neutral_us + uint16_t(val * (cfg.max_us - cfg.neutral_us));
    else         us = cfg.neutral_us - uint16_t(-val * (cfg.neutral_us - cfg.min_us));
    set_pwm_us(ch, us);
}
void ServoDriver::set_throttle(uint8_t ch, float t) {
    t = std::max(0.0f, std::min(1.0f, t));
    auto& cfg = m_cfgs[ch];
    set_pwm_us(ch, uint16_t(cfg.min_us + t*(cfg.max_us - cfg.min_us)));
}
void ServoDriver::disarm_all() {
    for(uint8_t i=0;i<m_count;i++) set_pwm_us(i, m_cfgs[i].disarm_us);
}
void ServoDriver::arm() {
    for(uint8_t i=0;i<m_count;i++) set_pwm_us(i, m_cfgs[i].neutral_us);
}
} // namespace apx::drivers
