// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once
#include "apx/drivers/driver_base.hpp"
#include "apx/hal/hal_pwm.hpp"
#include <cstdint>

namespace apx::drivers {

class PumpDriver : public DriverBase {
public:
    PumpDriver(apx::hal::IHalPwm& pwm, uint8_t channel, uint32_t freq_hz = 50);
    bool init() override;
    void update() override;
    void set_flow_rate(float rate_0_to_1);
    void enable();
    void disable();
    bool is_active() const { return m_enabled; }
    float get_flow_rate() const { return m_flow_rate; }
    float get_volume_sprayed_ml() const { return m_volume_ml; }
    void update_volume(float dt_s, float area_m2, float speed_ms);
private:
    apx::hal::IHalPwm& m_pwm;
    uint8_t  m_channel;
    uint32_t m_freq_hz;
    float    m_flow_rate{0.0f};
    bool     m_enabled{false};
    float    m_volume_ml{0.0f};
    static constexpr uint32_t kMinPwmUs = 1000;
    static constexpr uint32_t kMaxPwmUs = 2000;
};

} // namespace apx::drivers
