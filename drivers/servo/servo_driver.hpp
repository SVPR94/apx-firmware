// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once
#include "apx/drivers/driver_base.hpp"
#include "apx/hal/hal_pwm.hpp"
#include <cstdint>
#include <array>

namespace apx::drivers {

class ServoDriver : public DriverBase {
public:
    static constexpr uint8_t MAX_CHANNELS = 16;
    struct ChannelConfig {
        uint16_t min_us{1000}, max_us{2000}, neutral_us{1500}, disarm_us{900};
        bool reversed{false};
        float trim{0.0f};
    };
    ServoDriver(apx::hal::IHalPwm& pwm, uint8_t first_ch = 0, uint8_t count = 8);
    bool init() override;
    void set_pwm_us(uint8_t ch, uint16_t us);
    void set_normalized(uint8_t ch, float val_neg1_to_1);
    void set_throttle(uint8_t ch, float throttle_0_to_1);
    void disarm_all();
    void arm();
    void set_channel_config(uint8_t ch, const ChannelConfig& cfg);
    const ChannelConfig& get_channel_config(uint8_t ch) const;
private:
    apx::hal::IHalPwm& m_pwm;
    uint8_t m_first_ch, m_count;
    std::array<ChannelConfig, MAX_CHANNELS> m_cfgs{};
    std::array<uint16_t, MAX_CHANNELS> m_current{};
};
} // namespace apx::drivers
