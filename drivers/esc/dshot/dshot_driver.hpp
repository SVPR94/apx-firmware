// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once
#include "apx/drivers/driver_base.hpp"
#include <cstdint>
#include <array>

namespace apx::drivers {

enum class DshotSpeed { DSHOT150, DSHOT300, DSHOT600, DSHOT1200 };

enum class DshotCmd : uint16_t {
    MOTOR_STOP=0, BEEP1=1, BEEP2=2, BEEP3=3, BEEP4=4, BEEP5=5,
    ESC_INFO=6, SPIN_DIR_1=7, SPIN_DIR_2=8,
    SAVE_SETTINGS=12, SPIN_DIR_NORMAL=20, SPIN_DIR_REVERSED=21
};

struct DshotTelemetry { uint32_t rpm; uint8_t temperature_c; float voltage_v; float current_a; };

class DshotDriver : public DriverBase {
public:
    static constexpr uint8_t MAX_CHANNELS = 8;

    explicit DshotDriver(DshotSpeed speed = DshotSpeed::DSHOT600);

    bool init() override;
    void update() override;

    void set_throttle(uint8_t channel, uint16_t value_0_2047);
    void send_command(uint8_t channel, DshotCmd cmd, bool telemetry_req = false);
    void arm_all();
    bool is_armed() const { return m_armed; }
    const DshotTelemetry& get_telemetry(uint8_t ch) const { return m_telemetry[ch]; }

private:
    uint16_t encode_packet(uint16_t value, bool telemetry) const;
    void transmit_frame(uint8_t channel, uint16_t packet);

    std::array<uint16_t, MAX_CHANNELS>     m_throttle{};
    std::array<DshotTelemetry, MAX_CHANNELS> m_telemetry{};
    DshotSpeed m_speed;
    bool       m_armed{false};
    uint32_t   m_bit_time_ns{0};
};

} // namespace apx::drivers
