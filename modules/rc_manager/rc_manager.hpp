// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once
#include "apx/drivers/driver_base.hpp"
#include <cstdint>
#include <array>

namespace apx::modes { enum class FlightModeId; } // forward

namespace apx::rc {

class RcManager {
public:
    static RcManager& get_instance() { static RcManager i; return i; }

    void register_source(apx::drivers::DriverBase* src, int priority);
    void update();

    float get_channel(int ch) const;
    float get_roll()     const;
    float get_pitch()    const;
    float get_throttle() const;
    float get_yaw()      const;
    bool  get_aux(int n) const;

    bool     is_valid()      const { return m_valid; }
    bool     is_failsafe()   const { return m_failsafe; }
    uint64_t last_update_us() const { return m_last_us; }

    struct ChannelMap {
        int roll{0}, pitch{1}, throttle{2}, yaw{3};
        int mode_ch{4};
        int aux[8]{5,6,7,8,9,10,11,12};
    };
    void set_channel_map(const ChannelMap& m) { m_map=m; }
    void set_deadband(float db) { m_deadband=db; }
    void set_expo(float e)      { m_expo=e; }

    float apply_deadband(float input, float db) const;
    float apply_expo(float input, float expo) const;

    apx::modes::FlightModeId get_requested_mode() const;

private:
    RcManager() = default;
    ChannelMap m_map;
    std::array<float,18> m_channels{};
    float    m_deadband{0.03f};
    float    m_expo{0.0f};
    bool     m_valid{false};
    bool     m_failsafe{true};
    uint64_t m_last_us{0};
    static constexpr int MAX_SOURCES = 4;
    struct Source { apx::drivers::DriverBase* drv; int priority; };
    std::array<Source, MAX_SOURCES> m_sources{};
    int m_source_count{0};
};
} // namespace apx::rc
