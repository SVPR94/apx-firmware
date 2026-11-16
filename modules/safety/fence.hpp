// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
// - Configurable breach actions (Report Only, RTL, Land, Brake)
// - Pre-arm check validation
// - MAVLink FENCE_STATUS telemetry reporting

#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include "apx/kernel/mutex.hpp"

namespace apx::safety {

enum class FenceType : uint8_t {
    NONE = 0,
    ALT_MAX = 1,
    CIRCLE = 2,
    POLYGON = 4
};

enum class FenceBreachType : uint8_t {
    NONE = 0,
    ALTITUDE = 1,
    POLYGON = 4
};

enum class FenceAction : uint8_t {
    REPORT_ONLY = 0,
    RTL = 1,
    LAND = 2,
    BRAKE = 4
};

struct FencePoint {
    float n{0.0f}; // North offset from origin/home (meters)
    float e{0.0f}; // East offset from origin/home (meters)
};

class FenceManager {
public:
    static FenceManager& instance();

    FenceManager();
    ~FenceManager() = default;

    // Parameters / configuration
    bool is_enabled() const;
    uint8_t get_fence_type() const;
    FenceAction get_action() const;
    float get_alt_max() const;
    float get_margin() const;

    // Polygon boundary definition (inclusion area)
    void set_polygon_points(const std::vector<FencePoint>& points);
    void clear_polygon();
    std::vector<FencePoint> get_polygon_points() const;
    size_t get_polygon_point_count() const;

    // Real-time check called every tick from flight loop:
    // Returns current breach type (NONE, ALTITUDE, or POLYGON).
    FenceBreachType check(float rel_alt_m, float pos_n, float pos_e, float dt_s = 0.0025f);

    // Pre-arm check: returns true if safe to arm, false if vehicle is outside fence
    // or above max altitude.
    bool pre_arm_check(float rel_alt_m, float pos_n, float pos_e, std::string& failure_msg) const;

    // Breach status & telemetry
    bool is_breached() const { return m_breached; }
    FenceBreachType current_breach() const { return m_current_breach; }
    uint16_t breach_count() const { return m_breach_count; }
    uint32_t last_breach_time_ms() const { return m_last_breach_time_ms; }

    // Reset breach flags and count
    void reset();

    // 2D Ray-Casting Point-in-Polygon test (even-odd rule)
    static bool point_in_polygon(float px, float py, const std::vector<FencePoint>& poly);

private:
    mutable apx::kernel::Mutex m_mutex;
    std::vector<FencePoint> m_polygon;
    bool m_breached{false};
    FenceBreachType m_current_breach{FenceBreachType::NONE};
    uint16_t m_breach_count{0};
    uint32_t m_last_breach_time_ms{0};
    float m_breach_debounce_s{0.0f};
    float m_clear_debounce_s{0.0f};
    bool m_new_breach_event{false};
};

} // namespace apx::safety

