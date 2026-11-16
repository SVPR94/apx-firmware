// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "fence.hpp"
#include "param_manager.hpp"
#include <cmath>
#include <cstdio>
#include <chrono>

namespace apx::safety {

FenceManager& FenceManager::instance() {
    static FenceManager s_instance;
    return s_instance;
}

FenceManager::FenceManager() {
    reset();
}

void FenceManager::reset() {
    apx::kernel::LockGuard lock(m_mutex);
    m_breached = false;
    m_current_breach = FenceBreachType::NONE;
    m_breach_count = 0;
    m_last_breach_time_ms = 0;
    m_breach_debounce_s = 0.0f;
    m_clear_debounce_s = 0.0f;
}

bool FenceManager::is_enabled() const {
    auto& pm = apx::gcs::ParamManager::get_instance();
    return pm.get_int_or("FENCE_ENABLE", 0) != 0;
}

uint8_t FenceManager::get_fence_type() const {
    auto& pm = apx::gcs::ParamManager::get_instance();
    return static_cast<uint8_t>(pm.get_int_or("FENCE_TYPE", 5));
}

FenceAction FenceManager::get_action() const {
    auto& pm = apx::gcs::ParamManager::get_instance();
    int32_t val = pm.get_int_or("FENCE_ACTION", 1);
    switch (val) {
        case 0: return FenceAction::REPORT_ONLY;
        case 2: return FenceAction::LAND;
        case 4: return FenceAction::BRAKE;
        case 1:
        default: return FenceAction::RTL;
    }
}

float FenceManager::get_alt_max() const {
    auto& pm = apx::gcs::ParamManager::get_instance();
    return pm.get_float_or("FENCE_ALT_MAX", 100.0f);
}

float FenceManager::get_margin() const {
    auto& pm = apx::gcs::ParamManager::get_instance();
    return pm.get_float_or("FENCE_MARGIN", 2.0f);
}

void FenceManager::set_polygon_points(const std::vector<FencePoint>& points) {
    apx::kernel::LockGuard lock(m_mutex);
    m_polygon = points;
    std::printf("[APX Fence] Set %zu polygon points\n", m_polygon.size());
}

void FenceManager::clear_polygon() {
    apx::kernel::LockGuard lock(m_mutex);
    m_polygon.clear();
    std::printf("[APX Fence] Polygon cleared\n");
}

std::vector<FencePoint> FenceManager::get_polygon_points() const {
    apx::kernel::LockGuard lock(m_mutex);
    return m_polygon;
}

size_t FenceManager::get_polygon_point_count() const {
    apx::kernel::LockGuard lock(m_mutex);
    return m_polygon.size();
}

bool FenceManager::point_in_polygon(float px, float py, const std::vector<FencePoint>& poly) {
    const size_t n = poly.size();
    if (n < 3) {
        return true; // Not a valid closed polygon; do not falsely breach
    }
    bool inside = false;
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        const float xi = poly[i].n, yi = poly[i].e;
        const float xj = poly[j].n, yj = poly[j].e;
        const bool intersect = ((yi > py) != (yj > py)) &&
                               (px < (xj - xi) * (py - yi) / (yj - yi) + xi);
        if (intersect) {
            inside = !inside;
        }
    }
    return inside;
}

FenceBreachType FenceManager::check(float rel_alt_m, float pos_n, float pos_e, float dt_s) {
    if (!is_enabled()) {
        m_breached = false;
        m_current_breach = FenceBreachType::NONE;
        m_breach_debounce_s = 0.0f;
        m_clear_debounce_s = 0.0f;
        return FenceBreachType::NONE;
    }

    const uint8_t fence_type = get_fence_type();
    const float alt_max = get_alt_max();
    const float margin = get_margin();

    FenceBreachType raw_breach = FenceBreachType::NONE;

    // 1. Altitude fence check:
    // If FENCE_ALT_MAX <= 0.0f, no height restrictions apply.
    if ((fence_type & static_cast<uint8_t>(FenceType::ALT_MAX)) && alt_max > 0.0f) {
        if (rel_alt_m > (alt_max + margin)) {
            raw_breach = FenceBreachType::ALTITUDE;
        }
    }

    // 2. Polygon boundary fence check (if not already breached by altitude):
    if (raw_breach == FenceBreachType::NONE &&
        (fence_type & static_cast<uint8_t>(FenceType::POLYGON))) {
        apx::kernel::LockGuard lock(m_mutex);
        if (m_polygon.size() >= 3) {
            if (!point_in_polygon(pos_n, pos_e, m_polygon)) {
                raw_breach = FenceBreachType::POLYGON;
            }
        }
    }

    // Debounce to filter transient sensor noise
    constexpr float kBreachDebounceTimeS = 0.20f;
    constexpr float kClearDebounceTimeS = 1.0f;

    if (raw_breach != FenceBreachType::NONE) {
        m_breach_debounce_s += dt_s;
        m_clear_debounce_s = 0.0f;
        if (m_breach_debounce_s >= kBreachDebounceTimeS) {
            if (!m_breached || m_current_breach != raw_breach) {
                m_breached = true;
                m_current_breach = raw_breach;
                m_breach_count++;
                m_last_breach_time_ms = static_cast<uint32_t>(
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now().time_since_epoch()).count());

                const char* type_str = (raw_breach == FenceBreachType::ALTITUDE) ? "Altitude" : "Polygon";
                std::printf("[APX Fence] BREACH DETECTED: %s (count: %u, alt: %.1fm, pos: [%.1f, %.1f])\n",
                            type_str, m_breach_count, (double)rel_alt_m, (double)pos_n, (double)pos_e);
            }
            return m_current_breach;
        }
    } else {
        m_clear_debounce_s += dt_s;
        m_breach_debounce_s = 0.0f;
        if (m_clear_debounce_s >= kClearDebounceTimeS) {
            if (m_breached) {
                std::printf("[APX Fence] Breach cleared - vehicle returned inside fence boundaries\n");
                m_breached = false;
                m_current_breach = FenceBreachType::NONE;
            }
        }
    }

    return m_breached ? m_current_breach : FenceBreachType::NONE;
}

bool FenceManager::pre_arm_check(float rel_alt_m, float pos_n, float pos_e, std::string& failure_msg) const {
    if (!is_enabled()) {
        return true;
    }

    const uint8_t fence_type = get_fence_type();
    const float alt_max = get_alt_max();

    if ((fence_type & static_cast<uint8_t>(FenceType::ALT_MAX)) && alt_max > 0.0f) {
        if (rel_alt_m > alt_max) {
            failure_msg = "Fence: Altitude exceeds maximum limit (" + std::to_string(static_cast<int>(alt_max)) + "m)";
            return false;
        }
    }

    if (fence_type & static_cast<uint8_t>(FenceType::POLYGON)) {
        apx::kernel::LockGuard lock(m_mutex);
        if (m_polygon.size() >= 3) {
            if (!point_in_polygon(pos_n, pos_e, m_polygon)) {
                failure_msg = "Fence: Vehicle position is outside polygon enclosure";
                return false;
            }
        }
    }

    return true;
}

} // namespace apx::safety

