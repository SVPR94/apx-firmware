// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
// Dryden's altitude/airspeed dependent length scales). Seeded, so a given
// seed always replays the same gust sequence.
#pragma once

#include "sensors/sim_noise.hpp"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <string>

namespace apx::sitl {

struct WindConfig {
    float speed_ms{0.0f};       // mean speed
    float from_deg{0.0f};       // direction the wind blows FROM, degrees true (0 = north)
    float gust_sigma_ms{0.0f};  // gust standard deviation (horizontal)
    float gust_tau_s{2.0f};     // gust correlation time
    float vertical_ratio{0.5f}; // vertical gust sigma / horizontal
};

// "SPEED,FROM_DEG[,GUST_SIGMA]" e.g. "6,270,1.5". Returns false on junk.
inline bool parse_wind_spec(const std::string& spec, WindConfig& out) {
    if (spec.empty()) {
        return false;
    }
    float v[3] = {0.0f, 0.0f, 0.0f};
    int n = 0;
    const char* p = spec.c_str();
    while (*p != '\0' && n < 3) {
        char* end = nullptr;
        v[n] = std::strtof(p, &end);
        if (end == p) {
            return false;
        }
        ++n;
        p = end;
        if (*p == ',') {
            ++p;
        } else if (*p != '\0') {
            return false;
        }
    }
    if (n < 2 || *p != '\0' || v[0] < 0.0f || v[2] < 0.0f) {
        return false;
    }
    out.speed_ms = v[0];
    out.from_deg = v[1];
    out.gust_sigma_ms = v[2];
    return true;
}

class WindModel {
public:
    void init(const WindConfig& cfg, uint32_t seed) {
        m_cfg = cfg;
        m_rng.reseed(seed, 0x700);
        m_gust[0].init(cfg.gust_sigma_ms, cfg.gust_tau_s, m_rng);
        m_gust[1].init(cfg.gust_sigma_ms, cfg.gust_tau_s, m_rng);
        m_gust[2].init(cfg.gust_sigma_ms * cfg.vertical_ratio, cfg.gust_tau_s, m_rng);
        const float from = cfg.from_deg * 3.14159265f / 180.0f;
        // Air moves away from the "from" bearing.
        m_mean[0] = -cfg.speed_ms * std::cos(from);
        m_mean[1] = -cfg.speed_ms * std::sin(from);
        m_mean[2] = 0.0f;
        for (int a = 0; a < 3; ++a) {
            m_ned[a] = m_mean[a] + m_gust[a].value();
        }
    }

    // Advances the gusts by dt and returns the air velocity (NED, m/s).
    const float* update(float dt_s) {
        for (int a = 0; a < 3; ++a) {
            m_ned[a] = m_mean[a] + m_gust[a].step(dt_s, m_rng);
        }
        return m_ned;
    }

    const float* ned() const { return m_ned; }
    const WindConfig& config() const { return m_cfg; }

private:
    WindConfig m_cfg{};
    SimRng m_rng;
    GaussMarkov m_gust[3];
    float m_mean[3]{};
    float m_ned[3]{};
};

} // namespace apx::sitl
