// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once
#include <cstdint>
#include <random>
#include <cmath>

namespace apx::sim {

struct GpsSimParams {
    float pos_noise_m      = 1.5f;   // horizontal 1-sigma
    float vel_noise_ms     = 0.05f;
    float hdop_base        = 1.0f;
    float multipath_prob   = 0.01f;  // probability of large error per update
    float multipath_amp_m  = 5.0f;   // multipath error amplitude
    int   cold_start_s     = 30;     // fix acquisition delay
};
static const GpsSimParams kUbloxM9Params  = {1.5f, 0.05f, 1.0f, 0.01f, 5.0f, 30};
static const GpsSimParams kUbloxF9PFixed  = {0.02f, 0.01f, 0.8f, 0.001f, 0.1f, 5};
static const GpsSimParams kUbloxF9PFloat  = {0.5f, 0.03f, 0.9f, 0.005f, 1.0f, 10};

struct GpsOutput {
    double lat_deg, lon_deg;
    float  alt_m;
    float  vel_ned[3];
    float  hdop, vdop;
    uint8_t satellites;
    int     fix_type; // 0=none,2=2D,3=3D,5=float_rtk,6=fixed_rtk
    bool    valid;
    uint64_t ts_us;
};

class GpsSim {
public:
    explicit GpsSim(const GpsSimParams& p = kUbloxM9Params)
        : m_params(p), m_rng(std::random_device{}()), m_gauss(0.0f,1.0f), m_unif(0.0f,1.0f) {}
    GpsOutput simulate(double true_lat, double true_lon, float true_alt,
                       const float true_vel_ned[3], float dt_s, uint64_t ts_us);
private:
    GpsSimParams m_params;
    float m_elapsed_s{0};
    float m_pos_err_corr[3]{};  // correlated position error
    std::mt19937 m_rng;
    std::normal_distribution<float> m_gauss;
    std::uniform_real_distribution<float> m_unif;
    float randn() { return m_gauss(m_rng); }
};
} // namespace apx::sim
