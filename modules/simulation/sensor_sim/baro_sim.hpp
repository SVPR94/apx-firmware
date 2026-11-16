// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once
#include <cstdint>
#include <random>
#include <cmath>

namespace apx::sim {

struct BaroSimParams {
    float altitude_noise_m  = 0.1f;
    float pressure_noise_pa = 1.0f;
    float temp_noise_c      = 0.1f;
    float alt_bias_m        = 0.0f;      // static offset
    float drift_rate_mps    = 0.001f;    // slow drift m/s
};
static const BaroSimParams kMS5611Params = {0.08f, 0.8f, 0.05f, 0.0f, 0.001f};
static const BaroSimParams kBMP388Params = {0.05f, 0.5f, 0.04f, 0.0f, 0.0005f};

struct BaroOutput { float pressure_pa; float temperature_c; float altitude_m; uint64_t ts_us; };

class BaroSim {
public:
    explicit BaroSim(const BaroSimParams& p = kMS5611Params)
        : m_params(p), m_rng(std::random_device{}()), m_gauss(0.0f,1.0f) {}
    BaroOutput simulate(float true_altitude_m, float true_temp_c, float dt_s, uint64_t ts_us);
private:
    float altitude_to_pressure(float alt_m, float temp_c) const;
    BaroSimParams m_params;
    float m_bias{0.0f};
    std::mt19937 m_rng;
    std::normal_distribution<float> m_gauss;
    float randn() { return m_gauss(m_rng); }
};
} // namespace apx::sim
