// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once
#include <cstdint>
#include <cmath>
#include <random>

namespace apx::sim {

struct ImuSimParams {
    // Gyro Allan Deviation
    float gyro_noise_density;     // rad/s/sqrt(Hz) — angle random walk
    float gyro_bias_instability;  // rad/s
    float gyro_random_walk;       // rad/s/sqrt(s) — rate random walk
    float gyro_temp_sensitivity;  // rad/s/degC
    // Accel Allan Deviation
    float accel_noise_density;    // m/s^2/sqrt(Hz) — velocity random walk
    float accel_bias_instability; // m/s^2
    float accel_random_walk;      // m/s^2/sqrt(s)
    float accel_temp_sensitivity; // m/s^2/degC
    // Limits
    float accel_max_g;
    float gyro_max_dps;
    // Motor vibration
    float vibe_amplitude_ms2;
    float vibe_frequency_hz;
    // Misalignment (small angle in deg)
    float align_err_deg[3];
    float sample_rate_hz;
    float temperature_c;
};

// ICM-42688-P datasheet values
static const ImuSimParams kICM42688Params = {
    0.00523f, 0.00008f, 0.0001f, 0.00001f,   // gyro
    0.00785f, 0.00005f, 0.001f,  0.00002f,   // accel
    16.0f, 2000.0f,                           // limits
    0.1f, 80.0f,                              // vibration
    {0.1f, 0.1f, 0.1f},                       // alignment error
    1000.0f, 25.0f                            // rate, temp
};
static const ImuSimParams kBMI088Params = {
    0.014f, 0.0005f, 0.0002f, 0.00002f,
    0.018f, 0.0001f, 0.002f,  0.00003f,
    24.0f, 2000.0f, 0.15f, 80.0f,
    {0.15f, 0.15f, 0.15f}, 400.0f, 25.0f
};
static const ImuSimParams kBMI270Params = {
    0.007f, 0.0003f, 0.00015f, 0.000015f,
    0.012f, 0.00008f, 0.0015f, 0.000025f,
    16.0f, 2000.0f, 0.1f, 80.0f,
    {0.1f, 0.1f, 0.1f}, 1600.0f, 25.0f
};

class ImuSim {
public:
    explicit ImuSim(const ImuSimParams& p = kICM42688Params)
        : m_params(p), m_rng(std::random_device{}()), m_gauss(0.0f, 1.0f)
    { reset_biases(); }

    struct Output {
        float accel_ms2[3];
        float gyro_rads[3];
        float temp_c;
        uint64_t timestamp_us;
    };

    Output simulate(const float true_accel[3], const float true_gyro[3],
                    float dt_s, uint64_t ts_us);
    void reset_biases();
    void set_params(const ImuSimParams& p) { m_params = p; }
    void set_temperature(float t) { m_params.temperature_c = t; }
    void set_vibe(float amp, float freq) {
        m_params.vibe_amplitude_ms2 = amp;
        m_params.vibe_frequency_hz  = freq;
    }

private:
    ImuSimParams m_params;
    float m_gyro_bias[3]{};
    float m_accel_bias[3]{};
    float m_vibe_phase{0.0f};
    std::mt19937 m_rng;
    std::normal_distribution<float> m_gauss;

    float randn() { return m_gauss(m_rng); }
};

} // namespace apx::sim
