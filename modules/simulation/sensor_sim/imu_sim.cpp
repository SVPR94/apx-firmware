// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "imu_sim.hpp"
#include <cstring>
#include <cmath>
#include <algorithm>

namespace apx::sim {

void ImuSim::reset_biases() {
    for(int i=0;i<3;i++) m_gyro_bias[i]=m_accel_bias[i]=0.0f;
    m_vibe_phase=0.0f;
}

ImuSim::Output ImuSim::simulate(const float true_accel[3], const float true_gyro[3],
                                 float dt_s, uint64_t ts_us) {
    Output out;
    out.timestamp_us = ts_us;
    out.temp_c = m_params.temperature_c;

    float sqrt_rate = std::sqrt(m_params.sample_rate_hz);
    float sqrt_dt   = std::sqrt(dt_s);

    // Gyro processing
    for(int i=0;i<3;i++) {
        // White noise: sigma = ARW * sqrt(ODR)
        float noise = m_params.gyro_noise_density * sqrt_rate * randn();
        // Bias instability (slow walk)
        m_gyro_bias[i] += m_params.gyro_random_walk * sqrt_dt * randn();
        // Temperature sensitivity
        float temp_drift = m_params.gyro_temp_sensitivity * (m_params.temperature_c - 25.0f);
        out.gyro_rads[i] = true_gyro[i] + m_gyro_bias[i] + noise + temp_drift;
        // Clip to sensor limits
        float max_rads = m_params.gyro_max_dps * 3.14159265f / 180.0f;
        out.gyro_rads[i] = std::max(-max_rads, std::min(max_rads, out.gyro_rads[i]));
    }

    // Accel processing
    m_vibe_phase += 2.0f * 3.14159265f * m_params.vibe_frequency_hz * dt_s;
    if(m_vibe_phase > 2*3.14159265f) m_vibe_phase -= 2*3.14159265f;
    float vibe = m_params.vibe_amplitude_ms2 * std::sin(m_vibe_phase);

    for(int i=0;i<3;i++) {
        float noise = m_params.accel_noise_density * sqrt_rate * randn();
        m_accel_bias[i] += m_params.accel_random_walk * sqrt_dt * randn();
        float temp_drift = m_params.accel_temp_sensitivity * (m_params.temperature_c - 25.0f);
        // Add vibration on all axes (simplification; z dominant in practice)
        out.accel_ms2[i] = true_accel[i] + m_accel_bias[i] + noise + temp_drift + vibe;
        // Clip
        float max_ms2 = m_params.accel_max_g * 9.80665f;
        out.accel_ms2[i] = std::max(-max_ms2, std::min(max_ms2, out.accel_ms2[i]));
    }

    // Apply alignment error (small angle rotation)
    // For simplicity: just add cross-coupling (off-diagonal alignment)
    constexpr float kDeg2Rad = 3.14159265f / 180.0f;
    for(int i=0;i<3;i++) {
        float err = m_params.align_err_deg[i] * kDeg2Rad;
        int j=(i+1)%3, k=(i+2)%3;
        out.accel_ms2[i] += err * out.accel_ms2[j];
        out.gyro_rads[i] += err * out.gyro_rads[k];
    }

    return out;
}

} // namespace apx::sim
