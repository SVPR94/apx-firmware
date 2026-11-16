// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "baro_sim.hpp"

namespace apx::sim {

float BaroSim::altitude_to_pressure(float alt_m, float temp_c) const {
    // Barometric formula: P = P0 * (1 - L*h/T0)^(g*M/R*L)
    constexpr float P0 = 101325.0f, T0 = 288.15f, L = 0.0065f;
    constexpr float expo = 5.25588f;
    float T = T0 - L*alt_m;
    if(T < 1.0f) T = 1.0f;
    return P0 * std::pow(T/T0, expo);
}

BaroOutput BaroSim::simulate(float true_alt_m, float true_temp_c, float dt_s, uint64_t ts_us) {
    // Slow bias drift
    m_bias += m_params.drift_rate_mps * dt_s * randn();

    float alt_noisy = true_alt_m + m_params.alt_bias_m + m_bias
                    + m_params.altitude_noise_m * randn();
    float temp_noisy = true_temp_c + m_params.temp_noise_c * randn();

    // Ground effect: pressure increases near ground (within 1m)
    if(true_alt_m < 1.0f) alt_noisy -= 0.3f * (1.0f - true_alt_m);

    BaroOutput out;
    out.altitude_m    = alt_noisy;
    out.temperature_c = temp_noisy;
    out.pressure_pa   = altitude_to_pressure(alt_noisy, temp_noisy)
                      + m_params.pressure_noise_pa * randn();
    out.ts_us = ts_us;
    return out;
}
} // namespace apx::sim
