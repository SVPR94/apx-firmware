// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "gps_sim.hpp"
namespace apx::sim {
static constexpr double kRe = 6378137.0, kPi = 3.14159265358979;

GpsOutput GpsSim::simulate(double true_lat, double true_lon, float true_alt,
                             const float true_vel_ned[3], float dt_s, uint64_t ts_us) {
    m_elapsed_s += dt_s;
    GpsOutput out{};
    out.ts_us = ts_us;

    if(m_elapsed_s < m_params.cold_start_s) {
        out.fix_type=0; out.valid=false; out.satellites=0;
        return out;
    }

    // Correlated position error (time constant ~10s)
    float tau = 10.0f;
    for(int i=0;i<3;i++) {
        m_pos_err_corr[i] = m_pos_err_corr[i]*(1-dt_s/tau)
                           + m_params.pos_noise_m*std::sqrt(2*dt_s/tau)*randn();
    }

    // Multipath spike
    float mp = 0.0f;
    if(m_unif(m_rng) < m_params.multipath_prob) mp = m_params.multipath_amp_m*randn();

    // Convert NE errors to lat/lon offsets
    double cos_lat = std::cos(true_lat * kPi/180.0);
    out.lat_deg = true_lat + (m_pos_err_corr[0]+mp) / kRe * (180.0/kPi);
    out.lon_deg = true_lon + (m_pos_err_corr[1]+mp) / (kRe*cos_lat) * (180.0/kPi);
    out.alt_m   = true_alt + (m_pos_err_corr[2]+mp)*1.5f; // vert noise is 1.5x horizontal

    for(int i=0;i<3;i++) out.vel_ned[i] = true_vel_ned[i] + m_params.vel_noise_ms*randn();

    out.hdop = m_params.hdop_base + 0.1f*randn();
    out.vdop = out.hdop * 1.5f;
    out.satellites = 12 + (int)(3*randn());
    if(out.satellites < 4) out.satellites=4;
    out.fix_type = (m_elapsed_s < m_params.cold_start_s+5) ? 2 : 3;
    out.valid = true;
    return out;
}
} // namespace apx::sim
