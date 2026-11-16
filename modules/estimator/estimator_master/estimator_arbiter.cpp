// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "estimator_arbiter.hpp"
#include <cmath>
#include <cstdio>
#include <algorithm>

namespace apx::estimator {

EstimatorArbiter::EstimatorArbiter() {
    m_lane_diag[0].lane = EstimatorLaneId::LANE_EKF3;
    m_lane_diag[1].lane = EstimatorLaneId::LANE_UKF;
    m_lane_diag[2].lane = EstimatorLaneId::LANE_INVARIANT;
}

void EstimatorArbiter::init(double home_lat, double home_lon, float home_alt) {
    m_ekf3.init();
    m_ekf3.set_home(home_lat, home_lon, home_alt);

    m_ukf.init();
    m_ukf.set_home(home_lat, home_lon, home_alt);

    m_comp.reset();

    m_fused.primary_lane = EstimatorLaneId::LANE_EKF3;
    m_fused.valid = true;
    m_initialized = true;

    std::printf("[APX Estimator Master] Parallel multi-estimator active: EKF3 + UKF + Invariant Observer.\n");
}

void EstimatorArbiter::predict(const float gyro_body[3], const float accel_body[3], float dt_s, uint64_t now_us) {
    if (!m_initialized) return;

    // Check dynamic sensor crossbar routing
    m_crossbar.update_routing(now_us);

    // Propagate all three estimators in parallel
    m_ekf3.predict(gyro_body, accel_body, dt_s);
    m_ukf.predict(gyro_body, accel_body, dt_s);
    m_comp.update(gyro_body, accel_body, nullptr, dt_s);

    evaluate_lanes(now_us);
    compute_federated_blend(now_us);
}

void EstimatorArbiter::update_gps(double lat_deg, double lon_deg, float alt_m,
                                  const float vel_ned_ms[3], float hacc_m, float vacc_m, uint64_t now_us) {
    if (!m_initialized) return;

    m_crossbar.report_gps_metrics(0, hacc_m, 12, now_us);

    GpsMeas gm{};
    gm.lat_deg = lat_deg;
    gm.lon_deg = lon_deg;
    gm.alt_m   = alt_m;
    gm.vel_ned[0] = vel_ned_ms[0];
    gm.vel_ned[1] = vel_ned_ms[1];
    gm.vel_ned[2] = vel_ned_ms[2];
    gm.hacc_m  = hacc_m;
    gm.vacc_m  = vacc_m;

    m_ekf3.update_gps(gm);
    m_ukf.update_gps(lat_deg, lon_deg, alt_m, vel_ned_ms, hacc_m, vacc_m);
}

void EstimatorArbiter::update_baro(float pressure_pa, float altitude_m, uint64_t now_us) {
    if (!m_initialized) return;

    m_crossbar.report_baro_metrics(0, 1.0f, now_us);

    BaroMeas bm{};
    bm.altitude_m = altitude_m;
    m_ekf3.update_baro(bm);
    m_ukf.update_baro(altitude_m);
}

void EstimatorArbiter::update_mag(const float mag_gauss[3], uint64_t now_us) {
    if (!m_initialized) return;

    float norm = std::sqrt(mag_gauss[0]*mag_gauss[0] + mag_gauss[1]*mag_gauss[1] + mag_gauss[2]*mag_gauss[2]);
    m_crossbar.report_mag_metrics(0, std::abs(norm - 0.5f), now_us);

    MagMeas mm{};
    mm.mag_gauss[0] = mag_gauss[0];
    mm.mag_gauss[1] = mag_gauss[1];
    mm.mag_gauss[2] = mag_gauss[2];
    m_ekf3.update_mag(mm);
    m_ukf.update_mag(mag_gauss);
}

void EstimatorArbiter::evaluate_lanes(uint64_t now_us) {
    if (now_us - m_last_arbiter_eval_us < 50000) return; // 20 Hz evaluation
    m_last_arbiter_eval_us = now_us;

    const auto& nav_ekf = m_ekf3.get_nav_state();
    const auto& nav_ukf = m_ukf.get_nav_state();

    // 1. Gather diagnostics from EKF3 (Lane 0)
    m_lane_diag[0].nis = m_ekf3.get_innovation_gps_pos();
    m_lane_diag[0].trace_p = nav_ekf.pos_var[0] + nav_ekf.pos_var[1] + nav_ekf.pos_var[2];
    if (!m_ekf3.is_healthy() || std::isnan(m_lane_diag[0].trace_p)) {
        m_lane_diag[0].health = LaneHealthStatus::FAULTED;
    } else if (m_lane_diag[0].nis > 9.0f) {
        m_lane_diag[0].health = LaneHealthStatus::DEGRADED;
    } else if (m_lane_diag[0].nis > 4.0f) {
        m_lane_diag[0].health = LaneHealthStatus::ACCEPTABLE;
    } else {
        m_lane_diag[0].health = LaneHealthStatus::OPTIMAL;
    }

    // 2. Gather diagnostics from UKF (Lane 1)
    m_lane_diag[1].nis = m_ukf.get_nis();
    m_lane_diag[1].trace_p = nav_ukf.pos_var[0] + nav_ukf.pos_var[1] + nav_ukf.pos_var[2];
    if (!m_ukf.is_healthy() || std::isnan(m_lane_diag[1].trace_p)) {
        m_lane_diag[1].health = LaneHealthStatus::FAULTED;
    } else if (m_lane_diag[1].nis > 9.0f) {
        m_lane_diag[1].health = LaneHealthStatus::DEGRADED;
    } else if (m_lane_diag[1].nis > 4.0f) {
        m_lane_diag[1].health = LaneHealthStatus::ACCEPTABLE;
    } else {
        m_lane_diag[1].health = LaneHealthStatus::OPTIMAL;
    }

    // 3. Fast Invariant Observer (Lane 2 - attitude sanity backstop)
    m_lane_diag[2].health = LaneHealthStatus::OPTIMAL;
    m_lane_diag[2].nis = 0.5f;

    // 4. Decision Maker: Select Primary Lane
    EstimatorLaneId best_lane = EstimatorLaneId::LANE_EKF3;
    if (m_lane_diag[0].health == LaneHealthStatus::OPTIMAL) {
        best_lane = EstimatorLaneId::LANE_EKF3;
    } else if (m_lane_diag[1].health == LaneHealthStatus::OPTIMAL) {
        best_lane = EstimatorLaneId::LANE_UKF;
    } else if (m_lane_diag[0].health <= LaneHealthStatus::ACCEPTABLE) {
        best_lane = EstimatorLaneId::LANE_EKF3;
    } else {
        best_lane = EstimatorLaneId::LANE_UKF;
    }

    if (m_fused.primary_lane != best_lane) {
        std::printf("[APX Arbiter] Primary estimator switched: %s -> %s\n",
                    lane_name(m_fused.primary_lane).data(), lane_name(best_lane).data());
        m_fused.primary_lane = best_lane;
    }

    for (size_t i = 0; i < 3; ++i) {
        m_lane_diag[i].is_primary = (m_lane_diag[i].lane == best_lane);
    }
}

void EstimatorArbiter::compute_federated_blend(uint64_t now_us) {
    const auto& nav_ekf = m_ekf3.get_nav_state();
    const auto& nav_ukf = m_ukf.get_nav_state();
    const auto& state_cf = m_comp.get_state();

    // Compute continuous dynamic weights
    float raw_w[3]{0.0f, 0.0f, 0.0f};

    if (m_lane_diag[0].health != LaneHealthStatus::FAULTED) {
        float factor = (m_lane_diag[0].health == LaneHealthStatus::OPTIMAL) ? 1.0f : 0.4f;
        raw_w[0] = factor / std::max(0.01f, m_lane_diag[0].trace_p);
    }
    if (m_lane_diag[1].health != LaneHealthStatus::FAULTED) {
        float factor = (m_lane_diag[1].health == LaneHealthStatus::OPTIMAL) ? 1.0f : 0.4f;
        raw_w[1] = factor / std::max(0.01f, m_lane_diag[1].trace_p);
    }
    raw_w[2] = 0.05f; // Small stabilizing weight for Invariant Observer attitude

    float sum_w = raw_w[0] + raw_w[1] + raw_w[2];
    if (sum_w < 1e-6f) {
        raw_w[0] = 1.0f;
        sum_w = 1.0f;
    }

    float w0 = raw_w[0] / sum_w;
    float w1 = raw_w[1] / sum_w;
    float w2 = raw_w[2] / sum_w;

    m_fused.lane_weights[0] = w0;
    m_fused.lane_weights[1] = w1;
    m_fused.lane_weights[2] = w2;
    m_lane_diag[0].fusion_weight = w0;
    m_lane_diag[1].fusion_weight = w1;
    m_lane_diag[2].fusion_weight = w2;

    // Weight normalized position & velocity blending between EKF3 & UKF
    float sum_nav_w = w0 + w1;
    float nw0 = (sum_nav_w > 1e-4f) ? (w0 / sum_nav_w) : 1.0f;
    float nw1 = (sum_nav_w > 1e-4f) ? (w1 / sum_nav_w) : 0.0f;

    for (int k = 0; k < 3; ++k) {
        m_fused.pos_ned[k] = nw0 * nav_ekf.position_ned[k] + nw1 * nav_ukf.pos_ned[k];
        m_fused.vel_ned[k] = nw0 * nav_ekf.velocity_ned[k] + nw1 * nav_ukf.vel_ned[k];
        m_fused.gyro_bias[k] = nw0 * nav_ekf.gyro_bias[k] + nw1 * nav_ukf.gyro_bias[k];
        m_fused.accel_bias[k] = nw0 * nav_ekf.accel_bias[k] + nw1 * nav_ukf.accel_bias[k];

        m_fused.pos_var[k] = nw0 * nw0 * nav_ekf.pos_var[k] + nw1 * nw1 * nav_ukf.pos_var[k];
        m_fused.vel_var[k] = nw0 * nw0 * nav_ekf.vel_var[k] + nw1 * nw1 * nav_ukf.vel_var[k];
    }

    // Attitude Quaternion Spherical Blending (with sign alignment)
    float q_ekf[4] = {nav_ekf.quaternion[0], nav_ekf.quaternion[1], nav_ekf.quaternion[2], nav_ekf.quaternion[3]};
    float q_ukf[4] = {nav_ukf.quat[0], nav_ukf.quat[1], nav_ukf.quat[2], nav_ukf.quat[3]};
    float q_cf[4]  = {state_cf.quaternion[0], state_cf.quaternion[1], state_cf.quaternion[2], state_cf.quaternion[3]};

    // Ensure shortest path antipodal alignment
    float dot_ukf = q_ekf[0]*q_ukf[0] + q_ekf[1]*q_ukf[1] + q_ekf[2]*q_ukf[2] + q_ekf[3]*q_ukf[3];
    if (dot_ukf < 0.0f) {
        for (int k = 0; k < 4; ++k) q_ukf[k] = -q_ukf[k];
    }
    float dot_cf = q_ekf[0]*q_cf[0] + q_ekf[1]*q_cf[1] + q_ekf[2]*q_cf[2] + q_ekf[3]*q_cf[3];
    if (dot_cf < 0.0f) {
        for (int k = 0; k < 4; ++k) q_cf[k] = -q_cf[k];
    }

    for (int k = 0; k < 4; ++k) {
        m_fused.quat[k] = w0 * q_ekf[k] + w1 * q_ukf[k] + w2 * q_cf[k];
    }
    quat_normalize(m_fused.quat);
    quat_to_euler(m_fused.quat, m_fused.euler_rpy);

    m_fused.timestamp_us = now_us;
    m_fused.valid = true;
}

void EstimatorArbiter::quat_normalize(float q[4]) {
    float n = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (n > 1e-6f) {
        float inv = 1.0f / n;
        q[0] *= inv; q[1] *= inv; q[2] *= inv; q[3] *= inv;
    } else {
        q[0] = 1.0f; q[1] = 0.0f; q[2] = 0.0f; q[3] = 0.0f;
    }
}

void EstimatorArbiter::quat_to_euler(const float q[4], float rpy[3]) {
    float w = q[0], x = q[1], y = q[2], z = q[3];
    rpy[0] = std::atan2(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
    float sinp = 2.0f * (w * y - z * x);
    rpy[1] = (std::abs(sinp) >= 1.0f) ? std::copysign(1.5707963f, sinp) : std::asin(sinp);
    rpy[2] = std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
}

} // namespace apx::estimator
