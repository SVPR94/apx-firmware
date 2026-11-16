// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "altitude_estimator.hpp"

#include <cmath>

namespace apx::estimator {

namespace {
constexpr float kGravity = 9.80665f;
constexpr float kMaxDt = 0.05f;
// After this many baro samples in a row are gated out, accept them anyway:
// a persistent disagreement means the estimate, not the baro, is wrong.
constexpr uint32_t kMaxConsecutiveRejects = 50;
} // namespace

void AltitudeEstimator::reset() {
    const AltitudeEstimatorConfig cfg = m_cfg;
    *this = AltitudeEstimator(cfg);
}

void AltitudeEstimator::predict_imu(uint64_t t_us, const float f[3], const float q[4]) {
    if (m_first_imu_us == 0) {
        m_first_imu_us = t_us;
    }
    if (m_last_imu_us == 0 || t_us <= m_last_imu_us) {
        m_last_imu_us = t_us;
        return;
    }
    const float dt = std::fmin(static_cast<float>(t_us - m_last_imu_us) * 1.0e-6f, kMaxDt);
    m_last_imu_us = t_us;

    if (!m_init) {
        // Reference phase: average each baro lane, then start at h = 0.
        if (static_cast<float>(t_us - m_first_imu_us) * 1.0e-6f < m_cfg.init_seconds) {
            return;
        }
        bool any = false;
        for (int i = 0; i < kMaxBaro; ++i) {
            if (m_ref_n[i] > 0) {
                m_ref[i] = static_cast<float>(m_ref_sum[i] / m_ref_n[i]);
                m_ref_ok[i] = true;
                any = true;
            }
        }
        if (!any) {
            return;
        }
        m_x[0] = m_x[1] = m_x[2] = 0.0f;
        for (auto& row : m_p) {
            row[0] = row[1] = row[2] = 0.0f;
        }
        m_p[0][0] = 0.1f;
        m_p[1][1] = 0.1f;
        m_p[2][2] = 0.05f;
        m_init = true;
        return;
    }

    // Vertical acceleration (up) = -(R f)_down - g, R: body -> NED.
    const float w = q[0], x = q[1], y = q[2], z = q[3];
    const float r20 = 2.0f * (x * z - w * y);
    const float r21 = 2.0f * (y * z + w * x);
    const float r22 = 1.0f - 2.0f * (x * x + y * y);
    const float a_up = -(r20 * f[0] + r21 * f[1] + r22 * f[2]) - kGravity;

    const float a = a_up - m_x[2];
    m_x[0] += m_x[1] * dt + 0.5f * a * dt * dt;
    m_x[1] += a * dt;

    // P = F P F^T + Q, F = [[1, dt, -dt^2/2], [0, 1, -dt], [0, 0, 1]]
    const float h2 = 0.5f * dt * dt;
    const float F[3][3] = {{1.0f, dt, -h2}, {0.0f, 1.0f, -dt}, {0.0f, 0.0f, 1.0f}};
    float fp[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            fp[i][j] = F[i][0] * m_p[0][j] + F[i][1] * m_p[1][j] + F[i][2] * m_p[2][j];
        }
    }
    const float g[3] = {h2, dt, 0.0f};
    const float qa = m_cfg.accel_noise * m_cfg.accel_noise;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            m_p[i][j] = fp[i][0] * F[j][0] + fp[i][1] * F[j][1] + fp[i][2] * F[j][2] + qa * g[i] * g[j];
        }
    }
    m_p[2][2] += m_cfg.bias_walk * m_cfg.bias_walk * dt;
}

void AltitudeEstimator::update_baro(int lane, float baro_alt_m) {
    if (lane < 0 || lane >= kMaxBaro || !std::isfinite(baro_alt_m)) {
        return;
    }
    if (!m_init) {
        if (m_first_imu_us != 0) {
            m_ref_sum[lane] += static_cast<double>(baro_alt_m);
            ++m_ref_n[lane];
        }
        return;
    }
    if (!m_ref_ok[lane]) {
        // Lane that came up after startup: reference it to the estimate.
        m_ref[lane] = baro_alt_m - (m_x[0] - m_offset);
        m_ref_ok[lane] = true;
        return;
    }

    const float z = baro_alt_m - m_ref[lane] + m_offset;
    const float innov = z - m_x[0];
    const float s = m_p[0][0] + m_cfg.baro_noise * m_cfg.baro_noise;
    if (m_cfg.innov_gate > 0.0f && innov * innov > m_cfg.innov_gate * m_cfg.innov_gate * s &&
        m_rejects_in_row < kMaxConsecutiveRejects) {
        ++m_rejects;
        ++m_rejects_in_row;
        return;
    }
    m_rejects_in_row = 0;

    const float k[3] = {m_p[0][0] / s, m_p[1][0] / s, m_p[2][0] / s};
    for (int i = 0; i < 3; ++i) {
        m_x[i] += k[i] * innov;
    }
    const float p0[3] = {m_p[0][0], m_p[0][1], m_p[0][2]};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            m_p[i][j] -= k[i] * p0[j];
        }
    }
    // keep symmetric
    for (int i = 0; i < 3; ++i) {
        for (int j = i + 1; j < 3; ++j) {
            const float avg = 0.5f * (m_p[i][j] + m_p[j][i]);
            m_p[i][j] = m_p[j][i] = avg;
        }
    }
}

void AltitudeEstimator::rezero() {
    if (!m_init) {
        return;
    }
    m_offset -= m_x[0];
    m_x[0] = 0.0f;
}

} // namespace apx::estimator
