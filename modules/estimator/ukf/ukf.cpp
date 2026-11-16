// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "ukf.hpp"
#include <algorithm>
#include <cstring>

namespace apx::estimator {

namespace {
constexpr float kGravityMs2 = 9.80665f;
constexpr float kRadiusEarthM = 6378137.0f;
constexpr float kDegToRad = 0.017453292519943295f;
} // namespace

UKF::UKF() {
    init();
}

void UKF::init(const UkfConfig& config) {
    m_config = config;
    reset();
    compute_weights();
    m_initialized = true;
}

void UKF::compute_weights() {
    int L = UKF_STATE_DIM;
    float lambda = m_config.alpha * m_config.alpha * (static_cast<float>(L) + m_config.kappa) - static_cast<float>(L);
    float c = static_cast<float>(L) + lambda;
    m_gamma = std::sqrt(std::max(c, 1e-4f));

    m_Wm[0] = lambda / c;
    m_Wc[0] = lambda / c + (1.0f - m_config.alpha * m_config.alpha + m_config.beta);

    float w_i = 1.0f / (2.0f * c);
    for (int i = 1; i < UKF_SIGMA_COUNT; ++i) {
        m_Wm[i] = w_i;
        m_Wc[i] = w_i;
    }
}

void UKF::reset() {
    m_q[0] = 1.0f; m_q[1] = 0.0f; m_q[2] = 0.0f; m_q[3] = 0.0f;
    std::fill(std::begin(m_v), std::end(m_v), 0.0f);
    std::fill(std::begin(m_p), std::end(m_p), 0.0f);
    std::fill(std::begin(m_bg), std::end(m_bg), 0.0f);

    m_P.fill(0.0f);
    // Initialize diagonal covariance
    for (int i = 0; i < 3; ++i) {
        m_P[i * 12 + i] = 0.01f;             // Attitude error var (rad^2)
        m_P[(i + 3) * 12 + (i + 3)] = 0.05f; // Velocity var (m/s)^2
        m_P[(i + 6) * 12 + (i + 6)] = 0.5f;  // Position var (m^2)
        m_P[(i + 9) * 12 + (i + 9)] = 1e-4f; // Gyro bias var (rad/s)^2
    }

    m_state = UkfNavState{};
    m_state.healthy = true;
}

void UKF::set_home(double lat_deg, double lon_deg, float alt_m) {
    m_home_lat = lat_deg;
    m_home_lon = lon_deg;
    m_home_alt = alt_m;
    m_home_set = true;
}

void UKF::geodetic_to_ned(double lat, double lon, float alt, float ned[3]) const {
    if (!m_home_set) {
        ned[0] = 0.0f; ned[1] = 0.0f; ned[2] = 0.0f;
        return;
    }
    double dlat = (lat - m_home_lat) * kDegToRad;
    double dlon = (lon - m_home_lon) * kDegToRad;
    double lat_avg = (lat + m_home_lat) * 0.5 * kDegToRad;

    ned[0] = static_cast<float>(dlat * kRadiusEarthM);
    ned[1] = static_cast<float>(dlon * kRadiusEarthM * std::cos(lat_avg));
    ned[2] = -(alt - m_home_alt); // NED Down is negative altitude
}

void UKF::quat_to_dcm(const float q[4], float R[9]) {
    float w = q[0], x = q[1], y = q[2], z = q[3];
    float x2 = x * x, y2 = y * y, z2 = z * z;
    float xy = x * y, xz = x * z, yz = y * z;
    float wx = w * x, wy = w * y, wz = w * z;

    R[0] = 1.0f - 2.0f * (y2 + z2);
    R[1] = 2.0f * (xy - wz);
    R[2] = 2.0f * (xz + wy);

    R[3] = 2.0f * (xy + wz);
    R[4] = 1.0f - 2.0f * (x2 + z2);
    R[5] = 2.0f * (yz - wx);

    R[6] = 2.0f * (xz - wy);
    R[7] = 2.0f * (yz + wx);
    R[8] = 1.0f - 2.0f * (x2 + y2);
}

void UKF::quat_to_euler(const float q[4], float rpy[3]) {
    float w = q[0], x = q[1], y = q[2], z = q[3];
    rpy[0] = std::atan2(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
    float sinp = 2.0f * (w * y - z * x);
    rpy[1] = (std::abs(sinp) >= 1.0f) ? std::copysign(1.5707963f, sinp) : std::asin(sinp);
    rpy[2] = std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
}

void UKF::quat_normalize(float q[4]) {
    float norm = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (norm > 1e-6f) {
        float inv = 1.0f / norm;
        q[0] *= inv; q[1] *= inv; q[2] *= inv; q[3] *= inv;
    } else {
        q[0] = 1.0f; q[1] = 0.0f; q[2] = 0.0f; q[3] = 0.0f;
    }
}

void UKF::quat_multiply(const float a[4], const float b[4], float out[4]) {
    out[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    out[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    out[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    out[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}

void UKF::rot_vec_to_quat(const float r[3], float q[4]) {
    float theta = std::sqrt(r[0]*r[0] + r[1]*r[1] + r[2]*r[2]);
    if (theta < 1e-6f) {
        q[0] = 1.0f;
        q[1] = 0.5f * r[0];
        q[2] = 0.5f * r[1];
        q[3] = 0.5f * r[2];
        quat_normalize(q);
        return;
    }
    float half = 0.5f * theta;
    float s = std::sin(half) / theta;
    q[0] = std::cos(half);
    q[1] = r[0] * s;
    q[2] = r[1] * s;
    q[3] = r[2] * s;
}

bool UKF::cholesky_12x12(const float* A, float* L) const {
    std::fill(L, L + 144, 0.0f);
    for (int i = 0; i < 12; ++i) {
        for (int j = 0; j <= i; ++j) {
            float sum = 0.0f;
            for (int k = 0; k < j; ++k) {
                sum += L[i * 12 + k] * L[j * 12 + k];
            }
            if (i == j) {
                float val = A[i * 12 + i] - sum;
                if (val <= 0.0f) {
                    val = 1e-6f; // Regularize to prevent numerical breakdown
                }
                L[i * 12 + j] = std::sqrt(val);
            } else {
                float denom = L[j * 12 + j];
                L[i * 12 + j] = (std::abs(denom) > 1e-7f) ? ((A[i * 12 + j] - sum) / denom) : 0.0f;
            }
        }
    }
    return true;
}

void UKF::enforce_covariance_symmetry() {
    for (int i = 0; i < 12; ++i) {
        for (int j = i + 1; j < 12; ++j) {
            float avg = 0.5f * (m_P[i * 12 + j] + m_P[j * 12 + i]);
            m_P[i * 12 + j] = avg;
            m_P[j * 12 + i] = avg;
        }
        if (m_P[i * 12 + i] < 1e-7f) {
            m_P[i * 12 + i] = 1e-7f;
        }
    }
}

void UKF::predict(const float gyro_rads[3], const float accel_ms2[3], float dt_s) {
    if (!m_initialized || dt_s <= 0.0f || dt_s > 0.1f) return;

    // 1. Cholesky decomposition of current error covariance: P = L * L^T
    float L[144];
    cholesky_12x12(m_P.data(), L);

    // 2. Generate 25 Sigma Points in Error Space
    float chi_rot[UKF_SIGMA_COUNT][3]{};
    float chi_vel[UKF_SIGMA_COUNT][3]{};
    float chi_pos[UKF_SIGMA_COUNT][3]{};
    float chi_bg[UKF_SIGMA_COUNT][3]{};

    for (int col = 0; col < 12; ++col) {
        int idx_pos = col + 1;
        int idx_neg = col + 13;
        for (int row = 0; row < 3; ++row) {
            float pert_rot = m_gamma * L[(row + 0) * 12 + col];
            float pert_vel = m_gamma * L[(row + 3) * 12 + col];
            float pert_pos = m_gamma * L[(row + 6) * 12 + col];
            float pert_bg  = m_gamma * L[(row + 9) * 12 + col];

            chi_rot[idx_pos][row] = pert_rot;
            chi_rot[idx_neg][row] = -pert_rot;

            chi_vel[idx_pos][row] = m_v[row] + pert_vel;
            chi_vel[idx_neg][row] = m_v[row] - pert_vel;

            chi_pos[idx_pos][row] = m_p[row] + pert_pos;
            chi_pos[idx_neg][row] = m_p[row] - pert_pos;

            chi_bg[idx_pos][row]  = m_bg[row] + pert_bg;
            chi_bg[idx_neg][row]  = m_bg[row] - pert_bg;
        }
    }
    // Sigma point 0 is the nominal state
    for (int i = 0; i < 3; ++i) {
        chi_vel[0][i] = m_v[i];
        chi_pos[0][i] = m_p[i];
        chi_bg[0][i]  = m_bg[i];
    }

    // 3. Propagate each sigma point through true non-linear kinematics
    float chi_q_prop[UKF_SIGMA_COUNT][4];
    float chi_vel_prop[UKF_SIGMA_COUNT][3];
    float chi_pos_prop[UKF_SIGMA_COUNT][3];

    for (int s = 0; s < UKF_SIGMA_COUNT; ++s) {
        float q_pert[4];
        rot_vec_to_quat(chi_rot[s], q_pert);
        float q_s[4];
        quat_multiply(m_q, q_pert, q_s);

        // Corrected gyro rate for this sigma point
        float omega[3] = {
            gyro_rads[0] - chi_bg[s][0],
            gyro_rads[1] - chi_bg[s][1],
            gyro_rads[2] - chi_bg[s][2]
        };

        // Quaternion integration
        float dq[4];
        rot_vec_to_quat(omega, dq);
        quat_multiply(q_s, dq, chi_q_prop[s]);
        quat_normalize(chi_q_prop[s]);

        // Transform body acceleration to NED
        float R[9];
        quat_to_dcm(chi_q_prop[s], R);
        float a_ned[3] = {
            R[0]*accel_ms2[0] + R[1]*accel_ms2[1] + R[2]*accel_ms2[2],
            R[3]*accel_ms2[0] + R[4]*accel_ms2[1] + R[5]*accel_ms2[2],
            R[6]*accel_ms2[0] + R[7]*accel_ms2[1] + R[8]*accel_ms2[2] + kGravityMs2
        };

        // Velocity & Position integration
        for (int k = 0; k < 3; ++k) {
            chi_vel_prop[s][k] = chi_vel[s][k] + a_ned[k] * dt_s;
            chi_pos_prop[s][k] = chi_pos[s][k] + chi_vel_prop[s][k] * dt_s;
        }
    }

    // 4. Recover Predicted Mean State
    float new_v[3]{0, 0, 0};
    float new_p[3]{0, 0, 0};
    for (int s = 0; s < UKF_SIGMA_COUNT; ++s) {
        for (int k = 0; k < 3; ++k) {
            new_v[k] += m_Wm[s] * chi_vel_prop[s][k];
            new_p[k] += m_Wm[s] * chi_pos_prop[s][k];
        }
    }
    // Nominal attitude follows primary propagated sigma point (s=0)
    std::copy(std::begin(chi_q_prop[0]), std::end(chi_q_prop[0]), std::begin(m_q));
    std::copy(std::begin(new_v), std::end(new_v), std::begin(m_v));
    std::copy(std::begin(new_p), std::end(new_p), std::begin(m_p));

    // 5. Reconstruct Error Covariance P
    m_P.fill(0.0f);
    for (int s = 0; s < UKF_SIGMA_COUNT; ++s) {
        float dx[12];
        // Attitude error vector
        dx[0] = chi_rot[s][0];
        dx[1] = chi_rot[s][1];
        dx[2] = chi_rot[s][2];
        // Velocity error
        dx[3] = chi_vel_prop[s][0] - m_v[0];
        dx[4] = chi_vel_prop[s][1] - m_v[1];
        dx[5] = chi_vel_prop[s][2] - m_v[2];
        // Position error
        dx[6] = chi_pos_prop[s][0] - m_p[0];
        dx[7] = chi_pos_prop[s][1] - m_p[1];
        dx[8] = chi_pos_prop[s][2] - m_p[2];
        // Gyro bias error
        dx[9]  = chi_bg[s][0] - m_bg[0];
        dx[10] = chi_bg[s][1] - m_bg[1];
        dx[11] = chi_bg[s][2] - m_bg[2];

        float w = m_Wc[s];
        for (int r = 0; r < 12; ++r) {
            for (int c = 0; c < 12; ++c) {
                m_P[r * 12 + c] += w * dx[r] * dx[c];
            }
        }
    }

    // Add Process Noise Q
    float q_gyro = m_config.q_gyro_noise * m_config.q_gyro_noise * dt_s;
    float q_accel = m_config.q_accel_noise * m_config.q_accel_noise * dt_s;
    float q_bg = m_config.q_gyro_bias_noise * m_config.q_gyro_bias_noise * dt_s;

    for (int k = 0; k < 3; ++k) {
        m_P[(k + 0) * 12 + (k + 0)] += q_gyro;
        m_P[(k + 3) * 12 + (k + 3)] += q_accel;
        m_P[(k + 6) * 12 + (k + 6)] += 0.5f * q_accel * dt_s * dt_s;
        m_P[(k + 9) * 12 + (k + 9)] += q_bg;
    }

    enforce_covariance_symmetry();

    // Update NavState output
    std::copy(std::begin(m_q), std::end(m_q), std::begin(m_state.quat));
    std::copy(std::begin(m_v), std::end(m_v), std::begin(m_state.vel_ned));
    std::copy(std::begin(m_p), std::end(m_p), std::begin(m_state.pos_ned));
    std::copy(std::begin(m_bg), std::end(m_bg), std::begin(m_state.gyro_bias));
    quat_to_euler(m_q, m_state.euler_rpy);

    for (int k = 0; k < 3; ++k) {
        m_state.att_var[k] = m_P[(k + 0) * 12 + (k + 0)];
        m_state.vel_var[k] = m_P[(k + 3) * 12 + (k + 3)];
        m_state.pos_var[k] = m_P[(k + 6) * 12 + (k + 6)];
    }
}

void UKF::update_gps(double lat_deg, double lon_deg, float alt_m,
                      const float vel_ned_ms[3], float hacc_m, float vacc_m) {
    if (!m_initialized || !m_home_set) return;

    float meas_pos[3];
    geodetic_to_ned(lat_deg, lon_deg, alt_m, meas_pos);

    float r_pos = std::max(hacc_m * hacc_m, m_config.r_gps_pos * m_config.r_gps_pos);
    float r_vel = m_config.r_gps_vel * m_config.r_gps_vel;

    float total_nis = 0.0f;

    // Measurement update for Position (states 6, 7, 8)
    for (int k = 0; k < 3; ++k) {
        int idx = 6 + k;
        float innov = meas_pos[k] - m_p[k];
        float S = m_P[idx * 12 + idx] + (k == 2 ? std::max(vacc_m * vacc_m, 1.0f) : r_pos);
        float nis = (innov * innov) / S;
        total_nis += nis;

        if (nis < 25.0f) { // 5-sigma outlier gate
            float K = m_P[idx * 12 + idx] / S;
            m_p[k] += K * innov;
            for (int r = 0; r < 12; ++r) {
                m_P[r * 12 + idx] *= (1.0f - K);
            }
        }
    }

    // Measurement update for Velocity (states 3, 4, 5)
    for (int k = 0; k < 3; ++k) {
        int idx = 3 + k;
        float innov = vel_ned_ms[k] - m_v[k];
        float S = m_P[idx * 12 + idx] + r_vel;
        float nis = (innov * innov) / S;
        total_nis += nis;

        if (nis < 25.0f) {
            float K = m_P[idx * 12 + idx] / S;
            m_v[k] += K * innov;
            for (int r = 0; r < 12; ++r) {
                m_P[r * 12 + idx] *= (1.0f - K);
            }
        }
    }

    enforce_covariance_symmetry();
    m_state.nis_gps_pos = total_nis / 6.0f;
    m_state.overall_nis = m_state.nis_gps_pos;
    m_state.healthy = (m_state.overall_nis < 9.0f);
}

void UKF::update_baro(float altitude_m) {
    if (!m_initialized || !m_home_set) return;

    float meas_z = -(altitude_m - m_home_alt);
    int idx = 8; // pos_z (NED Down)
    float innov = meas_z - m_p[2];
    float r_baro = m_config.r_baro_alt * m_config.r_baro_alt;
    float S = m_P[idx * 12 + idx] + r_baro;
    float nis = (innov * innov) / S;
    m_state.nis_baro = nis;

    if (nis < 16.0f) { // 4-sigma gate
        float K = m_P[idx * 12 + idx] / S;
        m_p[2] += K * innov;
        for (int r = 0; r < 12; ++r) {
            m_P[r * 12 + idx] *= (1.0f - K);
        }
    }
    enforce_covariance_symmetry();
}

void UKF::update_mag(const float mag_gauss[3]) {
    if (!m_initialized) return;

    // Fast yaw update using predicted DCM
    float R[9];
    quat_to_dcm(m_q, R);

    // Compute magnetic heading from body readings
    float mag_h_x = mag_gauss[0] * R[0] + mag_gauss[1] * R[1] + mag_gauss[2] * R[2];
    float mag_h_y = mag_gauss[0] * R[3] + mag_gauss[1] * R[4] + mag_gauss[2] * R[5];
    float meas_yaw = std::atan2(-mag_h_y, mag_h_x);

    float pred_yaw = m_state.euler_rpy[2];
    float innov = meas_yaw - pred_yaw;
    while (innov > 3.14159265f)  innov -= 6.2831853f;
    while (innov < -3.14159265f) innov += 6.2831853f;

    int idx = 2; // yaw rotation error (state 2)
    float r_mag = m_config.r_mag * m_config.r_mag;
    float S = m_P[idx * 12 + idx] + r_mag;
    float nis = (innov * innov) / S;
    m_state.nis_mag = nis;

    if (nis < 12.0f) {
        float K = m_P[idx * 12 + idx] / S;
        float d_yaw = K * innov;
        float rot[3] = {0.0f, 0.0f, d_yaw};
        float dq[4];
        rot_vec_to_quat(rot, dq);
        float q_new[4];
        quat_multiply(m_q, dq, q_new);
        std::copy(std::begin(q_new), std::end(q_new), std::begin(m_q));
        quat_normalize(m_q);
        m_P[idx * 12 + idx] *= (1.0f - K);
    }
    enforce_covariance_symmetry();
}

} // namespace apx::estimator
