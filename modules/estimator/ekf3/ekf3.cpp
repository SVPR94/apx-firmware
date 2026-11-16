// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "ekf3.hpp"
#include "apx/kernel/timer.hpp"
#include "param_manager.hpp"
#include <cmath>
#include <cstring>
#include <algorithm>
#include <cstdio>

namespace apx::estimator {

static constexpr float kG        = 9.80665f;
static constexpr float kPi       = 3.14159265358979f;
static constexpr float kDegToRad = kPi / 180.0f;
static constexpr float kRadToDeg = 180.0f / kPi;
// Earth radius for geodetic conversion
static constexpr double kRe = 6378137.0;

// ─── Quaternion helpers ───────────────────────────────────────────────────────

void EKF3::quat_multiply(const float a[4], const float b[4], float out[4]) {
    out[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    out[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    out[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    out[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}

void EKF3::quat_normalize(float q[4]) {
    float n = std::sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    if (n < 1e-6f) { q[0]=1; q[1]=q[2]=q[3]=0; return; }
    float inv = 1.0f / n;
    q[0]*=inv; q[1]*=inv; q[2]*=inv; q[3]*=inv;
}

void EKF3::quat_to_euler(const float q[4], float rpy[3]) {
    float sinr_cosp = 2.0f*(q[0]*q[1]+q[2]*q[3]);
    float cosr_cosp = 1.0f-2.0f*(q[1]*q[1]+q[2]*q[2]);
    rpy[0] = std::atan2(sinr_cosp, cosr_cosp);

    float sinp = 2.0f*(q[0]*q[2]-q[3]*q[1]);
    rpy[1] = (std::abs(sinp)>=1.0f) ? std::copysign(kPi/2.0f, sinp) : std::asin(sinp);

    float siny_cosp = 2.0f*(q[0]*q[3]+q[1]*q[2]);
    float cosy_cosp = 1.0f-2.0f*(q[2]*q[2]+q[3]*q[3]);
    rpy[2] = std::atan2(siny_cosp, cosy_cosp);
}

// Body-to-NED DCM (3x3, row-major)
void EKF3::quat_to_dcm(const float q[4], float R[9]) {
    float q0=q[0], q1=q[1], q2=q[2], q3=q[3];
    R[0]=1-2*(q2*q2+q3*q3); R[1]=2*(q1*q2-q0*q3);   R[2]=2*(q1*q3+q0*q2);
    R[3]=2*(q1*q2+q0*q3);   R[4]=1-2*(q1*q1+q3*q3); R[5]=2*(q2*q3-q0*q1);
    R[6]=2*(q1*q3-q0*q2);   R[7]=2*(q2*q3+q0*q1);   R[8]=1-2*(q1*q1+q2*q2);
}

void EKF3::quat_from_euler(float roll, float pitch, float yaw, float q[4]) {
    float cr=std::cos(roll/2), sr=std::sin(roll/2);
    float cp=std::cos(pitch/2), sp=std::sin(pitch/2);
    float cy=std::cos(yaw/2),   sy=std::sin(yaw/2);
    q[0]=cr*cp*cy+sr*sp*sy;
    q[1]=sr*cp*cy-cr*sp*sy;
    q[2]=cr*sp*cy+sr*cp*sy;
    q[3]=cr*cp*sy-sr*sp*cy;
}

// ─── Geodetic conversion ──────────────────────────────────────────────────────

void EKF3::geodetic_to_ned(double lat, double lon, float alt, float ned[3]) const {
    double dlat = (lat - m_home_lat) * kDegToRad;
    double dlon = (lon - m_home_lon) * kDegToRad;
    double cos_lat = std::cos(m_home_lat * kDegToRad);
    ned[0] = static_cast<float>(dlat * kRe);
    ned[1] = static_cast<float>(dlon * kRe * cos_lat);
    ned[2] = -(alt - m_home_alt);  // Down is positive
}

// ─── Init ─────────────────────────────────────────────────────────────────────

EKF3::EKF3() {
    std::memset(&m_state, 0, sizeof(m_state));
    std::memset(&m_nav,   0, sizeof(m_nav));
}

void EKF3::init(const EKF3InitParams& params) {
    m_params = params;
    // Initialize state: quaternion = identity, everything else zero
    std::fill(m_state.x.begin(), m_state.x.end(), 0.0f);
    m_state.x[Q0] = 1.0f;  // identity quaternion

    // Initialize covariance (diagonal)
    std::fill(m_state.P.begin(), m_state.P.end(), 0.0f);
    auto P = [&](int i, int j) -> float& { return m_state.P[i*EKF3_N+j]; };
    P(Q0,Q0)=0.1f; P(Q1,Q1)=0.1f; P(Q2,Q2)=0.1f; P(Q3,Q3)=0.1f;
    P(VN,VN)=1.0f; P(VE,VE)=1.0f; P(VD,VD)=1.0f;
    P(PN,PN)=100.0f; P(PE,PE)=100.0f; P(PD,PD)=100.0f;
    P(BGX,BGX)=0.01f; P(BGY,BGY)=0.01f; P(BGZ,BGZ)=0.01f;
    P(BAX,BAX)=0.1f;  P(BAY,BAY)=0.1f;  P(BAZ,BAZ)=0.1f;
    for (int i=WN; i<=WW; i++) P(i,i)=10.0f;
    P(BBIAS,BBIAS)=10.0f;
    P(MDECL,MDECL)=1.0f;
    for (int i=MBX; i<=MBZ; i++) P(i,i)=1.0f;

    // COMPASS_DEC (registered in param_manager.cpp) was a dead parameter -
    // nothing read it, so update_mag() always compared the measured
    // magnetometer field against a reference assuming zero declination
    // (m_earth_mag_ned's East component was hardcoded to 0.0). Magnetic
    // declination is real-world location-dependent (commonly a few
    // degrees, occasionally much more at high latitude) and constant at
    // a given location, so an unmodeled declination shows up as exactly
    // a small, constant, non-growing heading bias once this filter is
    // actually driving heading from a real magnetometer - not a growing
    // drift, not noise, just a fixed offset the same size as whatever
    // COMPASS_DEC should have been. Rotating the reference field's
    // horizontal (N,E) component by the declination angle here is what
    // MDECL's own covariance/process-noise setup already assumed would
    // happen - that state existed but was never actually coupled into
    // this measurement model.
    float compass_dec_rad = apx::gcs::ParamManager::get_instance().get_float_or("COMPASS_DEC", 0.0f);
    float mag_n0 = m_earth_mag_ned[0];
    float mag_e0 = m_earth_mag_ned[1];
    m_earth_mag_ned[0] = mag_n0 * std::cos(compass_dec_rad) - mag_e0 * std::sin(compass_dec_rad);
    m_earth_mag_ned[1] = mag_n0 * std::sin(compass_dec_rad) + mag_e0 * std::cos(compass_dec_rad);

    m_initialized = true;
    m_att_valid = false;
    m_vel_valid = false;
    m_pos_valid = false;
    printf("EKF3: initialized 24-state filter\n");
}

void EKF3::set_home(double lat_deg, double lon_deg, float alt_m) {
    m_home_lat = lat_deg;
    m_home_lon = lon_deg;
    m_home_alt = alt_m;
    m_home_set = true;
}

// ─── Predict step ─────────────────────────────────────────────────────────────

void EKF3::predict(const float gyro_rads[3], const float accel_ms2[3], float dt_s) {
    if (!m_initialized) return;

    auto& x = m_state.x;

    // Correct gyro for estimated bias
    float wx = gyro_rads[0] - x[BGX];
    float wy = gyro_rads[1] - x[BGY];
    float wz = gyro_rads[2] - x[BGZ];

    // Quaternion derivative: q_dot = 0.5 * q ⊗ [0, wx, wy, wz]
    float q0=x[Q0], q1=x[Q1], q2=x[Q2], q3=x[Q3];
    float dq0 = 0.5f*(-q1*wx - q2*wy - q3*wz);
    float dq1 = 0.5f*( q0*wx + q2*wz - q3*wy);
    float dq2 = 0.5f*( q0*wy - q1*wz + q3*wx);
    float dq3 = 0.5f*( q0*wz + q1*wy - q2*wx);
    x[Q0] += dq0*dt_s; x[Q1] += dq1*dt_s;
    x[Q2] += dq2*dt_s; x[Q3] += dq3*dt_s;

    // Normalize quaternion
    float qn[4] = {x[Q0],x[Q1],x[Q2],x[Q3]};
    quat_normalize(qn);
    x[Q0]=qn[0]; x[Q1]=qn[1]; x[Q2]=qn[2]; x[Q3]=qn[3];

    // Body-to-NED rotation
    float R[9]; quat_to_dcm(qn, R);

    // Corrected specific force
    float ax = accel_ms2[0] - x[BAX];
    float ay = accel_ms2[1] - x[BAY];
    float az = accel_ms2[2] - x[BAZ];

    // Velocity: NED acceleration = R * a_body + g_NED
    float an_N = R[0]*ax + R[1]*ay + R[2]*az;
    float ae_N = R[3]*ax + R[4]*ay + R[5]*az;
    float ad_N = R[6]*ax + R[7]*ay + R[8]*az + kG;  // +g in down direction

    x[VN] += an_N * dt_s;
    x[VE] += ae_N * dt_s;
    x[VD] += ad_N * dt_s;

    // Position
    x[PN] += x[VN] * dt_s;
    x[PE] += x[VE] * dt_s;
    x[PD] += x[VD] * dt_s;

    // Biases and wind are modeled as constants (random walk via Q)

    // ─── Covariance propagation (simplified diagonal F for efficiency) ────────
    // Full F matrix would be 24x24; use diagonal approximation + key off-diagonal terms
    // Process noise Q diagonal
    float Q[EKF3_N]{};
    float dt2 = dt_s*dt_s;
    Q[Q0]=Q[Q1]=Q[Q2]=Q[Q3] = m_params.q_gyro_noise  * m_params.q_gyro_noise  * dt2;
    Q[VN]=Q[VE]=Q[VD]        = m_params.q_accel_noise * m_params.q_accel_noise * dt2;
    Q[PN]=Q[PE]=Q[PD]        = m_params.q_accel_noise * m_params.q_accel_noise * dt2*dt2/4;
    Q[BGX]=Q[BGY]=Q[BGZ]     = m_params.q_gyro_bias_noise  * m_params.q_gyro_bias_noise  * dt_s;
    Q[BAX]=Q[BAY]=Q[BAZ]     = m_params.q_accel_bias_noise * m_params.q_accel_bias_noise * dt_s;
    Q[WN]=Q[WE]=Q[WW]        = m_params.q_wind_noise * m_params.q_wind_noise * dt_s;
    Q[BBIAS]                  = m_params.q_baro_noise * m_params.q_baro_noise * dt_s;
    Q[MDECL]                  = 0.00001f * dt_s;
    Q[MBX]=Q[MBY]=Q[MBZ]     = m_params.q_mag_noise * m_params.q_mag_noise * dt_s;

    // P += Q (diagonal only for speed)
    for (int i=0; i<EKF3_N; i++) {
        m_state.P[i*EKF3_N+i] += Q[i];
    }

    // Velocity-position cross-coupling: P[V,P] grows with dt
    for (int v=VN, p=PN; v<=VD; v++,p++) {
        m_state.P[v*EKF3_N+p] += m_state.P[v*EKF3_N+v] * dt_s;
        m_state.P[p*EKF3_N+v] = m_state.P[v*EKF3_N+p];
        m_state.P[p*EKF3_N+p] += 2*m_state.P[v*EKF3_N+p]*dt_s + m_state.P[v*EKF3_N+v]*dt2;
    }

    m_att_valid = true;
    check_covariance_health();
    state_to_nav();
}

// ─── GPS update ───────────────────────────────────────────────────────────────

void EKF3::update_gps(const GpsMeas& m) {
    if (!m_initialized || !m_home_set) return;

    float ned[3];
    geodetic_to_ned(m.lat_deg, m.lon_deg, m.alt_m, ned);

    // Position measurement update (3 states: PN, PE, PD)
    auto& x = m_state.x;
    auto& P = m_state.P;

    for (int axis = 0; axis < 3; axis++) {
        int state_idx = PN + axis;
        float innov = ned[axis] - x[state_idx];
        float R_val = m_params.r_gps_pos * m_params.r_gps_pos;
        float S = P[state_idx*EKF3_N+state_idx] + R_val;
        if (S < 1e-6f) continue;

        // Chi-squared gate
        float gate_sq = m_params.innov_gate_gps_pos * m_params.innov_gate_gps_pos;
        if ((innov*innov)/S > gate_sq) continue;  // reject outlier

        float K_col[EKF3_N];
        for (int i=0; i<EKF3_N; i++) K_col[i] = P[i*EKF3_N+state_idx] / S;

        // State update
        for (int i=0; i<EKF3_N; i++) x[i] += K_col[i] * innov;

        // Covariance update: P = (I - K*H)*P
        for (int i=0; i<EKF3_N; i++) {
            for (int j=0; j<EKF3_N; j++) {
                P[i*EKF3_N+j] -= K_col[i] * P[state_idx*EKF3_N+j];
            }
        }

        if (axis == 0) m_innov_gps_pos = innov;
    }

    // Velocity update
    float vel_innov[3] = {m.vel_ned[0]-x[VN], m.vel_ned[1]-x[VE], m.vel_ned[2]-x[VD]};
    for (int axis=0; axis<3; axis++) {
        int state_idx = VN + axis;
        float R_val = m_params.r_gps_vel * m_params.r_gps_vel;
        float S = P[state_idx*EKF3_N+state_idx] + R_val;
        if (S < 1e-6f) continue;
        float gate_sq = m_params.innov_gate_gps_vel * m_params.innov_gate_gps_vel;
        if ((vel_innov[axis]*vel_innov[axis])/S > gate_sq) continue;
        float K_col[EKF3_N];
        for (int i=0; i<EKF3_N; i++) K_col[i] = P[i*EKF3_N+state_idx] / S;
        for (int i=0; i<EKF3_N; i++) x[i] += K_col[i] * vel_innov[axis];
        for (int i=0; i<EKF3_N; i++) for (int j=0; j<EKF3_N; j++)
            P[i*EKF3_N+j] -= K_col[i] * P[state_idx*EKF3_N+j];
    }

    quat_normalize(&x[Q0]);
    m_pos_valid = true;
    m_vel_valid = true;
    ensure_symmetric();
    state_to_nav();
}

void EKF3::update_gps_rtk(const GpsMeas& m) {
    // Use tighter noise for RTK
    EKF3InitParams saved_params = m_params;
    m_params.r_gps_pos = m_params.r_rtk_pos;
    update_gps(m);
    m_params = saved_params;
}

// ─── Baro update ──────────────────────────────────────────────────────────────

void EKF3::update_baro(const BaroMeas& m) {
    if (!m_initialized) return;
    auto& x = m_state.x;
    auto& P = m_state.P;

    // Altitude measurement: altitude = -PD + baro_bias
    float innov = m.altitude_m - (-x[PD] + x[BBIAS]);
    m_innov_baro = innov;

    // H = [0...0, 0,0,-1, 0...0, 1, 0...0]  (PD=-1, BBIAS=+1)
    float R_val = m_params.r_baro_alt * m_params.r_baro_alt;
    float S = P[PD*EKF3_N+PD] + P[BBIAS*EKF3_N+BBIAS]
            - 2*P[PD*EKF3_N+BBIAS] + R_val;
    if (S < 1e-6f) return;

    float gate_sq = m_params.innov_gate_baro * m_params.innov_gate_baro;
    if ((innov*innov)/S > gate_sq) return;

    // K = P * H^T / S  where H^T = [0...,-1,...,1,...]
    float K[EKF3_N];
    for (int i=0; i<EKF3_N; i++) {
        K[i] = (-P[i*EKF3_N+PD] + P[i*EKF3_N+BBIAS]) / S;
    }
    for (int i=0; i<EKF3_N; i++) x[i] += K[i] * innov;

    // P = (I-KH)*P
    for (int i=0; i<EKF3_N; i++) for (int j=0; j<EKF3_N; j++) {
        P[i*EKF3_N+j] -= K[i]*(-P[PD*EKF3_N+j]+P[BBIAS*EKF3_N+j]);
    }

    quat_normalize(&x[Q0]);
    ensure_symmetric();
    state_to_nav();
}

// ─── Mag update ───────────────────────────────────────────────────────────────

void EKF3::update_mag(const MagMeas& m) {
    if (!m_initialized || !m_att_valid) return;
    auto& x = m_state.x;

    // Predicted mag in body frame using current quaternion
    float R[9]; quat_to_dcm(&x[Q0], R);
    // Rotate earth field to body: mag_pred_body = R^T * earth_mag_ned
    float em[3] = {m_earth_mag_ned[0], m_earth_mag_ned[1], m_earth_mag_ned[2]};
    float pred[3] = {
        R[0]*em[0]+R[3]*em[1]+R[6]*em[2],
        R[1]*em[0]+R[4]*em[1]+R[7]*em[2],
        R[2]*em[0]+R[5]*em[1]+R[8]*em[2]
    };
    // Innovation
    float innov[3] = {m.mag_body[0]-pred[0], m.mag_body[1]-pred[1], m.mag_body[2]-pred[2]};
    float innov_sq = innov[0]*innov[0]+innov[1]*innov[1]+innov[2]*innov[2];
    float gate_sq = m_params.innov_gate_mag*m_params.innov_gate_mag*m_params.r_mag*m_params.r_mag*3;
    if (innov_sq > gate_sq) return;  // outlier rejection

    // Simple scalar update per axis using diagonal approximation
    float R_val = m_params.r_mag * m_params.r_mag;
    for (int axis=0; axis<3; axis++) {
        int state_idx = MBX+axis;
        float S = m_state.P[state_idx*EKF3_N+state_idx] + R_val;
        if (S < 1e-9f) continue;
        float K[EKF3_N];
        for (int i=0; i<EKF3_N; i++) K[i] = m_state.P[i*EKF3_N+state_idx] / S;
        for (int i=0; i<EKF3_N; i++) x[i] += K[i] * innov[axis];
        for (int i=0; i<EKF3_N; i++) for (int j=0; j<EKF3_N; j++)
            m_state.P[i*EKF3_N+j] -= K[i]*m_state.P[state_idx*EKF3_N+j];
    }

    quat_normalize(&x[Q0]);
    ensure_symmetric();
    state_to_nav();
}

void EKF3::update_flow(const FlowMeas& m) {
    (void)m; // placeholder — optical flow update similar to GPS velocity
}

// ─── Health & utilities ───────────────────────────────────────────────────────

bool EKF3::is_healthy() const {
    if (!m_initialized) return false;
    for (int i=0; i<EKF3_N; i++) {
        float p = m_state.P[i*EKF3_N+i];
        if (p < 0 || std::isnan(p) || std::isinf(p)) return false;
    }
    return true;
}

void EKF3::ensure_symmetric() {
    for (int i=0; i<EKF3_N; i++) for (int j=i+1; j<EKF3_N; j++) {
        float avg = 0.5f*(m_state.P[i*EKF3_N+j]+m_state.P[j*EKF3_N+i]);
        m_state.P[i*EKF3_N+j] = m_state.P[j*EKF3_N+i] = avg;
    }
}

void EKF3::check_covariance_health() {
    for (int i=0; i<EKF3_N; i++) {
        float& p = m_state.P[i*EKF3_N+i];
        if (std::isnan(p) || std::isinf(p) || p < 0) {
            // Reset diagonal to safe value
            p = 1.0f;
            printf("EKF3: covariance reset at state %d\n", i);
        }
    }
}

void EKF3::state_to_nav() {
    auto& x = m_state.x;
    float q[4] = {x[Q0],x[Q1],x[Q2],x[Q3]};
    memcpy(m_nav.quaternion, q, sizeof(q));
    quat_to_euler(q, m_nav.euler_rpy);
    m_nav.velocity_ned[0] = x[VN];
    m_nav.velocity_ned[1] = x[VE];
    m_nav.velocity_ned[2] = x[VD];
    m_nav.position_ned[0] = x[PN];
    m_nav.position_ned[1] = x[PE];
    m_nav.position_ned[2] = x[PD];
    memcpy(m_nav.gyro_bias,   &x[BGX], 3*sizeof(float));
    memcpy(m_nav.accel_bias,  &x[BAX], 3*sizeof(float));
    memcpy(m_nav.wind_vel,    &x[WN],  3*sizeof(float));
    m_nav.baro_bias = x[BBIAS];
    m_nav.mag_decl  = x[MDECL];
    memcpy(m_nav.mag_body, &x[MBX], 3*sizeof(float));
    // Extract variances from P diagonal
    for (int i=0; i<3; i++) {
        m_nav.att_var[i] = m_state.P[(Q1+i)*EKF3_N+(Q1+i)];
        m_nav.vel_var[i] = m_state.P[(VN+i)*EKF3_N+(VN+i)];
        m_nav.pos_var[i] = m_state.P[(PN+i)*EKF3_N+(PN+i)];
    }
    m_nav.valid = m_att_valid;
    m_nav.timestamp_us = apx::kernel::now_us();
}

} // namespace apx::estimator
