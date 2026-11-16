// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once
#include <cmath>
#include <cstdint>
#include <array>

namespace apx::filters {

// ─── 2nd-order Butterworth Low-Pass Filter ────────────────────────────────────
class ButterworthLPF {
public:
    void init(float cutoff_hz, float sample_rate_hz) {
        float omega = 2.0f * kPi * cutoff_hz / sample_rate_hz;
        float cos_omega = std::cos(omega);
        // RBJ biquad: alpha = sin(w) / (2Q); Butterworth Q = 1/sqrt(2), so
        // alpha = sin(w) / sqrt(2). (Was sin(w)/(2*sqrt(2)), i.e. Q = sqrt(2):
        // under-damped, ~+3 dB peak near cutoff instead of -3 dB.)
        float alpha = std::sin(omega) / kSqrt2;
        float a0_inv = 1.0f / (1.0f + alpha);
        m_b0 = ((1.0f - cos_omega) / 2.0f) * a0_inv;
        m_b1 = (1.0f - cos_omega) * a0_inv;
        m_b2 = m_b0;
        m_a1 = -2.0f * cos_omega * a0_inv;
        m_a2 = (1.0f - alpha) * a0_inv;
        reset();
    }
    float apply(float x) {
        float y = m_b0*x + m_b1*m_x1 + m_b2*m_x2 - m_a1*m_y1 - m_a2*m_y2;
        m_x2=m_x1; m_x1=x; m_y2=m_y1; m_y1=y;
        return y;
    }
    void reset() { m_x1=m_x2=m_y1=m_y2=0.0f; m_initialized=false; }
    void set_initial(float val) { m_x1=m_x2=m_y1=m_y2=val; m_initialized=true; }
    bool is_initialized() const { return m_initialized; }
private:
    static constexpr float kPi    = 3.14159265f;
    static constexpr float kSqrt2 = 1.41421356f;
    float m_b0{0}, m_b1{0}, m_b2{0}, m_a1{0}, m_a2{0};
    float m_x1{0}, m_x2{0}, m_y1{0}, m_y2{0};
    bool  m_initialized{false};
};

// ─── IIR Notch Filter (bilinear transform) ─────────────────────────────────────
class NotchFilter {
public:
    void init(float center_hz, float bandwidth_hz, float sample_rate_hz) {
        m_center_hz = center_hz;
        m_bw_hz     = bandwidth_hz;
        m_fs        = sample_rate_hz;
        update_coefficients();
        reset();
    }
    float apply(float x) {
        float y = m_b0*x + m_b1*m_x1 + m_b2*m_x2 - m_a1*m_y1 - m_a2*m_y2;
        m_x2=m_x1; m_x1=x; m_y2=m_y1; m_y1=y;
        return y;
    }
    // Retune without clearing the filter state (bumpless frequency tracking).
    void set_frequency(float hz) {
        m_center_hz = hz;
        update_coefficients();
    }
    void set_frequency(float hz, float bandwidth_hz) {
        m_bw_hz = bandwidth_hz;
        set_frequency(hz);
    }
    void reset() { m_x1=m_x2=m_y1=m_y2=0.0f; }

private:
    // Pole-zero notch: zeros on the unit circle at +-w, poles at radius r
    // at the same angle, numerator scaled for unity gain at DC:
    //   H(z) = g (1 - 2cos(w) z^-1 + z^-2) / (1 - 2r cos(w) z^-1 + r^2 z^-2)
    // with r = 1 - pi*BW/fs (-3 dB width ~= BW for narrow notches).
    void update_coefficients() {
        float center = m_center_hz;
        const float nyquist_guard = 0.45f * m_fs;
        if (center > nyquist_guard) center = nyquist_guard;
        if (center < 1.0f) center = 1.0f;
        const float omega = 2.0f * kPi * center / m_fs;
        float r = 1.0f - (kPi * m_bw_hz / m_fs);
        if (r < 0.0f) r = 0.0f;
        if (r > 0.9999f) r = 0.9999f;
        const float cos_w = std::cos(omega);
        const float gain = (1.0f - 2.0f * r * cos_w + r * r) / (2.0f - 2.0f * cos_w);
        m_b0 = gain;
        m_b1 = -2.0f * cos_w * gain;
        m_b2 = gain;
        m_a1 = -2.0f * r * cos_w;
        m_a2 = r * r;
    }
    static constexpr float kPi = 3.14159265f;
    float m_b0{0},m_b1{0},m_b2{0},m_a1{0},m_a2{0};
    float m_x1{0},m_x2{0},m_y1{0},m_y2{0};
    float m_center_hz{80}, m_bw_hz{20}, m_fs{1000};
};

// ─── Harmonic Notch Filter ─────────────────────────────────────────────────────
class HarmonicNotchFilter {
public:
    static constexpr int MAX_HARMONICS = 4;

    void init(float fundamental_hz, int harmonics, float bandwidth_hz, float fs_hz) {
        m_num_harmonics = harmonics < MAX_HARMONICS ? harmonics : MAX_HARMONICS;
        m_fs = fs_hz;
        m_bw_hz = bandwidth_hz;
        m_fundamental_hz = fundamental_hz;
        for (int i = 0; i < m_num_harmonics; i++) {
            m_notches[i].init(fundamental_hz * (i+1), m_bw_hz * (i+1), m_fs);
        }
    }
    float apply(float x) {
        for (int i = 0; i < m_num_harmonics; i++) x = m_notches[i].apply(x);
        return x;
    }
    // Retunes in place; filter state is kept so a moving RPM fundamental
    // doesn't produce a transient on every update.
    void update_fundamental(float hz) {
        m_fundamental_hz = hz;
        for (int i = 0; i < m_num_harmonics; i++) {
            m_notches[i].set_frequency(hz * (i+1), m_bw_hz * (i+1));
        }
    }
    void reset() { for (int i=0; i<MAX_HARMONICS; i++) m_notches[i].reset(); }

private:
    NotchFilter m_notches[MAX_HARMONICS];
    int   m_num_harmonics{1};
    float m_fundamental_hz{80};
    float m_bw_hz{20};
    float m_fs{1000};
};

// ─── 5-point Median Filter (spike rejection) ──────────────────────────────────
template<int N=5>
class MedianFilter {
public:
    float apply(float input) {
        m_buf[m_idx % N] = input;
        m_idx++;
        // Sort copy
        float sorted[N];
        int count = (m_idx < N) ? m_idx : N;
        for (int i=0; i<count; i++) sorted[i] = m_buf[i];
        for (int i=0; i<count-1; i++) for (int j=i+1; j<count; j++)
            if (sorted[j]<sorted[i]) { float t=sorted[i]; sorted[i]=sorted[j]; sorted[j]=t; }
        return sorted[count/2];
    }
    void reset() { m_idx=0; for (auto& v:m_buf) v=0; }
private:
    float m_buf[N]{};
    int   m_idx{0};
};

// ─── Alpha-Beta Tracker (aerospace position/velocity estimator) ────────────────
class AlphaBetaTracker {
public:
    void init(float alpha, float beta, float dt_s) {
        m_alpha=alpha; m_beta=beta; m_dt=dt_s;
        m_initialized=false;
    }
    void update(float measurement) {
        if (!m_initialized) { m_pos=measurement; m_vel=0; m_initialized=true; return; }
        float xk = m_pos + m_vel*m_dt;          // predicted position
        float rk = measurement - xk;            // residual
        m_pos = xk + m_alpha*rk;                // corrected position
        m_vel = m_vel + (m_beta/m_dt)*rk;       // corrected velocity
    }
    float get_position() const { return m_pos; }
    float get_velocity() const { return m_vel; }
    void reset() { m_initialized=false; m_pos=0; m_vel=0; }
private:
    float m_alpha{0.5f}, m_beta{0.05f}, m_dt{0.001f};
    float m_pos{0}, m_vel{0};
    bool  m_initialized{false};
};

// ─── IMU Pre-filter Pipeline ──────────────────────────────────────────────────
struct IMUPrefilterParams {
    float gyro_lpf_hz     = 50.0f;   // gyro low-pass cutoff
    float accel_lpf_hz    = 20.0f;   // accel low-pass cutoff
    float notch_freq_hz   = 80.0f;   // motor vibration fundamental
    int   notch_harmonics = 2;        // number of harmonics
    float notch_bw_hz     = 20.0f;   // notch bandwidth
    bool  enable_notch    = true;
    bool  enable_median   = false;    // 5-pt median for spike rejection
};

class IMUPrefilter {
public:
    void init(const IMUPrefilterParams& p, float sample_rate_hz) {
        m_params = p;
        for (int i=0; i<3; i++) {
            m_gyro_lpf[i].init(p.gyro_lpf_hz, sample_rate_hz);
            m_accel_lpf[i].init(p.accel_lpf_hz, sample_rate_hz);
            if (p.enable_notch) {
                m_notch[i].init(p.notch_freq_hz, p.notch_harmonics,
                                p.notch_bw_hz, sample_rate_hz);
            }
        }
    }
    void update_notch_frequency(float rpm) {
        // Motor frequency = RPM / 60 Hz
        float freq_hz = rpm / 60.0f;
        if (freq_hz > 10.0f && freq_hz < 500.0f) {
            for (int i=0; i<3; i++) m_notch[i].update_fundamental(freq_hz);
        }
    }
    void filter(const float gyro_in[3], const float accel_in[3],
                float gyro_out[3], float accel_out[3]) {
        for (int i=0; i<3; i++) {
            float g = gyro_in[i];
            if (m_params.enable_notch) g = m_notch[i].apply(g);
            gyro_out[i]  = m_gyro_lpf[i].apply(g);
            accel_out[i] = m_accel_lpf[i].apply(accel_in[i]);
        }
    }
private:
    IMUPrefilterParams  m_params;
    ButterworthLPF      m_gyro_lpf[3];
    ButterworthLPF      m_accel_lpf[3];
    HarmonicNotchFilter m_notch[3];
};

} // namespace apx::filters
