// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <random>

namespace apx::sitl {

// One independent random stream. Every simulated sensor instance owns one,
// keyed by (seed, stream id), so changing the number of lanes or the order
// in which sensors are sampled never changes another sensor's sequence.
class SimRng {
public:
    SimRng() { reseed(1, 0); }
    SimRng(uint32_t seed, uint32_t stream) { reseed(seed, stream); }

    void reseed(uint32_t seed, uint32_t stream) {
        std::seed_seq seq{seed, stream, 0x41505853u};
        m_engine.seed(seq);
        m_normal.reset();
    }

    float gauss(float sigma = 1.0f) { return sigma * m_normal(m_engine); }
    float uniform(float lo, float hi) { return lo + (hi - lo) * m_uniform(m_engine); }

private:
    std::mt19937 m_engine;
    std::normal_distribution<float> m_normal{0.0f, 1.0f};
    std::uniform_real_distribution<float> m_uniform{0.0f, 1.0f};
};

// First-order Gauss-Markov process with stationary standard deviation sigma
// and correlation time tau, discretised exactly for any step dt:
//   x[k+1] = a x[k] + sigma sqrt(1 - a^2) w,  a = exp(-dt / tau)
// tau <= 0 degenerates to white noise; sigma <= 0 to zero.
class GaussMarkov {
public:
    void init(float sigma, float tau_s, SimRng& rng) {
        m_sigma = sigma;
        m_tau_s = tau_s;
        m_x = sigma > 0.0f ? rng.gauss(sigma) : 0.0f;
    }

    float step(float dt_s, SimRng& rng) {
        if (m_sigma <= 0.0f) {
            m_x = 0.0f;
        } else if (m_tau_s <= 0.0f) {
            m_x = rng.gauss(m_sigma);
        } else if (dt_s > 0.0f) {
            const float a = std::exp(-dt_s / m_tau_s);
            m_x = a * m_x + m_sigma * std::sqrt(1.0f - a * a) * rng.gauss();
        }
        return m_x;
    }

    float value() const { return m_x; }

private:
    float m_sigma{0.0f};
    float m_tau_s{0.0f};
    float m_x{0.0f};
};

// Samples enter with the time they become visible to the flight code and
// leave in order once that time has passed. Release times must not go
// backwards (true for a fixed per-sensor delay). When full, the oldest
// pending sample is dropped and counted.
template <typename T, size_t N>
class DelayLine {
public:
    void push(uint64_t release_us, const T& value) {
        if (m_count == N) {
            m_head = (m_head + 1) % N;
            --m_count;
            ++m_overflows;
        }
        m_buf[(m_head + m_count) % N] = Entry{release_us, value};
        ++m_count;
    }

    // Releases every sample due at now_us; out receives the newest of them.
    bool pop_ready(uint64_t now_us, T& out) {
        bool any = false;
        while (m_count > 0 && m_buf[m_head].release_us <= now_us) {
            out = m_buf[m_head].value;
            m_head = (m_head + 1) % N;
            --m_count;
            any = true;
        }
        return any;
    }

    void clear() {
        m_head = 0;
        m_count = 0;
    }

    size_t pending() const { return m_count; }
    uint32_t overflows() const { return m_overflows; }

private:
    struct Entry {
        uint64_t release_us{0};
        T value{};
    };
    std::array<Entry, N> m_buf{};
    size_t m_head{0};
    size_t m_count{0};
    uint32_t m_overflows{0};
};

} // namespace apx::sitl
