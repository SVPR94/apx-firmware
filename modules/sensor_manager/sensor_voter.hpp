// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <type_traits>

namespace apx::sensor {

template<typename T, size_t N>
class SensorVoter {
public:
    struct Measurement { T data{}; float score{0.0f}; bool valid{false}; };

    void add_measurement(size_t index, const T& data, bool valid = true) {
        if (index >= N) return;
        m_meas[index].data  = data;
        m_meas[index].valid = valid;
    }

    void vote() {
        m_winner_idx = -1;
        float best_score = -1.0f;
        for (size_t i = 0; i < N; i++) {
            if (!m_meas[i].valid) { m_meas[i].score = 0.0f; continue; }
            float score = 0.0f;
            for (size_t j = 0; j < N; j++) {
                if (i == j || !m_meas[j].valid) continue;
                if (compute_deviation(m_meas[i].data, m_meas[j].data) <= m_max_deviation) {
                    score += 1.0f;
                }
            }
            m_meas[i].score = score;
            if (score > best_score) {
                best_score = score;
                m_winner_idx = static_cast<int>(i);
            }
        }
        if (m_winner_idx == -1) {
            for (size_t i = 0; i < N; i++) {
                if (m_meas[i].valid) { m_winner_idx = static_cast<int>(i); break; }
            }
        }
    }

    const T& get_best() const {
        static T empty{};
        if (m_winner_idx < 0) return empty;
        return m_meas[static_cast<size_t>(m_winner_idx)].data;
    }

    int  get_winner_index() const { return m_winner_idx; }
    void set_max_deviation(float dev) { m_max_deviation = dev; }
    bool has_winner() const { return m_winner_idx >= 0; }

private:
    std::array<Measurement, N> m_meas{};
    int   m_winner_idx{-1};
    float m_max_deviation{0.5f};

    float compute_deviation(const T& a, const T& b) const {
        if constexpr (std::is_floating_point_v<T>) {
            return std::abs(a - b);
        } else {
            return 0.0f;
        }
    }
};

} // namespace apx::sensor
