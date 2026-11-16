// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
//   h += v dt + 1/2 (a - b) dt^2,  v += (a - b) dt,  b random walk.
// Correction: barometric height (averaged over the healthy baro lanes, each
// relative to its own reference taken at startup).
// Barometer drift therefore shows up in the estimate exactly as it does
// on a real vehicle; GNSS height is not fused (its error is larger and
// it lags).
#pragma once

#include <cstdint>

namespace apx::estimator {

struct AltitudeEstimatorConfig {
    float accel_noise{1.0f};       // m/s^2, accel white noise incl. vibration (process)
    float bias_walk{0.02f};        // m/s^2/sqrt(s), accel bias random walk (process)
    float baro_noise{0.3f};        // m, baro height measurement noise
    float init_seconds{1.0f};      // baro averaging time for the zero reference
    float innov_gate{5.0f};        // reject baro innovations beyond this many sigma
};

class AltitudeEstimator {
public:
    static constexpr int kMaxBaro = 4;

    explicit AltitudeEstimator(const AltitudeEstimatorConfig& cfg = {}) : m_cfg(cfg) {}

    void reset();

    // Specific force in body FRD (m/s^2) and attitude quaternion body->NED
    // (w, x, y, z) at sample time t_us.
    void predict_imu(uint64_t t_us, const float accel_body[3], const float quat[4]);

    // Baro altitude (m, any datum) of one lane. Lanes are referenced to
    // their own startup average, then fused one by one.
    void update_baro(int lane, float baro_alt_m);

    // Makes the current position height 0 (call on arming on the ground,
    // like a baro ground calibration).
    void rezero();

    bool initialised() const { return m_init; }
    float height() const { return m_x[0]; }
    float climb_rate() const { return m_x[1]; }
    float accel_bias() const { return m_x[2]; }
    float height_variance() const { return m_p[0][0]; }
    uint32_t baro_rejects() const { return m_rejects; }

private:
    AltitudeEstimatorConfig m_cfg;
    bool m_init{false};
    float m_x[3]{0.0f, 0.0f, 0.0f};
    float m_p[3][3]{};
    uint64_t m_last_imu_us{0};
    float m_offset{0.0f};      // added to every referenced baro height

    // Startup reference per baro lane
    double m_ref_sum[kMaxBaro]{};
    uint32_t m_ref_n[kMaxBaro]{};
    float m_ref[kMaxBaro]{};
    bool m_ref_ok[kMaxBaro]{};
    uint64_t m_first_imu_us{0};
    uint32_t m_rejects{0};
    uint32_t m_rejects_in_row{0};
};

} // namespace apx::estimator
