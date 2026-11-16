// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include <gtest/gtest.h>
#include "eskf15/eskf15.hpp"

#include <chrono>
#include <cmath>
#include <random>
#include <vector>

using namespace apx::estimator;

namespace {
constexpr float kG = 9.80665f;
constexpr float kDegToRad = 0.017453292519943295f;
constexpr float kRadToDeg = 57.29577951308232f;
} // namespace

// ─── 1. Quaternion Math & Normalization ─────────────────────────────────────────

TEST(ESKF15Test, QuaternionMathAndNormalization) {
    // Identity quaternion
    float q_id[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float rpy[3] = {0.0f, 0.0f, 0.0f};
    ESKF15::quat_to_euler(q_id, rpy);
    EXPECT_NEAR(rpy[0], 0.0f, 1e-6f);
    EXPECT_NEAR(rpy[1], 0.0f, 1e-6f);
    EXPECT_NEAR(rpy[2], 0.0f, 1e-6f);

    // Round-trip 30 deg roll, -15 deg pitch, 45 deg yaw
    float roll = 30.0f * kDegToRad;
    float pitch = -15.0f * kDegToRad;
    float yaw = 45.0f * kDegToRad;
    float q_rot[4];
    ESKF15::quat_from_euler(roll, pitch, yaw, q_rot);
    ESKF15::quat_normalize(q_rot);

    float rpy_out[3];
    ESKF15::quat_to_euler(q_rot, rpy_out);
    EXPECT_NEAR(rpy_out[0], roll, 1e-4f);
    EXPECT_NEAR(rpy_out[1], pitch, 1e-4f);
    EXPECT_NEAR(rpy_out[2], yaw, 1e-4f);

    // Direction Cosine Matrix orthogonality: R * R^T = I
    float R[9];
    ESKF15::quat_to_dcm(q_rot, R);
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            float dot = 0.0f;
            for (int k = 0; k < 3; ++k) {
                dot += R[r * 3 + k] * R[c * 3 + k];
            }
            float expected = (r == c) ? 1.0f : 0.0f;
            EXPECT_NEAR(dot, expected, 1e-5f);
        }
    }
}

// ─── 2. Initial Coarse Alignment from Accel & Mag ─────────────────────────────

TEST(ESKF15Test, CoarseAlignmentFromAccelAndMag) {
    ESKF15 eskf;
    eskf.init();

    // Drone level on ground: specific force measured by IMU is [0, 0, -g]
    float accel_level[3] = {0.0f, 0.0f, -kG};
    eskf.align_tilt_from_accel(accel_level);

    float roll, pitch, yaw;
    eskf.get_euler(roll, pitch, yaw);
    EXPECT_NEAR(roll, 0.0f, 1e-4f);
    EXPECT_NEAR(pitch, 0.0f, 1e-4f);

    // Magnetometer pointing toward magnetic North (X body = 0.2 Gauss, Y body = 0.0)
    float mag_north[3] = {0.22f, 0.0f, 0.40f};
    eskf.align_yaw_from_mag(mag_north, 0.0f);
    eskf.get_euler(roll, pitch, yaw);
    EXPECT_NEAR(yaw, 0.0f, 1e-3f);

    // Magnetometer pointing 90 deg East (X body = 0, Y body = -0.22 Gauss)
    float mag_east[3] = {0.0f, -0.22f, 0.40f};
    eskf.align_yaw_from_mag(mag_east, 0.0f);
    eskf.get_euler(roll, pitch, yaw);
    EXPECT_NEAR(yaw, static_cast<float>(M_PI) / 2.0f, 1e-3f);
}

// ─── 3. Pure Rotation Integration ─────────────────────────────────────────────

TEST(ESKF15Test, PureRotationIntegration) {
    ESKF15 eskf;
    eskf.init();

    constexpr float dt = 0.0025f; // 400 Hz
    constexpr float omega_z = 0.5f; // 0.5 rad/s yaw rate
    float gyro[3] = {0.0f, 0.0f, omega_z};
    float accel[3] = {0.0f, 0.0f, -kG};

    // Integrate for 2.0 seconds -> expected delta yaw = 1.0 rad (~57.3 deg)
    constexpr int steps = 800; // 2.0s / 0.0025s
    for (int i = 0; i < steps; ++i) {
        eskf.predict(gyro, accel, dt);
    }

    float roll, pitch, yaw;
    eskf.get_euler(roll, pitch, yaw);
    EXPECT_NEAR(roll, 0.0f, 0.02f);
    EXPECT_NEAR(pitch, 0.0f, 0.02f);
    EXPECT_NEAR(yaw, 1.0f, 0.02f);
}

// ─── 4. Stationary ZUPT and Bias Convergence ──────────────────────────────────

TEST(ESKF15Test, StationaryZuptAndBiasConvergence) {
    ESKF15 eskf;
    eskf.init();
    eskf.set_home(37.6188, -122.3750, 10.0f);

    std::mt19937 rng(42);
    std::normal_distribution<float> noise_acc(0.0f, 0.02f);
    std::normal_distribution<float> noise_gyr(0.0f, 0.002f);

    // True sensor biases to be estimated
    const float true_bg[3] = { 0.015f, -0.020f,  0.010f }; // rad/s
    const float true_ba[3] = {-0.08f,   0.06f,   0.05f  }; // m/s^2

    constexpr float dt = 0.0025f; // 400 Hz
    // Run stationary for 10 seconds (4000 steps)
    for (int i = 0; i < 4000; ++i) {
        float gyro[3] = {
            true_bg[0] + noise_gyr(rng),
            true_bg[1] + noise_gyr(rng),
            true_bg[2] + noise_gyr(rng)
        };
        float accel[3] = {
            true_ba[0] + noise_acc(rng),
            true_ba[1] + noise_acc(rng),
            -kG + true_ba[2] + noise_acc(rng)
        };

        eskf.predict(gyro, accel, dt);

        // Stationary ZUPT at 50 Hz (every 8 ticks)
        if (i % 8 == 0) {
            eskf.update_zupt(0.02f);
        }
        // Baro update at 20 Hz (every 20 ticks)
        if (i % 20 == 0) {
            eskf.update_baro(10.0f);
        }
        // Magnetometer heading update at 20 Hz (every 20 ticks)
        if (i % 20 == 0) {
            float mag[3] = {0.22f, 0.0f, 0.40f};
            eskf.update_mag(mag, 0.0f);
        }
    }

    float est_bg[3];
    eskf.get_gyro_bias(est_bg);
    // Gyro bias should converge close to true values
    EXPECT_NEAR(est_bg[0], true_bg[0], 0.008f);
    EXPECT_NEAR(est_bg[1], true_bg[1], 0.008f);
    EXPECT_NEAR(est_bg[2], true_bg[2], 0.008f);

    // Velocity should remain zero within small noise margin
    float vel[3];
    eskf.get_vel_ned(vel);
    EXPECT_NEAR(vel[0], 0.0f, 0.05f);
    EXPECT_NEAR(vel[1], 0.0f, 0.05f);
    EXPECT_NEAR(vel[2], 0.0f, 0.05f);
}

// ─── 5. Multi-Sensor Baro & GPS Fusion ─────────────────────────────────────────

TEST(ESKF15Test, BaroAndGpsFusion) {
    ESKF15 eskf;
    eskf.init();
    eskf.set_home(37.6188, -122.3750, 10.0f);

    constexpr float dt = 0.0025f; // 400 Hz
    float gyro[3] = {0.0f, 0.0f, 0.0f};
    float accel[3] = {0.0f, 0.0f, -kG};

    // Simulate constant velocity flight: 5 m/s North, 2 m/s East, 0 m/s Down
    constexpr float true_vn = 5.0f;
    constexpr float true_ve = 2.0f;
    float current_pn = 0.0f;
    float current_pe = 0.0f;

    for (int i = 0; i < 2000; ++i) { // 5.0 seconds
        eskf.predict(gyro, accel, dt);

        current_pn += true_vn * dt;
        current_pe += true_ve * dt;

        // GPS updates at 10 Hz (every 40 ticks)
        if (i % 40 == 0) {
            ESKFGpsSample gps;
            gps.valid = true;
            gps.fix_type = 3;
            gps.satellites = 14;
            // Convert local NED displacement back to lat/lon for GPS sample
            gps.lat_deg = 37.6188 + static_cast<double>(current_pn) / 111319.5;
            double lat_rad = 37.6188 * kDegToRad;
            gps.lon_deg = -122.3750 + static_cast<double>(current_pe) / (111319.5 * std::cos(lat_rad));
            gps.alt_msl_m = 10.0f;
            gps.vel_ned[0] = true_vn;
            gps.vel_ned[1] = true_ve;
            gps.vel_ned[2] = 0.0f;
            gps.h_acc = 0.8f;
            gps.v_acc = 1.2f;
            gps.s_acc = 0.1f;
            eskf.update_gps(gps);
        }

        // Baro updates at 50 Hz (every 8 ticks)
        if (i % 8 == 0) {
            eskf.update_baro(10.0f);
        }
    }

    float pos[3], vel[3];
    eskf.get_pos_ned(pos);
    eskf.get_vel_ned(vel);

    EXPECT_NEAR(pos[0], current_pn, 0.5f);
    EXPECT_NEAR(pos[1], current_pe, 0.5f);
    EXPECT_NEAR(pos[2], 0.0f, 0.3f);

    EXPECT_NEAR(vel[0], true_vn, 0.2f);
    EXPECT_NEAR(vel[1], true_ve, 0.2f);
    EXPECT_NEAR(vel[2], 0.0f, 0.2f);
}

// ─── 6. Outlier Rejection and Gate Gating ──────────────────────────────────────

TEST(ESKF15Test, OutlierRejectionAndGateGating) {
    ESKF15 eskf;
    eskf.init();
    eskf.set_home(37.6188, -122.3750, 10.0f);

    float gyro[3] = {0.0f, 0.0f, 0.0f};
    float accel[3] = {0.0f, 0.0f, -kG};

    // Settle estimator at origin
    for (int i = 0; i < 400; ++i) {
        eskf.predict(gyro, accel, 0.0025f);
        if (i % 8 == 0) eskf.update_zupt(0.02f);
        if (i % 20 == 0) eskf.update_baro(10.0f);
    }

    float pos_before[3];
    eskf.get_pos_ned(pos_before);

    // Inject massive outlier in GPS (150-meter jump in single tick)
    ESKFGpsSample outlier_gps;
    outlier_gps.valid = true;
    outlier_gps.fix_type = 3;
    outlier_gps.satellites = 12;
    outlier_gps.lat_deg = 37.6188 + (150.0 / 111319.5); // 150m jump!
    outlier_gps.lon_deg = -122.3750;
    outlier_gps.alt_msl_m = 10.0f;
    outlier_gps.vel_ned[0] = 50.0f; // 50 m/s velocity spike!
    outlier_gps.vel_ned[1] = 0.0f;
    outlier_gps.vel_ned[2] = 0.0f;
    outlier_gps.h_acc = 1.0f;
    outlier_gps.v_acc = 1.5f;

    eskf.update_gps(outlier_gps);

    float pos_after[3];
    eskf.get_pos_ned(pos_after);

    // The filter should reject the 150m spike and not move significantly
    EXPECT_NEAR(pos_after[0], pos_before[0], 0.2f);

    // Inject massive outlier in Barometer (200-meter jump)
    eskf.update_baro(210.0f);
    eskf.get_pos_ned(pos_after);
    EXPECT_NEAR(pos_after[2], pos_before[2], 0.2f);
}

// ─── 7. Long-Term Covariance Stability (10,000 Steps) ──────────────────────────

TEST(ESKF15Test, LongTermCovarianceStability10000Steps) {
    ESKF15 eskf;
    eskf.init();
    eskf.set_home(37.6188, -122.3750, 10.0f);

    std::mt19937 rng(1337);
    std::normal_distribution<float> noise_acc(0.0f, 0.05f);
    std::normal_distribution<float> noise_gyr(0.0f, 0.005f);

    constexpr float dt = 0.0025f;
    for (int step = 0; step < 10000; ++step) {
        float gyro[3] = { noise_gyr(rng), noise_gyr(rng), noise_gyr(rng) };
        float accel[3] = { noise_acc(rng), noise_acc(rng), -kG + noise_acc(rng) };

        eskf.predict(gyro, accel, dt);

        if (step % 8 == 0) {
            eskf.update_baro(10.0f + 0.1f * noise_acc(rng));
        }
        if (step % 20 == 0) {
            eskf.update_mag(accel, 0.0f);
        }
        if (step % 40 == 0) {
            ESKFGpsSample gps;
            gps.valid = true;
            gps.fix_type = 3;
            gps.satellites = 12;
            gps.lat_deg = 37.6188;
            gps.lon_deg = -122.3750;
            gps.alt_msl_m = 10.0f;
            gps.vel_ned[0] = 0.0f;
            gps.vel_ned[1] = 0.0f;
            gps.vel_ned[2] = 0.0f;
            gps.h_acc = 1.0f;
            gps.v_acc = 1.5f;
            gps.s_acc = 0.1f;
            eskf.update_gps(gps);
        }
    }

    // Verify filter remains healthy, no NaNs or Infs
    EXPECT_TRUE(eskf.is_healthy());
    EXPECT_TRUE(eskf.is_attitude_valid());
    EXPECT_TRUE(eskf.is_velocity_valid());
    EXPECT_TRUE(eskf.is_position_valid());

    // Verify all diagonal covariance values are strictly positive and finite
    for (int i = 0; i < ESKF15_STATE_DIM; ++i) {
        float var = eskf.get_covariance(i, i);
        EXPECT_TRUE(std::isfinite(var));
        EXPECT_GT(var, 0.0f);
    }

    // Verify covariance symmetry P == P^T
    for (int r = 0; r < ESKF15_STATE_DIM; ++r) {
        for (int c = 0; c < ESKF15_STATE_DIM; ++c) {
            float p_rc = eskf.get_covariance(r, c);
            float p_cr = eskf.get_covariance(c, r);
            EXPECT_TRUE(std::isfinite(p_rc));
            EXPECT_NEAR(p_rc, p_cr, 1e-4f);
        }
    }
}

// ─── 8. Execution Timing Benchmark ─────────────────────────────────────────────

TEST(ESKF15Test, ExecutionTimingBenchmark) {
    ESKF15 eskf;
    eskf.init();
    eskf.set_home(37.6188, -122.3750, 10.0f);

    float gyro[3] = {0.01f, -0.01f, 0.02f};
    float accel[3] = {0.1f, -0.1f, -kG};

    constexpr int kIterations = 2000;
    auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < kIterations; ++i) {
        eskf.predict(gyro, accel, 0.0025f);
        if (i % 8 == 0) eskf.update_baro(10.0f);
        if (i % 40 == 0) {
            ESKFGpsSample gps;
            gps.valid = true;
            gps.fix_type = 3;
            gps.satellites = 12;
            gps.lat_deg = 37.6188;
            gps.lon_deg = -122.3750;
            gps.alt_msl_m = 10.0f;
            gps.vel_ned[0] = 0.0f;
            gps.vel_ned[1] = 0.0f;
            gps.vel_ned[2] = 0.0f;
            eskf.update_gps(gps);
        }
    }

    auto end = std::chrono::high_resolution_clock::now();
    double total_us = std::chrono::duration<double, std::micro>(end - start).count();
    double avg_us_per_iteration = total_us / static_cast<double>(kIterations);

    std::printf("[ESKF15 Benchmark] %d iterations: total %.2f us, average %.3f us/iteration\n",
                kIterations, total_us, avg_us_per_iteration);

    // Real-time requirement: average iteration time must be far below 100 microseconds
    EXPECT_LT(avg_us_per_iteration, 50.0);
}
