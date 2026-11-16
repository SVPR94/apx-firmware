// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
// ESKF15 against the SITL sensor suite (REALISTIC profile): noise, turn-on
// biases, bias walk, rotor vibration, baro offset, mag hard/soft iron,
// delayed GNSS. The estimator is fed exactly as apx_main.cpp feeds it and
// is checked against truth at a non-zero heading, where an attitude-error
// frame mismatch cannot hide (the level, yaw-0 tests in test_eskf15.cpp
// cannot see it).
#include <gtest/gtest.h>

#include "eskf15.hpp"
#include "sensors/sim_sensors.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

using apx::estimator::ESKF15;
using apx::sitl::SimSensorSuite;
using apx::sitl::SimTruth;

namespace {

constexpr double kG = 9.80665;
constexpr double kPi = 3.14159265358979323846;
constexpr double kD2R = kPi / 180.0;
constexpr uint64_t kStepUs = 2500; // 400 Hz truth

struct Vec3 { double x, y, z; };
Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 operator*(double s, Vec3 a) { return {s * a.x, s * a.y, s * a.z}; }
double dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
Vec3 cross(Vec3 a, Vec3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
Vec3 unit(Vec3 a) { return (1.0 / std::sqrt(dot(a, a))) * a; }

// R columns = body x, y, z axes expressed in NED (body FRD -> NED).
struct Mat3 { Vec3 c[3]; };

Mat3 attitude_from_thrust(Vec3 f_ned, double yaw) {
    const Vec3 zb = unit(-1.0 * f_ned);               // thrust is along -z body
    const Vec3 xc{std::cos(yaw), std::sin(yaw), 0.0};
    const Vec3 yb = unit(cross(zb, xc));
    const Vec3 xb = cross(yb, zb);
    return {{xb, yb, zb}};
}

Mat3 attitude_from_euler(double r, double p, double y) {
    const double cr = std::cos(r), sr = std::sin(r), cp = std::cos(p), sp = std::sin(p);
    const double cy = std::cos(y), sy = std::sin(y);
    return {{{cy * cp, sy * cp, -sp},
             {cy * sp * sr - sy * cr, sy * sp * sr + cy * cr, cp * sr},
             {cy * sp * cr + sy * sr, sy * sp * cr - cy * sr, cp * cr}}};
}

void mat_to_quat(const Mat3& m, double q[4]) {
    const double r00 = m.c[0].x, r11 = m.c[1].y, r22 = m.c[2].z;
    const double r21 = m.c[1].z, r12 = m.c[2].y, r02 = m.c[2].x, r20 = m.c[0].z;
    const double r10 = m.c[0].y, r01 = m.c[1].x;
    const double tr = r00 + r11 + r22;
    if (tr > 0) {
        const double s = 2.0 * std::sqrt(tr + 1.0);
        q[0] = 0.25 * s; q[1] = (r21 - r12) / s; q[2] = (r02 - r20) / s; q[3] = (r10 - r01) / s;
    } else if (r00 > r11 && r00 > r22) {
        const double s = 2.0 * std::sqrt(1.0 + r00 - r11 - r22);
        q[0] = (r21 - r12) / s; q[1] = 0.25 * s; q[2] = (r01 + r10) / s; q[3] = (r02 + r20) / s;
    } else if (r11 > r22) {
        const double s = 2.0 * std::sqrt(1.0 + r11 - r00 - r22);
        q[0] = (r02 - r20) / s; q[1] = (r01 + r10) / s; q[2] = 0.25 * s; q[3] = (r12 + r21) / s;
    } else {
        const double s = 2.0 * std::sqrt(1.0 + r22 - r00 - r11);
        q[0] = (r10 - r01) / s; q[1] = (r02 + r20) / s; q[2] = (r12 + r21) / s; q[3] = 0.25 * s;
    }
}

// Trajectory: on the ground (tilted, yawed) until t_takeoff, then a climb
// to 10 m and a 15 m-radius circle at 4 m/s with a constant yaw rate.
struct Scenario {
    double t_takeoff;     // s; infinity = never leaves the ground
    double yaw0;          // rad
    double ground_roll, ground_pitch;

    Vec3 pos(double t) const {
        if (t <= t_takeoff) return {0, 0, 0};
        const double tau = t - t_takeoff;
        const double h = 10.0 * (1.0 - std::exp(-tau / 2.0));
        const double r = 15.0, wc = 4.0 / 15.0, tr = 4.0;
        const double th = wc * (tau - tr * (1.0 - std::exp(-tau / tr)));
        return {r * std::sin(th), r * (1.0 - std::cos(th)), -h};
    }
    double yaw(double t) const {
        return t <= t_takeoff ? yaw0 : yaw0 + 0.25 * (t - t_takeoff);
    }
    Mat3 att(double t) const {
        if (t <= t_takeoff) return attitude_from_euler(ground_roll, ground_pitch, yaw0);
        const double h = 1e-3;
        const Vec3 a = (1.0 / (h * h)) * (pos(t + h) - 2.0 * pos(t) + pos(t - h));
        return attitude_from_thrust(a - Vec3{0, 0, kG}, yaw(t));
    }
    bool armed(double t) const { return t > t_takeoff - 2.0; }

    SimTruth truth(uint64_t t_us) const {
        const double t = t_us * 1e-6, h = 1e-3;
        SimTruth s;
        s.time_us = t_us;
        const Vec3 p = pos(t);
        const Vec3 v = (1.0 / (2 * h)) * (pos(t + h) - pos(t - h));
        const Vec3 a = t <= t_takeoff ? Vec3{0, 0, 0}
                                      : (1.0 / (h * h)) * (pos(t + h) - 2.0 * p + pos(t - h));
        const Mat3 R = att(t);
        const Vec3 f = a - Vec3{0, 0, kG};
        for (int i = 0; i < 3; ++i) {
            s.pos_ned[i] = i == 0 ? p.x : (i == 1 ? p.y : p.z);
            s.vel_ned[i] = i == 0 ? v.x : (i == 1 ? v.y : v.z);
            s.accel_body[i] = dot(R.c[i], f); // R^T f
        }
        // Body rates from R^T * dR/dt.
        const Mat3 Rp = att(t + h), Rm = att(t - h);
        Vec3 dR[3];
        for (int i = 0; i < 3; ++i) dR[i] = (1.0 / (2 * h)) * (Rp.c[i] - Rm.c[i]);
        // [w]x = R^T dR ; w_x = (R^T dR)(2,1) = z_b . d(y_b)
        s.gyro_body[0] = dot(R.c[2], dR[1]);
        s.gyro_body[1] = dot(R.c[0], dR[2]);
        s.gyro_body[2] = dot(R.c[1], dR[0]);
        mat_to_quat(R, s.quat);
        s.num_motors = 6;
        const float thr = armed(t) ? 0.5f : 0.0f;
        for (int i = 0; i < 6; ++i) s.motor_throttle[i] = thr;
        return s;
    }
};

struct ErrStats {
    double tilt_deg{0}, yaw_deg{0}, vel{0}, pos_h{0}, pos_v{0};
    void take(const ErrStats& e) {
        tilt_deg = std::max(tilt_deg, e.tilt_deg);
        yaw_deg = std::max(yaw_deg, e.yaw_deg);
        vel = std::max(vel, e.vel);
        pos_h = std::max(pos_h, e.pos_h);
        pos_v = std::max(pos_v, e.pos_v);
    }
};

ErrStats error_vs_truth(const ESKF15& eskf, const SimTruth& t) {
    ErrStats e;
    float q[4];
    eskf.get_quaternion(q);
    // Body z axis (third column of R) of estimate vs truth -> tilt error.
    auto zcol = [](const double w, const double x, const double y, const double z) {
        return Vec3{2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)};
    };
    auto xcol = [](const double w, const double x, const double y, const double z) {
        return Vec3{1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)};
    };
    const Vec3 ze = zcol(q[0], q[1], q[2], q[3]);
    const Vec3 zt = zcol(t.quat[0], t.quat[1], t.quat[2], t.quat[3]);
    e.tilt_deg = std::acos(std::clamp(dot(ze, zt), -1.0, 1.0)) / kD2R;
    const Vec3 xe = xcol(q[0], q[1], q[2], q[3]);
    const Vec3 xt = xcol(t.quat[0], t.quat[1], t.quat[2], t.quat[3]);
    double dy = std::atan2(xe.y, xe.x) - std::atan2(xt.y, xt.x);
    while (dy > kPi) dy -= 2 * kPi;
    while (dy < -kPi) dy += 2 * kPi;
    e.yaw_deg = std::fabs(dy) / kD2R;
    float p[3], v[3];
    eskf.get_pos_ned(p);
    eskf.get_vel_ned(v);
    double dv2 = 0;
    for (int i = 0; i < 3; ++i) dv2 += (v[i] - t.vel_ned[i]) * (v[i] - t.vel_ned[i]);
    e.vel = std::sqrt(dv2);
    e.pos_h = std::hypot(p[0] - t.pos_ned[0], p[1] - t.pos_ned[1]);
    e.pos_v = std::fabs(p[2] - t.pos_ned[2]);
    return e;
}

// Feeds the estimator the way apx_main.cpp does and returns the worst error
// over [t_check_from, t_end].
// The EKF_* parameter defaults (param_manager.cpp), which SITL flies with.
apx::estimator::ESKFTuningParams param_default_tuning() {
    apx::estimator::ESKFTuningParams t;
    t.q_accel_noise = 0.15f;
    t.q_gyro_noise = 0.015f;
    t.q_accel_bias_walk = 0.001f;
    t.q_gyro_bias_walk = 0.0001f;
    t.r_baro_alt = 0.8f;
    t.r_gps_pos_horiz = 0.5f;
    t.r_gps_vel = 0.15f;
    t.r_mag_heading = 0.05f;
    t.gate_gps_pos = t.gate_gps_vel = t.gate_baro = t.gate_mag = 3.0f;
    return t;
}

ErrStats run(const Scenario& sc, double t_end, double t_check_from, uint32_t seed,
             const apx::estimator::ESKFTuningParams& tuning) {
    auto cfg = apx::sitl::make_sensor_config(apx::sitl::SensorProfile::REALISTIC);
    cfg.seed = seed;
    SimSensorSuite sens;
    sens.init(cfg);

    ESKF15 eskf;
    eskf.init(tuning);
    eskf.set_home(cfg.home_lat_deg, cfg.home_lon_deg, cfg.home_alt_m);
    const float decl = static_cast<float>(cfg.mag.declination_deg * kD2R);

    uint32_t imu_seq[4]{}, baro_seq[4]{}, mag_seq[4]{}, gps_seq[4]{};
    uint64_t last_imu_us = 0;
    ErrStats worst;
    const uint64_t n = static_cast<uint64_t>(t_end * 1e6 / kStepUs);
    for (uint64_t k = 1; k <= n; ++k) {
        const SimTruth t = sc.truth(k * kStepUs);
        sens.update(t);

        float acc[3]{}, gyr[3]{};
        int cnt = 0;
        uint64_t imu_t = 0;
        for (int i = 0; i < sens.imu_count(); ++i) {
            if (sens.imu_seq(i) != imu_seq[i]) {
                imu_seq[i] = sens.imu_seq(i);
                for (int a = 0; a < 3; ++a) {
                    acc[a] += sens.imu(i).accel_xyz[a];
                    gyr[a] += sens.imu(i).gyro_xyz[a];
                }
                imu_t = sens.imu(i).timestamp_us;
                ++cnt;
            }
            if (sens.baro_seq(i) != baro_seq[i]) {
                baro_seq[i] = sens.baro_seq(i);
                if (i == 0) eskf.update_baro(sens.baro(0).altitude_m);
            }
            if (sens.mag_seq(i) != mag_seq[i]) {
                mag_seq[i] = sens.mag_seq(i);
                if (i == 0) eskf.update_mag(sens.mag(0).mag_xyz, decl);
            }
        }
        if (cnt > 0) {
            for (int a = 0; a < 3; ++a) { acc[a] /= cnt; gyr[a] /= cnt; }
            float dt = 0.0025f;
            if (last_imu_us > 0 && imu_t > last_imu_us) dt = (imu_t - last_imu_us) * 1e-6f;
            last_imu_us = imu_t;
            eskf.predict(gyr, acc, std::min(dt, 0.05f), imu_t);
            if (!sc.armed(t.time_us * 1e-6) && ESKF15::is_stationary(gyr, acc)) {
                eskf.update_zupt(0.01f);
            }
        }
        for (int i = 0; i < sens.gps_count(); ++i) {
            if (sens.gps_seq(i) != gps_seq[i]) {
                gps_seq[i] = sens.gps_seq(i);
                const auto& g = sens.gps(i);
                if (i != 0 || !g.valid) continue;
                apx::estimator::ESKFGpsSample s;
                s.timestamp_us = g.timestamp_us;
                s.lat_deg = g.lat_deg;
                s.lon_deg = g.lon_deg;
                s.alt_msl_m = g.alt_msl_m;
                s.vel_ned[0] = g.vel_n_ms;
                s.vel_ned[1] = g.vel_e_ms;
                s.vel_ned[2] = g.vel_d_ms;
                s.h_acc = g.hdop;
                s.v_acc = g.vdop * 1.5f;
                s.s_acc = 0.2f;
                s.fix_type = static_cast<uint8_t>(g.fix_type);
                s.satellites = g.satellites;
                s.valid = g.fix_type >= apx::drivers::GpsFixType::FIX_3D;
                eskf.update_gps(s);
            }
        }
        const double ts = t.time_us * 1e-6;
        if (ts >= t_check_from) {
            EXPECT_TRUE(eskf.is_healthy());
            EXPECT_TRUE(eskf.is_attitude_valid());
            worst.take(error_vs_truth(eskf, t));
        }
    }
    std::printf("[ESKF sim] worst: tilt %.2f deg, yaw %.2f deg, vel %.3f m/s, pos_h %.2f m, pos_v %.2f m\n",
                worst.tilt_deg, worst.yaw_deg, worst.vel, worst.pos_h, worst.pos_v);
    return worst;
}

} // namespace

// Disarmed on the ground, nose at 135 deg, slightly tilted: the estimate must
// sit still and match the truth (the reported failure: roll/pitch wander and
// velocity runs away with the vehicle motionless in Gazebo).
TEST(ESKF15SimTest, StationaryOnGroundAtNonZeroHeading) {
    for (const auto& tuning : {apx::estimator::ESKFTuningParams{}, param_default_tuning()})
    for (uint32_t seed : {1u, 2u, 3u}) {
        Scenario sc{1e9, 135.0 * kD2R, 1.5 * kD2R, -2.0 * kD2R};
        const ErrStats e = run(sc, 90.0, 10.0, seed, tuning);
        EXPECT_LT(e.tilt_deg, 1.0) << "seed " << seed;
        EXPECT_LT(e.yaw_deg, 4.0) << "seed " << seed;
        EXPECT_LT(e.vel, 0.10) << "seed " << seed;
        EXPECT_LT(e.pos_h, 2.5) << "seed " << seed;
        EXPECT_LT(e.pos_v, 1.5) << "seed " << seed;
    }
}

// Ground alignment, then climb and a sustained turn with continuous yaw
// rotation, rotor vibration and 150 ms GNSS latency.
TEST(ESKF15SimTest, CirclingFlightWithYawRotation) {
    for (const auto& tuning : {apx::estimator::ESKFTuningParams{}, param_default_tuning()})
    for (uint32_t seed : {1u, 2u, 3u}) {
        Scenario sc{20.0, -60.0 * kD2R, 0.0, 0.0};
        const ErrStats e = run(sc, 110.0, 30.0, seed, tuning);
        EXPECT_LT(e.tilt_deg, 2.0) << "seed " << seed;
        EXPECT_LT(e.yaw_deg, 5.0) << "seed " << seed;
        EXPECT_LT(e.vel, 0.5) << "seed " << seed;
        EXPECT_LT(e.pos_h, 3.0) << "seed " << seed;
        EXPECT_LT(e.pos_v, 2.0) << "seed " << seed;
    }
}
