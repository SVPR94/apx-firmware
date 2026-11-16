// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include <gtest/gtest.h>

#include "sensors/sim_sensors.hpp"
#include "physics/wind_model.hpp"
#include "physics/multirotor_dynamics.hpp"

#include <cmath>
#include <vector>

using namespace apx::sitl;

namespace {

constexpr uint64_t kStepUs = 2500; // 400 Hz truth, like the physics runners
constexpr double kG = 9.80665;
constexpr double kPi = 3.14159265358979323846;

SimTruth level_truth(uint64_t t_us) {
    SimTruth t;
    t.time_us = t_us;
    t.accel_body[2] = -kG;
    return t;
}

SimTruth yawed_truth(uint64_t t_us, double yaw_rad) {
    SimTruth t = level_truth(t_us);
    t.quat[0] = std::cos(yaw_rad / 2);
    t.quat[3] = std::sin(yaw_rad / 2);
    return t;
}

double stddev(const std::vector<double>& v) {
    double m = 0;
    for (double x : v) m += x;
    m /= static_cast<double>(v.size());
    double s = 0;
    for (double x : v) s += (x - m) * (x - m);
    return std::sqrt(s / static_cast<double>(v.size() - 1));
}

double correlation(const std::vector<double>& a, const std::vector<double>& b) {
    double ma = 0, mb = 0;
    for (size_t i = 0; i < a.size(); ++i) { ma += a[i]; mb += b[i]; }
    ma /= a.size(); mb /= b.size();
    double sab = 0, saa = 0, sbb = 0;
    for (size_t i = 0; i < a.size(); ++i) {
        sab += (a[i] - ma) * (b[i] - mb);
        saa += (a[i] - ma) * (a[i] - ma);
        sbb += (b[i] - mb) * (b[i] - mb);
    }
    return sab / std::sqrt(saa * sbb);
}

// Amplitude of the f_hz component of x sampled at fs_hz (single-bin DFT).
double tone_amplitude(const std::vector<double>& x, double fs_hz, double f_hz) {
    double re = 0, im = 0;
    for (size_t n = 0; n < x.size(); ++n) {
        const double ph = 2 * kPi * f_hz * static_cast<double>(n) / fs_hz;
        re += x[n] * std::cos(ph);
        im -= x[n] * std::sin(ph);
    }
    return 2.0 * std::sqrt(re * re + im * im) / static_cast<double>(x.size());
}

std::vector<double> gyro_x_sequence(SimSensorConfig cfg, int lane, int steps) {
    SimSensorSuite suite;
    suite.init(cfg);
    std::vector<double> out;
    for (int k = 0; k < steps; ++k) {
        suite.update(level_truth(static_cast<uint64_t>(k) * kStepUs));
        out.push_back(suite.imu(lane).gyro_xyz[0]);
    }
    return out;
}

// GPS north offset (m) relative to home from latitude.
double gps_north_m(const SimSensorConfig& cfg, const apx::drivers::GpsData& d) {
    return (d.lat_deg - cfg.home_lat_deg) * kPi / 180.0 * 6378137.0;
}

} // namespace

TEST(SimSensors, ProfilesParse) {
    SensorProfile p;
    EXPECT_TRUE(parse_sensor_profile("ideal", p));
    EXPECT_EQ(p, SensorProfile::IDEAL);
    EXPECT_TRUE(parse_sensor_profile("realistic", p));
    EXPECT_EQ(p, SensorProfile::REALISTIC);
    EXPECT_TRUE(parse_sensor_profile("harsh", p));
    EXPECT_EQ(p, SensorProfile::HARSH);
    EXPECT_FALSE(parse_sensor_profile("noisy", p));
    EXPECT_FALSE(parse_sensor_profile(nullptr, p));
}

TEST(SimSensors, LaneCountsAreClamped) {
    SimSensorConfig cfg = make_sensor_config(SensorProfile::IDEAL);
    cfg.imu_count = 0;
    cfg.gps_count = 9;
    SimSensorSuite suite;
    suite.init(cfg);
    EXPECT_EQ(suite.imu_count(), 1);
    EXPECT_EQ(suite.gps_count(), 4);
}

TEST(SimSensors, IdealProfileReproducesTruth) {
    SimSensorConfig cfg = make_sensor_config(SensorProfile::IDEAL);
    cfg.gps_count = 4;
    SimSensorSuite suite;
    suite.init(cfg);

    SimTruth t = yawed_truth(0, 0.3);
    t.gyro_body[0] = 0.1; t.gyro_body[1] = -0.2; t.gyro_body[2] = 0.05;
    t.accel_body[0] = 0.5;
    t.pos_ned[0] = 12.0; t.pos_ned[1] = -7.0; t.pos_ned[2] = -25.0;
    t.vel_ned[0] = 1.0;
    suite.update(t);

    for (int i = 0; i < 4; ++i) {
        ASSERT_GT(suite.imu_seq(i), 0u);
        EXPECT_FLOAT_EQ(suite.imu(i).gyro_xyz[0], 0.1f);
        EXPECT_FLOAT_EQ(suite.imu(i).gyro_xyz[1], -0.2f);
        EXPECT_FLOAT_EQ(suite.imu(i).accel_xyz[0], 0.5f);
        EXPECT_FLOAT_EQ(suite.imu(i).accel_xyz[2], static_cast<float>(-kG));
        EXPECT_NEAR(suite.baro(i).altitude_m, cfg.home_alt_m + 25.0f, 0.05f);
        ASSERT_GT(suite.gps_seq(i), 0u);
        EXPECT_NEAR(gps_north_m(cfg, suite.gps(i)), 12.0, 0.01);
        EXPECT_NEAR(suite.gps(i).alt_msl_m, cfg.home_alt_m + 25.0f, 0.01f);
        EXPECT_NEAR(suite.gps(i).vel_n_ms, 1.0f, 1e-6f);
        EXPECT_TRUE(suite.gps(i).valid);
    }
}

TEST(SimSensors, SameSeedReplaysDifferentSeedDiffers) {
    SimSensorConfig cfg = make_sensor_config(SensorProfile::REALISTIC);
    const auto a = gyro_x_sequence(cfg, 0, 2000);
    const auto b = gyro_x_sequence(cfg, 0, 2000);
    EXPECT_EQ(a, b);
    cfg.seed = 2;
    const auto c = gyro_x_sequence(cfg, 0, 2000);
    EXPECT_NE(a, c);
}

TEST(SimSensors, LaneCountDoesNotChangeOtherLanes) {
    SimSensorConfig cfg = make_sensor_config(SensorProfile::REALISTIC);
    cfg.imu_count = 1;
    const auto one = gyro_x_sequence(cfg, 0, 2000);
    cfg.imu_count = 4;
    const auto four = gyro_x_sequence(cfg, 0, 2000);
    EXPECT_EQ(one, four);
}

TEST(SimSensors, LaneNoiseIsIndependent) {
    SimSensorConfig cfg = make_sensor_config(SensorProfile::REALISTIC);
    cfg.imu.gyro_lsb = 0.0f; // quantisation would dominate at this noise level
    const auto l0 = gyro_x_sequence(cfg, 0, 8000);
    const auto l1 = gyro_x_sequence(cfg, 1, 8000);
    EXPECT_LT(std::fabs(correlation(l0, l1)), 0.1);
}

TEST(SimSensors, GyroWhiteNoiseMatchesNoiseDensity) {
    SimSensorConfig cfg = make_sensor_config(SensorProfile::IDEAL);
    cfg.imu.gyro_noise_density = 1.0e-3f;
    const auto x = gyro_x_sequence(cfg, 2, 20000);
    const double expected = 1.0e-3 * std::sqrt(cfg.imu.rate_hz / 2.0);
    EXPECT_NEAR(stddev(x), expected, 0.05 * expected);
}

TEST(SimSensors, TurnOnBiasDiffersPerLane) {
    SimSensorConfig cfg = make_sensor_config(SensorProfile::IDEAL);
    cfg.imu.gyro_bias_sigma = 0.01f;
    SimSensorSuite suite;
    suite.init(cfg);
    suite.update(level_truth(0));
    for (int i = 0; i < 4; ++i) {
        EXPECT_FLOAT_EQ(suite.imu(i).gyro_xyz[1], suite.gyro_bias(i, 1));
    }
    EXPECT_NE(suite.gyro_bias(0, 1), suite.gyro_bias(1, 1));
    EXPECT_NE(suite.gyro_bias(2, 1), suite.gyro_bias(3, 1));
}

TEST(SimSensors, BiasRandomWalkGrowsWithSqrtTime) {
    // std of the bias change after T seconds should be walk * sqrt(T).
    constexpr float kWalk = 1.0e-3f;
    constexpr double kSeconds = 100.0;
    SimSensorConfig cfg = make_sensor_config(SensorProfile::IDEAL);
    cfg.imu.gyro_bias_walk = kWalk;
    cfg.imu.rate_hz = 50.0f;
    std::vector<double> drift;
    for (uint32_t seed = 1; seed <= 50; ++seed) {
        cfg.seed = seed;
        SimSensorSuite suite;
        suite.init(cfg);
        const uint64_t step = 20000;
        for (uint64_t t = 0; t <= static_cast<uint64_t>(kSeconds * 1e6); t += step) {
            suite.update(level_truth(t));
        }
        for (int i = 0; i < 4; ++i) {
            for (int a = 0; a < 3; ++a) {
                drift.push_back(suite.gyro_bias(i, a));
            }
        }
    }
    const double expected = kWalk * std::sqrt(kSeconds);
    EXPECT_NEAR(stddev(drift), expected, 0.1 * expected);
}

TEST(SimSensors, VibrationFollowsRotorSpeedAndStopsWithMotors) {
    SimSensorConfig cfg = make_sensor_config(SensorProfile::IDEAL);
    cfg.imu.vib_accel_amp = 3.0f;
    cfg.imu.rotor_max_hz = 70.0f;
    SimSensorSuite suite;
    suite.init(cfg);

    std::vector<double> still;
    std::vector<double> spinning;
    for (int k = 0; k < 1600; ++k) {
        SimTruth t = level_truth(static_cast<uint64_t>(k) * kStepUs);
        t.num_motors = 6;
        const bool motors_on = k >= 800;
        for (int m = 0; m < 6; ++m) t.motor_throttle[m] = motors_on ? 0.5f : 0.0f;
        suite.update(t);
        (motors_on ? spinning : still).push_back(suite.imu(0).accel_xyz[0]);
    }
    for (double v : still) {
        EXPECT_EQ(v, 0.0);
    }
    const double f_rotor = 70.0 * std::sqrt(0.5);   // once per rev
    const double f_blade = 2.0 * f_rotor;           // 2 blades
    const double fs = 400.0;
    const double a_rotor = tone_amplitude(spinning, fs, f_rotor);
    const double a_blade = tone_amplitude(spinning, fs, f_blade);
    const double a_off = tone_amplitude(spinning, fs, 23.0);
    EXPECT_GT(a_rotor, 0.2);
    EXPECT_GT(a_blade, 0.1);
    EXPECT_LT(a_off, 0.2 * a_rotor);
}

TEST(SimSensors, ClippingSaturatesAndCounts) {
    SimSensorConfig cfg = make_sensor_config(SensorProfile::IDEAL);
    SimSensorSuite suite;
    suite.init(cfg);
    SimTruth t = level_truth(0);
    t.accel_body[2] = -300.0;
    t.gyro_body[0] = 50.0;
    suite.update(t);
    EXPECT_FLOAT_EQ(suite.imu(0).accel_xyz[2], -cfg.imu.accel_range);
    EXPECT_FLOAT_EQ(suite.imu(0).gyro_xyz[0], cfg.imu.gyro_range);
    EXPECT_EQ(suite.accel_clip_count(0), 1u);
    EXPECT_EQ(suite.gyro_clip_count(0), 1u);
}

TEST(SimSensors, BaroLatency) {
    SimSensorConfig cfg = make_sensor_config(SensorProfile::IDEAL);
    cfg.baro.delay_s = 0.02f;
    SimSensorSuite suite;
    suite.init(cfg);
    const uint64_t jump_us = 1000000;
    uint64_t first_seen = 0;
    for (uint64_t t = 0; t < 2000000; t += kStepUs) {
        SimTruth truth = level_truth(t);
        truth.pos_ned[2] = t >= jump_us ? -10.0 : 0.0;
        suite.update(truth);
        if (first_seen == 0 && suite.baro(0).altitude_m > cfg.home_alt_m + 5.0f) {
            first_seen = t;
            EXPECT_GE(t - suite.baro(0).timestamp_us, 20000u);
        }
    }
    ASSERT_GT(first_seen, 0u);
    EXPECT_GE(first_seen, jump_us + 20000);
    EXPECT_LE(first_seen, jump_us + 20000 + 20000 + kStepUs); // + one 50 Hz period
}

TEST(SimSensors, GpsRateAndDelay) {
    SimSensorConfig cfg = make_sensor_config(SensorProfile::IDEAL);
    cfg.gps.delay_s = 0.15f;
    cfg.gps.rate_hz = 10.0f;
    SimSensorSuite suite;
    suite.init(cfg);
    uint32_t last_seq = 0;
    int samples = 0;
    for (uint64_t t = 0; t <= 10000000; t += kStepUs) {
        SimTruth truth = level_truth(t);
        truth.vel_ned[0] = 5.0;
        truth.pos_ned[0] = 5.0 * static_cast<double>(t) * 1e-6;
        suite.update(truth);
        if (suite.gps_seq(0) != last_seq) {
            last_seq = suite.gps_seq(0);
            ++samples;
            const auto& g = suite.gps(0);
            EXPECT_EQ(t - g.timestamp_us, 150000u);
            // Position is truth at the time of validity, not at release.
            EXPECT_NEAR(gps_north_m(cfg, g), 5.0 * static_cast<double>(g.timestamp_us) * 1e-6, 0.01);
        }
    }
    // 10 s at 10 Hz, minus the samples still in the 0.15 s pipeline.
    EXPECT_NEAR(samples, 99, 1);
}

TEST(SimSensors, GpsCommonModeErrorIsSharedAcrossReceivers) {
    SimSensorConfig cfg = make_sensor_config(SensorProfile::IDEAL);
    cfg.gps_count = 4;
    cfg.gps.common_sigma_h_m = 1.0f;
    cfg.gps.common_tau_s = 30.0f;
    SimSensorSuite shared;
    shared.init(cfg);
    cfg.gps.common_sigma_h_m = 0.0f;
    cfg.gps.receiver_sigma_h_m = 1.0f;
    SimSensorSuite independent;
    independent.init(cfg);

    std::vector<double> shared_diff;
    std::vector<double> indep_diff;
    double shared_max = 0.0;
    for (uint64_t t = 0; t <= 120000000; t += 20000) {
        const SimTruth truth = level_truth(t);
        shared.update(truth);
        independent.update(truth);
        if (shared.gps_seq(3) > 0) {
            shared_max = std::max(shared_max, std::fabs(gps_north_m(cfg, shared.gps(0))));
            shared_diff.push_back(gps_north_m(cfg, shared.gps(0)) - gps_north_m(cfg, shared.gps(3)));
            indep_diff.push_back(gps_north_m(cfg, independent.gps(0)) - gps_north_m(cfg, independent.gps(3)));
        }
    }
    EXPECT_GT(shared_max, 0.1);        // the common error is really there...
    for (double d : shared_diff) {
        EXPECT_NEAR(d, 0.0, 1e-3);     // ...and identical in every receiver
    }
    EXPECT_GT(stddev(indep_diff), 0.3); // independent errors don't cancel
}

TEST(SimSensors, GpsOutagesDropFixAndRecover) {
    SimSensorConfig cfg = make_sensor_config(SensorProfile::IDEAL);
    cfg.gps.outage_mtbf_s = 20.0f;
    cfg.gps.outage_min_s = 1.0f;
    cfg.gps.outage_max_s = 2.0f;
    SimSensorSuite suite;
    suite.init(cfg);
    uint32_t last_seq = 0;
    int total = 0;
    int no_fix = 0;
    int recoveries = 0;
    bool prev_valid = true;
    for (uint64_t t = 0; t <= 600000000; t += 20000) {
        suite.update(level_truth(t));
        if (suite.gps_seq(0) != last_seq) {
            last_seq = suite.gps_seq(0);
            const bool valid = suite.gps(0).valid;
            ++total;
            no_fix += valid ? 0 : 1;
            if (valid && !prev_valid) {
                ++recoveries;
                EXPECT_EQ(suite.gps(0).fix_type, apx::drivers::GpsFixType::FIX_3D);
            }
            prev_valid = valid;
        }
    }
    // Expected outage fraction ~ 1.5 s / (20 s + 1.5 s) = 7 %.
    const double frac = static_cast<double>(no_fix) / total;
    EXPECT_GT(frac, 0.03);
    EXPECT_LT(frac, 0.15);
    EXPECT_GT(recoveries, 10);
}

TEST(SimSensors, MagRotatesWithYaw) {
    SimSensorConfig cfg = make_sensor_config(SensorProfile::IDEAL);
    SimSensorSuite suite;
    suite.init(cfg);
    const double incl = cfg.mag.inclination_deg * kPi / 180.0;
    const double decl = cfg.mag.declination_deg * kPi / 180.0;
    const double bn = cfg.mag.field_gauss * std::cos(incl) * std::cos(decl);
    const double be = cfg.mag.field_gauss * std::cos(incl) * std::sin(decl);
    const double bd = cfg.mag.field_gauss * std::sin(incl);

    suite.update(yawed_truth(0, 0.0));
    EXPECT_NEAR(suite.mag(0).mag_xyz[0], bn, 1e-5);
    EXPECT_NEAR(suite.mag(0).mag_xyz[1], be, 1e-5);
    EXPECT_NEAR(suite.mag(0).mag_xyz[2], bd, 1e-5);

    // Nose east: body x sees the east component, body y minus north.
    suite.update(yawed_truth(20000, kPi / 2));
    EXPECT_NEAR(suite.mag(0).mag_xyz[0], be, 1e-5);
    EXPECT_NEAR(suite.mag(0).mag_xyz[1], -bn, 1e-5);
    EXPECT_NEAR(suite.mag(0).mag_xyz[2], bd, 1e-5);
}

TEST(SimWind, ParsesSpec) {
    WindConfig w;
    EXPECT_TRUE(parse_wind_spec("6,270,1.5", w));
    EXPECT_FLOAT_EQ(w.speed_ms, 6.0f);
    EXPECT_FLOAT_EQ(w.from_deg, 270.0f);
    EXPECT_FLOAT_EQ(w.gust_sigma_ms, 1.5f);
    EXPECT_TRUE(parse_wind_spec("4,90", w));
    EXPECT_FLOAT_EQ(w.gust_sigma_ms, 0.0f);
    EXPECT_FALSE(parse_wind_spec("6", w));
    EXPECT_FALSE(parse_wind_spec("abc", w));
    EXPECT_FALSE(parse_wind_spec("-3,0", w));
    EXPECT_FALSE(parse_wind_spec("3,0,1,9", w));
}

TEST(SimWind, DirectionIsWhereItBlowsFrom) {
    WindModel north;
    north.init(WindConfig{10.0f, 0.0f, 0.0f}, 1);
    const float* w = north.update(0.01f);
    EXPECT_NEAR(w[0], -10.0f, 1e-4f); // from the north = moving south
    EXPECT_NEAR(w[1], 0.0f, 1e-4f);

    WindModel west;
    west.init(WindConfig{10.0f, 270.0f, 0.0f}, 1);
    w = west.update(0.01f);
    EXPECT_NEAR(w[0], 0.0f, 1e-3f);
    EXPECT_NEAR(w[1], 10.0f, 1e-3f);  // from the west = moving east
}

TEST(SimWind, GustStatisticsAndReplay) {
    WindConfig cfg{0.0f, 0.0f, 2.0f};
    WindModel a;
    WindModel b;
    a.init(cfg, 7);
    b.init(cfg, 7);
    std::vector<double> north;
    for (int k = 0; k < 200000; ++k) {
        const float* wa = a.update(0.01f);
        const float* wb = b.update(0.01f);
        ASSERT_EQ(wa[0], wb[0]);
        north.push_back(wa[0]);
    }
    EXPECT_NEAR(stddev(north), 2.0, 0.3);
}

TEST(SimDrag, VehiclePushedDownwindOnlyWhenAirborne) {
    auto params = MultirotorParams::make_hex_x(2.8f, 0.28f);
    params.drag_area_m2 = 0.05f;
    const float max_omega = params.max_rpm * 3.14159265f / 30.0f;
    params.kT = 10.0f / (max_omega * max_omega);
    const float wind[3] = {-10.0f, 0.0f, 0.0f}; // 10 m/s from the north

    // On the ground with motors off: friction holds it.
    MultirotorDynamics parked(params);
    parked.set_wind_ned(wind);
    for (int k = 0; k < 400; ++k) parked.step(0.0025f);
    EXPECT_EQ(parked.get_state().position[0], 0.0f);

    // Just over hover thrust: lifts off and drifts south.
    MultirotorDynamics flying(params);
    flying.set_wind_ned(wind);
    const float hover = params.mass_kg * 9.80665f / (6.0f * 10.0f);
    for (int m = 0; m < 6; ++m) flying.set_motor_throttle(m, hover * 1.05f);
    for (int k = 0; k < 400; ++k) flying.step(0.0025f);
    const auto& s = flying.get_state();
    EXPECT_LT(s.position[2], 0.0f);
    EXPECT_LT(s.velocity[0], -0.5f);
    // Initial drag accel 0.5*rho*CdA*v^2/m = 1.09 m/s^2; it falls as the
    // vehicle picks up speed, so after 1 s the speed is below 1.09 m/s.
    EXPECT_GT(s.velocity[0], -1.1f);
    EXPECT_NEAR(s.velocity[1], 0.0f, 1e-3f);
}
