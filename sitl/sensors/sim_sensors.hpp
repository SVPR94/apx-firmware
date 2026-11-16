// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
//   IMU   white noise (from a noise density), turn-on bias, bias random
//         walk, scale error, motor vibration (shared source, per-IC
//         mounting gain), range clipping, quantisation, latency.
//   Baro  ISA pressure, white noise, turn-on offset, Gauss-Markov drift,
//         static-port dynamic-pressure error, latency.
//   Mag   local earth field rotated into the body, hard/soft iron,
//         throttle-proportional motor interference, white noise.
//   GNSS  common-mode error shared by all receivers plus per-receiver
//         error (both Gauss-Markov), white noise, antenna lever arm,
//         fixed latency, optional random outages.
// Default numbers are order-of-magnitude values for consumer MEMS parts
// and standard-precision GNSS, standing in for *post-calibration*
// residuals. They are not taken from a specific datasheet: replace them
// with the figures for the fitted parts.
#pragma once

#include "sim_noise.hpp"
#include "imu/icm42688/icm42688.hpp"
#include "baro/ms5611/ms5611.hpp"
#include "mag/ist8310/ist8310.hpp"
#include "gps/ublox/ublox_driver.hpp"

#include <array>
#include <cstdint>

namespace apx::sitl {

struct ImuModelConfig {
    float rate_hz{400.0f};
    float gyro_noise_density{5.0e-5f};   // rad/s/sqrt(Hz)
    float accel_noise_density{7.0e-4f};  // m/s^2/sqrt(Hz)
    float gyro_bias_sigma{0.003f};       // rad/s, turn-on (1 sigma)
    float accel_bias_sigma{0.05f};       // m/s^2, turn-on (1 sigma)
    float gyro_bias_walk{2.0e-5f};       // rad/s/sqrt(s)
    float accel_bias_walk{2.0e-4f};      // m/s^2/sqrt(s)
    float scale_error_sigma{0.003f};     // fraction (1 sigma)
    float gyro_range{34.9f};             // rad/s (2000 deg/s)
    float accel_range{156.9f};           // m/s^2 (16 g)
    float gyro_lsb{0.00106f};            // rad/s per count (16-bit at 2000 deg/s); 0 = off
    float accel_lsb{0.00479f};           // m/s^2 per count (16-bit at 16 g); 0 = off
    float delay_s{0.0f};
    // Vibration: approximate per-axis amplitude at full throttle summed
    // over all motors. Rotor speed is taken as rotor_max_hz * sqrt(throttle)
    // (thrust roughly linear in throttle); each motor adds a once-per-rev
    // imbalance line and a blade-pass line at blades x that.
    float vib_accel_amp{3.0f};           // m/s^2
    float vib_gyro_amp{0.03f};           // rad/s
    float rotor_max_hz{70.0f};           // rev/s at full throttle
    int blades{2};
    float temp_c{35.0f};
};

struct BaroModelConfig {
    float rate_hz{50.0f};
    float noise_pa{1.5f};
    float offset_sigma_pa{30.0f};        // turn-on absolute error (1 sigma)
    float drift_sigma_pa{6.0f};          // Gauss-Markov drift (~0.5 m)
    float drift_tau_s{120.0f};
    float dyn_pressure_coef{0.05f};      // |coef| x 0.5 rho v^2, sign per IC
    float delay_s{0.01f};
    float temp_c{30.0f};
};

struct MagModelConfig {
    float rate_hz{100.0f};
    // Local earth field. The defaults are rough values for the SITL home
    // (Pune) - not a WMM evaluation. Override for other locations.
    float field_gauss{0.42f};
    float declination_deg{-0.5f};
    float inclination_deg{29.0f};
    float noise_gauss{0.003f};
    float hard_iron_sigma_gauss{0.01f};
    float soft_iron_sigma{0.01f};        // per-axis scale (1 sigma)
    float motor_interference_gauss{0.02f}; // at full throttle, per-IC direction
    float delay_s{0.0f};
};

struct GpsModelConfig {
    float rate_hz{10.0f};
    float delay_s{0.15f};
    float common_sigma_h_m{0.8f};        // shared by all receivers
    float common_tau_s{60.0f};
    float receiver_sigma_h_m{0.4f};      // independent per receiver
    float receiver_tau_s{30.0f};
    float white_sigma_h_m{0.1f};
    float vertical_ratio{1.6f};          // vertical / horizontal error
    float vel_sigma_h_ms{0.05f};
    float vel_sigma_v_ms{0.08f};
    float nominal_sats{18.0f};
    float nominal_hdop{0.7f};
    float outage_mtbf_s{0.0f};           // mean time between outages, 0 = never
    float outage_min_s{1.0f};
    float outage_max_s{5.0f};
    float antenna_offset_m[4][3]{};      // per receiver, body FRD
};

enum class SensorProfile { IDEAL, REALISTIC, HARSH };

struct SimSensorConfig {
    int imu_count{4};                    // IMU+baro+mag lanes, 1..4
    int gps_count{1};                    // GNSS receivers, 0..4
    uint32_t seed{1};
    double home_lat_deg{18.5204};
    double home_lon_deg{73.8567};
    float home_alt_m{540.0f};
    ImuModelConfig imu{};
    BaroModelConfig baro{};
    MagModelConfig mag{};
    GpsModelConfig gps{};
};

// Profile defaults. IDEAL removes every error source (rates and range
// limits stay), REALISTIC uses the defaults above, HARSH multiplies noise,
// drift and vibration and enables GNSS outages.
SimSensorConfig make_sensor_config(SensorProfile profile);
bool parse_sensor_profile(const char* name, SensorProfile& out);
const char* sensor_profile_name(SensorProfile profile);

// Truth from the physics backend.
struct SimTruth {
    uint64_t time_us{0};                 // simulation time
    double pos_ned[3]{};                 // m, relative to home
    double vel_ned[3]{};                 // m/s
    double accel_body[3]{};              // specific force, FRD, m/s^2
    double gyro_body[3]{};               // rad/s, FRD
    double quat[4]{1.0, 0.0, 0.0, 0.0};  // body FRD -> NED (w, x, y, z)
    double airspeed_ms{0.0};
    float motor_throttle[8]{};
    int num_motors{0};
};

class SimSensorSuite {
public:
    static constexpr int kMaxImu = 4;
    static constexpr int kMaxGps = 4;

    void init(const SimSensorConfig& cfg);

    // Call with each new truth sample. Generates whatever samples are due
    // and releases those whose latency has elapsed.
    void update(const SimTruth& truth);

    int imu_count() const { return m_imu_count; }
    int gps_count() const { return m_gps_count; }
    const SimSensorConfig& config() const { return m_cfg; }

    // Latest released sample per instance. The sequence number increases
    // each time a new sample is released (0 = nothing yet).
    const apx::drivers::ImuData&  imu(int i)  const { return m_imu[idx(i)].out; }
    const apx::drivers::BaroData& baro(int i) const { return m_baro[idx(i)].out; }
    const apx::drivers::MagData&  mag(int i)  const { return m_mag[idx(i)].out; }
    const apx::drivers::GpsData&  gps(int i)  const { return m_gps[idx(i)].out; }
    uint32_t imu_seq(int i)  const { return m_imu[idx(i)].seq; }
    uint32_t baro_seq(int i) const { return m_baro[idx(i)].seq; }
    uint32_t mag_seq(int i)  const { return m_mag[idx(i)].seq; }
    uint32_t gps_seq(int i)  const { return m_gps[idx(i)].seq; }

    uint32_t accel_clip_count(int i) const { return m_imu[idx(i)].accel_clips; }
    uint32_t gyro_clip_count(int i)  const { return m_imu[idx(i)].gyro_clips; }
    bool gps_in_outage(int i) const { return m_gps[idx(i)].outage_left_s > 0.0f; }

    // Current internal error states, for tests and logs.
    float gyro_bias(int i, int axis)  const { return m_imu[idx(i)].gyro_bias[axis % 3]; }
    float accel_bias(int i, int axis) const { return m_imu[idx(i)].accel_bias[axis % 3]; }

private:
    static int idx(int i) { return i < 0 ? 0 : (i >= kMaxImu ? kMaxImu - 1 : i); }

    struct ImuState {
        SimRng rng;
        float gyro_bias[3]{};
        float accel_bias[3]{};
        float gyro_scale[3]{};
        float accel_scale[3]{};
        float mount_gain[3]{};
        uint64_t next_due_us{0};
        uint64_t last_us{0};
        uint32_t accel_clips{0};
        uint32_t gyro_clips{0};
        DelayLine<apx::drivers::ImuData, 32> delay;
        apx::drivers::ImuData out{};
        uint32_t seq{0};
    };
    struct BaroState {
        SimRng rng;
        float offset_pa{0.0f};
        float dyn_coef{0.0f};
        GaussMarkov drift;
        uint64_t next_due_us{0};
        uint64_t last_us{0};
        DelayLine<apx::drivers::BaroData, 16> delay;
        apx::drivers::BaroData out{};
        uint32_t seq{0};
    };
    struct MagState {
        SimRng rng;
        float hard_iron[3]{};
        float soft_scale[3]{};
        float motor_dir[3]{};
        uint64_t next_due_us{0};
        DelayLine<apx::drivers::MagData, 16> delay;
        apx::drivers::MagData out{};
        uint32_t seq{0};
    };
    struct GpsState {
        SimRng rng;
        GaussMarkov err[3];
        GaussMarkov sats;
        float outage_left_s{0.0f};
        uint64_t next_due_us{0};
        uint64_t last_us{0};
        DelayLine<apx::drivers::GpsData, 32> delay;
        apx::drivers::GpsData out{};
        uint32_t seq{0};
    };

    void step_vibration(const SimTruth& t, float dt_s);
    void sample_imu(int i, const SimTruth& t);
    void sample_baro(int i, const SimTruth& t);
    void sample_mag(int i, const SimTruth& t);
    void sample_gps(int i, const SimTruth& t);
    static bool due(uint64_t now_us, uint64_t& next_due_us, float rate_hz);

    SimSensorConfig m_cfg{};
    int m_imu_count{0};
    int m_gps_count{0};
    bool m_have_time{false};
    uint64_t m_last_time_us{0};

    // Vibration source shared by all IMUs (the motors).
    SimRng m_vib_rng;
    float m_rotor_phase[8]{};
    float m_axis_phase[8][3]{};
    float m_vib_accel[3]{};
    float m_vib_gyro[3]{};

    // GNSS error common to every receiver (atmosphere, ephemeris).
    SimRng m_gps_common_rng;
    GaussMarkov m_gps_common[3];
    uint64_t m_gps_common_last_us{0};

    float m_mag_earth_ned[3]{};

    std::array<ImuState, kMaxImu> m_imu{};
    std::array<BaroState, kMaxImu> m_baro{};
    std::array<MagState, kMaxImu> m_mag{};
    std::array<GpsState, kMaxGps> m_gps{};
};

} // namespace apx::sitl
