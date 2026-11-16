// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "sim_sensors.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace apx::sitl {

namespace {

constexpr float kTwoPi = 6.28318530718f;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kEarthRadiusM = 6378137.0;
constexpr double kSeaLevelPa = 101325.0;

// Stream ids: one independent random stream per sensor instance.
constexpr uint32_t kStreamImu = 0x100;
constexpr uint32_t kStreamBaro = 0x200;
constexpr uint32_t kStreamMag = 0x300;
constexpr uint32_t kStreamGps = 0x400;
constexpr uint32_t kStreamGpsCommon = 0x500;
constexpr uint32_t kStreamVibration = 0x600;

// Body FRD -> NED rotation matrix from (w, x, y, z), row-major.
void quat_to_dcm(const double q[4], double R[9]) {
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    R[0] = 1 - 2 * (y * y + z * z); R[1] = 2 * (x * y - w * z);     R[2] = 2 * (x * z + w * y);
    R[3] = 2 * (x * y + w * z);     R[4] = 1 - 2 * (x * x + z * z); R[5] = 2 * (y * z - w * x);
    R[6] = 2 * (x * z - w * y);     R[7] = 2 * (y * z + w * x);     R[8] = 1 - 2 * (x * x + y * y);
}

// ISA troposphere, same constants as the physics runners.
double pressure_from_alt(double alt_m) {
    return kSeaLevelPa * std::pow(1.0 - alt_m / 44330.0, 5.255);
}

double alt_from_pressure(double p_pa) {
    return 44330.0 * (1.0 - std::pow(p_pa / kSeaLevelPa, 1.0 / 5.255));
}

float clip_quantise(float v, float range, float lsb, bool& clipped) {
    if (range > 0.0f && std::fabs(v) > range) {
        v = std::copysign(range, v);
        clipped = true;
    }
    if (lsb > 0.0f) {
        v = std::round(v / lsb) * lsb;
    }
    return v;
}

} // namespace

SimSensorConfig make_sensor_config(SensorProfile profile) {
    SimSensorConfig c{};
    switch (profile) {
    case SensorProfile::IDEAL: {
        auto& i = c.imu;
        i.gyro_noise_density = i.accel_noise_density = 0.0f;
        i.gyro_bias_sigma = i.accel_bias_sigma = 0.0f;
        i.gyro_bias_walk = i.accel_bias_walk = 0.0f;
        i.scale_error_sigma = 0.0f;
        i.gyro_lsb = i.accel_lsb = 0.0f;
        i.delay_s = 0.0f;
        i.vib_accel_amp = i.vib_gyro_amp = 0.0f;
        auto& b = c.baro;
        b.noise_pa = b.offset_sigma_pa = b.drift_sigma_pa = b.dyn_pressure_coef = b.delay_s = 0.0f;
        auto& m = c.mag;
        m.noise_gauss = m.hard_iron_sigma_gauss = m.soft_iron_sigma = m.motor_interference_gauss = 0.0f;
        m.delay_s = 0.0f;
        auto& g = c.gps;
        g.delay_s = 0.0f;
        g.common_sigma_h_m = g.receiver_sigma_h_m = g.white_sigma_h_m = 0.0f;
        g.vel_sigma_h_ms = g.vel_sigma_v_ms = 0.0f;
        g.outage_mtbf_s = 0.0f;
        break;
    }
    case SensorProfile::REALISTIC:
        break;
    case SensorProfile::HARSH: {
        auto& i = c.imu;
        i.gyro_noise_density *= 2.0f;
        i.accel_noise_density *= 2.0f;
        i.gyro_bias_sigma *= 2.0f;
        i.accel_bias_sigma *= 2.0f;
        i.gyro_bias_walk *= 3.0f;
        i.accel_bias_walk *= 3.0f;
        i.vib_accel_amp *= 3.0f;
        i.vib_gyro_amp *= 3.0f;
        c.baro.noise_pa *= 2.0f;
        c.baro.drift_sigma_pa *= 2.0f;
        c.baro.dyn_pressure_coef *= 2.0f;
        c.mag.noise_gauss *= 2.0f;
        c.mag.motor_interference_gauss *= 3.0f;
        auto& g = c.gps;
        g.common_sigma_h_m *= 2.0f;
        g.receiver_sigma_h_m *= 2.0f;
        g.white_sigma_h_m *= 2.0f;
        g.vel_sigma_h_ms *= 2.0f;
        g.vel_sigma_v_ms *= 2.0f;
        g.nominal_sats = 11.0f;
        g.nominal_hdop = 1.3f;
        g.outage_mtbf_s = 120.0f;
        g.outage_min_s = 2.0f;
        g.outage_max_s = 8.0f;
        break;
    }
    }
    return c;
}

bool parse_sensor_profile(const char* name, SensorProfile& out) {
    if (name == nullptr) {
        return false;
    }
    if (std::strcmp(name, "ideal") == 0) {
        out = SensorProfile::IDEAL;
    } else if (std::strcmp(name, "realistic") == 0) {
        out = SensorProfile::REALISTIC;
    } else if (std::strcmp(name, "harsh") == 0) {
        out = SensorProfile::HARSH;
    } else {
        return false;
    }
    return true;
}

const char* sensor_profile_name(SensorProfile profile) {
    switch (profile) {
    case SensorProfile::IDEAL: return "ideal";
    case SensorProfile::REALISTIC: return "realistic";
    case SensorProfile::HARSH: return "harsh";
    }
    return "?";
}

void SimSensorSuite::init(const SimSensorConfig& cfg) {
    m_cfg = cfg;
    m_imu_count = std::clamp(cfg.imu_count, 1, kMaxImu);
    m_gps_count = std::clamp(cfg.gps_count, 0, kMaxGps);
    m_have_time = false;
    m_last_time_us = 0;

    const uint32_t seed = cfg.seed;
    const auto& ic = cfg.imu;
    for (int i = 0; i < kMaxImu; ++i) {
        ImuState& s = m_imu[i];
        s = ImuState{};
        s.rng.reseed(seed, kStreamImu + i);
        for (int a = 0; a < 3; ++a) {
            s.gyro_bias[a] = s.rng.gauss(ic.gyro_bias_sigma);
            s.accel_bias[a] = s.rng.gauss(ic.accel_bias_sigma);
            s.gyro_scale[a] = s.rng.gauss(ic.scale_error_sigma);
            s.accel_scale[a] = s.rng.gauss(ic.scale_error_sigma);
            s.mount_gain[a] = s.rng.uniform(0.7f, 1.3f);
        }

        BaroState& b = m_baro[i];
        b = BaroState{};
        b.rng.reseed(seed, kStreamBaro + i);
        b.offset_pa = b.rng.gauss(cfg.baro.offset_sigma_pa);
        b.dyn_coef = cfg.baro.dyn_pressure_coef * b.rng.uniform(0.5f, 1.0f) *
                     (b.rng.uniform(0.0f, 1.0f) < 0.5f ? -1.0f : 1.0f);
        b.drift.init(cfg.baro.drift_sigma_pa, cfg.baro.drift_tau_s, b.rng);

        MagState& m = m_mag[i];
        m = MagState{};
        m.rng.reseed(seed, kStreamMag + i);
        float dir_norm = 0.0f;
        for (int a = 0; a < 3; ++a) {
            m.hard_iron[a] = m.rng.gauss(cfg.mag.hard_iron_sigma_gauss);
            m.soft_scale[a] = 1.0f + m.rng.gauss(cfg.mag.soft_iron_sigma);
            m.motor_dir[a] = m.rng.gauss();
            dir_norm += m.motor_dir[a] * m.motor_dir[a];
        }
        dir_norm = std::sqrt(dir_norm);
        for (float& d : m.motor_dir) {
            d = dir_norm > 1e-6f ? d / dir_norm : 0.0f;
        }
    }

    const auto& gc = cfg.gps;
    for (int i = 0; i < kMaxGps; ++i) {
        GpsState& g = m_gps[i];
        g = GpsState{};
        g.rng.reseed(seed, kStreamGps + i);
        g.err[0].init(gc.receiver_sigma_h_m, gc.receiver_tau_s, g.rng);
        g.err[1].init(gc.receiver_sigma_h_m, gc.receiver_tau_s, g.rng);
        g.err[2].init(gc.receiver_sigma_h_m * gc.vertical_ratio, gc.receiver_tau_s, g.rng);
        g.sats.init(1.5f, 20.0f, g.rng);
    }
    m_gps_common_rng.reseed(seed, kStreamGpsCommon);
    m_gps_common[0].init(gc.common_sigma_h_m, gc.common_tau_s, m_gps_common_rng);
    m_gps_common[1].init(gc.common_sigma_h_m, gc.common_tau_s, m_gps_common_rng);
    m_gps_common[2].init(gc.common_sigma_h_m * gc.vertical_ratio, gc.common_tau_s, m_gps_common_rng);

    m_vib_rng.reseed(seed, kStreamVibration);
    for (int k = 0; k < 8; ++k) {
        m_rotor_phase[k] = m_vib_rng.uniform(0.0f, kTwoPi);
        for (int a = 0; a < 3; ++a) {
            m_axis_phase[k][a] = m_vib_rng.uniform(0.0f, kTwoPi);
        }
    }
    std::memset(m_vib_accel, 0, sizeof(m_vib_accel));
    std::memset(m_vib_gyro, 0, sizeof(m_vib_gyro));

    const double incl = cfg.mag.inclination_deg * kDegToRad;
    const double decl = cfg.mag.declination_deg * kDegToRad;
    m_mag_earth_ned[0] = static_cast<float>(cfg.mag.field_gauss * std::cos(incl) * std::cos(decl));
    m_mag_earth_ned[1] = static_cast<float>(cfg.mag.field_gauss * std::cos(incl) * std::sin(decl));
    m_mag_earth_ned[2] = static_cast<float>(cfg.mag.field_gauss * std::sin(incl));
}

bool SimSensorSuite::due(uint64_t now_us, uint64_t& next_due_us, float rate_hz) {
    if (rate_hz <= 0.0f || now_us < next_due_us) {
        return false;
    }
    const uint64_t period = std::max<uint64_t>(1, static_cast<uint64_t>(std::llround(1.0e6 / rate_hz)));
    next_due_us += period;
    if (next_due_us <= now_us) {
        // Truth arrived late (or this is the first sample): resync rather
        // than burst out the missed samples from one truth value.
        next_due_us = now_us + period;
    }
    return true;
}

void SimSensorSuite::update(const SimTruth& t) {
    if (!m_have_time || t.time_us < m_last_time_us) {
        if (m_have_time) {
            init(m_cfg); // simulator restarted: start over from the seed
        }
        m_have_time = true;
        m_last_time_us = t.time_us;
        m_gps_common_last_us = t.time_us;
    }
    const float dt_s = static_cast<float>(t.time_us - m_last_time_us) * 1.0e-6f;
    m_last_time_us = t.time_us;

    step_vibration(t, dt_s);
    const float common_dt = static_cast<float>(t.time_us - m_gps_common_last_us) * 1.0e-6f;
    for (auto& e : m_gps_common) {
        e.step(common_dt, m_gps_common_rng);
    }
    m_gps_common_last_us = t.time_us;

    for (int i = 0; i < m_imu_count; ++i) {
        if (due(t.time_us, m_imu[i].next_due_us, m_cfg.imu.rate_hz)) {
            sample_imu(i, t);
        }
        if (m_imu[i].delay.pop_ready(t.time_us, m_imu[i].out)) {
            ++m_imu[i].seq;
        }
        if (due(t.time_us, m_baro[i].next_due_us, m_cfg.baro.rate_hz)) {
            sample_baro(i, t);
        }
        if (m_baro[i].delay.pop_ready(t.time_us, m_baro[i].out)) {
            ++m_baro[i].seq;
        }
        if (due(t.time_us, m_mag[i].next_due_us, m_cfg.mag.rate_hz)) {
            sample_mag(i, t);
        }
        if (m_mag[i].delay.pop_ready(t.time_us, m_mag[i].out)) {
            ++m_mag[i].seq;
        }
    }
    for (int i = 0; i < m_gps_count; ++i) {
        if (due(t.time_us, m_gps[i].next_due_us, m_cfg.gps.rate_hz)) {
            sample_gps(i, t);
        }
        if (m_gps[i].delay.pop_ready(t.time_us, m_gps[i].out)) {
            ++m_gps[i].seq;
        }
    }
}

void SimSensorSuite::step_vibration(const SimTruth& t, float dt_s) {
    const auto& c = m_cfg.imu;
    const int n = std::clamp(t.num_motors, 0, 8);
    const float per_motor = n > 0 ? 1.0f / std::sqrt(static_cast<float>(n)) : 0.0f;
    for (int a = 0; a < 3; ++a) {
        m_vib_accel[a] = 0.0f;
        m_vib_gyro[a] = 0.0f;
    }
    for (int k = 0; k < n; ++k) {
        const float thr = std::clamp(t.motor_throttle[k], 0.0f, 1.0f);
        const float rotor_hz = c.rotor_max_hz * std::sqrt(thr);
        m_rotor_phase[k] = std::fmod(m_rotor_phase[k] + kTwoPi * rotor_hz * dt_s, kTwoPi);
        // Imbalance force grows with rotor speed squared, i.e. ~throttle.
        const float level = thr * per_motor;
        for (int a = 0; a < 3; ++a) {
            const float ph = m_rotor_phase[k] + m_axis_phase[k][a];
            const float wave = std::sin(ph) +
                               0.5f * std::sin(static_cast<float>(c.blades) * m_rotor_phase[k] + 2.0f * m_axis_phase[k][a]);
            m_vib_accel[a] += c.vib_accel_amp * level * wave;
            m_vib_gyro[a] += c.vib_gyro_amp * level * wave;
        }
    }
}

void SimSensorSuite::sample_imu(int i, const SimTruth& t) {
    ImuState& s = m_imu[i];
    const auto& c = m_cfg.imu;
    const float dt_s = s.last_us > 0 && t.time_us > s.last_us
                           ? static_cast<float>(t.time_us - s.last_us) * 1.0e-6f : 0.0f;
    s.last_us = t.time_us;

    const float half_bw = 0.5f * c.rate_hz;
    const float gyro_sigma = c.gyro_noise_density * std::sqrt(half_bw);
    const float accel_sigma = c.accel_noise_density * std::sqrt(half_bw);
    const float sqrt_dt = std::sqrt(dt_s);

    apx::drivers::ImuData d{};
    bool gyro_clipped = false;
    bool accel_clipped = false;
    for (int a = 0; a < 3; ++a) {
        s.gyro_bias[a] += c.gyro_bias_walk * sqrt_dt * s.rng.gauss();
        s.accel_bias[a] += c.accel_bias_walk * sqrt_dt * s.rng.gauss();

        float g = static_cast<float>(t.gyro_body[a]) * (1.0f + s.gyro_scale[a]) + s.gyro_bias[a] +
                  s.mount_gain[a] * m_vib_gyro[a] + s.rng.gauss(gyro_sigma);
        float f = static_cast<float>(t.accel_body[a]) * (1.0f + s.accel_scale[a]) + s.accel_bias[a] +
                  s.mount_gain[a] * m_vib_accel[a] + s.rng.gauss(accel_sigma);
        d.gyro_xyz[a] = clip_quantise(g, c.gyro_range, c.gyro_lsb, gyro_clipped);
        d.accel_xyz[a] = clip_quantise(f, c.accel_range, c.accel_lsb, accel_clipped);
    }
    s.gyro_clips += gyro_clipped ? 1U : 0U;
    s.accel_clips += accel_clipped ? 1U : 0U;
    d.temp_c = c.temp_c;
    d.timestamp_us = t.time_us;
    s.delay.push(t.time_us + static_cast<uint64_t>(c.delay_s * 1.0e6f), d);
}

void SimSensorSuite::sample_baro(int i, const SimTruth& t) {
    BaroState& b = m_baro[i];
    const auto& c = m_cfg.baro;
    const float dt_s = b.last_us > 0 && t.time_us > b.last_us
                           ? static_cast<float>(t.time_us - b.last_us) * 1.0e-6f : 0.0f;
    b.last_us = t.time_us;

    const double alt_msl = static_cast<double>(m_cfg.home_alt_m) - t.pos_ned[2];
    const double p_true = pressure_from_alt(alt_msl);
    const double rho = 1.225 * p_true / kSeaLevelPa;
    const double q = 0.5 * rho * t.airspeed_ms * t.airspeed_ms;
    const double p = p_true + b.offset_pa + b.drift.step(dt_s, b.rng) + b.dyn_coef * q + b.rng.gauss(c.noise_pa);

    apx::drivers::BaroData d{};
    d.pressure_pa = static_cast<float>(p);
    d.temperature_c = c.temp_c;
    d.altitude_m = static_cast<float>(alt_from_pressure(p));
    d.timestamp_us = t.time_us;
    b.delay.push(t.time_us + static_cast<uint64_t>(c.delay_s * 1.0e6f), d);
}

void SimSensorSuite::sample_mag(int i, const SimTruth& t) {
    MagState& m = m_mag[i];
    const auto& c = m_cfg.mag;
    double R[9];
    quat_to_dcm(t.quat, R);

    float thr_sum = 0.0f;
    const int n = std::clamp(t.num_motors, 0, 8);
    for (int k = 0; k < n; ++k) {
        thr_sum += std::clamp(t.motor_throttle[k], 0.0f, 1.0f);
    }
    const float thr_mean = n > 0 ? thr_sum / static_cast<float>(n) : 0.0f;

    apx::drivers::MagData d{};
    for (int a = 0; a < 3; ++a) {
        // body = R^T * earth
        const double body = R[a] * m_mag_earth_ned[0] + R[3 + a] * m_mag_earth_ned[1] + R[6 + a] * m_mag_earth_ned[2];
        d.mag_xyz[a] = m.soft_scale[a] * static_cast<float>(body) + m.hard_iron[a] +
                       c.motor_interference_gauss * thr_mean * m.motor_dir[a] + m.rng.gauss(c.noise_gauss);
    }
    d.timestamp_us = t.time_us;
    m.delay.push(t.time_us + static_cast<uint64_t>(c.delay_s * 1.0e6f), d);
}

void SimSensorSuite::sample_gps(int i, const SimTruth& t) {
    GpsState& g = m_gps[i];
    const auto& c = m_cfg.gps;
    const float dt_s = g.last_us > 0 && t.time_us > g.last_us
                           ? static_cast<float>(t.time_us - g.last_us) * 1.0e-6f : 0.0f;
    g.last_us = t.time_us;

    // Antenna lever arm: position R*r, velocity R*(w x r).
    double R[9];
    quat_to_dcm(t.quat, R);
    const float* r = c.antenna_offset_m[i];
    const double wxr[3] = {
        t.gyro_body[1] * r[2] - t.gyro_body[2] * r[1],
        t.gyro_body[2] * r[0] - t.gyro_body[0] * r[2],
        t.gyro_body[0] * r[1] - t.gyro_body[1] * r[0],
    };

    double pos[3];
    double vel[3];
    for (int a = 0; a < 3; ++a) {
        const double lever = R[3 * a] * r[0] + R[3 * a + 1] * r[1] + R[3 * a + 2] * r[2];
        const double lever_vel = R[3 * a] * wxr[0] + R[3 * a + 1] * wxr[1] + R[3 * a + 2] * wxr[2];
        const float white = g.rng.gauss(a < 2 ? c.white_sigma_h_m : c.white_sigma_h_m * c.vertical_ratio);
        pos[a] = t.pos_ned[a] + lever + m_gps_common[a].value() + g.err[a].step(dt_s, g.rng) + white;
        vel[a] = t.vel_ned[a] + lever_vel + g.rng.gauss(a < 2 ? c.vel_sigma_h_ms : c.vel_sigma_v_ms);
    }

    const float sats = std::clamp(std::round(c.nominal_sats + g.sats.step(dt_s, g.rng)), 4.0f, 32.0f);

    bool in_outage = false;
    if (g.outage_left_s > 0.0f) {
        g.outage_left_s -= dt_s;
        in_outage = g.outage_left_s > 0.0f;
    } else if (c.outage_mtbf_s > 0.0f && dt_s > 0.0f &&
               g.rng.uniform(0.0f, 1.0f) < 1.0f - std::exp(-dt_s / c.outage_mtbf_s)) {
        g.outage_left_s = g.rng.uniform(c.outage_min_s, c.outage_max_s);
        in_outage = true;
    }

    apx::drivers::GpsData d{};
    const double lat0 = m_cfg.home_lat_deg * kDegToRad;
    d.lat_deg = m_cfg.home_lat_deg + (pos[0] / kEarthRadiusM) / kDegToRad;
    d.lon_deg = m_cfg.home_lon_deg + (pos[1] / (kEarthRadiusM * std::cos(lat0))) / kDegToRad;
    d.alt_msl_m = static_cast<float>(static_cast<double>(m_cfg.home_alt_m) - pos[2]);
    d.alt_ellipsoid_m = d.alt_msl_m; // geoid separation not modelled
    d.vel_n_ms = static_cast<float>(vel[0]);
    d.vel_e_ms = static_cast<float>(vel[1]);
    d.vel_d_ms = static_cast<float>(vel[2]);
    d.speed_ms = static_cast<float>(std::sqrt(vel[0] * vel[0] + vel[1] * vel[1]));
    d.satellites = static_cast<uint8_t>(sats);
    d.hdop = c.nominal_hdop * c.nominal_sats / sats;
    d.vdop = 1.4f * d.hdop;
    d.fix_type = apx::drivers::GpsFixType::FIX_3D;
    d.valid = true;
    if (in_outage) {
        d.fix_type = apx::drivers::GpsFixType::NO_FIX;
        d.valid = false;
        d.satellites = static_cast<uint8_t>(g.rng.uniform(0.0f, 4.0f));
        d.hdop = d.vdop = 99.99f;
    }
    d.timestamp_us = t.time_us;
    g.delay.push(t.time_us + static_cast<uint64_t>(c.delay_s * 1.0e6f), d);
}

} // namespace apx::sitl
