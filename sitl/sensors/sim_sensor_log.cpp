// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "sim_sensor_log.hpp"

#include <cinttypes>

namespace apx::sitl {

bool SimSensorLog::open(const char* path) {
    close();
    m_file = std::fopen(path, "w");
    if (m_file == nullptr) {
        return false;
    }
    std::setvbuf(m_file, nullptr, _IOFBF, 1 << 20);
    std::fprintf(m_file, "# APX SITL sensor log, see sitl/sensors/sim_sensor_log.hpp for columns\n");
    return true;
}

void SimSensorLog::close() {
    if (m_file != nullptr) {
        std::fclose(m_file);
        m_file = nullptr;
    }
}

void SimSensorLog::log_truth(const SimTruth& t) {
    if (m_file == nullptr) {
        return;
    }
    std::fprintf(m_file,
                 "%" PRIu64 ",T,0,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f,"
                 "%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%.3f\n",
                 t.time_us, t.pos_ned[0], t.pos_ned[1], t.pos_ned[2], t.vel_ned[0], t.vel_ned[1], t.vel_ned[2],
                 t.quat[0], t.quat[1], t.quat[2], t.quat[3], t.gyro_body[0], t.gyro_body[1], t.gyro_body[2],
                 t.accel_body[0], t.accel_body[1], t.accel_body[2], t.airspeed_ms);
}

void SimSensorLog::log_new_samples(uint64_t now_us, const SimSensorSuite& suite) {
    if (m_file == nullptr) {
        return;
    }
    for (int i = 0; i < suite.imu_count(); ++i) {
        if (suite.imu_seq(i) != m_imu_seq[i]) {
            m_imu_seq[i] = suite.imu_seq(i);
            const auto& d = suite.imu(i);
            std::fprintf(m_file, "%" PRIu64 ",I,%d,%" PRIu64 ",%.5f,%.5f,%.5f,%.6f,%.6f,%.6f,%.1f,%u,%u\n",
                         now_us, i, d.timestamp_us, d.accel_xyz[0], d.accel_xyz[1], d.accel_xyz[2],
                         d.gyro_xyz[0], d.gyro_xyz[1], d.gyro_xyz[2], d.temp_c,
                         suite.accel_clip_count(i), suite.gyro_clip_count(i));
        }
        if (suite.baro_seq(i) != m_baro_seq[i]) {
            m_baro_seq[i] = suite.baro_seq(i);
            const auto& d = suite.baro(i);
            std::fprintf(m_file, "%" PRIu64 ",B,%d,%" PRIu64 ",%.2f,%.3f,%.1f\n",
                         now_us, i, d.timestamp_us, d.pressure_pa, d.altitude_m, d.temperature_c);
        }
        if (suite.mag_seq(i) != m_mag_seq[i]) {
            m_mag_seq[i] = suite.mag_seq(i);
            const auto& d = suite.mag(i);
            std::fprintf(m_file, "%" PRIu64 ",M,%d,%" PRIu64 ",%.5f,%.5f,%.5f\n",
                         now_us, i, d.timestamp_us, d.mag_xyz[0], d.mag_xyz[1], d.mag_xyz[2]);
        }
    }
    for (int i = 0; i < suite.gps_count(); ++i) {
        if (suite.gps_seq(i) != m_gps_seq[i]) {
            m_gps_seq[i] = suite.gps_seq(i);
            const auto& d = suite.gps(i);
            std::fprintf(m_file, "%" PRIu64 ",G,%d,%" PRIu64 ",%d,%.8f,%.8f,%.3f,%.3f,%.3f,%.3f,%u,%.2f\n",
                         now_us, i, d.timestamp_us, d.valid ? 1 : 0, d.lat_deg, d.lon_deg, d.alt_msl_m,
                         d.vel_n_ms, d.vel_e_ms, d.vel_d_ms, static_cast<unsigned>(d.satellites), d.hdop);
        }
    }
}

} // namespace apx::sitl
