// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
//   time_us,B,lane,sample_us,pressure_pa,alt_m,temp
//   time_us,M,lane,sample_us,mx,my,mz
//   time_us,G,rx,sample_us,fix3d,lat,lon,alt_msl,vn,ve,vd,sats,hdop
// time_us is simulation time when the row was written (when the flight
// code first sees the sample); sample_us is the sample's time of validity,
// so time_us - sample_us is the modelled latency.
#pragma once

#include "sim_sensors.hpp"

#include <cstdio>

namespace apx::sitl {

class SimSensorLog {
public:
    SimSensorLog() = default;
    SimSensorLog(const SimSensorLog&) = delete;
    SimSensorLog& operator=(const SimSensorLog&) = delete;
    ~SimSensorLog() { close(); }

    bool open(const char* path);
    void close();
    bool is_open() const { return m_file != nullptr; }

    void log_truth(const SimTruth& t);
    // Writes every sample released since the previous call.
    void log_new_samples(uint64_t now_us, const SimSensorSuite& suite);

private:
    std::FILE* m_file{nullptr};
    uint32_t m_imu_seq[SimSensorSuite::kMaxImu]{};
    uint32_t m_baro_seq[SimSensorSuite::kMaxImu]{};
    uint32_t m_mag_seq[SimSensorSuite::kMaxImu]{};
    uint32_t m_gps_seq[SimSensorSuite::kMaxGps]{};
};

} // namespace apx::sitl
