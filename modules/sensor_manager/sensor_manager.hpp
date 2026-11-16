// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once
#include "sensor_health.hpp"
#include "sensor_voter.hpp"
#include "imu/icm42688/icm42688.hpp"
#include "baro/ms5611/ms5611.hpp"
#include "mag/ist8310/ist8310.hpp"
#include "gps/ublox_f9p/ublox_f9p.hpp"
#include "apx/kernel/ubus.hpp"
#include <array>
#include <cstdint>

namespace apx::sensor {

class SensorManager {
public:
    static constexpr int kMaxSensors = 4;

    static SensorManager& get_instance() {
        static SensorManager inst;
        return inst;
    }

    void register_imu(int idx, apx::drivers::DriverBase* drv);
    void register_baro(int idx, apx::drivers::DriverBase* drv);
    void register_mag(int idx, apx::drivers::DriverBase* drv);
    void register_gps(int idx, apx::drivers::DriverBase* drv);

    void update();  // call at 1kHz

    const apx::drivers::ImuData&  get_primary_imu()  const;
    const apx::drivers::BaroData& get_primary_baro() const;
    const apx::drivers::MagData&  get_primary_mag()  const;
    const apx::drivers::GpsData&  get_primary_gps()  const;

    int get_active_imu_index()  const { return m_voter_imu.get_winner_index(); }
    int get_active_baro_index() const { return m_voter_baro.get_winner_index(); }
    int get_active_mag_index()  const { return m_voter_mag.get_winner_index(); }
    int get_active_gps_index()  const { return m_voter_gps.get_winner_index(); }

private:
    SensorManager() = default;

    void update_imu();
    void update_baro();
    void update_mag();
    void update_gps();

    std::array<apx::drivers::DriverBase*, kMaxSensors> m_imu_drivers{};
    std::array<apx::drivers::DriverBase*, kMaxSensors> m_baro_drivers{};
    std::array<apx::drivers::DriverBase*, kMaxSensors> m_mag_drivers{};
    std::array<apx::drivers::DriverBase*, kMaxSensors> m_gps_drivers{};
    std::array<SensorHealth, kMaxSensors>              m_imu_health{};
    std::array<SensorHealth, kMaxSensors>              m_baro_health{};
    std::array<SensorHealth, kMaxSensors>              m_mag_health{};
    std::array<SensorHealth, kMaxSensors>              m_gps_health{};

    SensorVoter<apx::drivers::ImuData,  kMaxSensors> m_voter_imu;
    SensorVoter<apx::drivers::BaroData, kMaxSensors> m_voter_baro;
    SensorVoter<apx::drivers::MagData,  kMaxSensors> m_voter_mag;
    SensorVoter<apx::drivers::GpsData,  kMaxSensors> m_voter_gps;

    apx::drivers::ImuData  m_imu_fallback{};
    apx::drivers::BaroData m_baro_fallback{};
    apx::drivers::MagData  m_mag_fallback{};
    apx::drivers::GpsData  m_gps_fallback{};
};

} // namespace apx::sensor
