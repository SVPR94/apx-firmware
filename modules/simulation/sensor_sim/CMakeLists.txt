# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
add_library(apx_sim_sensors STATIC
    imu_sim.cpp
    baro_sim.cpp
    gps_sim.cpp
)
target_include_directories(apx_sim_sensors PUBLIC
    ${CMAKE_SOURCE_DIR}/modules/simulation/sensor_sim
    ${CMAKE_SOURCE_DIR}/modules
)
