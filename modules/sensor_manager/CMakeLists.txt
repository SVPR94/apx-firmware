# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
add_library(apx_sensor_manager STATIC
    sensor_health.cpp
    sensor_voter.cpp
    sensor_manager.cpp
)
target_include_directories(apx_sensor_manager PUBLIC
    ${CMAKE_SOURCE_DIR}/modules/sensor_manager
    ${CMAKE_SOURCE_DIR}/drivers
    ${CMAKE_SOURCE_DIR}/drivers/include
    ${CMAKE_SOURCE_DIR}/kernel/include
)
target_link_libraries(apx_sensor_manager PUBLIC apx_kernel)
