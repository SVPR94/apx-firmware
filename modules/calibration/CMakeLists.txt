# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
add_library(apx_calibration STATIC
    cal_accel.cpp
    cal_gyro.cpp
    cal_mag.cpp
    cal_esc.cpp
    cal_radio.cpp
)
target_include_directories(apx_calibration PUBLIC
    ${CMAKE_SOURCE_DIR}/modules/calibration
    ${CMAKE_SOURCE_DIR}/modules/sensor_manager
    ${CMAKE_SOURCE_DIR}/kernel/include
)
target_link_libraries(apx_calibration PUBLIC apx_kernel apx_sensor_manager)
