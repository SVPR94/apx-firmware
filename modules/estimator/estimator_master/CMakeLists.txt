# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
add_library(apx_estimator_master STATIC
    sensor_crossbar.cpp
    estimator_arbiter.cpp
)

target_include_directories(apx_estimator_master PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../ekf3
    ${CMAKE_CURRENT_SOURCE_DIR}/../ukf
    ${CMAKE_CURRENT_SOURCE_DIR}/../attitude_estimator
    ${CMAKE_SOURCE_DIR}/kernel/include
    ${CMAKE_SOURCE_DIR}/hal/include
)

target_link_libraries(apx_estimator_master PUBLIC
    apx_estimator_ekf3
    apx_estimator_ukf
    apx_attitude_estimator
    apx_kernel
    apx_hal
)
