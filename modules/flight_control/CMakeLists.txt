# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
add_library(apx_flight_control STATIC
    rate_controller/pid_controller.cpp
    rate_controller/rate_controller.cpp
    attitude_controller/attitude_controller.cpp
    position_controller/position_controller.cpp
    mixer/motor_mixer.cpp
)
target_include_directories(apx_flight_control PUBLIC
    ${CMAKE_SOURCE_DIR}/modules/flight_control
    ${CMAKE_SOURCE_DIR}/kernel/include
    ${CMAKE_SOURCE_DIR}/modules/params
)
target_link_libraries(apx_flight_control PUBLIC apx_kernel apx_params)
