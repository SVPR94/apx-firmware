# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
add_library(apx_gcs STATIC
    mavlink_handler.cpp
    command_handler.cpp
)
target_include_directories(apx_gcs PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/mavlink
    ${CMAKE_CURRENT_SOURCE_DIR}/mavlink/common
    ${CMAKE_SOURCE_DIR}/kernel/include
    ${CMAKE_SOURCE_DIR}/hal/include
    ${CMAKE_SOURCE_DIR}/modules
    ${CMAKE_SOURCE_DIR}/modules/safety
    ${CMAKE_SOURCE_DIR}/modules/radar
)
target_link_libraries(apx_gcs PUBLIC apx_kernel apx_hal apx_flight_modes apx_params apx_safety apx_radar)
