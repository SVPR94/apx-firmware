# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
add_library(apx_vehicles STATIC
    multirotor/quad_x.cpp
    multirotor/hex_x.cpp
    multirotor/octo_x.cpp
)
target_include_directories(apx_vehicles PUBLIC
    ${CMAKE_SOURCE_DIR}/vehicles
    ${CMAKE_SOURCE_DIR}/modules/flight_control
    ${CMAKE_SOURCE_DIR}/kernel/include
)
target_link_libraries(apx_vehicles PUBLIC apx_kernel apx_flight_control)
