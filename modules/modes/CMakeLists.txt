# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
add_library(apx_flight_modes STATIC
    navigation_cascade.cpp
    path_tracker.cpp
    scurve.cpp
    sprayer.cpp
    land_detector.cpp
    crash_detector.cpp
    stick_arm_detector.cpp
    mode_stabilize.cpp
    mode_althold.cpp
    mode_guided.cpp
    mode_poshold.cpp
    mode_loiter.cpp
    mode_brake.cpp
    mode_auto.cpp
    mode_rtl.cpp
    mode_land.cpp
    mode_agri_spray.cpp
    mode_agri_zigzag.cpp
    mode_manager.cpp
)
target_include_directories(apx_flight_modes PUBLIC
    ${CMAKE_SOURCE_DIR}/modules/modes
    ${CMAKE_SOURCE_DIR}/modules/flight_control
    ${CMAKE_SOURCE_DIR}/modules/radar
    ${CMAKE_SOURCE_DIR}/kernel/include
)
target_link_libraries(apx_flight_modes PUBLIC apx_kernel apx_flight_control apx_params apx_radar)
