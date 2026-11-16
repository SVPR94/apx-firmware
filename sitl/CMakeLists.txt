# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
cmake_minimum_required(VERSION 3.16)

add_subdirectory(sitl_bridge)

add_library(apx_sim_physics STATIC
    physics/multirotor_dynamics.cpp
)
target_include_directories(apx_sim_physics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Simulated sensors (IMU/baro/mag lanes + GNSS) generated from physics truth
# inside the SITL flight controller process.
add_library(apx_sim_sensors STATIC
    sensors/sim_sensors.cpp
    sensors/sim_sensor_log.cpp
)
target_include_directories(apx_sim_sensors PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Check for Gazebo Harmonic (gz-sim8) or Garden (gz-sim7)
find_package(gz-sim8 QUIET)
if(NOT gz-sim8_FOUND)
    find_package(gz-sim7 QUIET)
endif()

if(gz-sim8_FOUND OR gz-sim7_FOUND)
    add_subdirectory(gazebo_plugin)
    message(STATUS "Gazebo simulation plugin enabled")
else()
    message(STATUS "Gazebo (gz-sim) not found. Building standalone SITL with built-in 6DOF RK4 physics engine.")
endif()

add_executable(apx_sim_physics_runner
    physics/builtin_physics_runner.cpp
)
target_link_libraries(apx_sim_physics_runner PRIVATE
    apx_sim_physics
    sitl_bridge
)
