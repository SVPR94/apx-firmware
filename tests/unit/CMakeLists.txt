# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
cmake_minimum_required(VERSION 3.14)
project(ApxUnitTests)

add_executable(run_unit_tests
    test_pid_controller.cpp
    test_sensor_voter.cpp
    test_motor_mixer.cpp
    test_complementary_filter.cpp
    test_path_tracker.cpp
    test_mode_brake_sprayer.cpp
    test_mode_auto_condition_yaw.cpp
    test_noise_filters.cpp
    test_param_manager.cpp
    test_altitude_estimator.cpp
    test_fence.cpp
    test_radar_terrain.cpp
    test_set_home.cpp
)

target_include_directories(run_unit_tests PRIVATE
    ${CMAKE_SOURCE_DIR}/modules/flight_control
    ${CMAKE_SOURCE_DIR}/modules/modes
    ${CMAKE_SOURCE_DIR}/modules/sensor_manager
    ${CMAKE_SOURCE_DIR}/modules/estimator/attitude_estimator
    ${CMAKE_SOURCE_DIR}/modules/estimator
    ${CMAKE_SOURCE_DIR}/modules/params
    ${CMAKE_SOURCE_DIR}/modules/safety
    ${CMAKE_SOURCE_DIR}/modules/radar
    ${CMAKE_SOURCE_DIR}/modules/estimator/altitude_estimator
)

if(TARGET GTest::gtest_main)
    set(GTEST_TARGET GTest::gtest_main)
elseif(TARGET gtest_main)
    set(GTEST_TARGET gtest_main)
else()
    set(GTEST_TARGET ${GTEST_LIBRARIES} pthread)
endif()

target_link_libraries(run_unit_tests PRIVATE
    ${GTEST_TARGET}
    apx_flight_control
    apx_flight_modes
    apx_sensor_manager
    apx_attitude_estimator
    apx_kernel
    apx_hal
    apx_params
    apx_safety
    apx_radar
    apx_altitude_estimator
    apx_gcs
    pthread
)

# Airframe cross-check and sensor/wind models need the SITL libraries (APX_BUILD_SITL=ON).
if(TARGET apx_sim_physics)
    target_sources(run_unit_tests PRIVATE test_sim_airframe_geometry.cpp test_sim_sensors.cpp)
    target_link_libraries(run_unit_tests PRIVATE apx_sim_physics apx_sim_sensors)
endif()

add_test(NAME UnitTests COMMAND run_unit_tests)
