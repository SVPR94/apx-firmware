# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
# APX portable flight firmware: sources and include paths, for any OS port.
#
#   include(${APX_ROOT}/firmware/sources.cmake)
#   target_sources(<tgt> PRIVATE ${APX_FIRMWARE_SOURCES})
#   target_include_directories(<tgt> PRIVATE ${APX_FIRMWARE_INCLUDE_DIRS})
#
# The OS port adds: kernel/ports/<os> (kernel/sources.cmake), its board
# implementation (apx/firmware/board.hpp) and its startup code.

get_filename_component(APX_ROOT ${CMAKE_CURRENT_LIST_DIR}/.. ABSOLUTE)

set(APX_FIRMWARE_INCLUDE_DIRS
    ${APX_ROOT}
    ${APX_ROOT}/firmware/include
    ${APX_ROOT}/kernel/include
    ${APX_ROOT}/hal/include
    ${APX_ROOT}/drivers
    ${APX_ROOT}/drivers/include
    ${APX_ROOT}/drivers/iomcu
    ${APX_ROOT}/modules
    ${APX_ROOT}/modules/params
    ${APX_ROOT}/modules/gcs
    ${APX_ROOT}/modules/gcs/mavlink
    ${APX_ROOT}/modules/gcs/mavlink/common
    ${APX_ROOT}/modules/modes
    ${APX_ROOT}/modules/flight_control
    ${APX_ROOT}/modules/safety
    ${APX_ROOT}/modules/estimator
    ${APX_ROOT}/modules/estimator/altitude_estimator
)

set(_apx_modes
    navigation_cascade path_tracker scurve sprayer land_detector crash_detector stick_arm_detector
    mode_stabilize mode_althold mode_guided mode_poshold mode_loiter mode_brake mode_auto
    mode_rtl mode_land mode_agri_spray mode_agri_zigzag mode_manager
)
list(TRANSFORM _apx_modes PREPEND ${APX_ROOT}/modules/modes/)
list(TRANSFORM _apx_modes APPEND .cpp)

set(APX_FIRMWARE_SOURCES
    ${APX_ROOT}/firmware/src/firmware.cpp
    ${APX_ROOT}/drivers/iomcu/io_coprocessor.cpp
    ${APX_ROOT}/modules/params/param_manager.cpp
    ${APX_ROOT}/modules/safety/fence.cpp
    ${APX_ROOT}/modules/gcs/mavlink_handler.cpp
    ${APX_ROOT}/modules/gcs/command_handler.cpp
    ${APX_ROOT}/modules/flight_control/rate_controller/pid_controller.cpp
    ${APX_ROOT}/modules/flight_control/rate_controller/rate_controller.cpp
    ${APX_ROOT}/modules/flight_control/attitude_controller/attitude_controller.cpp
    ${APX_ROOT}/modules/flight_control/position_controller/position_controller.cpp
    ${APX_ROOT}/modules/flight_control/mixer/motor_mixer.cpp
    ${APX_ROOT}/modules/estimator/altitude_estimator/altitude_estimator.cpp
    ${_apx_modes}
)

# Every microcontroller build: no hosted OS services (sockets, std::thread).
set(APX_FIRMWARE_DEFINITIONS APX_EMBEDDED)
