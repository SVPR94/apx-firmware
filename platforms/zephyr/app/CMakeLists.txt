# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
cmake_minimum_required(VERSION 3.20.0)

# Out-of-tree boards (platforms/zephyr/boards/<vendor>/<board>)
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR}/..)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(apx_firmware LANGUAGES C CXX)

get_filename_component(APX_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../.. ABSOLUTE)

# APX_PLATFORM_ZEPHYR selects the Zephyr kernel shims (kernel/src);
# APX_PLATFORM_STM32H7 marks a non-hosted build (no sockets/threads).
target_compile_definitions(app PRIVATE
    APX_PLATFORM_ZEPHYR
    APX_PLATFORM_STM32H7
    APX_BOARD_STM32H7
)

target_include_directories(app PRIVATE
    ${APX_ROOT}
    ${APX_ROOT}/kernel/include
    ${APX_ROOT}/hal/include
    ${APX_ROOT}/drivers
    ${APX_ROOT}/drivers/include
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

set(APX_MODES
    navigation_cascade path_tracker scurve sprayer land_detector crash_detector stick_arm_detector
    mode_stabilize mode_althold mode_guided mode_poshold mode_loiter mode_brake mode_auto
    mode_rtl mode_land mode_agri_spray mode_agri_zigzag mode_manager
)
list(TRANSFORM APX_MODES PREPEND ${APX_ROOT}/modules/modes/)
list(TRANSFORM APX_MODES APPEND .cpp)

target_sources(app PRIVATE
    src/main.cpp
    ${APX_ROOT}/kernel/src/timer.cpp
    ${APX_ROOT}/kernel/src/mutex.cpp
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
    ${APX_MODES}
)

# MAVLink's generated headers take the address of packed members.
target_compile_options(app PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-Wno-address-of-packed-member>)

# Bootloader package: .apj for upload through the board's stock
# bootloader (APX Flight Manager / GCS "custom firmware").
set(APX_APJ_BOARD_ID 1063) # CubeOrange+ board id
# (An ordinary target: Zephyr reads extra_post_build_commands inside
# find_package(), before this file could append to it.)
set(APX_APJ ${CMAKE_BINARY_DIR}/zephyr/apx_cube_orange_plus.apj)
add_custom_command(
    OUTPUT ${APX_APJ}
    COMMAND ${PYTHON_EXECUTABLE} ${APX_ROOT}/tools/make_apj.py
            --bin ${CMAKE_BINARY_DIR}/zephyr/zephyr.bin
            --board-id ${APX_APJ_BOARD_ID}
            --summary "APX ${BOARD}"
            --flash-total 917504
            --out ${APX_APJ}
    DEPENDS zephyr_final ${CMAKE_BINARY_DIR}/zephyr/zephyr.bin ${APX_ROOT}/tools/make_apj.py
    COMMENT "APX: packaging .apj for the bootloader"
    VERBATIM
)
add_custom_target(apx_apj ALL DEPENDS ${APX_APJ})
