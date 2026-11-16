# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
cmake_minimum_required(VERSION 3.22)

# If configuring for stm32h7 and toolchain is not specified, default to the arm-none-eabi toolchain
if(DEFINED APX_BOARD AND APX_BOARD STREQUAL "stm32h7" AND NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/platforms/stm32h7/toolchain-arm-none-eabi.cmake" CACHE FILEPATH "Default ARM toolchain for STM32H7")
endif()

# ASM is only actually needed for platforms/stm32h7's startup_stm32h743.s
# (vector table + Reset_Handler) - SITL/Linux never compiles an .s file -
# but it must be enabled as a project language here regardless, or CMake
# has no rule for the .s extension at all and silently drops that source
# from the build instead of erroring: an earlier stm32h7 build attempt
# produced a "successfully" linked .elf missing its entire vector table
# and Reset_Handler (ld warned "cannot find entry symbol Reset_Handler"
# and the image was implausibly tiny) precisely because of this - not a
# linker-script problem, a missing-language one.
project(APX
    VERSION 0.1.0
    DESCRIPTION "Aerospace-Grade Modular Flight Controller & SITL Simulation Engine"
    LANGUAGES C CXX ASM
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to SITL if not specified
if(NOT DEFINED APX_BOARD)
    set(APX_BOARD "sitl" CACHE STRING "Target board: sitl | linux | stm32h7")
endif()

# The SITL bridge/physics use Linux sockets, so they can't build for a
# bare-metal target: default OFF there (an explicit -DAPX_BUILD_SITL=ON
# still overrides).
if(APX_BOARD STREQUAL "stm32h7")
    option(APX_BUILD_SITL "Build SITL simulation bridge and virtual sensors" OFF)
else()
    option(APX_BUILD_SITL "Build SITL simulation bridge and virtual sensors" ON)
endif()
option(APX_BUILD_TESTS "Build unit tests" OFF)

if(APX_BOARD STREQUAL "sitl")
    add_compile_definitions(APX_PLATFORM_LINUX APX_SITL APX_BOARD_SITL)
elseif(APX_BOARD STREQUAL "linux")
    add_compile_definitions(APX_PLATFORM_LINUX APX_BOARD_LINUX)
elseif(APX_BOARD STREQUAL "stm32h7")
    add_compile_definitions(APX_PLATFORM_STM32H7 APX_BOARD_STM32H7)

    # Which board under platforms/stm32h7/boards/ to build; see
    # platforms/stm32h7/README.md for how to add a new one.
    if(NOT DEFINED APX_BOARD_TARGET)
        set(APX_BOARD_TARGET "apx_h743_v1" CACHE STRING
            "STM32H7 board variant (directory name under platforms/stm32h7/boards/)")
    endif()

    # Routes into the STM32H7 toolchain/linker/startup/pin-map scaffold.
    # Added early (before the hosted-OS-assuming core subsystems below)
    # so it configures independently of them. Note: configuring the rest
    # of this file for APX_BOARD=stm32h7 is not yet expected to succeed
    # end-to-end -- kernel/CMakeLists.txt's find_package(Threads REQUIRED)
    # has no bare-metal branch yet. See platforms/stm32h7/README.md
    # "What's real vs. still needed".
    add_subdirectory(platforms/stm32h7)
endif()

# Global include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/..
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/drivers
    ${CMAKE_SOURCE_DIR}/kernel/include
    ${CMAKE_SOURCE_DIR}/hal/include
    ${CMAKE_SOURCE_DIR}/drivers/include
    ${CMAKE_SOURCE_DIR}/modules
    ${CMAKE_SOURCE_DIR}/modules/estimator
    ${CMAKE_SOURCE_DIR}/modules/estimator/eskf15
    ${CMAKE_SOURCE_DIR}/modules/flight_control
    ${CMAKE_SOURCE_DIR}/modules/modes
    ${CMAKE_SOURCE_DIR}/modules/gcs
    ${CMAKE_SOURCE_DIR}/modules/safety
    ${CMAKE_SOURCE_DIR}/modules/sensor_manager
    ${CMAKE_SOURCE_DIR}/modules/calibration
    ${CMAKE_SOURCE_DIR}/modules/rc_manager
    ${CMAKE_SOURCE_DIR}/sitl
)

# Core subsystems
add_subdirectory(kernel)
add_subdirectory(hal)
add_subdirectory(drivers)
add_subdirectory(modules)
add_subdirectory(vehicles)

if(APX_BUILD_SITL)
    add_subdirectory(sitl)
endif()

add_subdirectory(main)

if(APX_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
