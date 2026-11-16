# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
#   apx_firmware_apx_h743_v1  - the actual firmware .elf (see
#                                apx_main_stm32h7.cpp's file header for
#                                exactly what this does and doesn't do
#                                yet - real flight-control math linked in,
#                                no HAL backend/real I/O behind it).

set(APX_BOARD_DIR ${CMAKE_CURRENT_SOURCE_DIR})

add_library(apx_board_apx_h743_v1 INTERFACE)

target_include_directories(apx_board_apx_h743_v1 INTERFACE
    ${APX_BOARD_DIR}
)

target_sources(apx_board_apx_h743_v1 INTERFACE
    ${APX_BOARD_DIR}/startup_stm32h743.s
    ${APX_BOARD_DIR}/system_init.c
)

# Consumers link against this and add:
#   target_link_options(<exe> PRIVATE -T ${APX_BOARD_DIR}/linker.ld)
# CMake has no first-class "linker script" target property, so the path
# is exposed as a variable for the consumer to use explicitly.
set(APX_BOARD_LINKER_SCRIPT ${APX_BOARD_DIR}/linker.ld CACHE INTERNAL
    "Linker script for the selected STM32H7 board")

message(STATUS "APX board: apx_h743_v1 (STM32H743, linker script: ${APX_BOARD_LINKER_SCRIPT})")

# --- Firmware executable -------------------------------------------------
# Only apx_kernel/apx_params/apx_flight_control are linked in - those are
# the libraries confirmed to have no hosted-OS dependency (no pthread, no
# sockets - see platforms/stm32h7/README.md's "what's real vs. still
# needed"). apx_hal/apx_drivers/apx_gcs/apx_safety/apx_vehicles/
# apx_rc_manager/apx_flight_modes are deliberately NOT linked here yet:
# they either need a real HAL backend behind them (hal/stm32h7/ is still
# empty) or pull in main/apx_main.cpp-style hosted-OS assumptions
# transitively. Widening this list is the natural next step as each of
# those gets a bare-metal-compatible path.
add_executable(apx_firmware_apx_h743_v1
    ${APX_BOARD_DIR}/apx_main_stm32h7.cpp
)
target_link_libraries(apx_firmware_apx_h743_v1 PRIVATE
    apx_board_apx_h743_v1
    apx_kernel
    apx_params
    apx_flight_control
    apx_ekf3
    apx_eskf15
    apx_altitude_estimator
    apx_attitude_estimator
    apx_flight_modes
    apx_sensor_manager
    apx_safety
    apx_calibration
    apx_rc_manager
    apx_vehicles
)
target_link_options(apx_firmware_apx_h743_v1 PRIVATE
    -T${APX_BOARD_LINKER_SCRIPT}
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/apx_firmware_apx_h743_v1.map
)
set_target_properties(apx_firmware_apx_h743_v1 PROPERTIES
    SUFFIX ".elf"
)

# .bin and .hex for flashing - real tooling from the toolchain install.
add_custom_command(TARGET apx_firmware_apx_h743_v1 POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:apx_firmware_apx_h743_v1>
            ${CMAKE_CURRENT_BINARY_DIR}/apx_firmware_apx_h743_v1.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex
            $<TARGET_FILE:apx_firmware_apx_h743_v1>
            ${CMAKE_CURRENT_BINARY_DIR}/apx_firmware_apx_h743_v1.hex
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:apx_firmware_apx_h743_v1>
    COMMENT "apx_h743_v1: generating .bin / .hex and reporting section sizes"
)
