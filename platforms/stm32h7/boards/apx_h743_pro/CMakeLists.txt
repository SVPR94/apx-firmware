# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
set(APX_BOARD_DIR ${CMAKE_CURRENT_SOURCE_DIR})

add_library(apx_board_apx_h743_pro INTERFACE)

target_include_directories(apx_board_apx_h743_pro INTERFACE
    ${APX_BOARD_DIR}
)

target_sources(apx_board_apx_h743_pro INTERFACE
    ${APX_BOARD_DIR}/startup_stm32h743.s
    ${APX_BOARD_DIR}/system_init.c
)

set(APX_BOARD_LINKER_SCRIPT ${APX_BOARD_DIR}/linker.ld CACHE INTERNAL
    "Linker script for apx_h743_pro")

message(STATUS "APX board: apx_h743_pro (STM32H743xx, flash: 2048KB, RAM: 1024KB)")

add_executable(apx_firmware_apx_h743_pro
    ${APX_BOARD_DIR}/apx_main_stm32h7.cpp
)
target_link_libraries(apx_firmware_apx_h743_pro PRIVATE
    apx_board_apx_h743_pro
    apx_kernel
    apx_params
    apx_flight_control
    apx_eskf15
)
target_link_options(apx_firmware_apx_h743_pro PRIVATE
    -T${APX_BOARD_LINKER_SCRIPT}
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/apx_firmware_apx_h743_pro.map
)
set_target_properties(apx_firmware_apx_h743_pro PROPERTIES
    SUFFIX ".elf"
)

add_custom_command(TARGET apx_firmware_apx_h743_pro POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:apx_firmware_apx_h743_pro>
            ${CMAKE_CURRENT_BINARY_DIR}/apx_firmware_apx_h743_pro.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex
            $<TARGET_FILE:apx_firmware_apx_h743_pro>
            ${CMAKE_CURRENT_BINARY_DIR}/apx_firmware_apx_h743_pro.hex
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:apx_firmware_apx_h743_pro>
    COMMENT "apx_h743_pro: generating .bin / .hex firmware images"
)
