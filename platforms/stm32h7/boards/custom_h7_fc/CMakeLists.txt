# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
set(APX_BOARD_DIR ${CMAKE_CURRENT_SOURCE_DIR})

add_library(apx_board_custom_h7_fc INTERFACE)

target_include_directories(apx_board_custom_h7_fc INTERFACE
    ${APX_BOARD_DIR}
)

target_sources(apx_board_custom_h7_fc INTERFACE
    ${APX_BOARD_DIR}/startup_stm32h743.s
    ${APX_BOARD_DIR}/system_init.c
)

set(APX_BOARD_LINKER_SCRIPT ${APX_BOARD_DIR}/linker.ld CACHE INTERNAL
    "Linker script for custom_h7_fc")

message(STATUS "APX board: custom_h7_fc (STM32H743xx, flash: 2048KB, RAM: 1024KB)")

add_executable(apx_firmware_custom_h7_fc
    ${APX_BOARD_DIR}/apx_main_stm32h7.cpp
)
target_link_libraries(apx_firmware_custom_h7_fc PRIVATE
    apx_board_custom_h7_fc
    apx_kernel
    apx_params
    apx_flight_control
    apx_eskf15
)
target_link_options(apx_firmware_custom_h7_fc PRIVATE
    -T${APX_BOARD_LINKER_SCRIPT}
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/apx_firmware_custom_h7_fc.map
)
set_target_properties(apx_firmware_custom_h7_fc PROPERTIES
    SUFFIX ".elf"
)

add_custom_command(TARGET apx_firmware_custom_h7_fc POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:apx_firmware_custom_h7_fc>
            ${CMAKE_CURRENT_BINARY_DIR}/apx_firmware_custom_h7_fc.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex
            $<TARGET_FILE:apx_firmware_custom_h7_fc>
            ${CMAKE_CURRENT_BINARY_DIR}/apx_firmware_custom_h7_fc.hex
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:apx_firmware_custom_h7_fc>
    COMMENT "custom_h7_fc: generating .bin / .hex firmware images"
)
