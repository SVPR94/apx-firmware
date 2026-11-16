# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
set(APX_BOARD_DIR ${CMAKE_CURRENT_SOURCE_DIR})

add_library(apx_board_cube_orange_plus INTERFACE)

target_include_directories(apx_board_cube_orange_plus INTERFACE
    ${APX_BOARD_DIR}
)

target_sources(apx_board_cube_orange_plus INTERFACE
    ${APX_BOARD_DIR}/startup_stm32h757.s
    ${APX_BOARD_DIR}/system_init.c
)

set(APX_BOARD_LINKER_SCRIPT ${APX_BOARD_DIR}/linker.ld CACHE INTERNAL
    "Linker script for cube_orange_plus")

message(STATUS "APX board: cube_orange_plus (STM32H757xx, flash: 2048KB, RAM: 1024KB, 3 sensor lanes)")

add_executable(apx_firmware_cube_orange_plus
    ${APX_BOARD_DIR}/apx_main_stm32h7.cpp
)
target_link_libraries(apx_firmware_cube_orange_plus PRIVATE
    apx_board_cube_orange_plus
    apx_kernel
    apx_params
    apx_flight_control
)
target_link_options(apx_firmware_cube_orange_plus PRIVATE
    -T${APX_BOARD_LINKER_SCRIPT}
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/apx_firmware_cube_orange_plus.map
)
set_target_properties(apx_firmware_cube_orange_plus PROPERTIES
    SUFFIX ".elf"
)

set(APX_BIN ${CMAKE_CURRENT_BINARY_DIR}/apx_firmware_cube_orange_plus.bin)
set(APX_HEX ${CMAKE_CURRENT_BINARY_DIR}/apx_firmware_cube_orange_plus.hex)
set(APX_APJ ${CMAKE_CURRENT_BINARY_DIR}/apx_cube_orange_plus.apj)

add_custom_command(TARGET apx_firmware_cube_orange_plus POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:apx_firmware_cube_orange_plus>
            ${APX_BIN}
    COMMAND ${CMAKE_OBJCOPY} -O ihex
            $<TARGET_FILE:apx_firmware_cube_orange_plus>
            ${APX_HEX}
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:apx_firmware_cube_orange_plus>
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/make_apj.py
            --bin ${APX_BIN}
            --board-id 1063
            --summary "APX Cube Orange+ Baremetal"
            --out ${APX_APJ}
    COMMENT "cube_orange_plus: generating .bin, .hex, and .apj firmware images"
)
