# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
# APX bootloader - Zephyr port. Built together with the flight firmware by
# sysbuild (platforms/zephyr/app/sysbuild.cmake); not meant to be built on
# its own. Speaks the serial upload protocol in bootloader/ (portable core).

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(apx_bootloader LANGUAGES C CXX)

get_filename_component(APX_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../.. ABSOLUTE)
include(${APX_ROOT}/bootloader/sources.cmake)

execute_process(COMMAND git -C ${APX_ROOT} rev-parse --short HEAD
                OUTPUT_VARIABLE APX_GIT OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
if(NOT APX_GIT)
    set(APX_GIT "unknown")
endif()

target_sources(app PRIVATE
    src/main.cpp
    src/boot_jump.cpp
    src/port_zephyr.cpp
    ${APX_ROOT}/hal/ports/zephyr/uart_zephyr.cpp
    ${APX_BOOTLOADER_SOURCES}
)
target_include_directories(app PRIVATE
    ${APX_BOOTLOADER_INCLUDE_DIR}
    ${APX_ROOT}/hal/include
    ${APX_ROOT}/hal/ports/zephyr
)
target_compile_definitions(app PRIVATE APX_BL_VERSION="1.0-${APX_GIT}")

# Memory report (last step): must fit the bootloader partition.
add_custom_target(apx_bootloader_memory_report ALL
    COMMAND ${PYTHON_EXECUTABLE} ${APX_ROOT}/tools/fw_memory_report.py
            --elf ${CMAKE_BINARY_DIR}/zephyr/zephyr.elf
            --map ${CMAKE_BINARY_DIR}/zephyr/zephyr.map
            --bin ${CMAKE_BINARY_DIR}/zephyr/zephyr.bin
            --name "${BOARD} bootloader (Zephyr)"
            --top 5
            --out-txt ${CMAKE_BINARY_DIR}/zephyr/apx_memory_report.txt
            --out-json ${CMAKE_BINARY_DIR}/zephyr/apx_memory_report.json
    COMMENT "APX bootloader: memory analysis"
    VERBATIM
)
add_dependencies(apx_bootloader_memory_report zephyr_final)
