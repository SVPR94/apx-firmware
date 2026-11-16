# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
# Sysbuild: one `west build --sysbuild` builds the flight firmware (this
# app) and, with SB_CONFIG_APX_BOOTLOADER, the APX bootloader for the same
# board, then packages everything into <build>/firmware/.

get_filename_component(APX_ROOT ${APP_DIR}/../../.. ABSOLUTE)
set(APX_PACKAGE_ARGS)

if(SB_CONFIG_APX_BOOTLOADER)
    ExternalZephyrProject_Add(
        APPLICATION apx_bootloader
        SOURCE_DIR ${APX_ROOT}/platforms/zephyr/bootloader
    )
    # `west flash` writes the bootloader first, then the firmware.
    sysbuild_add_dependencies(FLASH ${DEFAULT_IMAGE} apx_bootloader)
    set(APX_PACKAGE_ARGS --bootloader-dir ${CMAKE_BINARY_DIR}/apx_bootloader/zephyr)
endif()

find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_custom_target(apx_firmware_package ALL
    COMMAND ${Python3_EXECUTABLE} ${APX_ROOT}/tools/make_firmware_package.py
            --board ${BOARD}
            --app-dir ${CMAKE_BINARY_DIR}/${DEFAULT_IMAGE}/zephyr
            ${APX_PACKAGE_ARGS}
            --out ${CMAKE_BINARY_DIR}/firmware
    COMMENT "APX: packaging firmware images into ${CMAKE_BINARY_DIR}/firmware"
    VERBATIM
)
add_dependencies(apx_firmware_package ${DEFAULT_IMAGE})
if(SB_CONFIG_APX_BOOTLOADER)
    add_dependencies(apx_firmware_package apx_bootloader)
endif()
