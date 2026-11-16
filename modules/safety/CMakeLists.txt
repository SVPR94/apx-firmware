# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
add_library(apx_safety STATIC
    failsafe.cpp
    fence.cpp
    preflight_check.cpp
    watchdog.cpp
)
target_include_directories(apx_safety PUBLIC
    ${CMAKE_SOURCE_DIR}/modules/safety
    ${CMAKE_SOURCE_DIR}/modules/params
    ${CMAKE_SOURCE_DIR}/kernel/include
)
target_link_libraries(apx_safety PUBLIC apx_kernel apx_params)
