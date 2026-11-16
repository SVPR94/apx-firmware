# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
add_library(apx_ekf3 STATIC
    ekf3.cpp
)
target_include_directories(apx_ekf3 PUBLIC
    ${CMAKE_SOURCE_DIR}/modules/estimator/ekf3
    ${CMAKE_SOURCE_DIR}/kernel/include
    ${CMAKE_SOURCE_DIR}/modules/params
)
target_link_libraries(apx_ekf3 PUBLIC apx_kernel apx_params)
