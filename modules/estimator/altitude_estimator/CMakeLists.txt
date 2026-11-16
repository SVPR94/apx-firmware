# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
add_library(apx_altitude_estimator STATIC
    altitude_estimator.cpp
)

target_include_directories(apx_altitude_estimator PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
