# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
add_library(apx_noise_filters INTERFACE)
target_include_directories(apx_noise_filters INTERFACE
    ${CMAKE_SOURCE_DIR}/modules/estimator/filters
)
