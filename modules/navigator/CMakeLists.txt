# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
set(NAVIGATOR_SOURCES
    mission_executor.cpp
)

add_library(apx_navigator STATIC ${NAVIGATOR_SOURCES})
target_include_directories(apx_navigator PUBLIC .)
