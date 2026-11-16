# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
cmake_minimum_required(VERSION 3.16)

add_library(sitl_bridge
    sitl_server.cpp
    sensor_injector.cpp
)
target_include_directories(sitl_bridge PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
