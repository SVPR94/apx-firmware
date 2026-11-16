# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
cmake_minimum_required(VERSION 3.16)
project(ApxGazeboPlugin)

find_package(gz-sim8 REQUIRED)

add_library(apx_gazebo_plugin SHARED apx_gazebo_plugin.cpp)
target_link_libraries(apx_gazebo_plugin PUBLIC gz-sim8::gz-sim8)
set_target_properties(apx_gazebo_plugin PROPERTIES PREFIX "")
