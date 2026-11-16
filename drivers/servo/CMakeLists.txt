# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
add_library(apx_servo STATIC servo_driver.cpp)
target_include_directories(apx_servo PUBLIC
    ${CMAKE_SOURCE_DIR}/drivers/servo
    ${CMAKE_SOURCE_DIR}/drivers/include
    ${CMAKE_SOURCE_DIR}/hal/include
)
target_link_libraries(apx_servo PUBLIC apx_hal)
