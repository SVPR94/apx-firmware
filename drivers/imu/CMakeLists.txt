# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
add_library(apx_imu_drivers STATIC
    icm42688/icm42688.cpp
    icm42605/icm42605.cpp
    icm20689/icm20689.cpp
    bmi088/bmi088.cpp
    bmi270/bmi270.cpp
)
target_include_directories(apx_imu_drivers PUBLIC
    ${CMAKE_SOURCE_DIR}/drivers
    ${CMAKE_SOURCE_DIR}/drivers/include
    ${CMAKE_SOURCE_DIR}/hal/include
    ${CMAKE_SOURCE_DIR}/kernel/include
)
target_link_libraries(apx_imu_drivers PUBLIC apx_hal apx_kernel)
