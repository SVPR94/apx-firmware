# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
add_library(apx_estimator_ukf STATIC
    ukf.cpp
)

target_include_directories(apx_estimator_ukf PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/kernel/include
    ${CMAKE_SOURCE_DIR}/hal/include
)

target_link_libraries(apx_estimator_ukf PUBLIC
    apx_kernel
    apx_hal
)
