# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
add_subdirectory(sensor_sim)
add_subdirectory(noise)

add_library(apx_simulation INTERFACE)
target_link_libraries(apx_simulation INTERFACE apx_sim_sensors)
