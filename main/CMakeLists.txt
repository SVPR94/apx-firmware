# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
if(NOT (APX_BOARD STREQUAL "stm32h7"))
add_executable(apx_flight_controller
    apx_main.cpp
)

target_link_libraries(apx_flight_controller PRIVATE
    apx_kernel
    apx_hal
    apx_drivers
    apx_ekf3
    apx_flight_control
    apx_flight_modes
    apx_safety
    apx_radar
    apx_gcs
    apx_vehicles
    apx_rc_manager
    pthread
    m
)

if(APX_BUILD_SITL)
    target_link_libraries(apx_flight_controller PRIVATE
        sitl_bridge
        apx_sim_physics
        apx_sim_sensors
        apx_altitude_estimator
    )

    add_executable(apx_sitl
        apx_main.cpp
    )
    target_compile_definitions(apx_sitl PRIVATE APX_SITL=1)
    target_link_libraries(apx_sitl PRIVATE
        apx_kernel
        apx_hal
        apx_drivers
        apx_ekf3
        apx_flight_control
        apx_flight_modes
        apx_safety
        apx_radar
        apx_gcs
        apx_vehicles
        apx_rc_manager
        sitl_bridge
        apx_sim_physics
        apx_sim_sensors
        apx_altitude_estimator
        pthread
        m
    )
endif()
endif()
