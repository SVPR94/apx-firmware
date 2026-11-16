# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
add_library(apx_drivers STATIC
    imu/icm42688/icm42688.cpp
    imu/icm42605/icm42605.cpp
    imu/icm20689/icm20689.cpp
    imu/bmi088/bmi088.cpp
    imu/bmi270/bmi270.cpp
    baro/ms5611/ms5611.cpp
    battery/ina226/ina226.cpp
    mag/ist8310/ist8310.cpp
    mag/lis3mdl/lis3mdl.cpp
    mag/rm3100/rm3100.cpp
    gps/ublox/ublox_driver.cpp
    gps/ublox_f9p/ublox_f9p.cpp
    esc/dshot/dshot_driver.cpp
    rc_input/sbus/sbus_driver.cpp
    rc_input/crsf/crsf_driver.cpp
    relay/relay_driver.cpp
    pump/pump_driver.cpp
    servo/servo_driver.cpp
    telemetry/uart_mavlink/uart_mavlink_driver.cpp
    rangefinder/can_radar/can_radar_driver.cpp
    flow/pmw3901/pmw3901.cpp
)

target_include_directories(apx_drivers PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/hal/include
    ${CMAKE_SOURCE_DIR}/kernel/include
)

target_link_libraries(apx_drivers PUBLIC apx_hal apx_kernel)
