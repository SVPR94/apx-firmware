# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
# APX bootloader core (portable): sources and include path for any OS port.
#   include(${APX_ROOT}/bootloader/sources.cmake)
set(APX_BOOTLOADER_INCLUDE_DIR ${CMAKE_CURRENT_LIST_DIR}/include)
set(APX_BOOTLOADER_SOURCES ${CMAKE_CURRENT_LIST_DIR}/src/protocol.cpp)
