# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
if(EXISTS "${CMAKE_SOURCE_DIR}/VERSION")
    file(READ "${CMAKE_SOURCE_DIR}/VERSION" APX_VERSION_RAW)
    string(STRIP "${APX_VERSION_RAW}" APX_VERSION)
else()
    set(APX_VERSION "0.1.0")
endif()
message(STATUS "APX Version: ${APX_VERSION}")
