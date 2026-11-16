# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
if(NOT DEFINED APX_BOARD)
    set(APX_BOARD "sitl")
endif()

message(STATUS "Configuring APX for board target: ${APX_BOARD}")
