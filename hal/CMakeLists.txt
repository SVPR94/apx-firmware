# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
cmake_minimum_required(VERSION 3.14)
project(apx_hal)

add_library(apx_hal INTERFACE)
target_include_directories(apx_hal INTERFACE include)
# Not target_compile_features(... cxx_std_17) - see kernel/CMakeLists.txt's
# comment on the same change; CMake 3.22 doesn't know GCC 15.2's compiler
# feature set, and the project-wide CMAKE_CXX_STANDARD 17 (set in the
# top-level CMakeLists.txt) already covers this without a per-target
# feature-database lookup.
