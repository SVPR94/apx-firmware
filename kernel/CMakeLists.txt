# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
cmake_minimum_required(VERSION 3.14)
project(apx_kernel)

add_library(apx_kernel src/ubus.cpp src/timer.cpp src/mutex.cpp)
target_include_directories(apx_kernel PUBLIC include)
# Not target_compile_features(... cxx_std_17): that looks up the
# compiler's known-features database, and CMake 3.22 (this project's
# cmake_minimum_required floor) predates GCC 15.2's release by several
# years, so it fails with "no known features for CXX compiler GNU
# version 15.2.1" against the STM32H7 cross toolchain - a CMake/compiler
# vintage mismatch, not anything about this target. The top-level
# CMakeLists.txt already sets CMAKE_CXX_STANDARD 17 (+ REQUIRED ON)
# globally before any add_subdirectory(), which every target in the
# project - this one included - already gets without needing its own
# feature-database lookup.

# Zephyr vs. hosted-OS (SITL/Linux) vs. bare-metal (platforms/stm32h7).
# Threads::Threads/pthread doesn't exist for a bare-metal arm-none-eabi
# Generic target. Checked via APX_BOARD (the actual CMake cache
# variable) rather than APX_PLATFORM_STM32H7 - that name only exists as
# a preprocessor macro (see the top-level CMakeLists.txt's
# add_compile_definitions() in the APX_BOARD=stm32h7 branch), which is
# invisible to CMake's own if() here; using it silently always took the
# else() branch below regardless of platform, defeating this check
# entirely until caught by an actual stm32h7 configure attempt.
if(ZEPHYR_BASE)
    target_link_libraries(apx_kernel PUBLIC zephyr_interface)
elseif(APX_BOARD STREQUAL "stm32h7")
    # No threading library at all yet (no RTOS wired in - see
    # platforms/stm32h7/README.md's "Bare-metal, not Zephyr/FreeRTOS"
    # section). apx::kernel::Mutex has a bare-metal Impl now (see
    # src/mutex.cpp - IRQ-masking, appropriate for this single-core,
    # no-RTOS-yet target). apx::kernel::Task still has no Impl on any
    # platform - fine while nothing constructs one.
else()
    find_package(Threads REQUIRED)
    target_link_libraries(apx_kernel PUBLIC Threads::Threads)
endif()
