# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
# APX OS abstraction (apx::kernel): sources for one OS port.
#
#   include(${APX_ROOT}/kernel/sources.cmake)
#   apx_kernel_sources(<os> <out_var>)
#
# <os> is a directory under kernel/ports/ (posix, zephyr, baremetal, or a
# new port such as freertos). Used by kernel/CMakeLists.txt (CMake builds)
# and by platforms/zephyr/app (Zephyr builds) so every build system picks
# the same files.

set(APX_KERNEL_DIR ${CMAKE_CURRENT_LIST_DIR})
set(APX_KERNEL_INCLUDE_DIR ${APX_KERNEL_DIR}/include)

function(apx_kernel_sources os out_var)
    set(port_dir ${APX_KERNEL_DIR}/ports/${os})
    if(os STREQUAL "template" OR NOT IS_DIRECTORY ${port_dir})
        file(GLOB ports LIST_DIRECTORIES true RELATIVE ${APX_KERNEL_DIR}/ports ${APX_KERNEL_DIR}/ports/*)
        list(REMOVE_ITEM ports template README.md)
        message(FATAL_ERROR "APX: no OS port '${os}' in kernel/ports/ (available: ${ports}). "
                            "See kernel/ports/README.md to add one.")
    endif()
    file(GLOB port_sources CONFIGURE_DEPENDS ${port_dir}/*.cpp ${port_dir}/*.c)
    set(${out_var}
        ${APX_KERNEL_DIR}/src/ubus.cpp   # portable kernel code
        ${port_sources}                  # the selected OS port
        PARENT_SCOPE)
endfunction()
