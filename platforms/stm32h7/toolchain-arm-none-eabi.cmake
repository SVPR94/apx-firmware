# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
# what the STM32H743/H750 silicon requires.
#
# Usage:
#   cmake -B build/stm32h7 \
#         -DCMAKE_TOOLCHAIN_FILE=platforms/stm32h7/toolchain-arm-none-eabi.cmake \
#         -DAPX_BOARD=stm32h7 -DAPX_BOARD_TARGET=apx_h743_v1 -G Ninja
#
# See platforms/stm32h7/README.md for toolchain install instructions and
# the full build walkthrough.

# --- Target system -----------------------------------------------------
# CMAKE_SYSTEM_NAME=Generic tells CMake this is a bare-metal target with
# no host OS underneath it (no fork/exec, no filesystem, no threads) --
# this project currently has no RTOS wired into the STM32H7 path (see
# README.md "Bare-metal, not Zephyr/FreeRTOS" section), so "Generic" is
# the correct and only choice here, not a placeholder.
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# A bare-metal target can't build+run a CMake test program (there's
# nothing to run it on), so make CMake's compiler sanity check just
# build a static library instead of a full executable.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(TOOLCHAIN_PREFIX arm-none-eabi-)

# Search order: an explicit $APX_ARM_TOOLCHAIN_BIN, then the pinned local
# install made by tools/setup_ubuntu_env.sh / setup_toolchain.sh (any
# version under toolchain/, newest first), then the system PATH.
set(TOOLCHAIN_HINTS "")
if(DEFINED ENV{APX_ARM_TOOLCHAIN_BIN})
    list(APPEND TOOLCHAIN_HINTS "$ENV{APX_ARM_TOOLCHAIN_BIN}")
endif()
file(GLOB _apx_local_toolchain_bins LIST_DIRECTORIES true "${CMAKE_CURRENT_LIST_DIR}/toolchain/*/bin")
list(SORT _apx_local_toolchain_bins ORDER DESCENDING)
list(APPEND TOOLCHAIN_HINTS ${_apx_local_toolchain_bins})

find_program(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}gcc HINTS ${TOOLCHAIN_HINTS})
if(NOT CMAKE_C_COMPILER)
    get_filename_component(_apx_repo_root "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
    message(FATAL_ERROR
        "\n"
        "  ARM cross-compiler (arm-none-eabi-gcc) not found.\n"
        "\n"
        "  Run the one-time environment setup, then open a new terminal:\n"
        "      ${_apx_repo_root}/tools/setup_ubuntu_env.sh\n"
        "\n"
        "  It downloads the pinned Arm GNU Toolchain into\n"
        "  platforms/stm32h7/toolchain/ and adds it to PATH. To use a toolchain\n"
        "  you installed yourself, put it on PATH or set APX_ARM_TOOLCHAIN_BIN\n"
        "  to its bin/ directory. Delete the build directory before re-running cmake.\n")
endif()
find_program(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}g++     HINTS ${TOOLCHAIN_HINTS} REQUIRED)
find_program(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}gcc     HINTS ${TOOLCHAIN_HINTS} REQUIRED)
find_program(CMAKE_AR           ${TOOLCHAIN_PREFIX}ar      HINTS ${TOOLCHAIN_HINTS} REQUIRED)
find_program(CMAKE_OBJCOPY      ${TOOLCHAIN_PREFIX}objcopy HINTS ${TOOLCHAIN_HINTS} REQUIRED)
find_program(CMAKE_OBJDUMP      ${TOOLCHAIN_PREFIX}objdump HINTS ${TOOLCHAIN_HINTS} REQUIRED)
find_program(CMAKE_SIZE         ${TOOLCHAIN_PREFIX}size    HINTS ${TOOLCHAIN_HINTS} REQUIRED)
find_program(CMAKE_GDB          ${TOOLCHAIN_PREFIX}gdb     HINTS ${TOOLCHAIN_HINTS})

# --- Core CPU flags ------------------------------------------------------
# H743/H750: Cortex-M7, single precision + double precision FPU (FPv5-D16),
# hard-float calling convention.
set(APX_CPU_FLAGS "-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb")

set(CMAKE_C_FLAGS_INIT
    "${APX_CPU_FLAGS} -ffunction-sections -fdata-sections -fno-common -Wall -Wextra")
set(CMAKE_CXX_FLAGS_INIT
    "${APX_CPU_FLAGS} -ffunction-sections -fdata-sections -fno-common -Wall -Wextra \
-fno-exceptions -fno-rtti -fno-threadsafe-statics")
set(CMAKE_ASM_FLAGS_INIT "${APX_CPU_FLAGS} -x assembler-with-cpp")

# No OS, no libc file/process backing: nano-newlib + nosys stubs. Each
# board's CMakeLists.txt appends -T <its linker.ld> to this on the
# executable target (not here, because the script path is per-board).
set(CMAKE_EXE_LINKER_FLAGS_INIT
    "${APX_CPU_FLAGS} -specs=nano.specs -specs=nosys.specs \
-Wl,--gc-sections -Wl,--print-memory-usage")

# MinSizeRel/Release definitions still apply on top of this; APX's
# STM32H7 CMakePresets.json entry defaults to MinSizeRel for flash budget.

# --- find_*() root behavior ---------------------------------------------
# Never search the host's /usr for the arm-none-eabi programs themselves,
# but do restrict library/include/package lookups to whatever sysroot the
# toolchain install provides (find_program for the compiler stays NEVER
# since it needs to find the cross-compiler on the host PATH).
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Skip compiler ABI detection -- it tries to run the produced binary,
# which is impossible for a bare-metal Cortex-M target from the host.
set(CMAKE_C_COMPILER_FORCED TRUE)
set(CMAKE_CXX_COMPILER_FORCED TRUE)
