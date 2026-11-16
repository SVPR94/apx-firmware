# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
if(NOT DEFINED APX_BOARD_TARGET)
    set(APX_BOARD_TARGET "apx_h743_v1" CACHE STRING
        "STM32H7 board variant to build, matching a directory under platforms/stm32h7/boards/")
endif()

set(APX_STM32H7_BOARD_PATH ${CMAKE_CURRENT_SOURCE_DIR}/boards/${APX_BOARD_TARGET})

# Automatic generation hook:
# If a custom hwdef.dat is specified via APX_HWDEF, or if a board folder
# contains a hwdef.dat without CMakeLists.txt or board_config.hpp, generate it.
if(DEFINED APX_HWDEF AND EXISTS ${APX_HWDEF})
    message(STATUS "APX: Generating board '${APX_BOARD_TARGET}' from ${APX_HWDEF}")
    execute_process(
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/generate_board.py
                --hwdef ${APX_HWDEF}
                --output-dir ${APX_STM32H7_BOARD_PATH}
                --board-name ${APX_BOARD_TARGET}
        RESULT_VARIABLE GEN_RESULT
    )
    if(NOT GEN_RESULT EQUAL 0)
        message(FATAL_ERROR "APX: Failed to generate board files from ${APX_HWDEF}")
    endif()
elseif(EXISTS ${APX_STM32H7_BOARD_PATH}/hwdef.dat AND NOT EXISTS ${APX_STM32H7_BOARD_PATH}/board_config.hpp)
    find_package(Python3 COMPONENTS Interpreter REQUIRED)
    message(STATUS "APX: Generating board '${APX_BOARD_TARGET}' from ${APX_STM32H7_BOARD_PATH}/hwdef.dat")
    execute_process(
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/generate_board.py
                --hwdef ${APX_STM32H7_BOARD_PATH}/hwdef.dat
                --output-dir ${APX_STM32H7_BOARD_PATH}
                --board-name ${APX_BOARD_TARGET}
        RESULT_VARIABLE GEN_RESULT
    )
    if(NOT GEN_RESULT EQUAL 0)
        message(FATAL_ERROR "APX: Failed to generate board files from ${APX_STM32H7_BOARD_PATH}/hwdef.dat")
    endif()
endif()

if(NOT EXISTS ${APX_STM32H7_BOARD_PATH}/CMakeLists.txt)
    message(FATAL_ERROR
        "APX_BOARD_TARGET='${APX_BOARD_TARGET}' has no board directory at "
        "${APX_STM32H7_BOARD_PATH} (expected a CMakeLists.txt or hwdef.dat there). "
        "See docs/board-management.md or platforms/stm32h7/README.md for how to add a new board.")
endif()

add_subdirectory(boards/${APX_BOARD_TARGET})
