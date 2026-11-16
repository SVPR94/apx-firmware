# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
# west flash / west debug runners for APX STM32 boards (SWD probe).
# A board's board.cmake sets APX_JLINK_DEVICE and APX_PYOCD_TARGET, then
# includes this file. Boards flashed through a bootloader (.apj) still get
# these for SWD recovery and debugging; the image is linked after the
# bootloader, so SWD flashing does not touch it.

board_runner_args(stm32cubeprogrammer "--port=swd" "--reset-mode=sw")
board_runner_args(openocd --cmd-post-verify "reset halt")
board_runner_args(openocd --target-handle=_CHIPNAME.cpu0)
board_runner_args(jlink "--device=${APX_JLINK_DEVICE}" "--speed=4000")
board_runner_args(pyocd "--target=${APX_PYOCD_TARGET}")

include(${ZEPHYR_BASE}/boards/common/openocd-stm32.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/stm32cubeprogrammer.board.cmake)
