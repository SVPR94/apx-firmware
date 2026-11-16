# Boards and operating systems

APX firmware has four layers. Each layer depends only on the layers below it,
so a board or an OS can be swapped without touching flight code.

```
 firmware/          portable flight firmware: loop, MAVLink, modes, params
   |  uses only the three interfaces below
   +-- apx/firmware/board.hpp   board interface: LED, power rails, GCS link
   +-- apx/hal/*.hpp            peripheral interfaces: IHalUart, IHalSpi, ...
   +-- apx/kernel/*.hpp         OS abstraction: time, Mutex, CriticalSection, PeriodicTimer
 -----------------------------------------------------------------------------
 one implementation per OS:
   kernel/ports/<os>/           OS abstraction            (posix, zephyr, baremetal)
   hal/ports/<os>/              peripheral drivers         (zephyr: UART/USB CDC)
   platforms/<os>/              startup, board interface, build glue, board files
```

| Layer | Folder | Changes when... |
|---|---|---|
| Flight firmware | `firmware/`, `modules/` | flight behaviour changes |
| Board interface | `firmware/include/apx/firmware/board.hpp` | the firmware needs a new kind of hardware service |
| Board description | `platforms/zephyr/boards/<vendor>/<board>/` | a new flight controller is added |
| OS port | `kernel/ports/<os>/`, `hal/ports/<os>/`, `platforms/<os>/` | a new OS is added |

## Which OS runs on which board

Every STM32H7 board builds on both OSes. Pick one with the **OS** list in
`tools/apx_build.py`, or with `--os` on the command line.

21 flight controllers build on Zephyr (full list in
[platforms/zephyr/README.md](../platforms/zephyr/README.md#boards)). Most are
generated from hwdef hardware descriptions by `tools/import_board.py`. Boards
with an IO coprocessor use [drivers/iomcu](../drivers/iomcu/README.md).
The APX boards also build bare-metal:

| Board | Zephyr RTOS (flight firmware) | Bare-metal (link test) |
|---|---|---|
| CubePilot Cube Orange+ (STM32H757) | yes, `.apj` (board id 1063) | yes |
| APX H743 Pro (STM32H743) | yes, `.apj` (board id 22336) | yes |
| APX H743 v1 (STM32H743) | yes, `.apj` (board id 22337, provisional) after a one-time combined SWD install | yes |
| Custom H7 FC (STM32H743) | yes, `.apj` (board id 22338, provisional) after a one-time combined SWD install | yes |
| SITL / Linux companion | - | host OS (`posix` port) |

## Bootloader

The APX bootloader (`bootloader/` portable protocol core, `platforms/zephyr/bootloader/`
Zephyr port) is built together with the firmware and packaged with it in
`build/zephyr_<board>/firmware/`, including a combined bootloader + firmware image. See
[bootloader/README.md](../bootloader/README.md).

## Adding a flight controller from another manufacturer

On Zephyr, you add board files only. You don't write any C++. See
[platforms/zephyr/README.md](../platforms/zephyr/README.md#adding-a-board).
Any MCU that Zephyr supports works, not only STM32. Port the board's sensor,
motor and RC drivers behind the HAL interfaces as they are written.

## Replacing Zephyr with FreeRTOS, ChibiOS or NuttX

Add a port. The flight code and the board interface stay unchanged. See
[kernel/ports/README.md](../kernel/ports/README.md) for the API, a template, and an
API-by-API mapping for each OS.
