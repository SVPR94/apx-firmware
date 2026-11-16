# APX OS abstraction - ports

APX code never calls an OS directly. It uses the `apx::kernel` API in
`kernel/include/apx/kernel/`, and each OS implements that API once, in its own
folder here. To change the OS (FreeRTOS, ChibiOS, NuttX, ...), you add a port
folder. The flight code, the HAL interfaces and the board files stay as they are.

| Port | Used by | Selected by |
|---|---|---|
| `posix/` | SITL, Linux companion | `APX_OS=posix` (default for `APX_BOARD=sitl` / `linux`) |
| `baremetal/` | STM32H7 bare-metal link test | `APX_OS=baremetal` (default for `APX_BOARD=stm32h7`) |
| `zephyr/` | Flight firmware on Zephyr RTOS | `platforms/zephyr/app` (west build) |
| `template/` | Starting point for a new port | not compiled |

Each build compiles exactly one port. `kernel/sources.cmake`
(`apx_kernel_sources(<os> <var>)`) returns its files, so CMake builds and Zephyr
builds pick the same sources. Each port also gets the compile definition
`APX_OS_<NAME>`, but code outside `kernel/ports/` and `platforms/` should not need it.

## The API a port implements

| Header | Functions | Contract |
|---|---|---|
| `timer.hpp` | `now_us`, `now_ms`, `elapsed_us`, `delay_us`, `delay_ms` | Monotonic 64-bit time since boot, at microsecond resolution. Use a hardware cycle counter, not the scheduler tick, because the flight loop computes `dt` from it. The delay functions sleep the calling thread. |
| `mutex.hpp` | `Mutex::Mutex/~Mutex/lock/unlock` | Recursive and blocking, with priority inheritance where available. Thread context only. |
| `critical.hpp` | `CriticalSection` ctor/dtor | Masks interrupts (or the scheduler). Nestable. Safe to share data with ISRs. |
| `periodic.hpp` | `PeriodicTimer::PeriodicTimer/~/start/stop/wait` | Drift-free fixed rate. `wait()` blocks until the next period and returns how many periods elapsed (≥ 1). |
| `os.hpp` | `os_name`, `yield` | Short OS id (`"freertos"`); yield to equal-priority threads. |

`task.hpp` (`Task`) is declared but not implemented by any port yet: the flight
loop runs on the OS's main thread. Implement it in your port when the firmware
needs extra threads.

The linker enforces the API: a port that misses a function fails to link,
naming the missing symbol.

## Adding a port (example: FreeRTOS)

1. **Copy the template** with `cp -r kernel/ports/template kernel/ports/freertos`, then
   remove the `#error` line. You can split the file into
   `time.cpp`/`mutex.cpp`/`periodic.cpp` like the other ports. All `*.cpp` files in
   the folder are compiled.
2. **Implement each function** using the mapping below.
3. **Add the OS startup code** under `platforms/freertos/`, the same way `platforms/zephyr/` does it:
   - the OS kernel sources and configuration (`FreeRTOSConfig.h`);
   - `main()`, which initialises the hardware, creates one task that calls
     `apx::firmware::run()` (`firmware/include/apx/firmware/firmware.hpp`), and
     starts the scheduler;
   - an implementation of the board interface (`firmware/include/apx/firmware/board.hpp`):
     status LED, power rails and the GCS serial link (an `apx::hal::IHalUart`);
   - a CMake file that uses `firmware/sources.cmake` (portable firmware) and
     `kernel/sources.cmake` with `apx_kernel_sources(freertos ...)`.
4. **Add it to the build tool:** extend `tools/build_tools/targets.py` with the new OS
   so it shows up in the GUI's **OS** list.

### API mapping

| APX | FreeRTOS | ChibiOS/RT | NuttX (POSIX) |
|---|---|---|---|
| `now_us` | DWT `CYCCNT` or a 32-bit timer (tick is too coarse) | `chVTGetSystemTimeX()` with a 1 MHz tickless timer, or DWT | `clock_gettime(CLOCK_MONOTONIC)` |
| `delay_ms` | `vTaskDelay(pdMS_TO_TICKS(ms))` | `chThdSleepMilliseconds` | `usleep` |
| `Mutex` | `xSemaphoreCreateRecursiveMutex` / `...TakeRecursive` | `chMtxObjectInit` / `chMtxLock` | `pthread_mutex` (recursive) |
| `CriticalSection` | `taskENTER_CRITICAL` / `taskEXIT_CRITICAL` (`..._FROM_ISR` in ISRs) | `chSysLock` / `chSysUnlock` | `enter_critical_section` / `leave_critical_section` |
| `PeriodicTimer::wait` | `xTaskDelayUntil` (returns missed periods via the tick count) | `chThdSleepUntilWindowed` | `clock_nanosleep(TIMER_ABSTIME)` |
| `yield` | `taskYIELD()` | `chThdYield()` | `sched_yield()` |

## Rules that keep the OS replaceable

- **OS headers stay in two places:** only files under `kernel/ports/<os>/`,
  `hal/ports/<os>/` and `platforms/<os>/` include them. The portable code
  (`firmware/`, `modules/`, `drivers/`) includes `apx/kernel/*.hpp` and `apx/hal/*.hpp` only.
- **Hardware details go in board files**, not in `firmware/`. On Zephyr that means
  the board's devicetree (see `platforms/zephyr/README.md`).
- **Use the right macro for hosted features:** to tell embedded builds from hosted ones,
  use `APX_EMBEDDED` ("no sockets / std::thread"). Don't use an OS or MCU macro for that.
