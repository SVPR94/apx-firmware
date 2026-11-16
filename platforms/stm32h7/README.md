# APX STM32H7 Platform Support

> **Not a flight firmware.** The image built here is a link test: it runs
> the controller math on constant inputs and has no HAL, drivers,
> scheduler or MAVLink transport (see
> `docs/assessment/stm32_firmware_audit.md`). The real hardware firmware
> is built on Zephyr in `platforms/zephyr/`, starting with the CubePilot
> Cube Orange+. Do not flash these images to a vehicle.

This directory provides real board-support scaffolding for building APX
on STM32H7 hardware (Cortex-M7). It maps logical peripheral roles onto a
specific board's pins/timers/buses using an independent, modern C++ design:
a plain constexpr C++ table instead of external text definition files with
custom parsers, and a clean CMake + GNU-ld + Cortex-M startup toolchain.

**Status: builds and links a real firmware image.** `cmake --preset
stm32h7` configures cleanly and `cmake --build build/stm32h7 --target
apx_firmware_apx_h743_v1` produces a genuine
`apx_firmware_apx_h743_v1.elf`/`.bin` containing a valid ARMv7-M vector
table, `Reset_Handler`, `SystemInit()`, and real APX flight-control code
(`AttitudeController`, `RateController`, `PositionController`,
`MotorMixer` - the same classes `modules/modes/navigation_cascade.cpp`
drives on SITL) - verified at the byte level (correct entry point,
correct initial stack pointer, real demangled symbols present), not just
"the linker didn't error." It has **not** been flashed to or run on real
hardware, and there is still no HAL backend behind it (so it can't read a
real sensor or drive a real motor yet) - see
[What's real vs. still needed](#whats-real-vs-still-needed) below for
exactly what that does and doesn't mean.

## Layout

```
platforms/stm32h7/
  README.md                          - this file
  toolchain-arm-none-eabi.cmake      - CMake toolchain file (arm-none-eabi-gcc, Cortex-M7)
  CMakeLists.txt                     - selects one board via APX_BOARD_TARGET
  boards/
    apx_h743_v1/                     - standard generic STM32H743 board
      board_config.hpp               - pin/peripheral map
      linker.ld                      - memory map + section layout
      startup_stm32h743.s            - vector table + Reset_Handler
      CMakeLists.txt                 - builds this board's support library
    apx_h743_pro/                    - professional H743 board layout
    cube_orange_plus/                - CubePilot Cube Orange+ (STM32H757, 3 sensor lanes)
      board_config.hpp               - triple-redundant IMU & 3 sensor lanes pinout
      hwdef.dat                      - hardware definition & bootloader ID 1063
      linker.ld                      - H757 memory map
      startup_stm32h757.s            - Cortex-M7 vector table
      system_init.c                  - FPU and VTOR setup
      CMakeLists.txt                 - builds ELF, BIN, HEX and bootloader .apj
  default_params/
    apx_h743_v1.parm                 - default parameter overrides
    cube_orange_plus.parm            - default parameter overrides for Cube Orange+
```

## Board description format

`board_config.hpp` is a `constexpr` C++ table in `namespace
apx::platform::stm32h7::<board_name>`, mapping logical roles
(`IMU_SPI_BUS`, `GPS_UART_ID`, `MOTOR1`, ...) to STM32H7
pins/peripherals/bus numbers. It is written to match the existing
`apx::hal::IHalSpi` / `IHalUart` / `IHalI2c` / `IHalPwm` / `IHalGpio`
interfaces in `hal/include/apx/hal/` (each of those takes a flat
`uint32_t` bus/pin/channel id, not a struct) -- a future
`hal/stm32h7/` backend (that directory exists but is currently empty)
would read these constants at board-init time to construct its HAL
objects, the same way `drivers/imu/icm42688/icm42688.hpp` etc. take an
`IHalSpi&`/`IHalUart&`/`IHalI2c&` reference and a bus/address number,
regardless of what backend is behind it.

This is deliberately **not** a text file with its own parser: it is compiled
directly, so a typo in a pin name is a compile error, not a runtime failure
discovered on real hardware.

### Adding a new board variant

1. Copy `boards/apx_h743_v1/` to `boards/<your_board_name>/`.
2. Edit `board_config.hpp`: update the namespace to
   `apx::platform::stm32h7::<your_board_name>` and change the
   pin/bus/channel assignments to match your board's schematic.
3. Edit `linker.ld` if your MCU variant has different flash/RAM sizes
   (H750 has 128 KB flash instead of H743's 2 MB, for example --
   check your exact part's datasheet).
4. `startup_stm32h743.s` can usually be reused unchanged unless your
   board needs additional named IRQ vectors (see the comments in that
   file -- most STM32H743 IRQ slots are placeholder `Default_Handler`
   entries and only get a real name once a driver needs one).
5. Copy `CMakeLists.txt`, updating `apx_board_apx_h743_v1` to
   `apx_board_<your_board_name>` throughout.
6. Add a `default_params/<your_board_name>.parm` (see format below).
7. Build with `-DAPX_BOARD_TARGET=<your_board_name>`.

## Default parameter files

`default_params/*.parm` use APX's format: one `NAME=value` per line, `#` for
full-line comments. Standard parameter *names* (`FRAME_CLASS`, `BATT_MONITOR`,
`ATC_RAT_RLL_P`, ...) and default values follow standard MAVLink flight
controller conventions for seamless GCS interoperability -- see
`modules/params/param_manager.cpp`.

**Every parameter name used in `default_params/apx_h743_v1.parm` is
cross-checked against `ParamManager::init_defaults()`
(`modules/params/param_manager.cpp`) and exists today.** Parameters this
board's wiring implies but that `ParamManager` doesn't define yet
(`SERIAL*_PROTOCOL`/`SERIAL*_BAUD` for routing GPS/telemetry/RC to the
right UART, `RCMAP_*` for stick-channel mapping) are listed at the
bottom of the file as commented-out `# TODO: not yet registered in
ParamManager` entries, not as real overrides -- loading this file today
only affects params `ParamManager` already knows about. There is no
loader wired up yet that reads a `.parm` file at boot; that's part of
what's still needed (below).

## Bare-metal, not Zephyr/FreeRTOS

Before writing `startup_stm32h743.s`, this task checked how the rest of
the tree currently decides between Zephyr, an RTOS, and bare-metal:

- `kernel/CMakeLists.txt` now branches on `if(ZEPHYR_BASE) ...
  elseif(APX_BOARD STREQUAL "stm32h7") ... else() find_package(Threads
  REQUIRED) ...` - Zephyr, this bare-metal platform, or a hosted OS with
  pthreads (SITL/Linux). (At the time this section was first written,
  that third branch didn't exist yet; it was added and verified via an
  actual build later in the same session - see "What's real vs. still
  needed" below.)
- `cmake/zephyr_integration.cmake` and the top-level `boards/apx_h7_v1/`
  directory (note: **not** under `platforms/`) contain an earlier,
  separate exploration of a Zephyr-based board -- a Zephyr devicetree
  (`apx_h7_v1.dts`) and a `board.cmake` setting `ZEPHYR_BOARD`. That
  path is **not currently wired into the build**: the top-level
  `CMakeLists.txt` never sets `ZEPHYR_BASE`, never includes
  `cmake/zephyr_integration.cmake`, and never does
  `add_subdirectory(boards)`. It is dead/orphaned as far as the CMake
  build graph goes.
- `hal/stm32h7/` exists as an empty directory: there is no STM32H7 HAL
  backend implementation (Zephyr-based or bare-metal) behind
  `IHalSpi`/`IHalUart`/etc. today.

Given that, `platforms/stm32h7/` takes the **bare-metal, no-OS** path:
`toolchain-arm-none-eabi.cmake` sets `CMAKE_SYSTEM_NAME Generic` (no
host OS assumed) and `startup_stm32h743.s` is a from-scratch Cortex-M
vector table + `Reset_Handler`, not a Zephyr board port. This is
consistent with `kernel/CMakeLists.txt`'s existing structure in that it
adds the third case that file is missing (`ZEPHYR_BASE` set → Zephyr;
unset + hosted OS → pthreads; unset + bare-metal → neither, needs adding
-- see "What's real vs. still needed" below) rather than trying to
resurrect the orphaned Zephyr board files. Reconciling with or removing
`boards/apx_h7_v1/` and `cmake/zephyr_integration.cmake` is a separate,
future decision (do we want a Zephyr-backed target *in addition to* this
bare-metal one, or should that exploration be removed?) -- this task
does not delete or modify that directory.

`cmake/toolchains/arm-none-eabi.cmake` (top-level, pre-existing) is a
shorter toolchain file already referenced by the `stm32h7` entry in
`CMakePresets.json`. `platforms/stm32h7/toolchain-arm-none-eabi.cmake`
supersedes it for this platform (adds `CMAKE_TRY_COMPILE_TARGET_TYPE
STATIC_LIBRARY` and forced-ABI-detection flags needed for a clean
bare-metal `cmake` configure, and `REQUIRED` on `find_program` so a
missing toolchain fails fast with a clear message) -- `CMakePresets.json`
has been updated to point at the new file. The old file has been left in
place rather than deleted, since it's outside this task's scope to judge
whether something else still depends on it.

## Building

### 1. Install the toolchain

```sh
tools/setup_ubuntu_env.sh      # one-time: deps + toolchain + permanent PATH entry
arm-none-eabi-gcc --version    # sanity check (in a new terminal)
```

`tools/setup_ubuntu_env.sh` calls `setup_toolchain.sh` below and adds the
toolchain to `PATH` permanently. To install only the toolchain for the current
shell:

```sh
platforms/stm32h7/setup_toolchain.sh
source platforms/stm32h7/toolchain/env.sh
```

If configuring without a toolchain, CMake stops with a message pointing to
`tools/setup_ubuntu_env.sh`.

`setup_toolchain.sh` downloads a *pinned* Arm GNU Toolchain release (see
the version comment at the top of that script - deliberately not "always
whatever's newest," so a build stays reproducible) into
`platforms/stm32h7/toolchain/<version>/` (gitignored - a local install,
not something committed) and writes `env.sh` to put it on `PATH`. It's
idempotent - safe to re-run, skips the download if that version is
already extracted. If you'd rather use your distro's package instead
(older GCC, but works offline once cached): `sudo apt install
gcc-arm-none-eabi`.

### 2. Configure

```sh
cmake --preset stm32h7
```

Or the equivalent manual invocation:

```sh
cmake -B build/stm32h7 \
      -DAPX_BOARD=stm32h7 \
      -DAPX_BOARD_TARGET=apx_h743_v1 \
      -DCMAKE_TOOLCHAIN_FILE=platforms/stm32h7/toolchain-arm-none-eabi.cmake \
      -DAPX_BUILD_SITL=OFF -DAPX_BUILD_TESTS=OFF \
      -G Ninja
```

### 3. Build

For the generic H743 board:
```sh
cmake --build build/stm32h7 --target apx_firmware_apx_h743_v1
```

For **CubePilot Cube Orange+** (STM32H757, 3 sensor lanes):
```sh
# Using CMake Preset:
cmake --preset stm32h7-cube-orange-plus
cmake --build build/stm32h7_cube_orange_plus --target apx_firmware_cube_orange_plus

# Or manual CMake invocation:
cmake -B build/stm32h7_cube_orange_plus \
      -DAPX_BOARD=stm32h7 \
      -DAPX_BOARD_TARGET=cube_orange_plus \
      -DCMAKE_TOOLCHAIN_FILE=platforms/stm32h7/toolchain-arm-none-eabi.cmake \
      -DAPX_BUILD_SITL=OFF -DAPX_BUILD_TESTS=OFF \
      -G Ninja
ninja -C build/stm32h7_cube_orange_plus apx_firmware_cube_orange_plus
```

This builds the ELF, BIN, HEX, and generates `apx_cube_orange_plus.apj` (packaged for the APX onboard bootloader with board ID `1063`).

For the full hardware firmware running on **Zephyr RTOS** (with USB CDC-ACM MAVLink):
```sh
source platforms/zephyr/workspace/env.sh
west build -b cube_orange_plus/stm32h757xx/m7 -d build/zephyr_cube_orange_plus platforms/zephyr/app
```
Outputs `build/zephyr_cube_orange_plus/zephyr/apx_cube_orange_plus.apj` ready to flash via APX Flight Manager / GCS.

This produces
`build/stm32h7/platforms/stm32h7/boards/apx_h743_v1/apx_firmware_apx_h743_v1.elf`
(+ `.bin`, + a `.map` file) - a real, linked image (see the status note
at the top of this file). **Don't use the bare `cmake --build
build/stm32h7`** (no `--target`) - that builds *every* target the top-level
CMakeLists.txt defines, including `apx_drivers`, which still has several
POSIX-only driver files (`nanosleep()` calls with no bare-metal
replacement yet) that fail to compile for this platform; building the
specific board target sidesteps those entirely.

### 4. Flashing (not yet exercised on real hardware)

```sh
# ST-Link, using ST's own tooling:
st-flash write build/stm32h7/platforms/stm32h7/boards/apx_h743_v1/apx_firmware_apx_h743_v1.bin 0x08000000

# or OpenOCD:
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg \
        -c "program build/stm32h7/platforms/stm32h7/boards/apx_h743_v1/apx_firmware_apx_h743_v1.elf verify reset exit"
```

Neither has been exercised in this session (no hardware available here)
- documented for when real hardware is available. The image should boot
(vector table/entry point/stack pointer are all verified correct - see
below) and spin in `apx_main_stm32h7.cpp`'s loop, but it doesn't do
anything observable yet (no UART/LED output wired up) since there's no
HAL backend behind it.

## What's real vs. still needed

**Real and verified (actual build/link, not just configure):**
- `setup_toolchain.sh`: downloads and installs a real, pinned Arm GNU
  Toolchain (15.2.Rel1) release - used to produce everything below.
- `toolchain-arm-none-eabi.cmake`: a working CMake toolchain file with
  correct `-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard` flags,
  `Generic` system, and the try-compile/ABI-detection settings a
  bare-metal cross toolchain needs to configure cleanly.
- `boards/apx_h743_v1/linker.ld`: a real STM32H743 memory map (2 MB
  flash at `0x08000000`, 128 KB DTCM, 512 KB AXI SRAM) with standard
  `.isr_vector`/`.text`/`.rodata`/`.data`/`.bss`/heap-stack sections.
  Its `/DISCARD/` block and `EXTERN(Reset_Handler)` were both fixed
  during this session after an actual build exposed real bugs in each
  (see below) - not just written and assumed correct.
- `boards/apx_h743_v1/startup_stm32h743.s`: a real vector table (all 16
  ARMv7-M core exceptions correctly ordered) and a `Reset_Handler` that
  copies `.data`, zeroes `.bss`, calls `SystemInit()`, runs C++ static
  constructors via `__libc_init_array`, and calls `main()`.
- `boards/apx_h743_v1/system_init.c`: a real (if deliberately minimal)
  `SystemInit()` - enables the FPU coprocessor (CP10/CP11) and sets
  VTOR. Does **not** configure the PLL/clock tree (see below).
- `boards/apx_h743_v1/apx_main_stm32h7.cpp`: a real bare-metal entry
  point (distinct from `main/apx_main.cpp`, which stays SITL/Linux-only)
  that constructs and exercises APX's actual `AttitudeController`/
  `RateController`/`PositionController`/`MotorMixer` classes.
- `boards/apx_h743_v1/board_config.hpp`: pin/bus assignments for every
  peripheral the existing `drivers/` tree supports for this board class.
- **`apx_firmware_apx_h743_v1.elf`/`.bin` actually link and were
  verified correct at the byte level**: `readelf -h` entry point
  `0x8000425` matches the weak `Reset_Handler` symbol (with the Thumb
  bit set); the vector table's first two words decode to `0x20020000`
  (top of DTCM, matching `linker.ld`'s `_estack`) and `0x08000425`
  (the same entry point) - not just "the linker didn't complain."
  `nm` shows real demangled symbols
  (`apx::control::AttitudeController::update`,
  `apx::control::RateController::update`,
  `apx::control::MotorMixer::mix`) actually present, at a plausible
  4.3 KB `.text` (not the ~164-byte near-empty image an earlier, broken
  configuration silently produced - see the CMake/linker bugs below).
- Top-level `CMakeLists.txt`: routes `-DAPX_BOARD=stm32h7` into
  `add_subdirectory(platforms/stm32h7)`, and now also enables `ASM` as a
  project language (`LANGUAGES C CXX ASM`) - required for
  `startup_stm32h743.s` to be compiled at all (see below). **The SITL
  preset was rebuilt end-to-end after every change in this session and
  still succeeds.**
- `kernel/CMakeLists.txt`: now has a real bare-metal branch (checked via
  the `APX_BOARD` CMake variable, not a preprocessor macro - see below)
  and `apx::kernel::Mutex` now has a real bare-metal `Impl` (IRQ-masking
  critical section, appropriate for this single-core, no-RTOS-yet
  target) alongside its hosted (`std::mutex`-backed) one.
- `modules/params/param_manager.cpp`/`.hpp`: switched from `std::mutex`
  to `apx::kernel::Mutex` (see below for why), and links cleanly for
  this target.
- **Bugs an actual build attempt found and fixed this session** (each
  one *silently* produced a "successful" but broken/tiny/wrong build
  before being caught and fixed - worth listing so the pattern doesn't
  recur elsewhere):
  1. `target_compile_features(... cxx_std_17)` in `kernel/` and `hal/`
     failed outright ("no known features for CXX compiler GNU version
     15.2.1") - CMake 3.22 predates GCC 15.2's compiler-feature database.
     Removed in favor of the project-wide `CMAKE_CXX_STANDARD 17` the
     top-level `CMakeLists.txt` already sets.
  2. `kernel/CMakeLists.txt`'s bare-metal branch checked
     `if(APX_PLATFORM_STM32H7)` - that name only exists as a
     *preprocessor* macro (`add_compile_definitions()`), invisible to
     CMake's own `if()`; it silently always fell through to
     `find_package(Threads REQUIRED)` regardless of platform. Fixed to
     check `APX_BOARD STREQUAL "stm32h7"` instead.
  3. `std::mutex`/`<mutex>` don't actually work on this toolchain: it's
     a single-threaded (no gthreads) libstdc++ build, so `<mutex>`
     compiles but `std::mutex` itself doesn't exist
     ("'mutex' is not a member of 'std'"). `param_manager.cpp` switched
     to `apx::kernel::Mutex`, which needed a real bare-metal `Impl`
     written for it (it had none before, on any platform).
  4. `param_manager.cpp` used `std::clamp` without `#include <algorithm>`
     - previously worked by luck (something else pulled it in
     transitively on the host toolchain); this toolchain's header graph
     doesn't, and it's a real missing include either way.
  5. **The linker script's `/DISCARD/` block discarded `libc.a(*)` and
     `libm.a(*)` wholesale** (not just debug/comment sections from them)
     - would have silently thrown away the entire C/math library the
     moment anything actually needed it. Fixed to discard only
     `.comment`/`.ARM.attributes`.
  6. **`ENTRY(Reset_Handler)` alone did not reliably keep `Reset_Handler`
     live under `--gc-sections`** - ld warned "cannot find entry symbol
     Reset_Handler" and produced a ~164-byte image with almost
     everything garbage-collected away, *without erroring* (a "successful"
     link that was actually badly broken). Fixed by adding
     `EXTERN(Reset_Handler)` to force it to be treated as a real
     unresolved reference.
  7. **The most consequential one: `ASM` was never enabled as a CMake
     project language** (`LANGUAGES C CXX` only). CMake has no rule for
     `.s` files without it and silently drops them from the build
     instead of erroring - `startup_stm32h743.s` (vector table +
     `Reset_Handler`) was never actually being compiled at all, which
     was the real cause of bugs 6's symptom (there was no
     `Reset_Handler` object code in the link to begin with). Fixed by
     adding `ASM` to the top-level `project(... LANGUAGES C CXX ASM)`.

**Explicitly stubbed / not attempted (do not assume otherwise):**
- **No STM32H7 HAL backend.** `hal/stm32h7/` is still an empty
  directory. `board_config.hpp`'s constants are not consumed by any
  code yet. This is *why* `apx_firmware_apx_h743_v1` only links
  `apx_kernel`/`apx_params`/`apx_flight_control` and not
  `apx_hal`/`apx_drivers`/`apx_gcs`/`apx_safety`/`apx_vehicles`/
  `apx_rc_manager`/`apx_flight_modes` - those either need a real HAL
  backend behind them or still have hosted-OS-only code in them (see
  `apx_drivers`' `nanosleep()` calls, noted in "Building" above).
- **No PLL/clock-tree init.** `system_init.c` enables the FPU and sets
  VTOR (both real and necessary) but deliberately does not attempt the
  HSI→400MHz PLL sequence - getting the flash wait-state/voltage-scaling
  register values wrong on real H743 silicon is a well-known way to hang
  the boot path, and there's no hardware here to verify it against. The
  core will run on its ~64MHz HSI reset clock until this is written and
  verified on real `apx_h743_v1` hardware - `kernel/src/timer.cpp`'s
  `kAssumedCoreClockHz` assumes this same default and must be updated
  together with a real clock-tree init.
- **No `.parm` file loader.** `default_params/apx_h743_v1.parm` is a
  plain text file; nothing in `modules/params/` reads `.parm` files yet
  (`ParamManager::load_from_storage()` is a `// TODO` stub today).
- **`SERIAL*_PROTOCOL`/`SERIAL*_BAUD` parameters don't exist yet** in
  `ParamManager` (routing GPS/telemetry/RC to the right UART) - see the
  TODO block at the bottom of the `.parm` file. (`RCMAP_*` and per-
  channel `RC1_MIN/MAX/TRIM`.. `RC8_*` calibration *were* added to
  `ParamManager` this session - the `.parm` file's TODO list has been
  updated to only list what's genuinely still missing.)
- **IRQ vector table is not exhaustively verified.** Only the vectors
  this board's drivers need are named; the remaining STM32H743 IRQ
  slots are `Default_Handler` placeholders and the whole table should be
  checked against ST's RM0433 reference manual before relying on any
  unlisted IRQ number.
- **No DMA, no double-buffering, no interrupt-driven UART/SPI.** The
  existing `IHalUart`/`IHalSpi` interfaces and drivers are written
  against a simple synchronous/polling model; real STM32H7 firmware
  would normally want DMA for the SPI IMU and UART ports, which isn't
  designed here.
- **No real sensor/actuator I/O.** `apx_main_stm32h7.cpp` runs the real
  control-loop math on synthetic (constant) input in a free-running
  loop - there's no SysTick-driven scheduling, no real IMU sample, no
  real PWM output, until an `hal/stm32h7/` backend exists.
- **`main/CMakeLists.txt` is unmodified** and still unconditionally
  links `pthread` and builds `apx_flight_controller`/`apx_sitl` for
  SITL/Linux - the STM32H7 firmware target lives entirely in
  `platforms/stm32h7/boards/apx_h743_v1/CMakeLists.txt` instead
  (`apx_firmware_apx_h743_v1`), deliberately not routed through `main/`.
- **Not flashed or run on real hardware** -- no board was available in
  this session. Everything above marked "real and verified" was checked
  by actually building, linking, and inspecting the resulting binary
  (readelf/nm/objdump), which is real evidence the toolchain/linker/
  startup pipeline works correctly - but it is still "should boot
  correctly," not "confirmed booting on silicon."
