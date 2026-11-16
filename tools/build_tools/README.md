# APX firmware build utility

GUI and command-line front end for building APX firmware on **Windows, Linux and macOS**.
It uses only the Python standard library: Tkinter for the GUI, and subprocess to run CMake/Ninja
or west. Requires Python 3.8 or newer.

```bash
python3 tools/apx_build.py            # GUI
python3 tools/apx_build.py --help     # CLI
```

On Linux, install Tk once if `doctor` reports it missing: `sudo apt install python3-tk`.
The python.org installers for Windows and macOS already include Tk.

## Targets

Targets are discovered from the repo, so a new board shows up without editing this tool.

In the GUI you pick a **Board** (21 flight controllers, sorted A–Z, then the host builds), then an **OS**
from the ones that board supports. The info line says when a board has an IO coprocessor.
Every STM32H7 board builds on **Zephyr RTOS** and on **bare-metal**. A new OS port shows up
in the OS list once `targets.py` knows about it (see [kernel/ports/README.md](../../kernel/ports/README.md)).

| Target id | What | Backend |
|---|---|---|
| `zephyr:<board>` | Zephyr boards in `platforms/zephyr/boards/` (Cube Orange+, APX H743 Pro/v1, Custom H7 FC) | `west build` from the workspace venv |
| `stm32h7:<board>` | Bare-metal boards in `platforms/stm32h7/boards/` | CMake + Arm GNU toolchain |
| `host:sitl`, `host:linux` | Native SITL / Linux companion builds | CMake + host compiler |

On the command line, use the target id, or the board name plus `--os`:
`build apx_h743_v1 --os zephyr`.

Build directories match `CMakePresets.json` and the README (`build/zephyr_cube_orange_plus`, `build/stm32h7`, ...),
so the tool and manual builds share outputs.

## Options

Each target shows only the options that apply to it, with its own defaults:

- **Build**: build type (CMake) or optimization level (Zephyr); build backend (Auto/Ninja/Make/NMake);
  parallel jobs; clean (pristine) build; verbose output; build directory; `compile_commands.json`; LTO (Zephyr).
- **Features**: SITL bridge and unit tests (host); main stack size (Zephyr); warnings as errors;
  per-function stack usage (`-fstack-usage`).
- **Debug**: debug symbols (`-g`/`-g3`/none), frame pointer, keep `assert()` in release builds,
  sanitizers (host); thread-aware debugging, kernel asserts, stack overflow detection (MPU guard or sentinel),
  stack canaries, stack high-water mark, extra fault info, disassembly listing (Zephyr).
- **Debug probe**: OpenOCD (ST-Link / CMSIS-DAP / J-Link), pyOCD, SEGGER J-Link, STM32CubeProgrammer
  (flash only), or USB DFU through `dfu-util` (flash only, no probe: hold the board's BOOT button while plugging in USB); image to flash on Zephyr boards (firmware only, bootloader + firmware, or bootloader
  only, with a warning before replacing a vendor bootloader); probe device (defaults from the MCU),
  SWD speed, GDB port.
- **Bootloader** (Zephyr): build the APX bootloader with the firmware (default on); power-up listening
  window, stay time after a reboot-to-bootloader request, idle timeout; UART baud rate (UART-link boards)
  or USB vendor/product ID and product string (USB boards); optimization; extra bootloader Kconfig.
  These options go to the bootloader image only, through `build/.apx_builder/<target>/bootloader.conf`
  (`-Dapx_bootloader_EXTRA_CONF_FILE`). They grey out when the bootloader is not built. The outputs land in
  `build/zephyr_<board>/firmware/`, the folder that **Open binary folder** opens.
- **Advanced**: extra compiler flags, extra CMake arguments, extra Kconfig lines.

**Profiles** (`Defaults`, `Debug`, `Release`, `Smallest`) set a group of these options at once for the selected target.
The GUI saves your options per target.

How the options reach the build:

- **Compiler flags** go into a generated `apx_builder_options.cmake` inside the build directory (via `CMAKE_PROJECT_INCLUDE`),
  so the toolchain file's CPU flags are never overwritten.
- **Zephyr Kconfig settings** go into `build/.apx_builder/<target>/extra.conf` (via `EXTRA_CONF_FILE`).
  A setting is written there only when it differs from its default, so `prj.conf` and board defaults stay in effect.

> Options you build with stay in that build directory's CMake cache. A later manual `cmake --preset ...` in the same
> directory keeps them, so reset to **Defaults** and build once to return to the preset configuration.

## Statistics

After every build the **Statistics** tab (and the CLI) shows:

- result, duration per step, compiled units, warnings and errors
- memory regions with usage bars (from `tools/fw_memory_report.py`), and flash/RAM size changes since the last successful build
- flash and RAM breakdown, the largest components and symbols, and every artifact with its size
- the deepest stack frames (when stack usage is on)

**Problems** lists warnings and errors (double-click opens the file in VS Code). **History** keeps the last 200 builds.
**Environment** checks every tool and lets you point at a different toolchain, Zephyr base or SDK.

Buttons: **Build** (Ctrl+B/F5), **Stop**, **Clean**, **Open binary folder**, **Flash**, **Debug server**
(starts a GDB server and copies the `arm-none-eabi-gdb ... target extended-remote :3333` command), **Copy commands**.

## Board diagram

The **Board diagram** tab draws the selected flight controller and redraws it whenever the board changes:
- **Header:** board name, main MCU (core, clock, memory), and a summary row: IMUs, barometers, compasses,
  serial ports, outputs, CAN, battery sensing, microSD, IO MCU yes/no.
- **Main MCU** drawn as a chip in the middle, with a pad for every pin the board uses on its edge.
- **Left:** USB, the serial ports (SERIALn with role TELEM1/2, GPS1/2, ESC telemetry or spare; TX/RX/RTS/CTS),
  RC input, CAN, power and analog inputs, and LEDs, buzzer, heater and power-rail switches.
- **Right:** SPI buses (SCK/MISO/MOSI and each device's CS pin), I2C buses (internal or external, with device
  addresses), the SD card bus, and PWM outputs (AUX n on boards with an IO MCU, OUT n or Mn otherwise).
- **Device cards and icons:** every bus block is wired to a card per device with an icon: IMU (chip with
  X/Y/Z axes), compass (needle), barometer (gauge), power monitor, FRAM, microSD, OSD, GPS, telemetry radio,
  RC receiver, ESC, ground station. Pin rows of the output, analog and GPIO blocks each get their own icon:
  a motor (or servo) per output, battery / current / voltage meters, RSSI bars, airspeed tube, LEDs in their
  real colour, buzzer, heater, rail switch.
- **IO MCU:** a second chip on the far right, linked to the main MCU's UART (TX and RX arrows, 1.5 Mbaud),
  with its own pins: MAIN 1–8 motors, RC in, DSM in, SBUS out, the safety switch and LED, servo-rail and RSSI
  sensing, status LEDs.

Hover any pin or wire to see its full function. Drag to pan, and use Ctrl+wheel, **Zoom ±**, **100%** or
**Fit width** to zoom. **Pin table** lists every pin (sortable), and **Export SVG...** saves the diagram.

The data comes from the board's hardware description: `platforms/zephyr/boards/<vendor>/<board>/board_pins.json`
(written by `tools/import_board.py`), or the APX boards' own `platforms/stm32h7/boards/<board>/hwdef.dat` (and `iomcu.dat` for the IO MCU's
pin map). After changing the device model, refresh the imported boards with
`tools/import_board.py --hwdef-root DIR --pins-only` (rewrites only `board_pins.json`).
The parser and model are in `board_desc.py`; the layout and the canvas/SVG renderers are in `diagram.py`.

## CLI

```bash
python3 tools/apx_build.py list
python3 tools/apx_build.py options zephyr:cube_orange_plus
python3 tools/apx_build.py build zephyr:cube_orange_plus --profile Debug
python3 tools/apx_build.py build stm32h7:apx_h743_v1 -s build_type=Debug -s stack_usage=on --json stats.json
python3 tools/apx_build.py build host:sitl --pristine -j 8 --dry-run
python3 tools/apx_build.py stats zephyr:cube_orange_plus --full
python3 tools/apx_build.py open zephyr:cube_orange_plus
python3 tools/apx_build.py flash zephyr:cube_orange_plus --probe openocd_stlink
python3 tools/apx_build.py debug zephyr:cube_orange_plus
python3 tools/apx_build.py diagram pixhawk6x --os zephyr --svg pixhawk6x.svg   # pin table + SVG block diagram
python3 tools/apx_build.py doctor
```

`build` exits non-zero on failure (including a memory overflow), so it can be used in CI.
Target ids also accept a unique part of the name (`zephyr`, `apx_h743_v1`).

Settings and history are stored in `%APPDATA%\apx_builder` (Windows), `~/Library/Application Support/apx_builder` (macOS),
or `~/.config/apx_builder` (Linux). Set `APX_BUILDER_CONFIG_DIR` to use a different location.

## Layout

| File | Role |
|---|---|
| `tools/apx_build.py` | Launcher |
| `targets.py` | Finds the targets in the repo |
| `options.py` | Option schema, per-target defaults, profiles |
| `builder.py` | Turns options into commands and runs them (progress, diagnostics, stop) |
| `stats.py` | Artifacts, memory and stack statistics, history |
| `probe.py` | Flash and GDB-server commands for each probe backend |
| `env.py` | Tool discovery, Zephyr environment, settings, opening folders |
| `board_desc.py`, `diagram.py` | Board pin model; block-diagram layout (Tk canvas and SVG) |
| `cli.py`, `gui.py` | The two front ends |
