# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
"""Build option schema shared by the GUI and the CLI.

Each option declares which target kinds it applies to, its default (which
may differ per target or kind), and how it reaches the build:

  cmake    -> -D<VAR>=<value> on the CMake command line
  kconfig  -> line(s) in a generated Zephyr .conf fragment (EXTRA_CONF_FILE);
              only emitted when the value differs from the default, so
              prj.conf / board defaults are left alone
  cflags   -> compiler flags (CMAKE_PROJECT_INCLUDE add_compile_options for
              CMake builds, EXTRA_CFLAGS/EXTRA_CXXFLAGS for Zephyr)
  lflags   -> linker flags (host/bare-metal only)
  (none)   -> consumed by the builder / probe code itself
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from .env import GENERATORS, cpu_count

ALL = ("host", "baremetal", "zephyr")
CMAKE_KINDS = ("host", "baremetal")
HW = ("baremetal", "zephyr")


@dataclass
class Option:
    key: str
    label: str
    group: str
    kind: str                        # bool | enum | int | hex | str | text
    default: Any
    kinds: tuple = ALL
    choices: tuple = ()              # enum: ((value, label), ...)
    help: str = ""
    cmake: str | None = None
    kconfig: Any = None              # bool: "SYM" or [..]; int/str: "SYM"; enum: {value: [lines]}
    cflags: Any = None               # bool: [flags]; enum: {value: [flags]}
    lflags: Any = None
    minimum: int | None = None
    maximum: int | None = None
    targets: tuple | None = None     # restrict to these target ids
    requires: str | None = None      # target feature needed, e.g. "usb" / "uart_link"
    image: str = "app"               # Zephyr sysbuild image the kconfig lines go to

    def default_for(self, target) -> Any:
        d = self.default
        if isinstance(d, dict):
            for k in (target.id, target.kind, "*"):
                if k in d:
                    return d[k]
            return None
        return d() if callable(d) else d

    def applies(self, target) -> bool:
        if self.targets is not None:
            return target.id in self.targets
        if self.requires and self.requires not in getattr(target, "features", ()):
            return False
        return target.kind in self.kinds

    def choice_label(self, value) -> str:
        return dict(self.choices).get(value, str(value))

    def parse(self, text: str) -> Any:
        """Parse a CLI/str value into the option's type."""
        if self.kind == "bool":
            if isinstance(text, bool):
                return text
            low = str(text).strip().lower()
            if low in ("1", "on", "yes", "true", "y"):
                return True
            if low in ("0", "off", "no", "false", "n"):
                return False
            raise ValueError(f"{self.key}: expected on/off, got '{text}'")
        if self.kind == "int":
            val = int(str(text), 0)
            if self.minimum is not None and val < self.minimum:
                raise ValueError(f"{self.key}: {val} < minimum {self.minimum}")
            if self.maximum is not None and val > self.maximum:
                raise ValueError(f"{self.key}: {val} > maximum {self.maximum}")
            return val
        if self.kind == "hex":  # "" = keep the default
            s = str(text).strip()
            if not s:
                return ""
            try:
                val = int(s, 16)
            except ValueError:
                raise ValueError(f"{self.key}: '{s}' is not a hex number (e.g. 0x1209)") from None
            if self.maximum is not None and not 0 <= val <= self.maximum:
                raise ValueError(f"{self.key}: 0x{val:X} out of range 0..0x{self.maximum:X}")
            return f"0x{val:04X}"
        if self.kind == "enum":
            values = [c[0] for c in self.choices]
            if text in values:
                return text
            for v, label in self.choices:
                if str(text).lower() in (v.lower(), label.lower()):
                    return v
            raise ValueError(f"{self.key}: '{text}' not one of {', '.join(values)}")
        return str(text)


OPTIONS: list[Option] = [
    # ---------------------------------------------------------------- Build
    Option("build_type", "Build type", "Build", "enum",
           {"host:sitl": "Debug", "host": "Release", "baremetal": "MinSizeRel"},
           kinds=CMAKE_KINDS,
           choices=(("Debug", "Debug (-O0/-Og, asserts, symbols)"),
                    ("RelWithDebInfo", "RelWithDebInfo (-O2 -g)"),
                    ("Release", "Release (-O2/-O3)"),
                    ("MinSizeRel", "MinSizeRel (-Os)")),
           help="CMAKE_BUILD_TYPE. MinSizeRel is the default for flash-limited MCUs.",
           cmake="CMAKE_BUILD_TYPE"),
    Option("optimization", "Optimization", "Build", "enum", "size", kinds=("zephyr",),
           choices=(("size", "Size (-Os)"), ("speed", "Speed (-O2)"),
                    ("debug", "Debug (-Og)"), ("none", "None (-O0)")),
           help="Zephyr compiler optimization level (Kconfig choice).",
           kconfig={"size": ["CONFIG_SIZE_OPTIMIZATIONS=y"], "speed": ["CONFIG_SPEED_OPTIMIZATIONS=y"],
                    "debug": ["CONFIG_DEBUG_OPTIMIZATIONS=y"], "none": ["CONFIG_NO_OPTIMIZATIONS=y"]}),
    Option("generator", "Build backend", "Build", "enum", "Auto", kinds=CMAKE_KINDS,
           choices=tuple((g, "Auto (Ninja, else Make)" if g == "Auto" else g) for g in GENERATORS),
           help="CMake generator. Changing it on an existing build dir forces a clean build. "
                "Zephyr builds always use Ninja via west."),
    Option("jobs", "Parallel jobs", "Build", "int", cpu_count, minimum=1, maximum=256,
           help="Compiler processes run in parallel."),
    Option("pristine", "Clean (pristine) build", "Build", "bool", False,
           help="Delete the build directory and configure from scratch."),
    Option("verbose", "Verbose output", "Build", "bool", False,
           help="Show full compiler / linker command lines."),
    Option("build_dir", "Build directory", "Build", "str", None,
           help="Relative to the repo root. Default matches CMakePresets.json."),
    Option("compile_commands", "Export compile_commands.json", "Build", "bool", True, kinds=CMAKE_KINDS,
           help="For IDEs / clangd. Zephyr always generates it.",
           cmake="CMAKE_EXPORT_COMPILE_COMMANDS"),
    Option("lto", "Link-time optimization (experimental)", "Build", "bool", False, kinds=("zephyr",),
           help="CONFIG_LTO: smaller/faster image, slower link, harder debugging.", kconfig="CONFIG_LTO"),

    # -------------------------------------------------------------- Features
    Option("sitl_bridge", "SITL bridge + virtual sensors", "Features", "bool",
           {"host:sitl": True, "*": False}, kinds=("host",), cmake="APX_BUILD_SITL",
           help="APX_BUILD_SITL: physics runner, Gazebo plugin, simulated sensors."),
    Option("unit_tests", "Unit tests", "Features", "bool", {"host:sitl": True, "*": False},
           kinds=("host",), cmake="APX_BUILD_TESTS", help="APX_BUILD_TESTS (GoogleTest)."),
    Option("main_stack", "Main thread stack (bytes)", "Features", "int", 16384, kinds=("zephyr",),
           minimum=1024, maximum=262144, kconfig="CONFIG_MAIN_STACK_SIZE",
           help="CONFIG_MAIN_STACK_SIZE. The APX control loop runs on the main thread."),
    Option("warnings_as_errors", "Treat warnings as errors", "Features", "bool", False,
           kconfig="CONFIG_COMPILER_WARNINGS_AS_ERRORS", cflags=["-Werror"],
           help="-Werror (Zephyr: CONFIG_COMPILER_WARNINGS_AS_ERRORS)."),
    Option("stack_usage", "Per-function stack usage (-fstack-usage)", "Features", "bool", False,
           kinds=HW, cflags=["-fstack-usage"],
           help="Writes .su files; the Statistics tab lists the deepest functions."),

    # ----------------------------------------------------------------- Debug
    Option("debug_symbols", "Debug symbols", "Debug", "enum", "default", kinds=CMAKE_KINDS,
           choices=(("default", "As build type"), ("g", "-g (normal)"),
                    ("g3", "-g3 (incl. macros)"), ("g0", "None (-g0)")),
           cflags={"g": ["-g"], "g3": ["-g3"], "g0": ["-g0"]},
           help="Symbols live in the .elf only; they never increase the flashed image."),
    Option("frame_pointer", "Keep frame pointer", "Debug", "bool", False, kinds=CMAKE_KINDS,
           cflags=["-fno-omit-frame-pointer"], help="Better backtraces in GDB / profilers."),
    Option("keep_asserts", "Keep assert() in optimized builds", "Debug", "bool", False,
           kinds=CMAKE_KINDS, cflags=["-UNDEBUG"], help="Undefines NDEBUG after the build type sets it."),
    Option("sanitizer", "Sanitizers", "Debug", "enum", "none", kinds=("host",),
           choices=(("none", "None"), ("asan_ubsan", "Address + UndefinedBehavior"),
                    ("asan", "AddressSanitizer"), ("ubsan", "UndefinedBehaviorSanitizer"),
                    ("tsan", "ThreadSanitizer")),
           cflags={"asan_ubsan": ["-fsanitize=address,undefined", "-fno-omit-frame-pointer"],
                   "asan": ["-fsanitize=address", "-fno-omit-frame-pointer"],
                   "ubsan": ["-fsanitize=undefined"], "tsan": ["-fsanitize=thread"]},
           lflags={"asan_ubsan": ["-fsanitize=address,undefined"], "asan": ["-fsanitize=address"],
                   "ubsan": ["-fsanitize=undefined"], "tsan": ["-fsanitize=thread"]},
           help="GCC/Clang only (not MSVC). Use with Debug for SITL runs."),
    Option("thread_info", "Thread-aware debugging", "Debug", "bool", False, kinds=("zephyr",),
           kconfig=["CONFIG_DEBUG_THREAD_INFO", "CONFIG_THREAD_NAME"],
           help="CONFIG_DEBUG_THREAD_INFO + THREAD_NAME: OpenOCD/GDB can list Zephyr threads."),
    Option("asserts", "Kernel asserts", "Debug", "bool", False, kinds=("zephyr",),
           kconfig="CONFIG_ASSERT", help="CONFIG_ASSERT: __ASSERT() checks in kernel and drivers."),
    Option("stack_guard", "Stack overflow detection", "Debug", "enum", "mpu", kinds=("zephyr",),
           choices=(("mpu", "MPU guard (default)"), ("sentinel", "Software sentinel"),
                    ("none", "None")),
           kconfig={"sentinel": ["CONFIG_HW_STACK_PROTECTION=n", "CONFIG_STACK_SENTINEL=y"],
                    "none": ["CONFIG_HW_STACK_PROTECTION=n"]},
           help="The MPU guard faults on overflow in hardware. The sentinel (a magic word checked "
                "on context switch) needs the MPU guard off."),
    Option("stack_canaries", "Stack canaries", "Debug", "bool", False, kinds=("zephyr",),
           kconfig="CONFIG_STACK_CANARIES", help="CONFIG_STACK_CANARIES (-fstack-protector)."),
    Option("init_stacks", "Measure stack high-water mark", "Debug", "bool", False, kinds=("zephyr",),
           kconfig=["CONFIG_INIT_STACKS", "CONFIG_THREAD_STACK_INFO"],
           help="Fill stacks with 0xAA so unused stack can be measured from the debugger."),
    Option("exception_info", "Extra fault/exception info", "Debug", "bool", False, kinds=("zephyr",),
           kconfig="CONFIG_EXTRA_EXCEPTION_INFO", help="CONFIG_EXTRA_EXCEPTION_INFO: full register dump."),
    Option("disassembly", "Disassembly listing (zephyr.lst)", "Debug", "bool", False, kinds=("zephyr",),
           kconfig="CONFIG_OUTPUT_DISASSEMBLY", help="CONFIG_OUTPUT_DISASSEMBLY."),

    # ------------------------------------------------------------ Bootloader
    # Zephyr boards: the APX bootloader image built by sysbuild with the
    # firmware (platforms/zephyr/bootloader, Kconfig options APX_BL_*).
    Option("bootloader", "Build bootloader + combined image", "Bootloader", "bool", True, kinds=("zephyr",),
           help="Builds the APX bootloader with the firmware (sysbuild) and packages "
                "build/<dir>/firmware/: bootloader, firmware, .apj and the combined image."),
    Option("bl_boot_delay_ms", "Wait for GCS at power-up (ms)", "Bootloader", "int", 2000, kinds=("zephyr",),
           minimum=0, maximum=60000, kconfig="CONFIG_APX_BL_BOOT_DELAY_MS", image="apx_bootloader",
           help="How long the bootloader listens after a normal reset before starting a valid "
                "firmware. Long enough for USB to enumerate and a ground station to connect; "
                "shorter = faster boot."),
    Option("bl_stay_timeout_s", "Stay after reboot-to-bootloader (s)", "Bootloader", "int", 30,
           kinds=("zephyr",), minimum=1, maximum=3600, kconfig="CONFIG_APX_BL_STAY_TIMEOUT_S",
           image="apx_bootloader",
           help="After a MAVLink reboot-to-bootloader request, wait this long for an upload."),
    Option("bl_idle_timeout_s", "Start firmware after idle (s)", "Bootloader", "int", 30, kinds=("zephyr",),
           minimum=1, maximum=3600, kconfig="CONFIG_APX_BL_IDLE_TIMEOUT_S", image="apx_bootloader",
           help="Once a ground station has connected, start the (valid) firmware after this long "
                "without commands."),
    Option("bl_uart_baud", "UART link baud rate", "Bootloader", "enum", "115200", kinds=("zephyr",),
           requires="uart_link", image="apx_bootloader",
           choices=tuple((b, b) for b in ("57600", "115200", "230400", "460800", "921600")),
           kconfig={b: [f"CONFIG_APX_BL_UART_BAUD={b}"] for b in ("57600", "115200", "230400", "460800", "921600")},
           help="Baud rate of the bootloader on the TELEM UART. Uploaders use 115200 by default."),
    Option("bl_usb_vid", "USB vendor ID (hex)", "Bootloader", "hex", "", kinds=("zephyr",), requires="usb",
           maximum=0xFFFF, kconfig="CONFIG_CDC_ACM_SERIAL_VID", image="apx_bootloader",
           help="Empty = Zephyr's development ID (0x2FE3). Ground stations that detect bootloaders "
                "by USB ID (e.g. APX Flight Manager) need an ID they know."),
    Option("bl_usb_pid", "USB product ID (hex)", "Bootloader", "hex", "", kinds=("zephyr",), requires="usb",
           maximum=0xFFFF, kconfig="CONFIG_CDC_ACM_SERIAL_PID", image="apx_bootloader",
           help="Empty = default (0x0004)."),
    Option("bl_usb_product", "USB product string", "Bootloader", "str", "", kinds=("zephyr",), requires="usb",
           kconfig="CONFIG_CDC_ACM_SERIAL_PRODUCT_STRING", image="apx_bootloader",
           help="Empty = \"APX Bootloader (<board>)\". Shown by the host OS for the bootloader's serial port."),
    Option("bl_optimization", "Optimization", "Bootloader", "enum", "size", kinds=("zephyr",),
           image="apx_bootloader",
           choices=(("size", "Size (-Os)"), ("speed", "Speed (-O2)"), ("debug", "Debug (-Og)")),
           kconfig={"speed": ["CONFIG_SPEED_OPTIMIZATIONS=y"], "debug": ["CONFIG_DEBUG_OPTIMIZATIONS=y"]},
           help="Debug (-Og) for stepping through the bootloader with a probe. Must still fit 128 KB."),
    Option("bl_extra_kconfig", "Extra bootloader Kconfig", "Bootloader", "text", "", kinds=("zephyr",),
           help="One CONFIG_X=y per line, for the bootloader image only."),

    # ------------------------------------------------- Debug probe / backend
    Option("probe", "Debug probe backend", "Debug probe", "enum", "openocd_stlink", kinds=HW,
           choices=(("openocd_stlink", "OpenOCD + ST-Link"), ("openocd_cmsisdap", "OpenOCD + CMSIS-DAP"),
                    ("openocd_jlink", "OpenOCD + J-Link"), ("pyocd", "pyOCD"),
                    ("jlink", "SEGGER J-Link tools"), ("cubeprog", "STM32CubeProgrammer (flash only)"),
                    ("dfu", "USB DFU, no probe (dfu-util, flash only)")),
           help="Used by Flash and Debug server. ST-Link is the usual SWD probe. USB DFU needs no probe: "
                "hold the board's BOOT button while plugging in USB (typical for F4/H7 FPV and wing boards)."),
    Option("flash_image", "Image to flash", "Debug probe", "enum", "firmware", kinds=("zephyr",),
           choices=(("firmware", "Firmware only (keeps the bootloader)"),
                    ("combined", "Bootloader + firmware (first install)"),
                    ("bootloader", "Bootloader only")),
           help="What Flash writes over SWD. Use the combined image once on a board without a "
                "bootloader; afterwards update the firmware alone or by .apj over USB/UART."),
    Option("probe_device", "Probe target / device", "Debug probe", "str", None, kinds=HW,
           help="OpenOCD target cfg, pyOCD target or J-Link device name. Empty = from the MCU."),
    Option("swd_speed", "SWD speed (kHz)", "Debug probe", "int", 4000, kinds=HW, minimum=100,
           maximum=50000),
    Option("gdb_port", "GDB server port", "Debug probe", "int", 3333, kinds=HW, minimum=1,
           maximum=65535),

    # -------------------------------------------------------------- Advanced
    Option("extra_cflags", "Extra compiler flags", "Advanced", "str", "",
           help="Appended to every C/C++ compile, e.g. -DAPX_TRACE=1"),
    Option("extra_cmake", "Extra CMake arguments", "Advanced", "str", "",
           help="Passed verbatim to CMake configure, e.g. -DFOO=ON"),
    Option("extra_kconfig", "Extra Kconfig lines", "Advanced", "text", "", kinds=("zephyr",),
           help="One CONFIG_X=y per line, appended to the generated .conf fragment."),
]

BY_KEY = {o.key: o for o in OPTIONS}
GROUPS = ("Build", "Features", "Debug", "Bootloader", "Debug probe", "Advanced")

# Profiles: overrides on top of defaults, per target kind.
PROFILES = {
    "Defaults": {},
    "Debug": {
        "host": {"build_type": "Debug", "debug_symbols": "g3", "frame_pointer": True},
        "baremetal": {"build_type": "Debug", "debug_symbols": "g3", "stack_usage": True},
        "zephyr": {"optimization": "debug", "thread_info": True, "asserts": True,
                   "init_stacks": True, "exception_info": True,
                   "disassembly": True, "stack_usage": True},
    },
    "Release": {
        "host": {"build_type": "Release", "unit_tests": False},
        "baremetal": {"build_type": "Release"},
        "zephyr": {"optimization": "speed"},
    },
    "Smallest": {
        "host": {"build_type": "MinSizeRel"},
        "baremetal": {"build_type": "MinSizeRel", "debug_symbols": "g0"},
        "zephyr": {"optimization": "size", "lto": True},
    },
}


def options_for(target) -> list[Option]:
    return [o for o in OPTIONS if o.applies(target)]


def defaults(target) -> dict:
    vals = {o.key: o.default_for(target) for o in options_for(target)}
    vals["build_dir"] = target.build_dir
    return vals


def apply_profile(target, profile: str, base: dict | None = None) -> dict:
    vals = dict(defaults(target) if base is None else base)
    for k, v in PROFILES.get(profile, {}).get(target.kind, {}).items():
        if k in vals:
            vals[k] = v
    return vals


def merge(target, saved: dict | None) -> dict:
    """Defaults overlaid with saved/user values (unknown keys dropped)."""
    vals = defaults(target)
    for k, v in (saved or {}).items():
        if k in vals and v is not None:
            vals[k] = v
    return vals
