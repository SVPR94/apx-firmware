# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
"""Flash / debug-server commands for SWD probes (OpenOCD, pyOCD, J-Link, CubeProgrammer).

Only builds argv lists; the builder's runner executes them. Host targets
get a GDB/LLDB command line to run in a terminal instead.
"""

from __future__ import annotations

import os
from pathlib import Path

from . import env as E
from .builder import Step
from .stats import binary_dir
from .targets import Target

_JLINK_DEVICE = {"STM32H743XX": "STM32H743ZI", "STM32H757XX": "STM32H757XI_M7"}


def default_device(probe: str, mcu: str) -> str:
    mcu_u = mcu.upper()
    if probe.startswith("openocd"):
        return "target/stm32h7x.cfg"
    if probe == "pyocd":
        return mcu.lower()
    if probe == "jlink":
        return _JLINK_DEVICE.get(mcu_u, mcu_u.replace("XX", "XI"))
    return ""


def firmware_files(target: Target, build_dir: Path, image: str = "firmware") -> dict:
    """Files to flash. Zephyr image: firmware | combined | bootloader."""
    b = binary_dir(target, build_dir)
    if target.kind == "zephyr" and b.name == "firmware":
        name = target.board.split("/")[0]
        base = name if name.startswith("apx_") else f"apx_{name}"
        stem = {"firmware": base, "combined": f"{base}_combined", "bootloader": f"{base}_bootloader"}[image]
        elf = b / f"{stem}.elf"
        hexf = b / f"{stem}.hex"
        # The combined image has no ELF: program its HEX (absolute addresses).
        return {"elf": elf if elf.exists() else hexf, "hex": hexf, "bin": b / f"{stem}.bin"}
    stem = "zephyr" if target.kind == "zephyr" else f"apx_firmware_{target.board}"
    return {"elf": b / f"{stem}.elf", "hex": b / f"{stem}.hex", "bin": b / f"{stem}.bin"}


def flash_warning(target: Target, values: dict) -> str | None:
    """Message to confirm before flashing, or None."""
    from .targets import has_factory_bootloader
    if target.overwrites_bootloader:
        return target.notes[0]
    image = values.get("flash_image", "firmware")
    if target.kind == "zephyr" and image != "firmware" and has_factory_bootloader(target):
        return ("This board ships with a factory bootloader, which already supports .apj uploads. "
                f"Flashing the {image} image replaces it with the APX bootloader; restoring the "
                "original needs its binary and an SWD probe.")
    return None


def _openocd_base(values: dict, device: str) -> list:
    iface = {"openocd_stlink": "interface/stlink.cfg", "openocd_cmsisdap": "interface/cmsis-dap.cfg",
             "openocd_jlink": "interface/jlink.cfg"}[values["probe"]]
    argv = [E.which("openocd") or "openocd", "-f", iface]
    if values["probe"] != "openocd_stlink":  # stlink.cfg selects its own (HLA) SWD transport
        argv += ["-c", "transport select swd"]
    return argv + ["-f", device, "-c", f"adapter speed {values.get('swd_speed', 4000)}"]


def _jlink_exe(name_win: str, name_unix: str) -> str:
    return E.which(name_win if E.IS_WIN else name_unix) or (name_win if E.IS_WIN else name_unix)


def flash_step(target: Target, build_dir: Path, values: dict, scratch: Path) -> Step:
    files = firmware_files(target, build_dir, values.get("flash_image", "firmware"))
    missing = [str(p) for p in (files["elf"], files["hex"]) if not p.exists()]
    if missing:
        raise FileNotFoundError("build the firmware first, missing: " + ", ".join(missing))
    probe = values.get("probe", "openocd_stlink")
    device = values.get("probe_device") or default_device(probe, target.mcu)
    khz = values.get("swd_speed", 4000)
    elf, hexf = files["elf"].as_posix(), files["hex"].as_posix()
    if probe.startswith("openocd"):
        argv = _openocd_base(values, device) + ["-c", f"program {elf} verify reset exit"]
    elif probe == "pyocd":
        argv = [E.which("pyocd") or "pyocd", "flash", "-t", device, "-f", f"{khz}k", elf]
    elif probe == "dfu":
        # STM32 ROM bootloader over USB: hold BOOT0 while plugging in USB.
        binf = files["bin"]
        if not binf.exists():
            raise FileNotFoundError(f"build the firmware first, missing: {binf}")
        addr = image_address(target, build_dir, values.get("flash_image", "firmware"))
        argv = [E.which("dfu-util") or "dfu-util", "-a", "0", "-s", f"0x{addr:08x}:leave", "-D", binf.as_posix()]
    elif probe == "jlink":
        script = scratch / "jlink_flash.jlink"
        scratch.mkdir(parents=True, exist_ok=True)
        script.write_text(f"r\nh\nloadfile {hexf}\nverify\nr\ng\nexit\n")
        argv = [_jlink_exe("JLink.exe", "JLinkExe"), "-device", device, "-if", "SWD", "-speed",
                str(khz), "-autoconnect", "1", "-NoGui", "1", "-CommandFile", str(script)]
    else:  # cubeprog
        argv = [E.which("STM32_Programmer_CLI") or "STM32_Programmer_CLI", "-c", f"port=SWD freq={khz}",
                "-w", hexf, "-v", "-rst"]
    return Step("Flash", argv, dict(os.environ))


def image_address(target: Target, build_dir: Path, image: str) -> int:
    """Flash address of a packaged image (firmware/manifest.json), for raw .bin writes."""
    import json
    try:
        manifest = json.loads((binary_dir(target, build_dir) / "manifest.json").read_text())
        key = {"firmware": "firmware", "combined": "combined", "bootloader": "bootloader"}[image]
        return int(manifest["images"][key]["address"], 16)
    except (OSError, KeyError, ValueError):
        return 0x08000000


def debug_step(target: Target, build_dir: Path, values: dict) -> Step:
    probe = values.get("probe", "openocd_stlink")
    device = values.get("probe_device") or default_device(probe, target.mcu)
    port = int(values.get("gdb_port", 3333))
    khz = values.get("swd_speed", 4000)
    if probe.startswith("openocd"):
        argv = _openocd_base(values, device) + ["-c", f"gdb_port {port}"]
    elif probe == "pyocd":
        argv = [E.which("pyocd") or "pyocd", "gdbserver", "-t", device, "-p", str(port), "-f", f"{khz}k"]
    elif probe == "jlink":
        argv = [_jlink_exe("JLinkGDBServerCL.exe", "JLinkGDBServerCLExe"), "-device", device, "-if", "SWD",
                "-speed", str(khz), "-port", str(port), "-nogui"]
    else:
        raise ValueError("This backend has no GDB server; pick OpenOCD, pyOCD or J-Link.")
    return Step("GDB server", argv, dict(os.environ))


def gdb_hint(target: Target, build_dir: Path, values: dict, env_settings: dict) -> str:
    """Command the user runs (in a terminal) to attach GDB."""
    if target.kind == "host":
        x = ".exe" if E.IS_WIN else ""
        exe = build_dir / "main" / f"apx_flight_controller{x}"
        if E.IS_MAC or (not E.which("gdb") and E.which("lldb")):
            return f"lldb -- {exe}"
        return f"gdb --args {exe}"
    gdb = E.arm_tool("gdb", env_settings) or E.which("gdb-multiarch") or "arm-none-eabi-gdb"
    elf = firmware_files(target, build_dir)["elf"]
    return f'{gdb} {elf} -ex "target extended-remote :{values.get("gdb_port", 3333)}"'
