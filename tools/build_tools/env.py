# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
"""Host environment: repo paths, tool discovery, Zephyr env, settings dir.

Everything here is cross-platform (Windows / Linux / macOS) and uses the
standard library only.
"""

from __future__ import annotations

import json
import os
import platform
import shutil
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
HOST = platform.system().lower()  # "linux" | "windows" | "darwin"
IS_WIN = HOST == "windows"
IS_MAC = HOST == "darwin"

ZEPHYR_WS = REPO_ROOT / "platforms" / "zephyr" / "workspace"
ZEPHYR_APP = REPO_ROOT / "platforms" / "zephyr" / "app"
ARM_TOOLCHAIN_ROOT = REPO_ROOT / "platforms" / "stm32h7" / "toolchain"
STM32H7_TOOLCHAIN_FILE = REPO_ROOT / "platforms" / "stm32h7" / "toolchain-arm-none-eabi.cmake"
MEMORY_REPORT_TOOL = REPO_ROOT / "tools" / "fw_memory_report.py"

DEFAULT_ENV_SETTINGS = {
    "arm_toolchain": "",          # dir containing bin/arm-none-eabi-gcc ("" = auto)
    "zephyr_base": "",            # "" = platforms/zephyr/workspace/zephyr
    "zephyr_toolchain_variant": "gnuarmemb",  # gnuarmemb | zephyr
    "zephyr_sdk": "",             # Zephyr SDK dir when variant = zephyr
    "west": "",                   # explicit west executable ("" = workspace venv)
}


def exe(name: str) -> str:
    return name + ".exe" if IS_WIN and not name.endswith(".exe") else name


def cpu_count() -> int:
    return max(1, os.cpu_count() or 1)


def which(name: str, extra_dirs=()) -> str | None:
    path = os.pathsep.join([str(d) for d in extra_dirs if d] + [os.environ.get("PATH", "")])
    return shutil.which(name, path=path)


# --------------------------------------------------------------------------
# Toolchains
# --------------------------------------------------------------------------
def arm_toolchain_dir(env_settings: dict | None = None) -> Path | None:
    """Root of the Arm GNU toolchain (has bin/arm-none-eabi-gcc)."""
    env_settings = env_settings or {}
    gcc = exe("arm-none-eabi-gcc")
    candidates = []
    if env_settings.get("arm_toolchain"):
        candidates.append(Path(env_settings["arm_toolchain"]))
    if os.environ.get("APX_ARM_TOOLCHAIN_BIN"):
        candidates.append(Path(os.environ["APX_ARM_TOOLCHAIN_BIN"]).parent)
    if ARM_TOOLCHAIN_ROOT.is_dir():
        candidates += sorted((p for p in ARM_TOOLCHAIN_ROOT.iterdir() if p.is_dir()), reverse=True)
    for c in candidates:
        if (c / "bin" / gcc).exists():
            return c
    found = which(gcc)
    return Path(found).resolve().parent.parent if found else None


def arm_tool(name: str, env_settings: dict | None = None) -> str | None:
    root = arm_toolchain_dir(env_settings)
    return which(exe("arm-none-eabi-" + name), [root / "bin"] if root else [])


def venv_bin() -> Path:
    return ZEPHYR_WS / ".venv" / ("Scripts" if IS_WIN else "bin")


def zephyr_base(env_settings: dict) -> Path:
    return Path(env_settings.get("zephyr_base") or ZEPHYR_WS / "zephyr")


def west_command(env_settings: dict) -> list[str] | None:
    """argv prefix that runs west (workspace venv preferred)."""
    if env_settings.get("west"):
        return [env_settings["west"]]
    py = venv_bin() / exe("python")
    if py.exists():
        return [str(py), "-m", "west"]
    w = which("west")
    return [w] if w else None


def zephyr_env(env_settings: dict) -> dict:
    """Process environment equivalent to sourcing workspace/env.sh."""
    env = dict(os.environ)
    extra = []
    if venv_bin().is_dir():
        extra.append(str(venv_bin()))
        env["VIRTUAL_ENV"] = str(venv_bin().parent)
    env["ZEPHYR_BASE"] = str(zephyr_base(env_settings))
    # APX boards for every sysbuild image (platforms/zephyr/zephyr/module.yml)
    env["ZEPHYR_EXTRA_MODULES"] = str(REPO_ROOT / "platforms" / "zephyr")
    variant = env_settings.get("zephyr_toolchain_variant") or "gnuarmemb"
    env["ZEPHYR_TOOLCHAIN_VARIANT"] = variant
    if variant == "gnuarmemb":
        root = arm_toolchain_dir(env_settings)
        if root:
            env["GNUARMEMB_TOOLCHAIN_PATH"] = str(root)
            extra.append(str(root / "bin"))
    elif env_settings.get("zephyr_sdk"):
        env["ZEPHYR_SDK_INSTALL_DIR"] = env_settings["zephyr_sdk"]
    env["PATH"] = os.pathsep.join(extra + [env.get("PATH", "")])
    return env


def cmake_env(env_settings: dict) -> dict:
    env = dict(os.environ)
    root = arm_toolchain_dir(env_settings)
    if root:
        env["APX_ARM_TOOLCHAIN_BIN"] = str(root / "bin")
    return env


# --------------------------------------------------------------------------
# Generators (build backends)
# --------------------------------------------------------------------------
GENERATORS = ("Auto", "Ninja", "Unix Makefiles", "MinGW Makefiles", "NMake Makefiles")


def resolve_generator(choice: str) -> str:
    if choice and choice != "Auto":
        return choice
    if which("ninja"):
        return "Ninja"
    if IS_WIN:
        return "MinGW Makefiles" if which("mingw32-make") else "NMake Makefiles"
    return "Unix Makefiles"


# --------------------------------------------------------------------------
# Misc helpers
# --------------------------------------------------------------------------
def open_path(path: Path) -> None:
    """Open a folder/file in the OS file manager / default application."""
    path = Path(path)
    if IS_WIN:
        os.startfile(str(path))  # noqa: S606 - user-requested open
    elif IS_MAC:
        subprocess.Popen(["open", str(path)])
    else:
        subprocess.Popen(["xdg-open", str(path)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def open_in_editor(path: str, line: int | None = None) -> None:
    code = which("code")
    if code and line:
        subprocess.Popen([code, "-g", f"{path}:{line}"])
    elif code:
        subprocess.Popen([code, path])
    else:
        open_path(Path(path))


def tool_version(argv: list[str]) -> str:
    try:
        out = subprocess.run(argv, capture_output=True, text=True, timeout=15)
        text = (out.stdout or out.stderr).strip().splitlines()
        return text[0].strip() if text else ""
    except (OSError, subprocess.SubprocessError):
        return ""


def settings_dir() -> Path:
    if os.environ.get("APX_BUILDER_CONFIG_DIR"):
        d = Path(os.environ["APX_BUILDER_CONFIG_DIR"])
    elif IS_WIN:
        d = Path(os.environ.get("APPDATA", Path.home())) / "apx_builder"
    elif IS_MAC:
        d = Path.home() / "Library" / "Application Support" / "apx_builder"
    else:
        d = Path(os.environ.get("XDG_CONFIG_HOME", Path.home() / ".config")) / "apx_builder"
    d.mkdir(parents=True, exist_ok=True)
    return d


def load_settings() -> dict:
    try:
        data = json.loads((settings_dir() / "settings.json").read_text())
    except (OSError, ValueError):
        data = {}
    data.setdefault("values", {})
    data["env"] = {**DEFAULT_ENV_SETTINGS, **data.get("env", {})}
    data.setdefault("warn_pct", 85)
    return data


def save_settings(data: dict) -> None:
    try:
        (settings_dir() / "settings.json").write_text(json.dumps(data, indent=2))
    except OSError:
        pass


# --------------------------------------------------------------------------
# Doctor
# --------------------------------------------------------------------------
def doctor(env_settings: dict) -> list[dict]:
    """Check every tool the builder can use. Returns rows for display."""
    rows = []

    def add(name, path, needed, version_args=("--version",), ok=None, detail=""):
        version = tool_version([path, *version_args]) if path and version_args else ""
        rows.append({"tool": name, "ok": bool(path) if ok is None else ok,
                     "detail": detail or (f"{version}  [{path}]" if path else "not found"),
                     "needed": needed})

    rows.append({"tool": "Python", "ok": sys.version_info >= (3, 8),
                 "detail": f"{platform.python_version()} [{sys.executable}]", "needed": "this tool"})
    try:
        import tkinter  # noqa: F401
        rows.append({"tool": "Tkinter", "ok": True, "detail": f"Tk {tkinter.TkVersion}", "needed": "GUI"})
    except ImportError:
        rows.append({"tool": "Tkinter", "ok": False, "detail": "missing (install python3-tk)",
                     "needed": "GUI"})
    add("CMake", which("cmake"), "all builds")
    add("Ninja", which("ninja"), "fast builds (optional)")
    make = which("make") or which("mingw32-make") or which("nmake")
    add("Make", make, "fallback backend", version_args=("--version",) if make and "nmake" not in make else ())
    gcc = arm_tool("gcc", env_settings)
    add("arm-none-eabi-gcc", gcc, "STM32H7 / Zephyr")
    add("arm-none-eabi-gdb", arm_tool("gdb", env_settings) or which("gdb-multiarch"), "debugging (optional)")
    host_cc = which("c++") or which("g++") or which("clang++") or which("cl")
    add("Host C++ compiler", host_cc, "SITL / Linux", version_args=() if host_cc and host_cc.endswith("cl.exe") else ("--version",))
    west = west_command(env_settings)
    zb = zephyr_base(env_settings)
    rows.append({"tool": "west", "ok": bool(west), "needed": "Zephyr",
                 "detail": " ".join(west) if west else "not found (run platforms/zephyr/setup_zephyr.sh)"})
    rows.append({"tool": "Zephyr base", "ok": (zb / "VERSION").exists(), "needed": "Zephyr",
                 "detail": f"{_zephyr_version(zb)}  [{zb}]" if (zb / "VERSION").exists() else f"missing [{zb}]"})
    add("OpenOCD", which("openocd"), "flash/debug (optional)")
    add("pyOCD", which("pyocd"), "flash/debug (optional)")
    jl = which("JLinkGDBServerCL") or which("JLinkGDBServerCLExe") or which("JLinkGDBServer")
    add("SEGGER J-Link", jl, "flash/debug (optional)", version_args=())
    add("STM32CubeProgrammer", which("STM32_Programmer_CLI"), "flash (optional)", version_args=("--version",))
    add("dfu-util", which("dfu-util"), "USB DFU flash (optional)", version_args=("--version",))
    return rows


def _zephyr_version(zb: Path) -> str:
    try:
        kv = dict(map(str.strip, line.split("=", 1)) for line in (zb / "VERSION").read_text().split("\n") if "=" in line)
        return "Zephyr {}.{}.{}".format(*(kv.get(k, "?").strip() for k in
                                          ("VERSION_MAJOR", "VERSION_MINOR", "PATCHLEVEL")))
    except (OSError, ValueError):
        return "Zephyr"
