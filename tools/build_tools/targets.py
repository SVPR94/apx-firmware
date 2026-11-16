# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
"""Firmware target catalogue, discovered from the repository.

  host:sitl / host:linux        native builds (CMake presets sitl / linux-release)
  stm32h7:<board>               bare-metal boards in platforms/stm32h7/boards/
  zephyr:<board>                Zephyr boards in platforms/zephyr/boards/<vendor>/<board>/

New boards (e.g. generated from a hwdef.dat) show up automatically.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from pathlib import Path

from .env import REPO_ROOT

# Build dirs that match CMakePresets.json, so preset users and this tool share them.
_PRESET_DIRS = {
    "host:sitl": "build/sitl",
    "host:linux": "build/linux-release",
    "stm32h7:apx_h743_v1": "build/stm32h7",
    "stm32h7:cube_orange_plus": "build/stm32h7_cube_orange_plus",
}


OS_LABELS = {"zephyr": "Zephyr RTOS", "baremetal": "Bare-metal (no RTOS)", "posix": "Host OS (POSIX)"}


@dataclass
class Target:
    """One (board, OS) build. The same board can appear once per OS."""
    id: str
    kind: str                 # host | baremetal | zephyr
    label: str
    board: str                # board dir name / Zephyr board qualifier / APX_BOARD
    mcu: str                  # e.g. STM32H757xx, or "host"
    build_dir: str            # relative to REPO_ROOT
    cmake_target: str | None = None
    description: str = ""
    hosts: tuple = ("linux", "windows", "darwin")
    flash_origin: int | None = None
    notes: list = field(default_factory=list)
    overwrites_bootloader: bool = False
    board_key: str = ""       # board name shared by all OS builds of it
    os: str = ""              # zephyr | baremetal | posix
    board_title: str = ""     # human-readable board name
    features: tuple = ()      # e.g. ("usb",) or ("uart_link",): GCS/bootloader link type

    @property
    def os_label(self) -> str:
        return OS_LABELS.get(self.os, self.os)

    @property
    def build_path(self) -> Path:
        return REPO_ROOT / self.build_dir


def _hwdef_mcu(board_dir: Path) -> str:
    try:
        for line in (board_dir / "hwdef.dat").read_text().splitlines():
            if line.startswith("MCU "):
                return line.split()[-1]
    except OSError:
        pass
    return "STM32H7xx"


def _linker_flash_origin(board_dir: Path) -> int | None:
    try:
        text = (board_dir / "linker.ld").read_text()
    except OSError:
        return None
    m = re.search(r"^\s*FLASH\s*\([^)]*\)\s*:\s*ORIGIN\s*=\s*(0x[0-9A-Fa-f]+)", text, re.M)
    return int(m.group(1), 16) if m else None


def _baremetal_targets() -> list[Target]:
    root = REPO_ROOT / "platforms" / "stm32h7" / "boards"
    out = []
    for d in sorted(p for p in root.iterdir() if p.is_dir()) if root.is_dir() else []:
        if not ((d / "CMakeLists.txt").exists() or (d / "hwdef.dat").exists()):
            continue
        tid = f"stm32h7:{d.name}"
        mcu = _hwdef_mcu(d)
        t = Target(id=tid, kind="baremetal", label=f"{d.name} ({mcu}) - bare-metal",
                   board_key=d.name, os="baremetal", board_title=d.name,
                   board=d.name, mcu=mcu,
                   build_dir=_PRESET_DIRS.get(tid, f"build/stm32h7_{d.name}"),
                   cmake_target=f"apx_firmware_{d.name}",
                   description=f"Bare-metal APX firmware for {d.name} ({mcu}, Cortex-M7)",
                   flash_origin=_linker_flash_origin(d))
        try:
            makes_apj = "make_apj.py" in (d / "CMakeLists.txt").read_text()
        except OSError:
            makes_apj = False
        if makes_apj and t.flash_origin == 0x08000000:
            t.overwrites_bootloader = True
            t.notes.append("Linked at 0x08000000 but packaged as .apj: the bootloader loads apps at "
                           "0x08020000, so the .apj will not boot and SWD flashing erases the "
                           "bootloader.")
        out.append(t)
    return out


def _zephyr_targets() -> list[Target]:
    root = REPO_ROOT / "platforms" / "zephyr" / "boards"
    out = []
    for yml in sorted(root.glob("*/*/board.yml")) if root.is_dir() else []:
        text = yml.read_text()
        m = re.search(r"^\s*name:\s*(\S+)", text, re.M)
        if not m:
            continue
        name = m.group(1)
        full = re.search(r"^\s*full_name:\s*(.+)$", text, re.M)
        full_name = full.group(1).strip() if full else name
        quals = []
        for dts in sorted(yml.parent.glob(f"{name}*.dts")):
            rest = dts.stem[len(name):].lstrip("_")
            quals.append("/".join([name] + [p for p in rest.split("_", 1) if p]) if rest else name)
        dts_text = "".join(p.read_text(errors="replace") for p in yml.parent.glob("*.dts"))
        features = ("usb",) if "zephyr,cdc-acm-uart" in dts_text else ("uart_link",)
        if "apx,iomcu-uart" in dts_text:
            features += ("iomcu",)
        if "factory-apj-bootloader" in dts_text:
            features += ("factory_bootloader",)
        for q in quals or [name]:
            yml_soc = re.search(r"^\s*-\s*name:\s*(\S+)", text, re.M)
            soc = q.split("/")[1] if "/" in q else (yml_soc.group(1) if yml_soc else "")
            soc = soc[:-2].upper() + soc[-2:] if soc.endswith("xx") else soc.upper()
            out.append(Target(
                id=f"zephyr:{q.replace('/', ':')}" if len(quals) > 1 else f"zephyr:{name}",
                kind="zephyr", label=f"{full_name} ({soc or name}) - Zephyr RTOS",
                board_key=name, os="zephyr", board_title=full_name, features=features,
                board=q, mcu=soc or name, build_dir=f"build/zephyr_{name}",
                description=f"APX flight firmware on Zephyr for {full_name}"
                            + (", IO coprocessor (main outputs + RC input on the IO MCU)" if "iomcu" in features else ""),
                flash_origin=None))
    return out


def _host_targets() -> list[Target]:
    note = "Host builds are developed on Linux; Windows/macOS are experimental."
    return [
        Target(id="host:sitl", kind="host", label="SITL simulation - host", board="sitl", mcu="host",
               board_key="sitl", os="posix", board_title="SITL simulation",
               build_dir=_PRESET_DIRS["host:sitl"],
               description="Autopilot + physics simulator + Gazebo bridge on this computer",
               hosts=("linux",), notes=[note]),
        Target(id="host:linux", kind="host", label="Linux companion - host", board="linux",
               board_key="linux", os="posix", board_title="Linux companion computer",
               mcu="host", build_dir=_PRESET_DIRS["host:linux"],
               description="Flight controller for embedded Linux (Raspberry Pi, Jetson)",
               hosts=("linux",), notes=[note]),
    ]


_OS_ORDER = {"zephyr": 0, "baremetal": 1, "posix": 2}

def has_factory_bootloader(target) -> bool:
    """Board sold with an .apj bootloader (devicetree: factory-apj-bootloader)."""
    return "factory_bootloader" in getattr(target, "features", ())


def discover() -> list[Target]:
    targets = _zephyr_targets() + _baremetal_targets() + _host_targets()
    # Same board under several OSes: share the most descriptive title.
    titles = {}
    for t in targets:
        if t.board_title and t.board_title != t.board_key:
            titles.setdefault(t.board_key, t.board_title)
    for t in targets:
        t.board_title = titles.get(t.board_key, t.board_title or t.board_key)
    return targets


def boards(targets: list[Target]) -> list[tuple]:
    """[(board_key, title, [targets ordered by OS])]: flight controllers A-Z, host builds last."""
    out = {}
    for t in targets:
        out.setdefault(t.board_key, []).append(t)
    rows = [(k, v[0].board_title, sorted(v, key=lambda t: _OS_ORDER.get(t.os, 9))) for k, v in out.items()]
    return sorted(rows, key=lambda r: (r[2][0].kind == "host", r[1].lower()))


def find(targets: list[Target], query: str, os: str | None = None) -> Target:
    """Exact id, board name (+ os), or a unique id substring ('cube' alone is ambiguous)."""
    if os:
        targets = [t for t in targets if t.os == os or t.kind == os]
    for t in targets:
        if t.id == query:
            return t
    exact = [t for t in targets if t.board_key == query]
    if len(exact) == 1:
        return exact[0]
    hits = exact or [t for t in targets if query.lower() in t.id.lower()]
    if len(hits) == 1:
        return hits[0]
    if not hits:
        raise KeyError(f"no target matches '{query}'. Run 'list' to see targets.")
    raise KeyError(f"'{query}' is ambiguous: " + ", ".join(t.id for t in hits)
                   + " (add --os zephyr|baremetal)")
