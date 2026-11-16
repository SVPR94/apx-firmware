# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
"""Compilation statistics: artifacts, memory usage, stack usage, history."""

from __future__ import annotations

import json
import re
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

from . import env as E
from .targets import Target

HISTORY_LIMIT = 200
# .su line: "<file>:<line>:<col>:<function signature>\t<bytes>\t<static|dynamic|bounded>"
SU_RE = re.compile(r"^(?P<file>.*?):(?P<line>\d+):(?P<col>\d+):(?P<func>.*)$")


# --------------------------------------------------------------------------
# Where things are
# --------------------------------------------------------------------------
def zephyr_image_dir(build_dir: Path, image: str = "app") -> Path:
    """Sysbuild layout: <build>/<image>/zephyr; plain west build: <build>/zephyr."""
    d = build_dir / image / "zephyr"
    return d if d.is_dir() or not (build_dir / "zephyr").is_dir() else build_dir / "zephyr"


def binary_dir(target: Target, build_dir: Path) -> Path:
    if target.kind == "zephyr":
        pkg = build_dir / "firmware"
        return pkg if pkg.is_dir() else zephyr_image_dir(build_dir)
    if target.kind == "baremetal":
        return build_dir / "platforms" / "stm32h7" / "boards" / target.board
    return build_dir / "main"


def _artifact_paths(target: Target, build_dir: Path) -> list[Path]:
    b = binary_dir(target, build_dir)
    if target.kind == "zephyr":
        if b.name == "firmware":  # packaged by sysbuild: everything to flash or ship
            return sorted(p for p in b.iterdir() if p.is_file())
        names = ["zephyr.elf", "zephyr.bin", "zephyr.hex", "zephyr.map", "zephyr.lst",
                 "apx_memory_report.txt", "apx_memory_report.json", ".config"]
        return [b / n for n in names] + sorted(b.glob("*.apj"))
    if target.kind == "baremetal":
        stem = f"apx_firmware_{target.board}"
        return [b / f"{stem}{ext}" for ext in (".elf", ".bin", ".hex", ".map", ".memory.txt",
                                                ".memory.json")] + sorted(b.glob("*.apj"))
    x = ".exe" if E.IS_WIN else ""
    paths = [build_dir / "main" / f"apx_flight_controller{x}", build_dir / "main" / f"apx_sitl{x}",
             build_dir / "sitl" / f"apx_sim_physics_runner{x}"]
    plug = build_dir / "sitl" / "gazebo_plugin"
    if plug.is_dir():
        paths += sorted(p for p in plug.iterdir() if p.suffix in (".so", ".dll", ".dylib"))
    tests = build_dir / "tests"
    if tests.is_dir():
        paths += sorted(p for p in tests.iterdir() if p.is_file() and p.name.startswith("test")
                        and (p.suffix == ".exe" or (not E.IS_WIN and not p.suffix)))
    return paths


def artifacts(target: Target, build_dir: Path, since: float = 0.0) -> list[dict]:
    out = []
    for p in _artifact_paths(target, build_dir):
        if p.is_file():
            st = p.stat()
            out.append({"name": p.name, "path": str(p), "size": st.st_size,
                        "mtime": st.st_mtime, "fresh": st.st_mtime >= since - 1})
    return out


def memory_report(target: Target, build_dir: Path, regenerate: bool = False, image: str = "app") -> dict | None:
    """Memory JSON written by the build (or produced on demand from elf + map).
    image: "app" (flight firmware) or "apx_bootloader" (Zephyr sysbuild builds)."""
    if target.kind == "host":
        return None
    b = zephyr_image_dir(build_dir, image) if target.kind == "zephyr" else binary_dir(target, build_dir)
    if target.kind == "zephyr":
        js, elf, mp, bn = (b / "apx_memory_report.json", b / "zephyr.elf", b / "zephyr.map",
                           b / "zephyr.bin")
    else:
        stem = f"apx_firmware_{target.board}"
        js, elf, mp, bn = b / f"{stem}.memory.json", b / f"{stem}.elf", b / f"{stem}.map", b / f"{stem}.bin"
    stale = js.exists() and elf.exists() and js.stat().st_mtime < elf.stat().st_mtime
    if (regenerate or not js.exists() or stale) and elf.exists() and mp.exists():
        argv = [sys.executable, str(E.MEMORY_REPORT_TOOL), "--elf", str(elf), "--map", str(mp),
                "--name", target.label, "--out-json", str(js)]
        if bn.exists():
            argv += ["--bin", str(bn)]
        subprocess.run(argv, capture_output=True, text=True)
    try:
        return json.loads(js.read_text())
    except (OSError, ValueError):
        return None


def stack_usage(build_dir: Path, since: float = 0.0, top: int = 25) -> list[dict]:
    rows = []
    for su in build_dir.rglob("*.su"):
        try:
            if su.stat().st_mtime < since - 1:
                continue
            for line in su.read_text(errors="replace").splitlines():
                parts = line.split("\t")
                if len(parts) < 3:
                    continue
                loc, size, qual = parts[0], parts[1], parts[2]
                m = SU_RE.match(loc)
                src, func = (m.group("file"), m.group("func")) if m else (su.stem, loc)
                rows.append({"function": func.strip(), "bytes": int(size), "qualifier": qual,
                             "file": Path(src).name + (f":{m.group('line')}" if m else "")})
        except (OSError, ValueError):
            continue
    rows.sort(key=lambda r: -r["bytes"])
    return rows[:top]


# --------------------------------------------------------------------------
# Memory summary helpers
# --------------------------------------------------------------------------
def _is_flash_region(r: dict) -> bool:
    n = r["name"].upper()
    return "FLASH" in n or "ROM" in n or ("x" in r.get("attrs", "") and "w" not in r.get("attrs", ""))


def memory_totals(mem: dict | None) -> dict:
    if not mem:
        return {}
    regions = [r for r in mem.get("regions", []) if r.get("used")]
    flash = [r for r in regions if _is_flash_region(r)]
    ram = [r for r in regions if not _is_flash_region(r)]
    out = {"image": mem.get("image_bytes"), "result": mem.get("result")}
    if flash:
        f = max(flash, key=lambda r: r["used"])
        out.update(flash_used=f["used"], flash_cap=f["capacity"])
    if ram:
        out.update(ram_used=sum(r["used"] for r in ram), ram_cap=sum(r["capacity"] for r in ram))
    return out


# --------------------------------------------------------------------------
# Report assembly + history
# --------------------------------------------------------------------------
def build_report(plan, state) -> dict:
    t = plan.target
    ok = state.returncode == 0 and not state.cancelled
    mem = memory_report(t, plan.build_dir) if ok else None
    report = {
        "target": t.id, "label": t.label, "mcu": t.mcu, "host": E.HOST,
        "ok": ok, "cancelled": state.cancelled, "returncode": state.returncode,
        "started": datetime.fromtimestamp(state.started).isoformat(timespec="seconds"),
        "duration": round(time.time() - state.started, 2),
        "steps": [{"name": n, "seconds": round(s, 2), "returncode": rc} for n, s, rc in state.steps],
        "units_compiled": state.units, "warnings": state.warnings, "errors": state.errors,
        "diagnostics": state.diagnostics[:500],
        "build_dir": str(plan.build_dir), "binary_dir": str(binary_dir(t, plan.build_dir)),
        "artifacts": artifacts(t, plan.build_dir, since=state.started),
        "memory": mem, "totals": memory_totals(mem),
        "bootloader_totals": memory_totals(memory_report(t, plan.build_dir, image="apx_bootloader"))
        if ok and t.kind == "zephyr" and plan.values.get("bootloader", True) else {},
        "stack": stack_usage(plan.build_dir, since=state.started) if plan.values.get("stack_usage") else [],
        "values": plan.values, "commands": plan.command_lines(),
    }
    report["previous"] = previous_build(t.id)
    return report


def _history_path() -> Path:
    return E.settings_dir() / "history.json"


def load_history() -> list[dict]:
    try:
        return json.loads(_history_path().read_text())
    except (OSError, ValueError):
        return []


def previous_build(target_id: str) -> dict | None:
    for h in reversed(load_history()):
        if h["target"] == target_id and h["ok"]:
            return h
    return None


def record(report: dict) -> None:
    tot = report.get("totals", {})
    v = report["values"]
    entry = {"time": report["started"], "target": report["target"], "ok": report["ok"],
             "cancelled": report["cancelled"], "duration": report["duration"],
             "warnings": report["warnings"], "errors": report["errors"],
             "config": v.get("build_type") or v.get("optimization") or "",
             "flash_used": tot.get("flash_used"), "flash_cap": tot.get("flash_cap"),
             "ram_used": tot.get("ram_used"), "ram_cap": tot.get("ram_cap"),
             "image": tot.get("image")}
    hist = load_history()[-(HISTORY_LIMIT - 1):] + [entry]
    try:
        _history_path().write_text(json.dumps(hist, indent=1))
    except OSError:
        pass


# --------------------------------------------------------------------------
# Text formatting (CLI)
# --------------------------------------------------------------------------
def fmt_bytes(n) -> str:
    if n is None:
        return "-"
    n = int(n)
    if abs(n) >= 1048576:
        return f"{n / 1048576:.2f} MiB"
    if abs(n) >= 1024:
        return f"{n / 1024:.1f} KiB"
    return f"{n} B"


def fmt_delta(now, before) -> str:
    if now is None or before is None:
        return ""
    d = now - before
    return " (no change)" if d == 0 else f" ({'+' if d > 0 else '-'}{fmt_bytes(abs(d))} vs last build)"


def format_text(rep: dict) -> str:
    w = 72
    lines = ["", "=" * w, f" Build statistics - {rep['label']}", "=" * w]
    res = "CANCELLED" if rep["cancelled"] else ("SUCCESS" if rep["ok"] else f"FAILED (rc={rep['returncode']})")
    lines.append(f" Result        : {res}")
    lines.append(f" Duration      : {rep['duration']:.1f} s  (" +
                 ", ".join(f"{s['name']} {s['seconds']:.1f}s" for s in rep["steps"]) + ")")
    lines.append(f" Units compiled: {rep['units_compiled']}")
    lines.append(f" Warnings      : {rep['warnings']}    Errors: {rep['errors']}")
    lines.append(f" Binary dir    : {rep['binary_dir']}")
    tot, prev = rep.get("totals") or {}, rep.get("previous") or {}
    if tot.get("flash_used") is not None:
        pct = 100 * tot["flash_used"] / tot["flash_cap"] if tot.get("flash_cap") else 0
        lines.append(f" Flash         : {fmt_bytes(tot['flash_used'])} / {fmt_bytes(tot['flash_cap'])} "
                     f"({pct:.1f}%){fmt_delta(tot['flash_used'], prev.get('flash_used'))}")
    if tot.get("ram_used") is not None:
        pct = 100 * tot["ram_used"] / tot["ram_cap"] if tot.get("ram_cap") else 0
        lines.append(f" RAM           : {fmt_bytes(tot['ram_used'])} / {fmt_bytes(tot['ram_cap'])} "
                     f"({pct:.1f}%){fmt_delta(tot['ram_used'], prev.get('ram_used'))}")
    btot = rep.get("bootloader_totals") or {}
    if btot.get("flash_used") is not None:
        pct = 100 * btot["flash_used"] / btot["flash_cap"] if btot.get("flash_cap") else 0
        lines.append(f" Bootloader    : {fmt_bytes(btot['flash_used'])} / {fmt_bytes(btot['flash_cap'])} "
                     f"({pct:.1f}%), RAM {fmt_bytes(btot.get('ram_used'))}")
    if tot.get("result"):
        lines.append(f" Memory check  : {tot['result']}")
    if rep["artifacts"]:
        lines.append(" Artifacts:")
        for a in rep["artifacts"]:
            lines.append(f"   {'*' if a['fresh'] else ' '} {a['name']:<34}{fmt_bytes(a['size']):>12}")
    if rep["stack"]:
        lines.append(" Deepest stack frames:")
        for s in rep["stack"][:10]:
            lines.append(f"   {s['bytes']:>7} B  {s['qualifier']:<8} {s['function'][:60]}  ({s['file']})")
    errs = [d for d in rep["diagnostics"] if d["sev"] == "error"][:10]
    if errs:
        lines.append(" First errors:")
        for d in errs:
            loc = f"{d['file']}:{d['line']}" if d["line"] else d["file"]
            lines.append(f"   {loc}: {d['msg'][:90]}")
    lines.append("=" * w)
    return "\n".join(lines)
