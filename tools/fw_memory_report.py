#!/usr/bin/env python3
# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
"""Post-build firmware memory analysis for APX microcontroller images.

Reads the linked ELF (sections, load addresses, symbols) and the linker
.map file (resolved MEMORY regions, per-object contributions) and prints:

  * per-region usage (flash / RAM / TCM ...) with free space and % used
  * flash split into code / read-only data / initialised-data copy
  * RAM split into .data / .bss / no-init (stacks, heaps)
  * the components (libraries / APX source files) using the most memory
  * the largest flash and RAM symbols

It fails (exit 1) when anything overflows a region or the image does not
fit the flash available to it, and warns above --warn percent. Works for
both the Zephyr build (zephyr.elf + zephyr.map) and the bare-metal
STM32H7 boards. Pure standard library; no pyelftools needed.

Usage:
  fw_memory_report.py --elf zephyr.elf --map zephyr.map [--bin zephyr.bin]
                      [--region-limit FLASH=896K] [--warn 85]
                      [--name "Cube Orange+"] [--out-txt r.txt] [--out-json r.json]
"""

import argparse
import json
import re
import shutil
import struct
import subprocess
import sys
from pathlib import Path

SHT_SYMTAB, SHT_NOBITS = 2, 8
SHF_WRITE, SHF_ALLOC, SHF_EXECINSTR = 0x1, 0x2, 0x4
PT_LOAD = 1
STT_OBJECT, STT_FUNC = 1, 2


# --------------------------------------------------------------------------
# ELF32 little-endian reader (ARM Cortex-M)
# --------------------------------------------------------------------------
class Elf:
    def __init__(self, path):
        data = Path(path).read_bytes()
        if data[:4] != b"\x7fELF" or data[4] != 1 or data[5] != 1:
            raise SystemExit(f"fw_memory_report: {path} is not a 32-bit little-endian ELF")
        (_, _, _, self.entry, phoff, shoff, _, _, phentsize, phnum,
         shentsize, shnum, shstrndx) = struct.unpack_from("<HHIIIIIHHHHHH", data, 16)

        self.segments = []
        for i in range(phnum):
            p_type, _, vaddr, paddr, filesz, memsz, _, _ = struct.unpack_from(
                "<IIIIIIII", data, phoff + i * phentsize)
            if p_type == PT_LOAD:
                self.segments.append((vaddr, paddr, filesz, memsz))

        raw = [struct.unpack_from("<IIIIIIIIII", data, shoff + i * shentsize) for i in range(shnum)]
        strtab_off = raw[shstrndx][4]

        def cstr(off):
            end = data.index(b"\0", off)
            return data[off:end].decode("utf-8", "replace")

        self.sections = []
        symtab = None
        for idx, (name, stype, flags, addr, off, size, link, _, _, _) in enumerate(raw):
            sec = {"idx": idx, "name": cstr(strtab_off + name), "type": stype, "flags": flags,
                   "addr": addr, "size": size, "lma": addr}
            self.sections.append(sec)
            if stype == SHT_SYMTAB:
                symtab = (off, size, raw[link][4])
        for sec in self.sections:
            if sec["flags"] & SHF_ALLOC and sec["type"] != SHT_NOBITS:
                sec["lma"] = self._lma(sec["addr"])

        self.symbols = []
        if symtab:
            off, size, stroff = symtab
            for i in range(size // 16):
                name, value, ssize, info, _, shndx = struct.unpack_from("<IIIBBH", data, off + i * 16)
                stt = info & 0xF
                if ssize and stt in (STT_OBJECT, STT_FUNC) and 0 < shndx < len(self.sections):
                    if self.sections[shndx]["flags"] & SHF_ALLOC:
                        self.symbols.append({"name": cstr(stroff + name), "addr": value & ~1,
                                             "size": ssize, "func": stt == STT_FUNC,
                                             "section": self.sections[shndx]})

    def _lma(self, vaddr):
        for seg_v, seg_p, _, memsz in self.segments:
            if seg_v <= vaddr < seg_v + max(memsz, 1):
                return seg_p + (vaddr - seg_v)
        return vaddr

    def alloc_sections(self):
        return [s for s in self.sections if s["flags"] & SHF_ALLOC and s["size"]]


# --------------------------------------------------------------------------
# Linker map: memory regions + per input-object contributions
# --------------------------------------------------------------------------
REGION_RE = re.compile(r"^(\S+)\s+0x([0-9a-fA-F]+)\s+0x([0-9a-fA-F]+)\s*(\S*)\s*$")
INPUT_FULL_RE = re.compile(r"^ (\S+)\s+0x([0-9a-fA-F]+)\s+0x([0-9a-fA-F]+)\s+(\S.*)$")
INPUT_CONT_RE = re.compile(r"^\s+0x([0-9a-fA-F]+)\s+0x([0-9a-fA-F]+)\s+(\S.*)$")
INPUT_NAME_RE = re.compile(r"^ (\S+)\s*$")
OUTPUT_RE = re.compile(r"^([A-Za-z_.][^\s]*)")


def parse_map(path, alloc_names):
    lines = Path(path).read_text(errors="replace").splitlines()
    regions, contributions = [], []
    i = 0
    while i < len(lines) and lines[i].strip() != "Memory Configuration":
        i += 1
    i += 1
    while i < len(lines) and not lines[i].startswith("Linker script and memory map"):
        m = REGION_RE.match(lines[i].strip())
        if m and m.group(1) not in ("Name", "*default*"):
            regions.append({"name": m.group(1), "origin": int(m.group(2), 16),
                            "length": int(m.group(3), 16), "attrs": m.group(4)})
        i += 1

    current_out, pending_name = None, None
    for line in lines[i:]:
        if not line:
            continue
        if not line[0].isspace():
            m = OUTPUT_RE.match(line)
            current_out = m.group(1) if m else None
            pending_name = None
            continue
        if current_out not in alloc_names:
            continue
        m = INPUT_FULL_RE.match(line)
        if m:
            name, addr, size, src = m.group(1), int(m.group(2), 16), int(m.group(3), 16), m.group(4)
        else:
            m = INPUT_CONT_RE.match(line)
            if m and pending_name:
                name, addr, size, src = pending_name, int(m.group(1), 16), int(m.group(2), 16), m.group(3)
            else:
                m = INPUT_NAME_RE.match(line)
                pending_name = m.group(1) if m and not line.startswith("  ") else None
                continue
        pending_name = None
        if name == "*fill*" or not size or src.startswith("load address"):
            continue
        contributions.append({"out": current_out, "addr": addr, "size": size, "src": src.strip()})
    return regions, contributions


def component_of(src):
    """libfoo.a(bar.o) -> libfoo.a ; APX app sources are expanded per file."""
    m = re.match(r"^(.*?)([^/\\]+\.a)\((.+)\)$", src)
    if m:
        archive, obj = m.group(2), m.group(3)
        if archive == "libapp.a":
            return "app/" + re.sub(r"\.(c|cpp|cc|S|s)\.obj$", r".\1", obj)
        return archive
    if src.startswith("linker stubs") or src.startswith("*"):
        return "<linker>"
    return Path(src).name


# --------------------------------------------------------------------------
# Analysis
# --------------------------------------------------------------------------
def parse_size(text):
    m = re.fullmatch(r"(?i)\s*(0x[0-9a-f]+|\d+)\s*([KM]?)(i?B)?\s*", text)
    if not m:
        raise argparse.ArgumentTypeError(f"bad size '{text}'")
    val = int(m.group(1), 0)
    return val * {"": 1, "K": 1024, "M": 1024 * 1024}[m.group(2).upper()]


def fmt_b(n):
    if n >= 1024 * 1024:
        return f"{n / 1048576:.2f} MiB"
    if n >= 1024:
        return f"{n / 1024:.1f} KiB"
    return f"{n} B"


def is_flash(region):
    name = region["name"].upper()
    return "FLASH" in name or "ROM" in name or ("x" in region["attrs"] and "w" not in region["attrs"])


def region_for(regions, addr):
    for r in regions:  # map order: first match wins (Zephyr lists RAM before its alias SRAM0)
        if r["origin"] <= addr < r["origin"] + r["length"]:
            return r
    return None


def demangle(names):
    tool = shutil.which("arm-none-eabi-c++filt") or shutil.which("c++filt")
    if not tool or not names:
        return names
    try:
        out = subprocess.run([tool], input="\n".join(names), capture_output=True,
                             text=True, timeout=30, check=True).stdout.splitlines()
        return out if len(out) == len(names) else names
    except (OSError, subprocess.SubprocessError):
        return names


def analyse(args):
    elf = Elf(args.elf)
    alloc = elf.alloc_sections()
    regions, contribs = parse_map(args.map, {s["name"] for s in alloc})
    if not regions:
        raise SystemExit(f"fw_memory_report: no 'Memory Configuration' found in {args.map}")

    for r in regions:
        r["capacity"] = args.region_limit.get(r["name"], r["length"])
        r["placements"] = []
        r["sections"] = []

    flash_split = {"code": 0, "rodata": 0, "data_init": 0}
    ram_split = {"data": 0, "bss": 0, "noinit": 0}
    unplaced = []
    for s in alloc:
        r = region_for(regions, s["addr"])
        if r is None:
            unplaced.append(s["name"])
            continue
        r["placements"].append((s["addr"], s["addr"] + s["size"]))
        r["sections"].append((s["name"], s["addr"], s["size"]))
        nobits = s["type"] == SHT_NOBITS
        if is_flash(r):
            flash_split["code" if s["flags"] & SHF_EXECINSTR else "rodata"] += s["size"]
        elif nobits:
            ram_split["noinit" if "noinit" in s["name"] or "heap" in s["name"]
                      or "stack" in s["name"] else "bss"] += s["size"]
        else:
            ram_split["data"] += s["size"]
        if not nobits and s["lma"] != s["addr"]:  # initialised data copied from flash at boot
            lr = region_for(regions, s["lma"])
            if lr is not None:
                lr["placements"].append((s["lma"], s["lma"] + s["size"]))
                lr["sections"].append((s["name"] + " (load)", s["lma"], s["size"]))
                if is_flash(lr):
                    flash_split["data_init"] += s["size"]

    for r in regions:
        r["used"] = max((end for _, end in r["placements"]), default=r["origin"]) - r["origin"]
        if r["placements"]:
            r["used"] = max(r["used"], 0)

    # Components (from the map). Initialised data counts against flash too.
    by_lma = {s["name"]: s["lma"] != s["addr"] and s["type"] != SHT_NOBITS for s in alloc}
    comps = {}
    for c in contribs:
        r = region_for(regions, c["addr"])
        if r is None:
            continue
        e = comps.setdefault(component_of(c["src"]), {"flash": 0, "ram": 0})
        if is_flash(r):
            e["flash"] += c["size"]
        else:
            e["ram"] += c["size"]
            if by_lma.get(c["out"]):
                e["flash"] += c["size"]

    # Symbols
    seen, syms = set(), []
    for s in elf.symbols:
        key = (s["addr"], s["size"])
        if key in seen:
            continue
        seen.add(key)
        r = region_for(regions, s["addr"])
        if r is not None:
            syms.append({**s, "flash": is_flash(r), "region": r["name"]})
    top_flash = sorted((s for s in syms if s["flash"]), key=lambda s: -s["size"])[:args.top]
    top_ram = sorted((s for s in syms if not s["flash"]), key=lambda s: -s["size"])[:args.top]
    names = demangle([s["name"] for s in top_flash + top_ram])
    for s, n in zip(top_flash + top_ram, names):
        s["pretty"] = n

    image = Path(args.bin).stat().st_size if args.bin else None
    return elf, regions, flash_split, ram_split, comps, top_flash, top_ram, image, unplaced


# --------------------------------------------------------------------------
# Report
# --------------------------------------------------------------------------
def bar(pct, width=20):
    filled = min(width, int(round(pct / 100 * width)))
    return "[" + "#" * filled + "." * (width - filled) + "]"


def short(text, width):
    return text if len(text) <= width else text[:width - 3] + "..."


def build_report(args, elf, regions, flash_split, ram_split, comps, top_flash, top_ram, image, unplaced):
    out, problems, warnings = [], [], []
    w = 78
    out.append("=" * w)
    out.append(f" APX firmware memory report{(' - ' + args.name) if args.name else ''}")
    out.append("=" * w)
    out.append(f" ELF   : {args.elf}")
    if image is not None:
        out.append(f" Image : {args.bin} ({image:,} B)")
    out.append(f" Entry : 0x{elf.entry:08x}")
    out.append("")
    out.append(" Memory regions")
    out.append(f"  {'Region':<10}{'Origin':<12}{'Size':>11}{'Used':>12}{'Free':>12}{'Use%':>8}")
    shown = [r for r in regions if r["used"] or (is_flash(r) and r is regions[0])]
    for r in shown:
        cap, used = r["capacity"], r["used"]
        pct = 100.0 * used / cap if cap else 0.0
        note = "" if cap == r["length"] else f"  (limit {fmt_b(cap)})"
        out.append(f"  {r['name']:<10}0x{r['origin']:08x}  {fmt_b(cap):>11}{fmt_b(used):>12}"
                   f"{fmt_b(max(cap - used, 0)):>12}{pct:7.2f}%  {bar(pct)}{note}")
        if used > cap:
            problems.append(f"{r['name']} overflow: {used:,} B used > {cap:,} B available "
                            f"(over by {used - cap:,} B)")
        elif pct >= args.warn:
            warnings.append(f"{r['name']} is {pct:.1f}% full ({cap - used:,} B free)")
    empty = [r["name"] for r in regions if r not in shown]
    if empty:
        out.append(f"  unused: {', '.join(empty)}")

    flash_regions = [r for r in regions if is_flash(r)]
    if image is not None and flash_regions:
        fr = max(flash_regions, key=lambda r: r["used"])
        out.append("")
        pct = 100.0 * image / fr["capacity"] if fr["capacity"] else 0
        out.append(f" Image fit: {image:,} B of {fr['capacity']:,} B in {fr['name']} "
                   f"({pct:.2f}%, {fr['capacity'] - image:,} B free)")
        if image > fr["capacity"]:
            problems.append(f"image {image:,} B does not fit {fr['name']} ({fr['capacity']:,} B)")

    out.append("")
    tot_f = sum(flash_split.values())
    tot_r = sum(ram_split.values())
    out.append(f" Flash content {fmt_b(tot_f):>10}   code {fmt_b(flash_split['code'])}, "
               f"rodata {fmt_b(flash_split['rodata'])}, .data init {fmt_b(flash_split['data_init'])}")
    out.append(f" RAM content   {fmt_b(tot_r):>10}   .data {fmt_b(ram_split['data'])}, "
               f".bss {fmt_b(ram_split['bss'])}, noinit/stack/heap {fmt_b(ram_split['noinit'])}")

    if comps:
        out.append("")
        out.append(f" Top components by flash{'':<32}{'Flash':>10}{'RAM':>12}")
        for name, e in sorted(comps.items(), key=lambda kv: -kv[1]["flash"])[:args.top]:
            out.append(f"  {short(name, 52):<54}{fmt_b(e['flash']):>10}{fmt_b(e['ram']):>12}")
        out.append(f" Top components by RAM{'':<34}{'Flash':>10}{'RAM':>12}")
        for name, e in sorted(comps.items(), key=lambda kv: -kv[1]["ram"])[:args.top]:
            if e["ram"]:
                out.append(f"  {short(name, 52):<54}{fmt_b(e['flash']):>10}{fmt_b(e['ram']):>12}")

    for title, lst in (("flash", top_flash), ("RAM", top_ram)):
        if lst:
            out.append("")
            out.append(f" Largest {title} symbols")
            for s in lst:
                kind = "fn " if s["func"] else "obj"
                out.append(f"  {fmt_b(s['size']):>10}  {kind} {s['region']:<8} "
                           f"{short(s.get('pretty', s['name']), 54)}")

    if unplaced:
        warnings.append("sections outside every MEMORY region: " + ", ".join(unplaced))

    out.append("")
    for p in problems:
        out.append(f" ERROR  : {p}")
    for wmsg in warnings:
        out.append(f" WARNING: {wmsg}")
    verdict = "FAIL" if problems else ("PASS (with warnings)" if warnings else "PASS")
    out.append(f" Result : {verdict} - firmware {'does NOT fit' if problems else 'fits'} "
               f"in target memory (warn at {args.warn:g}%)")
    out.append("=" * w)

    data = {
        "name": args.name, "elf": str(args.elf), "image_bytes": image, "result": verdict,
        "regions": [{k: r[k] for k in ("name", "origin", "length", "capacity", "used", "attrs")}
                    for r in regions],
        "flash": flash_split, "ram": ram_split,
        "components": comps,
        "top_flash": [{"name": s.get("pretty", s["name"]), "size": s["size"]} for s in top_flash],
        "top_ram": [{"name": s.get("pretty", s["name"]), "size": s["size"]} for s in top_ram],
        "errors": problems, "warnings": warnings,
    }
    return "\n".join(out), data, bool(problems)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--elf", required=True, type=Path)
    p.add_argument("--map", required=True, type=Path)
    p.add_argument("--bin", type=Path, help="flashable image; checked against the flash region")
    p.add_argument("--name", default="", help="label for the report header")
    p.add_argument("--region-limit", action="append", default=[], metavar="NAME=SIZE",
                   help="override a region's usable size, e.g. FLASH=896K (repeatable)")
    p.add_argument("--warn", type=float, default=85.0, help="warn when a region is above this %%")
    p.add_argument("--top", type=int, default=10, help="rows in the component/symbol tables")
    p.add_argument("--out-txt", type=Path)
    p.add_argument("--out-json", type=Path)
    args = p.parse_args()

    limits = {}
    for item in args.region_limit:
        name, _, size = item.partition("=")
        limits[name] = parse_size(size)
    args.region_limit = limits

    text, data, failed = build_report(args, *analyse(args))
    print(text)
    if args.out_txt:
        args.out_txt.write_text(text + "\n")
    if args.out_json:
        args.out_json.write_text(json.dumps(data, indent=2) + "\n")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
