#!/usr/bin/env python3
# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
"""Package APX firmware images into one folder (run by sysbuild after a build).

  (<stem> = apx_<board>, without doubling an existing apx_ prefix)
  <out>/<stem>.{bin,hex,elf,apj}           flight firmware (application)
  <out>/<stem>_bootloader.{bin,hex,elf}    APX bootloader
  <out>/<stem>_combined.{bin,hex}          bootloader + firmware in one image,
                                                for the first install over SWD
  <out>/<stem>*_memory.txt                 memory reports
  <out>/manifest.json                           addresses, sizes, SHA-256

The combined image is merged from the two Intel HEX files, so each part
lands at the address it was linked for (bootloader at the start of flash,
firmware after it); gaps are filled with 0xFF (erased flash). Overlapping
images are an error.

Usage: make_firmware_package.py --board B --app-dir D [--bootloader-dir D] --out D
"""

import argparse
import hashlib
import json
import shutil
import sys
from pathlib import Path


def read_hex(path):
    """Intel HEX -> {address: byte}."""
    data = {}
    base = 0
    for lineno, line in enumerate(Path(path).read_text().splitlines(), 1):
        line = line.strip()
        if not line:
            continue
        if not line.startswith(":"):
            raise ValueError(f"{path}:{lineno}: not Intel HEX")
        raw = bytes.fromhex(line[1:])
        if sum(raw) & 0xFF:
            raise ValueError(f"{path}:{lineno}: bad checksum")
        count, addr, rtype, payload = raw[0], (raw[1] << 8) | raw[2], raw[3], raw[4:4 + raw[0]]
        if len(payload) != count:
            raise ValueError(f"{path}:{lineno}: truncated record")
        if rtype == 0:
            for i, b in enumerate(payload):
                data[base + addr + i] = b
        elif rtype == 1:
            break
        elif rtype == 2:
            base = int.from_bytes(payload, "big") << 4
        elif rtype == 4:
            base = int.from_bytes(payload, "big") << 16
        # 3 / 5: start address records - not needed for a flash image
    return data


def write_hex(path, data, start=None):
    """{address: byte} -> Intel HEX (32-byte records, type-4 extended addresses)."""
    lines = []
    upper = None
    addrs = sorted(data)
    i = 0
    while i < len(addrs):
        a = addrs[i]
        run = [data[a]]
        while i + len(run) < len(addrs) and addrs[i + len(run)] == a + len(run) and len(run) < 32 \
                and ((a + len(run)) & 0xFFFF) != 0:
            run.append(data[a + len(run)])
        if a >> 16 != upper:
            upper = a >> 16
            rec = bytes([2, 0, 0, 4]) + upper.to_bytes(2, "big")
            lines.append(":" + (rec + bytes([(-sum(rec)) & 0xFF])).hex().upper())
        rec = bytes([len(run), (a >> 8) & 0xFF, a & 0xFF, 0]) + bytes(run)
        lines.append(":" + (rec + bytes([(-sum(rec)) & 0xFF])).hex().upper())
        i += len(run)
    if start is not None:
        rec = bytes([4, 0, 0, 5]) + start.to_bytes(4, "big")
        lines.append(":" + (rec + bytes([(-sum(rec)) & 0xFF])).hex().upper())
    lines.append(":00000001FF")
    Path(path).write_text("\n".join(lines) + "\n")


def to_bin(data):
    lo, hi = min(data), max(data) + 1
    out = bytearray(b"\xff" * (hi - lo))
    for a, b in data.items():
        out[a - lo] = b
    return lo, bytes(out)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def copy(src, dst, manifest, role, kind):
    if Path(src).exists():
        shutil.copyfile(src, dst)
        manifest["files"].append({"file": Path(dst).name, "role": role, "type": kind,
                                  "size": Path(dst).stat().st_size, "sha256": sha256(dst)})
        return True
    return False


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--board", required=True)
    p.add_argument("--app-dir", required=True, type=Path, help="application zephyr/ build dir")
    p.add_argument("--bootloader-dir", type=Path, help="bootloader zephyr/ build dir")
    p.add_argument("--out", required=True, type=Path)
    a = p.parse_args()

    a.out.mkdir(parents=True, exist_ok=True)
    for old in a.out.glob("apx_*"):
        old.unlink()
    stem = a.board if a.board.startswith("apx_") else f"apx_{a.board}"
    manifest = {"board": a.board, "files": [], "images": {}}

    app_hex = a.app_dir / "zephyr.hex"
    if not app_hex.exists():
        sys.exit(f"make_firmware_package: {app_hex} missing")
    for ext in ("bin", "hex", "elf"):
        copy(a.app_dir / f"zephyr.{ext}", a.out / f"{stem}.{ext}", manifest, "firmware", ext)
    apj = next(iter(sorted(a.app_dir.glob("*.apj"))), None)
    if apj:
        copy(apj, a.out / f"{stem}.apj", manifest, "firmware", "apj")
        manifest["board_id"] = json.loads(apj.read_text()).get("board_id")
    copy(a.app_dir / "apx_memory_report.txt", a.out / f"{stem}_memory.txt", manifest, "firmware", "report")

    app = read_hex(app_hex)
    lo, img = to_bin(app)
    manifest["images"]["firmware"] = {"address": f"0x{lo:08x}", "size": len(img)}
    print(f"  firmware    0x{lo:08x}  {len(img):>8} B")

    if a.bootloader_dir:
        bl_hex = a.bootloader_dir / "zephyr.hex"
        if not bl_hex.exists():
            sys.exit(f"make_firmware_package: {bl_hex} missing")
        for ext in ("bin", "hex", "elf"):
            copy(a.bootloader_dir / f"zephyr.{ext}", a.out / f"{stem}_bootloader.{ext}", manifest, "bootloader", ext)
        copy(a.bootloader_dir / "apx_memory_report.txt", a.out / f"{stem}_bootloader_memory.txt",
             manifest, "bootloader", "report")
        bl = read_hex(bl_hex)
        bl_lo, bl_img = to_bin(bl)
        manifest["images"]["bootloader"] = {"address": f"0x{bl_lo:08x}", "size": len(bl_img)}
        print(f"  bootloader  0x{bl_lo:08x}  {len(bl_img):>8} B")

        overlap = set(bl) & set(app)
        if overlap:
            sys.exit(f"make_firmware_package: bootloader and firmware overlap at 0x{min(overlap):08x}")
        combined = {**bl, **app}
        c_lo, c_img = to_bin(combined)
        write_hex(a.out / f"{stem}_combined.hex", combined)
        (a.out / f"{stem}_combined.bin").write_bytes(c_img)
        for ext in ("bin", "hex"):
            f = a.out / f"{stem}_combined.{ext}"
            manifest["files"].append({"file": f.name, "role": "combined", "type": ext,
                                      "size": f.stat().st_size, "sha256": sha256(f)})
        manifest["images"]["combined"] = {"address": f"0x{c_lo:08x}", "size": len(c_img)}
        print(f"  combined    0x{c_lo:08x}  {len(c_img):>8} B  (bootloader + firmware, flash over SWD)")

    (a.out / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"make_firmware_package: {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
