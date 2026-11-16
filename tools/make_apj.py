#!/usr/bin/env python3
# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
"""Package a firmware .bin as an .apj file for an APX-standard bootloader.

The .apj is JSON: the image zlib-compressed and base64-encoded, plus the
board id the bootloader checks before accepting an upload. APX Flight Manager
and Ground Control Station can flash it as "custom firmware".

Usage: make_apj.py --bin zephyr.bin --board-id 1063 --out apx.apj [--summary TEXT]
"""

import argparse
import base64
import json
import subprocess
import time
import zlib
from pathlib import Path


def git_identity(repo):
    try:
        return subprocess.check_output(["git", "-C", str(repo), "rev-parse", "--short", "HEAD"],
                                       stderr=subprocess.DEVNULL, text=True).strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--bin", required=True, type=Path)
    p.add_argument("--board-id", required=True, type=int)
    p.add_argument("--out", required=True, type=Path)
    p.add_argument("--summary", default="APX")
    p.add_argument("--flash-total", type=int, default=0, help="bytes available for the image (0 = unchecked)")
    args = p.parse_args()

    image = args.bin.read_bytes()
    if args.flash_total and len(image) > args.flash_total:
        raise SystemExit(f"make_apj: image {len(image)} B exceeds flash {args.flash_total} B")
    desc = {
        "board_id": args.board_id,
        "magic": "APJFWv1",
        "description": f"APX firmware ({args.summary})",
        "image": base64.b64encode(zlib.compress(image, 9)).decode("ascii"),
        "summary": args.summary,
        "version": "0.1",
        "image_size": len(image),
        "board_revision": 0,
        "git_identity": git_identity(Path(__file__).resolve().parent.parent),
        "build_time": int(time.time()),
    }
    if args.flash_total:
        desc["flash_total"] = args.flash_total
        desc["image_maxsize"] = args.flash_total
        desc["flash_free"] = args.flash_total - len(image)
    args.out.write_text(json.dumps(desc, indent=4))
    print(f"make_apj: {args.out} ({len(image)} B image, board id {args.board_id})")


if __name__ == "__main__":
    main()
