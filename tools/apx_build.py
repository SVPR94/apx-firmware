#!/usr/bin/env python3
# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
"""APX firmware build utility - GUI (no arguments) or CLI (see --help).

Runs on Windows, Linux and macOS with the Python standard library only
(Tkinter for the GUI). Examples:

  python3 tools/apx_build.py                       # GUI
  python3 tools/apx_build.py list
  python3 tools/apx_build.py build zephyr:cube_orange_plus --profile Debug
"""

import sys
from pathlib import Path

if sys.version_info < (3, 8):
    sys.exit("apx_build.py needs Python 3.8 or newer")

sys.path.insert(0, str(Path(__file__).resolve().parent))

from build_tools.cli import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
