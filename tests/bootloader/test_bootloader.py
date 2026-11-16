#!/usr/bin/env python3
# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
"""Host tests for the APX bootloader protocol core (bootloader/src/protocol.cpp).

Builds tests/bootloader/bl_sim (the core on simulated flash behind a pseudo-
terminal) and drives it with an independent protocol client:

  python3 tests/bootloader/test_bootloader.py
  python3 tests/bootloader/test_bootloader.py --uploader /path/to/uploader.py --apj fw.apj

--uploader additionally runs a ground-station command-line uploader against
the simulator, end to end. POSIX hosts only (pseudo-terminals).
"""

import argparse
import base64
import json
import os
import struct
import subprocess
import sys
import tempfile
import time
import zlib
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]

INSYNC, EOC, OK, FAILED, INVALID = 0x12, 0x20, 0x10, 0x11, 0x13
GET_SYNC, GET_DEVICE, CHIP_ERASE, PROG_MULTI, READ_MULTI, GET_CRC = 0x21, 0x22, 0x23, 0x27, 0x28, 0x29
GET_SN, GET_CHIP, GET_CHIP_DES, GET_SOFTWARE, REBOOT, CHIP_VERIFY = 0x2B, 0x2C, 0x2E, 0x2F, 0x30, 0x24


def crc32_raw(data, state=0):
    """CRC-32 reflected 0xEDB88320, init 0, no final inversion (independent check)."""
    for b in data:
        state ^= b
        for _ in range(8):
            state = (state >> 1) ^ (0xEDB88320 if state & 1 else 0)
    return state


class Sim:
    def __init__(self, exe, workdir, board_id=1063, app_size=65536, unit=32, timeout_ms=4000):
        self.image = Path(workdir) / f"flash_{time.monotonic_ns()}.bin"
        self.proc = subprocess.Popen(
            [str(exe), "--board-id", str(board_id), "--app-size", str(app_size), "--unit", str(unit),
             "--image", str(self.image), "--timeout-ms", str(timeout_ms)],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        self.pty = self.proc.stdout.readline().strip()
        self.fd = os.open(self.pty, os.O_RDWR | os.O_NOCTTY)
        import termios
        import tty
        tty.setraw(self.fd)
        attrs = termios.tcgetattr(self.fd)
        attrs[3] &= ~termios.ECHO
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)

    def send(self, *parts):
        os.write(self.fd, bytes(b for p in parts for b in (p if isinstance(p, (bytes, bytearray)) else [p])))

    def recv(self, n, timeout=25.0):
        out = b""
        deadline = time.time() + timeout
        import select
        while len(out) < n and time.time() < deadline:
            r, _, _ = select.select([self.fd], [], [], 0.05)
            if r:
                out += os.read(self.fd, n - len(out))
        if len(out) < n:
            raise TimeoutError(f"wanted {n} bytes, got {out!r}")
        return out

    def status(self):
        s = self.recv(2)
        assert s[0] == INSYNC, f"expected INSYNC, got {s!r}"
        return s[1]

    def word(self):
        v = struct.unpack("<I", self.recv(4))[0]
        assert self.status() == OK
        return v

    def text(self):
        n = struct.unpack("<I", self.recv(4))[0]
        t = self.recv(n).decode()
        assert self.status() == OK
        return t

    def finish(self):
        os.close(self.fd)
        rc = self.proc.wait(timeout=10)
        err = self.proc.stderr.read()
        return rc, self.image.read_bytes(), err


def build_sim(workdir):
    exe = Path(workdir) / "bl_sim"
    cxx = os.environ.get("CXX", "c++")
    cmd = [cxx, "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror", f"-I{ROOT / 'bootloader/include'}",
           str(HERE / "bl_sim.cpp"), str(ROOT / "bootloader/src/protocol.cpp"), "-o", str(exe), "-lutil"]
    subprocess.run(cmd, check=True)
    return exe


def check(name, cond):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}")
    if not cond:
        check.failed += 1


check.failed = 0


def test_protocol(exe, work):
    print("protocol core:")
    app_size = 65536
    s = Sim(exe, work, board_id=4242, app_size=app_size)
    s.send(GET_SYNC, EOC)
    check("GET_SYNC -> INSYNC OK", s.status() == OK)
    s.send(GET_SYNC, 0x00)
    check("bad terminator -> INVALID", s.status() == INVALID)
    s.send(0x99, 0x55)  # unknown bytes are ignored silently
    s.send(GET_DEVICE, 1, EOC)
    check("protocol revision 5", s.word() == 5)
    s.send(GET_DEVICE, 2, EOC)
    check("board id", s.word() == 4242)
    s.send(GET_DEVICE, 4, EOC)
    check("app size", s.word() == app_size)
    s.send(GET_DEVICE, 6, EOC)
    check("no external flash", s.word() == 0)
    s.send(GET_DEVICE, 9, EOC)
    check("unknown GET_DEVICE param -> INVALID", s.status() == INVALID)
    s.send(GET_SN, struct.pack("<I", 4), EOC)
    check("serial number word", s.word() == 0x41505804)
    s.send(GET_CHIP, EOC)
    check("chip id", s.word() == 0x10016450)
    s.send(GET_CHIP_DES, EOC)
    check("chip description", s.text() == "SIMULATED,X")
    s.send(GET_SOFTWARE, EOC)
    check("software string", s.text().startswith("APX bootloader"))
    s.send(PROG_MULTI, 4, b"\x01\x02\x03\x04", EOC)
    check("program before erase -> FAILED", s.status() == FAILED)
    s.send(CHIP_ERASE, EOC)
    check("erase -> OK", s.status() == OK)
    s.send(PROG_MULTI, 3, b"\x01\x02\x03", EOC)
    check("length not multiple of 4 -> INVALID", s.status() == INVALID)
    s.send(GET_CRC, EOC)
    check("CRC of erased area", s.word() == crc32_raw(b"\xff" * app_size))

    image = bytes((i * 7 + 3) & 0xFF for i in range(1000))  # 1000: last unit partial
    for off in range(0, len(image), 252):
        chunk = image[off:off + 252]
        s.send(PROG_MULTI, len(chunk), chunk, EOC)
        assert s.status() == OK
    s.send(GET_CRC, EOC)
    expect = crc32_raw(image + b"\xff" * (app_size - len(image)))
    check("CRC matches image + erased padding", s.word() == expect)
    s.send(CHIP_VERIFY, EOC)
    s.status()
    s.send(READ_MULTI, 64, EOC)
    data = s.recv(64)
    check("READ_MULTI returns image (incl. held first unit)", data == image[:64] and s.status() == OK)
    s.send(REBOOT, EOC)
    check("REBOOT -> OK", s.status() == OK)
    rc, flash, err = s.finish()
    check("simulator reports boot", rc == 0 and "result=boot" in err)
    check("flash holds the image", flash[:len(image)] == image)
    check("first unit programmed last", "first_unit_written_last=1" in err)

    print("interrupted upload:")
    s = Sim(exe, work, app_size=app_size, timeout_ms=1500)
    s.send(CHIP_ERASE, EOC)
    s.status()
    s.send(PROG_MULTI, 252, image[:252], EOC)
    s.status()
    rc, flash, err = s.finish()  # host disappears without REBOOT
    check("times out (no boot)", rc == 2)
    check("first unit still erased -> image not bootable", flash[:32] == b"\xff" * 32)
    # Whole program units after the first are written; the trailing partial
    # unit (bytes 224..251) waits for more data or REBOOT.
    check("complete units after the first were written", flash[32:224] == image[32:224])
    check("trailing partial unit not written yet", flash[224:256] == b"\xff" * 32)

    print("size limit:")
    s = Sim(exe, work, app_size=256)
    s.send(CHIP_ERASE, EOC)
    s.status()
    s.send(PROG_MULTI, 252, bytes(252), EOC)
    s.status()
    s.send(PROG_MULTI, 8, bytes(8), EOC)
    check("programming past the app area -> FAILED", s.status() == FAILED)
    s.send(REBOOT, EOC)
    s.status()
    s.finish()


def test_uploader(exe, work, uploader, apj):
    print(f"ground-station uploader ({Path(uploader).name}):")
    desc = json.loads(Path(apj).read_text())
    image = zlib.decompress(base64.b64decode(desc["image"]))
    s = Sim(exe, work, board_id=desc["board_id"], app_size=desc.get("flash_total", 917504), timeout_ms=20000)
    os.close(s.fd)
    r = subprocess.run([sys.executable, uploader, "--port", s.pty, "--no-extf", apj],
                       capture_output=True, text=True, timeout=300)
    rc = s.proc.wait(timeout=30)
    flash = s.image.read_bytes()
    err = s.proc.stderr.read()
    check("uploader exit code 0", r.returncode == 0)
    check("bootloader booted after upload", rc == 0 and "result=boot" in err)
    check("flash equals the .apj image", flash[:len(image)] == image)
    if r.returncode != 0:
        print(r.stdout[-2000:], r.stderr[-2000:])


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--uploader", help="ground-station command-line uploader script to test against")
    p.add_argument("--apj", help=".apj firmware for --uploader")
    args = p.parse_args()
    with tempfile.TemporaryDirectory() as work:
        exe = build_sim(work)
        test_protocol(exe, work)
        if args.uploader:
            if not args.apj:
                sys.exit("--uploader needs --apj")
            test_uploader(exe, work, args.uploader, args.apj)
    print("FAILED" if check.failed else "ALL PASSED", f"({check.failed} failure(s))" if check.failed else "")
    return 1 if check.failed else 0


if __name__ == "__main__":
    sys.exit(main())
