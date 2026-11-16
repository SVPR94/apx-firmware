# APX bootloader

The APX bootloader is a small program in the first 128 KB of flash. It lets
a ground station install new firmware over USB or a serial port (an `.apj`
file), so you don't need an SWD probe for updates. It speaks the standard
`.apj` upload protocol (revision 5) that APX Flight Manager and standard ground control stations and
the usual command-line uploaders use.

## Do I need it?

| Board | Bootloader today | Recommendation |
|---|---|---|
| Pixhawk-class: Cube Orange / Orange+, Pixhawk 6X / 6C, Durandal, SIYI N7, CUAV Nora / X7 / 7-Nano, ARK V6X | Factory bootloader, same protocol | **Not needed.** Keep the factory one and upload `.apj` files as usual (APX uses the same board ids). The APX bootloader is built too, but replacing the factory bootloader is optional. |
| FPV / wing: Matek H743, Kakute H7 / H7 Mini, MicoAir H743, Matek F405-Wing / F405-TE, SpeedyBee F405 V4 / Wing | Usually other firmware, no `.apj` bootloader | **Yes.** Flash the combined image once over USB DFU (hold BOOT while plugging in; no probe needed); then update by `.apj`. |
| APX H743 Pro | None (layout reserves 128 KB) | **Yes.** Flash the combined image once over SWD; then update by `.apj`. |
| APX H743 v1, Custom H7 FC | None | **Yes**, if you want updates without a probe. Flash the combined image once over SWD; then update by `.apj` over the TELEM UART. |

Without a bootloader, every update needs an SWD probe (`west flash`, or
**Flash** in `tools/apx_build.py`).

## How it is built

The bootloader is never built on its own. Building the firmware builds it as
well, in the same command (Zephyr *sysbuild*, the workspace default):

```bash
source platforms/zephyr/workspace/env.sh
west build --sysbuild -b apx_h743_pro -d build/zephyr_apx_h743_pro platforms/zephyr/app
```

Everything to flash or ship ends up in `build/zephyr_<board>/firmware/`:

| File | What | Flash with |
|---|---|---|
| `<stem>_combined.hex` / `.bin` | Bootloader + firmware, one image at 0x08000000 | SWD, once (first install or recovery) |
| `<stem>_bootloader.hex` / `.bin` / `.elf` | Bootloader only, 0x08000000 | SWD |
| `<stem>.apj` | Firmware for the bootloader | APX Flight Manager / standard GCS / uploader |
| `<stem>.hex` / `.bin` / `.elf` | Firmware only, 0x08020000 | SWD (keeps the bootloader) |
| `*_memory.txt`, `manifest.json` | Memory reports; addresses, sizes, SHA-256 | |

`<stem>` is `apx_<board>`, without doubling an existing `apx_` prefix
(for example `apx_h743_pro`, `apx_cube_orange_plus`).

Bootloader settings (listening window, timeouts, UART baud, USB IDs,
optimization) are Kconfig options in `platforms/zephyr/bootloader/Kconfig`.
They're also in the *Bootloader* group of `tools/apx_build.py`; see the table
in the [main README](../README.md#firmware-build-utility-gui--cli).

The build fails if the bootloader doesn't fit its partition: 128 KB on H7,
where it uses about 56 KB with USB or 27 KB with a UART link; 64 KB on F405,
where it uses about 50 KB, or if the two images overlap.
To skip the bootloader, build with `-DSB_CONFIG_APX_BOOTLOADER=n`, or untick
**Build bootloader** in `tools/apx_build.py`.

## First install (board without a bootloader)

1. **Flash the combined image over SWD or USB DFU.** In `tools/apx_build.py`, set
   **Image to flash** to *Bootloader + firmware* and click **Flash**. Without a
   probe, set **Debug probe backend** to *USB DFU* and hold the board's BOOT
   button while plugging in USB. On the command line:
   `python3 tools/apx_build.py flash apx_h743_pro --os zephyr -s flash_image=combined`
   (add `--probe dfu` for USB DFU).
2. **Power-cycle the board.** The status LED blinks fast for about 2 s (the
   bootloader is listening), then the firmware starts.

## Updating the firmware

- **APX Flight Manager / standard GCS:** use *custom firmware file* and pick
  the `.apj`. The ground station sends the MAVLink reboot-to-bootloader
  command; the firmware accepts it only when disarmed. Then the upload runs.
- **Command-line uploader:** `--port <USB or UART port>` and the `.apj`. On
  UART boards (APX H743 v1, Custom H7 FC) the bootloader listens on the TELEM
  port at 115200 baud.

The upload is power-loss safe. The first 32 bytes of the image (stack
pointer and reset vector) are written last, after the CRC check. An
interrupted upload therefore leaves an image the bootloader refuses to start,
and it waits for a new upload.

## Boot behaviour

| Situation | Bootloader does |
|---|---|
| Power-up, valid firmware | listens `CONFIG_APX_BL_BOOT_DELAY_MS` (2 s), then starts the firmware |
| Firmware requested reboot-to-bootloader | listens up to `CONFIG_APX_BL_STAY_TIMEOUT_S` (30 s) |
| No valid firmware | stays in the bootloader until one is uploaded |
| Upload finished | starts the new firmware |

The status LED blinks at 5 Hz while the bootloader runs. To start the firmware,
the bootloader records the request and resets the MCU, then jumps before any
driver starts, so the firmware begins with every peripheral in its reset state.

Firmware and bootloader share a 16-byte block at the end of DTCM
(`apx_boot_control` in each board's devicetree,
[include/apx/bootloader/boot_control.hpp](include/apx/bootloader/boot_control.hpp)).
It's read only after a warm reset. After power-on its contents are random, so
the bootloader initialises it. The bootloader also stores the reset-cause flags
there for the firmware.

## Layout and portability

```
bootloader/                       portable protocol core (no OS headers)
  include/apx/bootloader/port.hpp     what an OS port provides: link I/O, flash, time, id
  include/apx/bootloader/protocol.hpp commands, CRC, Protocol::run()
  src/protocol.cpp
platforms/zephyr/bootloader/      Zephyr port: Port on Zephyr flash/UART/USB, boot policy, jump
platforms/zephyr/app/sysbuild.cmake   builds firmware + bootloader together, packages firmware/
tools/make_firmware_package.py    combined image (HEX merge), manifest
tests/bootloader/                 host simulator + protocol tests
```

Moving to another OS means implementing `Port` (about 100 lines) and the
boot policy for that OS. The protocol core stays unchanged.

## Protocol reference

Every command ends with `0x20` (EOC). Every reply is the command's payload
(if any), then `0x12` (INSYNC), then a status byte: `0x10` OK, `0x11` FAILED,
or `0x13` INVALID. Multi-byte values are little-endian.

| Cmd | Name | Arguments | Reply payload |
|---|---|---|---|
| 0x21 | GET_SYNC | - | - |
| 0x22 | GET_DEVICE | 1: protocol rev, 2: board id, 3: board rev, 4: firmware area size, 6: external flash size | u32 |
| 0x23 / 0x40 | CHIP_ERASE / FULL_ERASE | - | - (replies when done; blank sectors are skipped) |
| 0x27 | PROG_MULTI | len (multiple of 4, ≤ 252), data | - |
| 0x29 | GET_CRC | - | u32: CRC-32 (0xEDB88320, init 0, no final XOR) over the whole firmware area, unwritten bytes 0xFF |
| 0x24 / 0x28 | CHIP_VERIFY / READ_MULTI | - / len | - / data |
| 0x2a | GET_OTP | u32 offset | u32 (0xFFFFFFFF: no OTP) |
| 0x2b | GET_SN | u32 offset | u32 word of the MCU unique id |
| 0x2c | GET_CHIP | - | u32 MCU id code |
| 0x2d | SET_BOOT_DELAY | u8 | - (accepted, not stored) |
| 0x2e | GET_CHIP_DES | - | u32 length + `"<soc>,<revision>"` |
| 0x2f | GET_SOFTWARE | - | u32 length + version string |
| 0x30 | REBOOT | - | - (writes the held first block, then starts the firmware) |
| 0x33 | SET_BAUD | u32 baud | - (UART links only) |
| 0x34–0x37 | external flash | | INVALID (no external flash) |

Unknown bytes are ignored without a reply. A known command with a bad
terminator gets INVALID.

## Tests

```bash
python3 tests/bootloader/test_bootloader.py
python3 tests/bootloader/test_bootloader.py --uploader <uploader.py> --apj build/zephyr_<board>/firmware/<stem>.apj
```

The first command checks every command and the error cases: bad terminator,
programming without an erase, size limit, and an interrupted upload leaving an
unbootable image. The second runs a ground-station command-line uploader end to
end against the simulator.
