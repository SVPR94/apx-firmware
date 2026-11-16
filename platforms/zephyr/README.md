# APX hardware firmware (Zephyr RTOS)

This is the APX flight firmware for real flight controllers, built on
[Zephyr RTOS](https://zephyrproject.org/) v4.4.2.

**First target:** CubePilot **Cube Orange+** (STM32H757, Cortex-M7 at
400 MHz), flashed through the board's existing APX onboard bootloader.

`platforms/stm32h7/` is the older bare-metal link test. It is not a flight
firmware; see `docs/assessment/stm32_firmware_audit.md`.

## Status: bring-up milestone 1

This build is for bring-up only. **It cannot fly, and it refuses to arm.**

**What works**

- **Boot:** boots behind the APX onboard bootloader. The image is linked at
  0x08020000; the bootloader keeps 0x08000000–0x0801FFFF.
- **Clock and power:** 24 MHz crystal → PLL → 400 MHz (VOS1), SMPS-direct
  core supply. These match the board's known-good configuration.
- **Power rails:** turns on the sensor 3V3 rail (PE3) and the peripheral 5V
  rail (PA8).
- **LED:** blinks the amber FMU LED at 1 Hz while the firmware runs.
- **MAVLink over USB:** runs APX's real MAVLink handler, parameter manager,
  command handling and mode manager.

**What the GCS is told**

- **Sensors:** none present.
- **Estimates:** no attitude and no position messages.
- **Battery:** unknown.
- **GPS:** `GPS_RAW_INT` reports no GPS.
- **Arming:** refused, with a STATUSTEXT saying why. The same warning
  repeats once a minute.

**Not implemented yet**

- **Sensors:** IMU, baro and compass drivers.
- **Motor outputs:** none at all. No PWM or DShot pin is configured.
- **RC input.**
- **CAN:** DroneCAN GPS (Here3/Here4).
- **Parameter storage:** FRAM not used yet, so parameters reset at reboot.
- **Logging.**

On the Cube, MAIN 1–8 and RCIN belong to the IOMCU co-processor (USART6).
The H757 drives only **AUX 1–6** directly (TIM1 CH1–4, TIM4 CH2–3).

## Set up (one time)

```bash
platforms/zephyr/setup_zephyr.sh
```

The script:

- creates `platforms/zephyr/workspace/` (gitignored, about 2.6 GB);
- installs Zephyr v4.4.2 with only the STM32 HAL and CMSIS modules;
- installs a Python 3.12 virtualenv (via `uv` when the system Python is
  older; Zephyr 4.4 needs 3.12);
- uses the pinned Arm GNU toolchain from `platforms/stm32h7/`.

## Build

```bash
source platforms/zephyr/workspace/env.sh
west build -b cube_orange_plus/stm32h757xx/m7 -d build/zephyr_cube_orange_plus platforms/zephyr/app
```

**Outputs** in `build/zephyr_cube_orange_plus/zephyr/`:

- `apx_cube_orange_plus.apj`: the file to flash through the bootloader
  (board id 1063, `AP_HW_CUBEORANGEPLUS`);
- `zephyr.bin`, `zephyr.hex`, `zephyr.elf`.

**Size:** about 218 KB of flash and 47 KB of RAM.

## Flash (through the bootloader)

1. **Make the vehicle safe:** remove the propellers, and disconnect the
   ESC signal leads or power.
2. **In APX Flight Manager:** Vehicle Setup → Firmware. Plug in the Cube's USB
   when asked, tick **Advanced settings**, choose **Custom firmware file…**
   and select the `.apj`. Ground Control Station works too: Install Firmware →
   *Load custom firmware*.
3. **Wait for completion:** the bootloader checks the board id, writes the
   image and reboots into APX.

**Recovery.** The APX onboard bootloader runs first on every reset and is
never overwritten. If an APX image misbehaves, reflash any `.apj`
(including apx) the same way.

## First boot checklist (props off)

| Check | Expected |
|---|---|
| Amber LED | blinks at 1 Hz a few seconds after power-up |
| USB device | `APX Cube Orange+`, CDC-ACM serial, VID:PID `2fe3:0004` (Zephyr's development ID; CubePilot's VID is reserved for Hex) |
| Connect | GCS: Application Settings → Comm Links → add a Serial link on that port (115200 is fine; USB ignores it), then Connect. Ground Control Station: select the COM port, Connect |
| Heartbeat | vehicle type hexarotor, mode STABILIZE |
| Messages | "APX bring-up build: no sensors or motor outputs yet, arming disabled" |
| Parameters | the full APX parameter list loads (values reset at reboot until FRAM storage is added) |
| Arm button | refused with "Arming refused: APX bring-up build has no sensors or motor outputs yet" |

If the LED never blinks, note what you see (LED state, whether a USB
device appears at all) and report it. Without an SWD probe, those
observations are the only diagnostics.

## Layout

```
platforms/zephyr/
  setup_zephyr.sh                         workspace setup (pinned Zephyr)
  boards/cubepilot/cube_orange_plus/      out-of-tree board definition
  app/                                    the firmware application
    CMakeLists.txt                        APX sources + .apj packaging
    prj.conf                              Zephyr configuration
    src/main.cpp                          board bring-up + 400 Hz loop
tools/make_apj.py                         .bin -> .apj packager
```

APX's portable code builds unchanged:

- **Kernel layer:** `kernel/src/timer.cpp` and `mutex.cpp` have Zephyr
  implementations: the 64-bit cycle counter for time, `k_mutex` for locking.
- **MAVLink:** `MavlinkHandler` takes a serial byte writer
  (`set_byte_writer`) and `receive_bytes()` for USB/UART links.
