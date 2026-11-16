# STM32 Firmware Audit and Hardware Port Plan

**Question (2026-10-20):** is everything compiled into and used by the
STM32 firmware, given that `apx_firmware_apx_h743_pro.hex` is only
24.4 KB?

## Findings

The bare-metal STM32 build (`platforms/stm32h7/`) is a **link test, not a
flight firmware**.

**The image** (`apx_h743_pro`, MinSizeRel):

- the `.hex` is 24,356 B; the binary inside is 8,636 B (Intel HEX text is
  about 2.8× the binary);
- `main()` runs the attitude, rate and position controllers and the mixer
  in a loop on constant inputs: level attitude, zero rates, quad frame;
- linked objects: attitude, rate, position and PID controllers, the motor
  mixer, plus a parameter-manager and mutex object.
- **Not linked:** flight modes, path tracker, navigation cascade,
  estimators, sensor manager, drivers, MAVLink, RC, failsafes, sprayer,
  logging, parameter storage.

**What is missing below the flight code**

| Layer | State |
|---|---|
| HAL (SPI/I2C/UART/PWM/CAN/storage/timer) | Interface headers only, zero implementation (`hal/stm32h7/` empty) |
| Clock tree | Not configured: the core runs on the 64 MHz reset clock |
| Scheduler | None: `main()` is a free-running loop |
| Motor output | `DshotDriver::transmit_frame()` is an empty placeholder. `set_throttle()` maps to `48 + value`, which can exceed 2047 |
| Sensor drivers | Device register logic exists for some ICs, unusable without a HAL |
| CAN GPS | No CAN driver, no DroneCAN |
| MAVLink on hardware | `write_bytes()` discarded everything; the clock came from `std::chrono`, whose bare-metal stub fails, so heartbeats would never send; most of `mavlink_handler.cpp` (state setters, parser) was compiled only for hosted builds |
| Parameters | Not persisted |
| Board files | `apx_h743_pro` lists 1 IMU while its hwdef has 3 IMU and 2 baro chip-selects; no CAN; SBUS baud 57600 (SBUS is 100000) |
| Flash layout | Linked at 0x08000000 although the hwdef reserves 128 KB for a bootloader |

**What already worked**

- **Cross-compile:** all 14 APX libraries compile for Cortex-M7 without
  errors.
- **Full-stack link test:** an image containing the real flight stack
  (parameters, MAVLink, every mode, mixer, altitude estimator, sensor
  manager) linked at 74 KB of flash.

**On size:** apx reaches 1–2 MB with an RTOS, hundreds of drivers,
scripting, multiple EKF cores, logging, OSD and more. Image size follows
features; it isn't a measure of readiness to fly.

## Decisions (user, 2026-10-20)

| Decision | Choice |
|---|---|
| Foundation | Zephyr RTOS |
| Board | CubePilot Cube Orange+ (STM32H757, CM7) |
| Flashing | Keep the APX onboard bootloader (`.apj` upload) |
| Debug probe | Not yet; bring-up over USB only |

## Done in this step: milestone 1 (boots and talks)

The firmware is in `platforms/zephyr/`; build, flash and checklist are in
its README.

- **Toolchain:** Zephyr v4.4.2 workspace with pinned modules
  (`platforms/zephyr/setup_zephyr.sh`), using the existing Arm GCC 15.2 and
  a uv-provided Python 3.12.
- **Board definition:** out-of-tree Cube Orange+.
  - **Clock and power:** 400 MHz, SMPS-direct. These are cross-checked
    against the board's apx configuration and ST's H757 evaluation
    board, because a wrong supply setting deadlocks the MCU.
  - **App partition:** at 0x08020000.
  - **USB:** OTG FS.
  - **LED and power rails:** amber LED; sensor and peripheral power
    enables.
- **Firmware:**
  - **Bring-up:** power rails and LED.
  - **MAVLink:** APX's real handler, parameter and mode code over USB
    CDC-ACM, driven by a 400 Hz kernel-timer loop.
  - **Status reporting:** the GCS is told "no sensors, no estimates,
    battery unknown, no GPS", and arming is refused.
- **Shared code** (SITL behaviour unchanged, 79/79 unit tests pass):
  - **Zephyr kernel shims:** 64-bit cycle-counter time and `k_mutex`.
  - **`MavlinkHandler` serial transport:** `set_byte_writer` and
    `receive_bytes`, using its own parse channel.
  - **Kernel-time MAVLink clock** on non-hosted builds.
  - **State setters and parser** now build for hardware.
  - **Reported-state controls:** sensor status, estimate validity,
    battery unknown and the arming block.
  - **`BATTERY_STATUS`** no longer truncates volts.
- **`tools/make_apj.py`:** an `.apj` packager (board id 1063).
- **Verified on the build output:**
  - vector table at 0x08020000;
  - reset vector inside the app;
  - initial SP in AXI SRAM;
  - `.apj` image byte-identical to `zephyr.bin`;
  - 218 KB flash, 47 KB RAM.
  - The first build linked at 0x08000000; this check caught it, and
    `CONFIG_USE_DT_CODE_PARTITION=y` fixed it.

**Not verified:** the firmware has not run on a Cube yet. No hardware or
probe is available here. The README's first-boot checklist is the
acceptance test.

## Next milestones

| # | Milestone | Content | Hardware check (props off) |
|---|---|---|---|
| 2 | Sensors | SPI1/SPI4 + I2C buses in the board DTS; drivers for the fitted IMUs (ICM-42688 ×2 / ICM-45686 or ICM-20649 variants), MS5611 ×2, AK09916 compass, in APX's driver layer on Zephyr SPI/I2C; IMU heater; `HIGHRES_IMU`/`SCALED_PRESSURE` telemetry | Values in GCS MAVLink Inspector, temperatures sane, rotations correct |
| 3 | Estimation | Attitude estimator, then the altitude estimator, from real sensors; sensor-manager voting bugs (driver cast, struct deviation) fixed | Horizon in GCS follows the board by hand |
| 4 | Outputs + RC | AUX 1–6 PWM/DShot on TIM1/TIM4 with arming interlocks; RC via SBUS/CRSF on a TELEM UART (RCIN needs the IOMCU) | Motor test without props, one motor at a time |
| 5 | GPS + storage | FDCAN1/2 + DroneCAN (libcanard) for Here3/Here4; parameters in FRAM (SPI2) | GPS fix, parameters survive reboot |
| 6 | Integration | Full flight stack on real sensors, failsafes, arming checks, logging (SD) | Tethered hover, then normal test flights |
