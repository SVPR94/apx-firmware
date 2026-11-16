# APX Build System & Toolchain Guide

This document covers prerequisites, toolchains, build presets, board compilation, and documentation building for the APX flight controller.

---

## 1. Build System Architecture

APX uses **CMake (>= 3.22)** with `CMakePresets.json` to provide consistent, reproducible builds across three distinct execution environments:

| Preset Name | Target System | Compiler | Primary Output |
|---|---|---|---|
| `sitl` | Host Linux (x86_64) | Native GCC / Clang | `main/apx_sitl`, `tests/unit/run_unit_tests`, Gazebo plugin |
| `linux-release` | Companion Computer (ARM/x86) | Native GCC / Clang (`-O3`) | `main/apx_flight_controller` |
| `stm32h7` | Cortex-M7 Bare-Metal | `arm-none-eabi-gcc` | `apx_firmware_<board>.elf`, `.bin`, `.hex` |

---

## 2. Host Build Prerequisites (Ubuntu) — one-time setup

Run the environment setup script once per machine:

```bash
tools/setup_ubuntu_env.sh                 # apt packages + Python + ARM toolchain + PATH
tools/setup_ubuntu_env.sh --with-gazebo   # also install Gazebo Harmonic
tools/setup_ubuntu_env.sh --no-apt        # without sudo: Python + ARM toolchain + PATH only
```

It installs the apt build dependencies (cmake, ninja, compilers, python3,
protobuf), the Python packages in `requirements.txt`, and the pinned Arm GNU
Toolchain into `platforms/stm32h7/toolchain/` (gitignored). It also adds a marked
block to `~/.bashrc` and `~/.profile` (and `~/.zshrc` if present) that puts the
toolchain on `PATH`. Re-running it is safe. Open a new terminal afterwards.

If an STM32H7 configure stops with *"ARM cross-compiler (arm-none-eabi-gcc) not
found"*, this script has not been run on the machine yet.

---

## 3. Building SITL (Simulation)

Configures debug binaries with GoogleTest unit tests and the Gazebo simulation plugin:

```bash
# Configure using the sitl preset
cmake --preset sitl

# Build all targets in parallel
cmake --build build/sitl -j$(nproc)

# Run the unified test suite
./build/sitl/tests/unit/run_unit_tests
```

---

## 4. Building Linux Release Firmware

Produces an optimized standalone flight controller binary for companion computers (e.g., Raspberry Pi 4/5, Nvidia Jetson):

```bash
cmake --preset linux-release
cmake --build build/linux-release -j$(nproc)
```

---

## 5. Cross-Compiling for STM32H7 Hardware

APX includes a bundled ARM Cortex-M7 toolchain located in `platforms/stm32h7/toolchain/15.2.rel1/bin/`.

```bash
# Configure for the default reference board (apx_h743_v1)
cmake --preset stm32h7

# Build the firmware image (.elf, .bin, .hex)
cmake --build build/stm32h7 -j$(nproc)
```

### 5.1 Reference Board (`apx_h743_v1`)
```bash
cmake --preset stm32h7
cmake --build build/stm32h7 -j$(nproc)
```

### 5.2 APX H743 Pro Flight Controller (`apx_h743_pro`)
```bash
cmake -B build/stm32h7 \
      -DCMAKE_TOOLCHAIN_FILE=platforms/stm32h7/toolchain-arm-none-eabi.cmake \
      -DAPX_BOARD=stm32h7 \
      -DAPX_BOARD_TARGET=apx_h743_pro \
      -DAPX_BUILD_SITL=OFF \
      -DAPX_BUILD_TESTS=OFF \
      -GNinja

ninja -C build/stm32h7
```

Output artifacts in `build/stm32h7/platforms/stm32h7/boards/apx_h743_pro/`:
- `apx_firmware_apx_h743_pro.bin`
- `apx_firmware_apx_h743_pro.hex`
- `apx_firmware_apx_h743_pro.elf`
- `apx_firmware_apx_h743_pro.map`

### 5.3 Generic Custom Board (`custom_h7_fc`)
```bash
cmake -B build/stm32h7 \
      -DCMAKE_TOOLCHAIN_FILE=platforms/stm32h7/toolchain-arm-none-eabi.cmake \
      -DAPX_BOARD=stm32h7 \
      -DAPX_BOARD_TARGET=custom_h7_fc \
      -DAPX_BUILD_SITL=OFF \
      -DAPX_BUILD_TESTS=OFF \
      -GNinja

ninja -C build/stm32h7
```

### 5.4 Flashing Firmware
- **Direct Flash via SWD / ST-Link (Address `0x08000000`):**
  ```bash
  STM32_Programmer_CLI -c port=SWD -w build/stm32h7/platforms/stm32h7/boards/apx_h743_pro/apx_firmware_apx_h743_pro.bin 0x08000000 -v -rst
  ```
- **Flashing via Bootloader (Address `0x08020000` / 128 KB Offset):**
  ```bash
  STM32_Programmer_CLI -c port=SWD -w build/stm32h7/platforms/stm32h7/boards/apx_h743_pro/apx_firmware_apx_h743_pro.bin 0x08020000 -v -rst
  ```

---

## 6. Generating New Hardware Boards

APX allows defining new STM32 flight controllers from a concise `hwdef.dat` file:

```bash
python3 tools/generate_board.py \
    --hwdef path/to/my_hwdef.dat \
    --output-dir platforms/stm32h7/boards/my_custom_board \
    --board-name my_custom_board
```

For complete syntax and pin mapping rules, refer to [Board Management Guide](board-management.md).

---

## 7. Generating Documentation Locally

To generate C++ API Markdown references, validate links, and produce the static documentation website:

```bash
./scripts/generate-docs.sh
```

To validate documentation integrity only (broken links, Mermaid syntax, compilation database):

```bash
python3 scripts/validate-docs.py
```

To live-preview the documentation with hot reload:

```bash
mkdocs serve
# Open http://127.0.0.1:8000 in your browser
```

---

## 8. Git Pre-Push Hook Integration

To automatically validate documentation, internal links, and the compilation database before pushing to Git, enable the bundled pre-push hook:

```bash
ln -sf ../../scripts/git-hooks/pre-push .git/hooks/pre-push
chmod +x .git/hooks/pre-push
```

Now every `git push` will execute `scripts/validate-docs.py` and abort the push if broken links or syntax issues are detected.

