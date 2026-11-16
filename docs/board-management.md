# APX Hardware Board Management & Definition Guide

This guide describes how to define, generate, configure, and build firmware for custom STM32-based flight controller boards in APX.

---

## 1. Overview

APX provides an automated board generation framework. Instead of writing verbose boilerplate C++, linker scripts, and startup code by hand, you can define your board layout in a declarative `hwdef.dat` file. The APX generator tool (`tools/generate_board.py`) parses this definition and produces:

- **`board_config.hpp`:** Type-safe, compile-time verified C++17 `constexpr` pin/bus/timer map.
- **`CMakeLists.txt`:** Board library target and firmware executable target.
- **`linker.ld`:** MCU memory map tailored to Flash and RAM sizes.
- **`startup_stm32h743.s`:** Vector table and `Reset_Handler`.
- **`system_init.c`:** Clock tree and memory cache setup.
- **`default_params/<board>.parm`:** Default parameter overrides matching the board's sensor and actuator configuration.

---

## 2. Hardware Definition File Syntax (`hwdef.dat`)

The syntax is compatible with standard flight controller pinout definitions (such as those found in `libraries/AP_HAL_ChibiOS/hwdef`):

```text
# MCU and Clock Specifications
MCU STM32H743xx
FLASH_SIZE 2048
RAM_SIZE 1024
OSCILLATOR_HZ 25000000

# SPI Devices (IMU, Baro, Flash)
PA4 SPI1_NSS SPI1
PA5 SPI1_SCK SPI1
PA6 SPI1_MISO SPI1
PA7 SPI1_MOSI SPI1
SPIDEV icm42688 SPI1 PA4 MODE3 10*MHZ

# I2C Devices (Magnetometer, Power Monitor)
PB6 I2C1_SCL I2C1
PB7 I2C1_SDA I2C1
I2CDEV ist8310 I2C1 0x0E
I2CDEV ina226 I2C1 0x40

# UART Serial Ports (GPS, Telemetry, RC)
PA9 USART1_TX USART1
PA10 USART1_RX USART1
UART 1 PA9 PA10 115200 GPS

PA2 USART2_TX USART2
PA3 USART2_RX USART2
UART 2 PA2 PA3 57600 TELEM

PD8 USART3_TX USART3
PD9 USART3_RX USART3
UART 3 PD8 PD9 100000 RC

# Motors (PWM / DShot outputs 1..4 on TIM1)
PE9 TIM1_CH1 TIM1 PWM(1)
PE11 TIM1_CH2 TIM1 PWM(2)
PE13 TIM1_CH3 TIM1 PWM(3)
PE14 TIM1_CH4 TIM1 PWM(4)
MOTOR 1 1 1 PE9
MOTOR 2 1 2 PE11
MOTOR 3 1 3 PE13
MOTOR 4 1 4 PE14

# Auxiliary Servos (Channels 9..10 on TIM4)
PD12 TIM4_CH1 TIM4 PWM(9)
PD13 TIM4_CH2 TIM4 PWM(10)
SERVO 9 4 1 PD12
SERVO 10 4 2 PD13

# Status LEDs
PC13 LED1 ACTIVE_LOW
LED 1 PC13 ACTIVE_LOW

# Relays / Sprayer Triggers
PE3 RELAY1
RELAY 1 PE3
```

---

## 3. Supported Hardware List

| Category | Supported Hardware & Drivers |
|---|---|
| **Microcontrollers** | STM32H743, STM32H753, STM32H750, STM32F745, STM32F405 |
| **IMUs (SPI / I2C)** | ICM-42688-P, ICM-42605, ICM-20689, ICM-20948, BMI088, BMI270, BMI323 |
| **Barometers** | MS5611, BMP388 |
| **Magnetometers** | IST8310, RM3100, LIS3MDL |
| **GNSS & RTK** | u-blox M9, u-blox M10, u-blox ZED-F9P (RTCM injection) |
| **Power Monitors** | INA226 (I2C), Analog ADC shunt/voltage dividers |
| **RC Inputs** | SBUS (inverted UART), CRSF / ExpressLRS (420 kBaud) |
| **Motor Protocols** | DShot (150/300/600), Analog PWM (50-400 Hz) |
| **Actuators** | PWM Servos, Spray Pumps, Atomizing Spinners, GPIO Relays |

---

## 4. Step-by-Step: Adding a New Board

### Step 1: Create Board Directory & `hwdef.dat`
Create a folder under `platforms/stm32h7/boards/<your_board_name>/` and place your `hwdef.dat` there:

```bash
mkdir -p platforms/stm32h7/boards/my_custom_fc
# Edit platforms/stm32h7/boards/my_custom_fc/hwdef.dat with your pin assignments
```

### Step 2: Run the Board Generator
```bash
python3 tools/generate_board.py \
    --hwdef platforms/stm32h7/boards/my_custom_fc/hwdef.dat \
    --output-dir platforms/stm32h7/boards/my_custom_fc \
    --board-name my_custom_fc
```

### Step 3: Build Firmware Image
```bash
cmake -B build/stm32h7 -DAPX_BOARD_TARGET=my_custom_fc
cmake --build build/stm32h7 -j$(nproc)
```

The output firmware images will be generated in:
- `build/stm32h7/platforms/stm32h7/boards/my_custom_fc/apx_firmware_my_custom_fc.elf`
- `build/stm32h7/platforms/stm32h7/boards/my_custom_fc/apx_firmware_my_custom_fc.bin`
- `build/stm32h7/platforms/stm32h7/boards/my_custom_fc/apx_firmware_my_custom_fc.hex`

---

## 5. Built-in Boards Reference & Compilation Commands

### 5.1 APX H743 Pro (`apx_h743_pro`)
- **Target Name:** `apx_h743_pro`
- **MCU:** STM32H743xx (2048 KB Flash, 1024 KB RAM)
- **Clock:** 24 MHz Crystal (`OSCILLATOR_HZ 24000000`)
- **Board ID:** `22336` (USB: "APX" - "APX H743 Pro")
- **Bootloader Load Offset:** 128 KB (`0x08020000`)
- **Sensors & Peripherals:**
  - Onboard IMU: ICM-42688 (CS: `PE4`)
  - External IMU 1: ICM-42688 (CS: `PC2`)
  - External IMU 2: ICM-40605 (CS: `PC13`)
  - Onboard Barometer: MS5611 (CS: `PC14`)
  - External Barometer: Baro Ext (CS: `PD7`)
  - Storage: FRAM (CS: `PD10`), SD Card (CS: `PD2`)
  - Telem 1: USART2 (TX: `PD5`, RX: `PD6`)
  - Telem 2: USART3 (TX: `PD8`, RX: `PD9`)
  - Serial 5 / Payload: UART7 (TX: `PE8`, RX: `PE7`)
  - LEDs: Activity Red (`PB3`), Bootloader Blue (`PE12`)

**Compilation Command:**
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

### 5.2 Reference Board (`apx_h743_v1`)
- **Target Name:** `apx_h743_v1`
- **MCU:** STM32H743xx (25 MHz Crystal)

**Compilation Command:**
```bash
cmake --preset stm32h7
cmake --build build/stm32h7 -j$(nproc)
```

### 5.3 Generic Custom Board (`custom_h7_fc`)
- **Target Name:** `custom_h7_fc`

**Compilation Command:**
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

