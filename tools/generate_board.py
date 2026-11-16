#!/usr/bin/env python3
# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
"""APX Board Generator & Hardware Definition Parser.

Parses hardware definition files (e.g., hwdef.dat or APX board definition files)
and generates C++ constexpr platform maps (board_config.hpp), CMake target files,
linker scripts, and default parameter files for APX flight controller boards.

Usage:
    python3 tools/generate_board.py --hwdef <path_to_hwdef.dat> --output-dir <path_to_board_dir> [--board-name <name>]
"""

import argparse
import os
import re
import sys
from typing import Dict, List, Optional, Tuple, Any


class Pin:
    def __init__(self, port: str, pin: int, function: str = "", extra: str = ""):
        self.port = port.upper()
        self.pin = pin
        self.function = function
        self.extra = extra

    @property
    def apx_pin_call(self) -> str:
        return f"apx_pin(GpioPort::{self.port}, {self.pin})"

    @property
    def name(self) -> str:
        return f"P{self.port}{self.pin}"


class SpiDevice:
    def __init__(self, name: str, bus: int, cs_pin: Pin, mode: int = 3, speed_hz: int = 10000000):
        self.name = name.lower()
        self.bus = bus
        self.cs_pin = cs_pin
        self.mode = mode
        self.speed_hz = speed_hz


class I2cDevice:
    def __init__(self, name: str, bus: int, addr: int, speed_hz: int = 400000):
        self.name = name.lower()
        self.bus = bus
        self.addr = addr
        self.speed_hz = speed_hz


class UartPort:
    def __init__(self, id: int, tx_pin: Optional[Pin] = None, rx_pin: Optional[Pin] = None,
                 baud: int = 115200, role: str = ""):
        self.id = id
        self.tx_pin = tx_pin
        self.rx_pin = rx_pin
        self.baud = baud
        self.role = role.upper()  # GPS, TELEM, RC, ESC_TELEM, GENERAL


class MotorOutput:
    def __init__(self, index: int, timer: int, channel: int, pin: Pin, is_servo: bool = False):
        self.index = index
        self.timer = timer
        self.channel = channel
        self.pin = pin
        self.is_servo = is_servo


class Led:
    def __init__(self, index: int, pin: Pin, active_low: bool = True, role: str = "STATUS"):
        self.index = index
        self.pin = pin
        self.active_low = active_low
        self.role = role


class Relay:
    def __init__(self, index: int, pin: Pin):
        self.index = index
        self.pin = pin


class BoardDef:
    def __init__(self, name: str):
        self.name = name
        self.mcu = "STM32H743xx"
        self.flash_kb = 2048
        self.ram_kb = 1024
        self.oscillator_hz = 24000000
        self.board_id = 0
        self.usb_manufacturer = "APX"
        self.usb_product = name
        self.bootloader_offset_kb = 0
        
        # Peripherals
        self.pins: Dict[str, Pin] = {}
        self.spi_devices: List[SpiDevice] = []
        self.i2c_devices: List[I2cDevice] = []
        self.uarts: Dict[int, UartPort] = {}
        self.motors: List[MotorOutput] = []
        self.servos: List[MotorOutput] = []
        self.leds: List[Led] = []
        self.relays: List[Relay] = []
        
        # Sensor mappings
        self.imu_type = "ICM42688"
        self.mag_type = "IST8310"
        self.baro_type = "MS5611"
        self.batt_type = "INA226"
        self.batt_shunt_ohm = 0.0005
        self.batt_cells = 6
        
        # Features
        self.defines: Dict[str, str] = {}


def parse_pin_name(name: str) -> Optional[Tuple[str, int]]:
    """Matches PA5, PC13, etc."""
    m = re.match(r"^P([A-K])(\d{1,2})$", name.strip(), re.IGNORECASE)
    if m:
        port = m.group(1).upper()
        pin = int(m.group(2))
        if 0 <= pin <= 15:
            return port, pin
    return None


def parse_hwdef(file_path: str, board_name: str) -> BoardDef:
    """Parses a hwdef.dat or APX board definition file."""
    bdef = BoardDef(board_name)
    
    with open(file_path, "r", encoding="utf-8") as f:
        lines = f.readlines()

    timer_pins: Dict[str, Tuple[int, int]] = {}

    for line_num, line in enumerate(lines, start=1):
        # Strip comments
        line = line.strip()
        if not line or line.startswith("#") or line.startswith("//"):
            continue
        
        # Remove trailing comments
        if "#" in line:
            line = line.split("#", 1)[0].strip()
        if "//" in line:
            line = line.split("//", 1)[0].strip()

        tokens = line.split()
        if not tokens:
            continue

        cmd = tokens[0].upper()

        # MCU & Clock directives
        if cmd == "MCU" and len(tokens) >= 2:
            if len(tokens) >= 3 and "STM32" in tokens[2].upper():
                bdef.mcu = tokens[2]
            else:
                bdef.mcu = tokens[1]
        elif cmd in ("FLASH_SIZE", "FLASH_SIZE_KB") and len(tokens) >= 2:
            bdef.flash_kb = int(tokens[1].upper().replace("KB", "").replace("K", ""))
        elif cmd in ("RAM_SIZE", "RAM_SIZE_KB") and len(tokens) >= 2:
            bdef.ram_kb = int(tokens[1].upper().replace("KB", "").replace("K", ""))
        elif cmd in ("FLASH_BOOTLOADER_LOAD_KB", "FLASH_RESERVE_START_KB") and len(tokens) >= 2:
            if cmd == "FLASH_BOOTLOADER_LOAD_KB":
                bdef.bootloader_offset_kb = int(tokens[1].upper().replace("KB", "").replace("K", ""))
        elif cmd == "APJ_BOARD_ID" and len(tokens) >= 2:
            bdef.board_id = int(tokens[1])
        elif cmd == "USB_STRING_MANUFACTURER" and len(tokens) >= 2:
            bdef.usb_manufacturer = tokens[1].strip('"\'')
        elif cmd == "USB_STRING_PRODUCT" and len(tokens) >= 2:
            bdef.usb_product = " ".join(tokens[1:]).strip('"\'')
        elif cmd in ("OSCILLATOR_HZ", "HSE_VALUE") and len(tokens) >= 2:
            bdef.oscillator_hz = int(tokens[1].replace("*MHZ", "000000"))
        elif cmd == "SERIAL_ORDER" and len(tokens) >= 2:
            serial_order = tokens[1:]
            for idx, port_name in enumerate(serial_order):
                u_match = re.search(r"(?:USART|UART)(\d+)", port_name, re.IGNORECASE)
                if u_match:
                    u_id = int(u_match.group(1))
                    if u_id not in bdef.uarts:
                        bdef.uarts[u_id] = UartPort(u_id)
                    if idx == 1:
                        bdef.uarts[u_id].role = "TELEM"
                        bdef.uarts[u_id].baud = 57600
                    elif idx == 2:
                        bdef.uarts[u_id].role = "TELEM2"
                        bdef.uarts[u_id].baud = 57600
                    elif idx == 3:
                        bdef.uarts[u_id].role = "GPS"
                        bdef.uarts[u_id].baud = 115200

        # Pin mapping directive: PIN <pin_id> <function> [extra...]
        elif cmd == "PIN" and len(tokens) >= 3:
            pin_parsed = parse_pin_name(tokens[1])
            if pin_parsed:
                p_obj = Pin(pin_parsed[0], pin_parsed[1], tokens[2], " ".join(tokens[3:]))
                bdef.pins[p_obj.name] = p_obj
                
        # Direct pin line (standard hwdef format: e.g. "PA4 SPI1_NSS SPI1")
        elif parse_pin_name(tokens[0]):
            port, pin_idx = parse_pin_name(tokens[0])
            func = tokens[1] if len(tokens) >= 2 else ""
            extra = " ".join(tokens[2:]) if len(tokens) >= 3 else ""
            p_obj = Pin(port, pin_idx, func, extra)
            bdef.pins[p_obj.name] = p_obj

            # Timer PWM assignments: e.g. PE9 TIM1_CH1 TIM1 PWM(1)
            tim_match = re.search(r"TIM(\d{1,2})_CH(\d)", func)
            if tim_match:
                t_num = int(tim_match.group(1))
                ch_num = int(tim_match.group(2))
                timer_pins[p_obj.name] = (t_num, ch_num)
                pwm_match = re.search(r"PWM\((\d+)\)", extra)
                if pwm_match:
                    pwm_idx = int(pwm_match.group(1))
                    bdef.motors.append(MotorOutput(pwm_idx, t_num, ch_num, p_obj, is_servo=(pwm_idx > 8)))

            # UART direct pin lines (e.g. PD5 USART2_TX USART2, PD6 USART2_RX USART2)
            uart_tx_match = re.search(r"(?:USART|UART)(\d+)_TX", func, re.IGNORECASE)
            if uart_tx_match:
                u_id = int(uart_tx_match.group(1))
                if u_id not in bdef.uarts:
                    bdef.uarts[u_id] = UartPort(u_id)
                bdef.uarts[u_id].tx_pin = p_obj

            uart_rx_match = re.search(r"(?:USART|UART)(\d+)_RX", func, re.IGNORECASE)
            if uart_rx_match:
                u_id = int(uart_rx_match.group(1))
                if u_id not in bdef.uarts:
                    bdef.uarts[u_id] = UartPort(u_id)
                bdef.uarts[u_id].rx_pin = p_obj

            # Direct CS pins (e.g. PE4 42688_CS CS, PC2 42688_EXT_CS CS, PC14 BARO_CS CS, PD10 FRAM_CS CS)
            if func.upper().endswith("_CS") or extra.upper().startswith("CS") or " CS" in extra.upper():
                dev_clean = func.upper().replace("_CS", "").replace("_EXT", "").lower()
                full_dev_name = func.lower().replace("_cs", "")
                
                speed_hz = 10000000
                if "42688" in dev_clean or "40605" in dev_clean:
                    speed_hz = 10000000
                    bdef.imu_type = "ICM42688"
                elif "baro" in dev_clean or "5611" in dev_clean:
                    speed_hz = 10000000
                    bdef.baro_type = "MS5611"
                elif "fram" in dev_clean:
                    speed_hz = 20000000
                elif "sdcard" in dev_clean:
                    speed_hz = 20000000

                bdef.spi_devices.append(SpiDevice(full_dev_name, 1, p_obj, 3, speed_hz))

            # Direct LED lines (e.g. PB3 LED_ACTIVITY OUTPUT OPENDRAIN HIGH)
            if func.upper().startswith("LED_"):
                role = func.upper().replace("LED_", "")
                is_low = "OPENDRAIN" in extra.upper() or "HIGH" in extra.upper()
                led_idx = len(bdef.leds) + 1
                bdef.leds.append(Led(led_idx, p_obj, active_low=is_low, role=role))

        # SPI Device directive: SPIDEV <name> <bus> <cs_pin> [mode] [speed_hz]
        elif cmd == "SPIDEV" and len(tokens) >= 4:
            dev_name = tokens[1]
            bus_str = tokens[2].upper().replace("SPI", "")
            bus_id = int(bus_str) if bus_str.isdigit() else 1
            
            cs_str = tokens[3]
            if cs_str.upper() == "CS" and len(tokens) >= 5:
                cs_str = tokens[4]
            cs_parsed = parse_pin_name(cs_str)
            if cs_parsed:
                cs_pin = Pin(cs_parsed[0], cs_parsed[1], f"SPI{bus_id}_CS")
                speed = 10000000
                if "20*MHZ" in line or "20000000" in line:
                    speed = 20000000
                bdef.spi_devices.append(SpiDevice(dev_name, bus_id, cs_pin, 3, speed))
                if any(x in dev_name.lower() for x in ("icm", "bmi", "mpu", "iim", "imu")):
                    bdef.imu_type = dev_name.upper()

        # I2C Device directive: I2CDEV <name> <bus> <addr_hex>
        elif cmd == "I2CDEV" and len(tokens) >= 4:
            dev_name = tokens[1]
            bus_str = tokens[2].upper().replace("I2C", "")
            bus_id = int(bus_str) if bus_str.isdigit() else 1
            addr = int(tokens[3], 16) if tokens[3].startswith("0x") else int(tokens[3])
            bdef.i2c_devices.append(I2cDevice(dev_name, bus_id, addr))
            if "ist8310" in dev_name.lower() or "rm3100" in dev_name.lower() or "mag" in dev_name.lower():
                bdef.mag_type = dev_name.upper()
            elif "ina226" in dev_name.lower():
                bdef.batt_type = "INA226"

        # UART Port directive: UART <id> <tx_pin> <rx_pin> [baud] [role]
        elif cmd == "UART" and len(tokens) >= 4:
            uart_id = int(tokens[1].upper().replace("USART", "").replace("UART", ""))
            tx = parse_pin_name(tokens[2])
            rx = parse_pin_name(tokens[3])
            baud = int(tokens[4]) if len(tokens) >= 5 and tokens[4].isdigit() else 115200
            role = tokens[5].upper() if len(tokens) >= 6 else "GENERAL"
            tx_pin = Pin(tx[0], tx[1], f"UART{uart_id}_TX") if tx else None
            rx_pin = Pin(rx[0], rx[1], f"UART{uart_id}_RX") if rx else None
            bdef.uarts[uart_id] = UartPort(uart_id, tx_pin, rx_pin, baud, role)

        # Motor Output directive: MOTOR <index> <timer> <channel> <pin>
        elif cmd == "MOTOR" and len(tokens) >= 5:
            m_idx = int(tokens[1])
            t_num = int(tokens[2].upper().replace("TIM", ""))
            ch_num = int(tokens[3].upper().replace("CH", ""))
            pin_p = parse_pin_name(tokens[4])
            if pin_p:
                bdef.motors.append(MotorOutput(m_idx, t_num, ch_num, Pin(pin_p[0], pin_p[1])))

        # Servo Output directive: SERVO <index> <timer> <channel> <pin>
        elif cmd == "SERVO" and len(tokens) >= 5:
            s_idx = int(tokens[1])
            t_num = int(tokens[2].upper().replace("TIM", ""))
            ch_num = int(tokens[3].upper().replace("CH", ""))
            pin_p = parse_pin_name(tokens[4])
            if pin_p:
                bdef.servos.append(MotorOutput(s_idx, t_num, ch_num, Pin(pin_p[0], pin_p[1]), is_servo=True))

        # LED directive: LED <index> <pin> [ACTIVE_LOW/ACTIVE_HIGH]
        elif cmd == "LED" and len(tokens) >= 3:
            l_idx = int(tokens[1])
            pin_p = parse_pin_name(tokens[2])
            is_low = True
            if len(tokens) >= 4 and tokens[3].upper() == "ACTIVE_HIGH":
                is_low = False
            if pin_p:
                bdef.leds.append(Led(l_idx, Pin(pin_p[0], pin_p[1]), is_low))

        # Relay directive: RELAY <index> <pin>
        elif cmd == "RELAY" and len(tokens) >= 3:
            r_idx = int(tokens[1])
            pin_p = parse_pin_name(tokens[2])
            if pin_p:
                bdef.relays.append(Relay(r_idx, Pin(pin_p[0], pin_p[1])))

        # Generic DEFINES
        elif cmd in ("DEFINE", "DEFINES") and len(tokens) >= 2:
            key = tokens[1]
            val = tokens[2] if len(tokens) >= 3 else "1"
            bdef.defines[key] = val

    # Sort motors and servos by index
    bdef.motors.sort(key=lambda m: m.index)
    bdef.servos.sort(key=lambda s: s.index)

    # Defaults if missing:
    if not bdef.spi_devices:
        bdef.spi_devices.append(SpiDevice("icm42688", 1, Pin("A", 4, "SPI1_CS"), 3, 10000000))
    if not bdef.i2c_devices:
        bdef.i2c_devices.append(I2cDevice("ist8310", 1, 0x0E))
        bdef.i2c_devices.append(I2cDevice("ina226", 1, 0x40))
    if not bdef.uarts:
        bdef.uarts[1] = UartPort(1, Pin("A", 9), Pin("A", 10), 115200, "GPS")
        bdef.uarts[2] = UartPort(2, Pin("A", 2), Pin("A", 3), 57600, "TELEM")
        bdef.uarts[3] = UartPort(3, Pin("B", 10), Pin("B", 11), 100000, "RC")
    if not bdef.motors:
        bdef.motors.append(MotorOutput(1, 1, 1, Pin("E", 9)))
        bdef.motors.append(MotorOutput(2, 1, 2, Pin("E", 11)))
        bdef.motors.append(MotorOutput(3, 1, 3, Pin("E", 13)))
        bdef.motors.append(MotorOutput(4, 1, 4, Pin("E", 14)))
        bdef.motors.append(MotorOutput(5, 4, 1, Pin("D", 12)))
        bdef.motors.append(MotorOutput(6, 4, 2, Pin("D", 13)))
        bdef.motors.append(MotorOutput(7, 4, 3, Pin("D", 14)))
        bdef.motors.append(MotorOutput(8, 4, 4, Pin("D", 15)))
    if not bdef.servos:
        bdef.servos.append(MotorOutput(9, 2, 1, Pin("A", 15), is_servo=True))
        bdef.servos.append(MotorOutput(10, 2, 2, Pin("B", 3), is_servo=True))
        bdef.servos.append(MotorOutput(11, 2, 3, Pin("B", 10), is_servo=True))
        bdef.servos.append(MotorOutput(12, 2, 4, Pin("B", 11), is_servo=True))
    if not bdef.leds:
        bdef.leds.append(Led(1, Pin("B", 3), True, "ACTIVITY"))

    return bdef


def generate_board_config_hpp(bdef: BoardDef) -> str:
    """Generates the C++ constexpr board_config.hpp content."""
    clean_name = re.sub(r"[^a-zA-Z0-9_]", "_", bdef.name.lower())
    
    imu_dev = next((d for d in bdef.spi_devices if any(x in d.name for x in ("icm", "bmi", "mpu", "imu"))), bdef.spi_devices[0])
    mag_dev = next((d for d in bdef.i2c_devices if any(x in d.name for x in ("ist", "rm", "mag"))), bdef.i2c_devices[0])
    batt_dev = next((d for d in bdef.i2c_devices if "ina" in d.name or "batt" in d.name), None)

    gps_uart = next((u for u in bdef.uarts.values() if u.role == "GPS" or u.id == 1), list(bdef.uarts.values())[0])
    telem_uart = next((u for u in bdef.uarts.values() if u.role == "TELEM" or u.id == 2), list(bdef.uarts.values())[min(1, len(bdef.uarts)-1)])
    rc_uart = next((u for u in bdef.uarts.values() if u.role == "RC" or u.id == 3), list(bdef.uarts.values())[min(2, len(bdef.uarts)-1)])

    lines = []
    lines.append("// SPDX-License-Identifier: CC-BY-NC-4.0")
    lines.append(f"// APX Board Configuration for \"{bdef.name}\"")
    lines.append("// Auto-generated by tools/generate_board.py. Do not edit directly.")
    lines.append("#pragma once")
    lines.append("#include <cstdint>")
    lines.append("")
    lines.append(f"namespace apx::platform::stm32h7::{clean_name} {{")
    lines.append("")
    lines.append("// --- MCU Specifications -------------------------------------------------")
    lines.append(f"constexpr const char* BOARD_NAME     = \"{bdef.name}\";")
    lines.append(f"constexpr const char* MCU_NAME       = \"{bdef.mcu}\";")
    lines.append(f"constexpr uint32_t FLASH_SIZE_KB     = {bdef.flash_kb};")
    lines.append(f"constexpr uint32_t RAM_SIZE_KB       = {bdef.ram_kb};")
    lines.append(f"constexpr uint32_t OSCILLATOR_HZ     = {bdef.oscillator_hz};")
    lines.append("")
    lines.append("// --- GPIO Port Definition -----------------------------------------------")
    lines.append("enum class GpioPort : uint32_t { A = 0, B = 1, C = 2, D = 3, E = 4, F = 5, G = 6, H = 7, I = 8, J = 9, K = 10 };")
    lines.append("")
    lines.append("constexpr uint32_t apx_pin(GpioPort port, uint8_t pin_number) {")
    lines.append("    return static_cast<uint32_t>(port) * 16u + pin_number;")
    lines.append("}")
    lines.append("")
    lines.append("// --- IMU Sensor ---------------------------------------------------------")
    lines.append(f"constexpr uint32_t IMU_SPI_BUS       = {imu_dev.bus};")
    lines.append(f"constexpr uint32_t IMU_CS_PIN        = {imu_dev.cs_pin.apx_pin_call};")
    lines.append(f"constexpr uint32_t IMU_SPI_HZ        = {imu_dev.speed_hz};")
    lines.append("")
    lines.append("// --- Magnetometer -------------------------------------------------------")
    lines.append(f"constexpr uint32_t MAG_I2C_BUS       = {mag_dev.bus};")
    lines.append(f"constexpr uint32_t MAG_I2C_ADDR      = 0x{mag_dev.addr:02X};")
    lines.append(f"constexpr uint32_t MAG_I2C_HZ        = {mag_dev.speed_hz};")
    lines.append("")
    lines.append("// --- Battery Monitor ----------------------------------------------------")
    if batt_dev:
        lines.append(f"constexpr uint32_t BATT_I2C_BUS      = {batt_dev.bus};")
        lines.append(f"constexpr uint32_t BATT_I2C_ADDR     = 0x{batt_dev.addr:02X};")
    else:
        lines.append(f"constexpr uint32_t BATT_I2C_BUS      = 1;")
        lines.append(f"constexpr uint32_t BATT_I2C_ADDR     = 0x40;")
    lines.append(f"constexpr float    BATT_SHUNT_OHM    = {bdef.batt_shunt_ohm}f;")
    lines.append(f"constexpr uint8_t  BATT_CELL_COUNT_DEFAULT = {bdef.batt_cells};")
    lines.append("")
    lines.append("// --- GPS UART -----------------------------------------------------------")
    lines.append(f"constexpr uint32_t GPS_UART_ID       = {gps_uart.id};")
    if gps_uart.tx_pin:
        lines.append(f"constexpr uint32_t GPS_TX_PIN        = {gps_uart.tx_pin.apx_pin_call};")
    if gps_uart.rx_pin:
        lines.append(f"constexpr uint32_t GPS_RX_PIN        = {gps_uart.rx_pin.apx_pin_call};")
    lines.append(f"constexpr uint32_t GPS_BAUD          = {gps_uart.baud};")
    lines.append("")
    lines.append("// --- Telemetry UART -----------------------------------------------------")
    lines.append(f"constexpr uint32_t TELEM_UART_ID     = {telem_uart.id};")
    if telem_uart.tx_pin:
        lines.append(f"constexpr uint32_t TELEM_TX_PIN      = {telem_uart.tx_pin.apx_pin_call};")
    if telem_uart.rx_pin:
        lines.append(f"constexpr uint32_t TELEM_RX_PIN      = {telem_uart.rx_pin.apx_pin_call};")
    lines.append(f"constexpr uint32_t TELEM_BAUD        = {telem_uart.baud};")
    lines.append("")
    lines.append("// --- RC Input UART ------------------------------------------------------")
    lines.append(f"constexpr uint32_t RC_UART_ID        = {rc_uart.id};")
    if rc_uart.tx_pin:
        lines.append(f"constexpr uint32_t RC_TX_PIN         = {rc_uart.tx_pin.apx_pin_call};")
    if rc_uart.rx_pin:
        lines.append(f"constexpr uint32_t RC_RX_PIN         = {rc_uart.rx_pin.apx_pin_call};")
    lines.append(f"constexpr uint32_t RC_SBUS_BAUD      = {rc_uart.baud};")
    lines.append("")
    lines.append("// --- Motor Outputs ------------------------------------------------------")
    lines.append("struct MotorPwmChannel {")
    lines.append("    uint32_t timer_id;")
    lines.append("    uint32_t channel;")
    lines.append("    uint32_t gpio_pin;")
    lines.append("};")
    lines.append("")
    for m in bdef.motors:
        lines.append(f"constexpr MotorPwmChannel MOTOR{m.index}{{{m.timer}, {m.channel}, {m.pin.apx_pin_call}}};")
    for s in bdef.servos:
        lines.append(f"constexpr MotorPwmChannel SERVO{s.index}{{{s.timer}, {s.channel}, {s.pin.apx_pin_call}}};")
    lines.append("constexpr uint32_t MOTOR_PWM_FREQ_HZ = 400;")
    lines.append("")
    lines.append("// --- Status LEDs --------------------------------------------------------")
    for led in bdef.leds:
        lines.append(f"constexpr uint32_t LED_{led.role}_PIN = {led.pin.apx_pin_call};")
        lines.append(f"constexpr bool     LED_{led.role}_ACTIVE_LOW = {str(led.active_low).lower()};")
    lines.append("")
    lines.append("// --- Relays & Actuators -------------------------------------------------")
    for r in bdef.relays:
        lines.append(f"constexpr uint32_t RELAY{r.index}_PIN = {r.pin.apx_pin_call};")
    lines.append("")
    lines.append(f"}} // namespace apx::platform::stm32h7::{clean_name}")
    return "\n".join(lines) + "\n"


def generate_cmakelists_txt(bdef: BoardDef) -> str:
    """Generates the board's CMakeLists.txt target."""
    clean_name = re.sub(r"[^a-zA-Z0-9_]", "_", bdef.name.lower())
    target_lib = f"apx_board_{clean_name}"
    target_exe = f"apx_firmware_{clean_name}"

    return f"""# APX -- board target for {bdef.name} ({bdef.mcu})
# Auto-generated by tools/generate_board.py. Do not edit directly.

set(APX_BOARD_DIR ${{CMAKE_CURRENT_SOURCE_DIR}})

add_library({target_lib} INTERFACE)

target_include_directories({target_lib} INTERFACE
    ${{APX_BOARD_DIR}}
)

target_sources({target_lib} INTERFACE
    ${{APX_BOARD_DIR}}/startup_stm32h743.s
    ${{APX_BOARD_DIR}}/system_init.c
)

set(APX_BOARD_LINKER_SCRIPT ${{APX_BOARD_DIR}}/linker.ld CACHE INTERNAL
    "Linker script for {bdef.name}")

message(STATUS "APX board: {bdef.name} ({bdef.mcu}, flash: {bdef.flash_kb}KB, RAM: {bdef.ram_kb}KB)")

add_executable({target_exe}
    ${{APX_BOARD_DIR}}/apx_main_stm32h7.cpp
)
target_link_libraries({target_exe} PRIVATE
    {target_lib}
    apx_kernel
    apx_params
    apx_flight_control
)
target_link_options({target_exe} PRIVATE
    -T${{APX_BOARD_LINKER_SCRIPT}}
    -Wl,-Map=${{CMAKE_CURRENT_BINARY_DIR}}/{target_exe}.map
)
set_target_properties({target_exe} PROPERTIES
    SUFFIX ".elf"
)

add_custom_command(TARGET {target_exe} POST_BUILD
    COMMAND ${{CMAKE_OBJCOPY}} -O binary
            $<TARGET_FILE:{target_exe}>
            ${{CMAKE_CURRENT_BINARY_DIR}}/{target_exe}.bin
    COMMAND ${{CMAKE_OBJCOPY}} -O ihex
            $<TARGET_FILE:{target_exe}>
            ${{CMAKE_CURRENT_BINARY_DIR}}/{target_exe}.hex
    COMMAND ${{CMAKE_SIZE}} $<TARGET_FILE:{target_exe}>
    COMMENT "{bdef.name}: generating .bin / .hex firmware images"
)
"""


def generate_linker_ld(bdef: BoardDef) -> str:
    """Generates a GNU ld script tailored to the MCU flash and RAM sizes."""
    return f"""/* SPDX-License-Identifier: CC-BY-NC-4.0 */
/* Linker script for {bdef.name} ({bdef.mcu}) */
/* Auto-generated by tools/generate_board.py */

ENTRY(Reset_Handler)

MEMORY
{{
  FLASH (rx)      : ORIGIN = 0x08000000, LENGTH = {bdef.flash_kb}K
  DTCMRAM (xrw)   : ORIGIN = 0x20000000, LENGTH = 128K
  AXISRAM (xrw)   : ORIGIN = 0x24000000, LENGTH = 512K
  SRAM1 (xrw)     : ORIGIN = 0x30000000, LENGTH = 128K
  SRAM2 (xrw)     : ORIGIN = 0x30020000, LENGTH = 128K
  SRAM3 (xrw)     : ORIGIN = 0x30040000, LENGTH = 32K
  SRAM4 (xrw)     : ORIGIN = 0x38000000, LENGTH = 64K
  ITCMRAM (xrw)   : ORIGIN = 0x00000000, LENGTH = 64K
}}

_estack = ORIGIN(DTCMRAM) + LENGTH(DTCMRAM);
_min_stack_size = 0x2000; /* 8 KB */
_min_heap_size  = 0x4000; /* 16 KB */

SECTIONS
{{
  .isr_vector :
  {{
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  }} >FLASH

  .text :
  {{
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.rodata)
    *(.rodata*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;
  }} >FLASH

  _sidata = LOADADDR(.data);

  .data :
  {{
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  }} >AXISRAM AT> FLASH

  .bss :
  {{
    . = ALIGN(4);
    _sbss = .;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
  }} >AXISRAM

  ._user_heap_stack :
  {{
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _min_heap_size;
    . = . + _min_stack_size;
    . = ALIGN(8);
  }} >DTCMRAM
}}
"""


def generate_default_parm(bdef: BoardDef) -> str:
    """Generates default parameter overrides for the board."""
    lines = [
        f"# APX default parameter file for {bdef.name}",
        f"# Auto-generated by tools/generate_board.py",
        "",
        "# --- Frame configuration ---",
        "FRAME_CLASS=1      # 1 = Quad",
        "FRAME_TYPE=1       # 1 = X",
        "",
        "# --- System IDs ---",
        "SYSID_THISMAV=1",
        "SYSID_MYGCS=255",
        "",
        "# --- Failsafes ---",
        "ARMING_CHECK=1",
        "FS_THR_ENABLE=1",
        "FS_GCS_ENABLE=1",
        "",
        "# --- Battery settings ---",
        "BATT_MONITOR=4",
        "BATT_CAPACITY=5000",
        f"BATT_LOW_VOLT={3.5 * bdef.batt_cells:.1f}",
        f"BATT_CRT_VOLT={3.3 * bdef.batt_cells:.1f}",
        "",
        "# --- Speed & Navigation ---",
        "WPNAV_SPEED=250.0",
        "WPNAV_RADIUS=200.0",
        "RTL_ALT=1500.0",
        "",
        "# --- Rate PID Controller Defaults ---",
        "ATC_RAT_RLL_P=0.135",
        "ATC_RAT_RLL_I=0.135",
        "ATC_RAT_RLL_D=0.0036",
        "ATC_RAT_PIT_P=0.135",
        "ATC_RAT_PIT_I=0.135",
        "ATC_RAT_PIT_D=0.0036",
        "ATC_RAT_YAW_P=0.180",
        "ATC_RAT_YAW_I=0.018",
        "ATC_RAT_YAW_D=0.0000",
        "",
        "# --- RC Channel Mapping ---",
        "RCMAP_ROLL=1",
        "RCMAP_PITCH=2",
        "RCMAP_THROTTLE=3",
        "RCMAP_YAW=4",
        "",
    ]
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description="APX Board Generator")
    parser.add_argument("--hwdef", required=True, help="Path to hwdef.dat or board.def file")
    parser.add_argument("--output-dir", required=True, help="Directory to output generated board files")
    parser.add_argument("--board-name", default=None, help="Optional board name override")
    args = parser.parse_args()

    if not os.path.exists(args.hwdef):
        print(f"Error: Hardware definition file not found: {args.hwdef}", file=sys.stderr)
        sys.exit(1)

    board_name = args.board_name or os.path.basename(os.path.dirname(os.path.abspath(args.hwdef)))
    if not board_name or board_name in (".", "/"):
        board_name = os.path.splitext(os.path.basename(args.hwdef))[0]

    print(f"[APX BoardGen] Parsing definition: {args.hwdef} for board '{board_name}'...")
    bdef = parse_hwdef(args.hwdef, board_name)

    os.makedirs(args.output_dir, exist_ok=True)

    # 1. board_config.hpp
    cfg_path = os.path.join(args.output_dir, "board_config.hpp")
    with open(cfg_path, "w", encoding="utf-8") as f:
        f.write(generate_board_config_hpp(bdef))
    print(f"[APX BoardGen] Generated: {cfg_path}")

    # 2. CMakeLists.txt
    cmake_path = os.path.join(args.output_dir, "CMakeLists.txt")
    with open(cmake_path, "w", encoding="utf-8") as f:
        f.write(generate_cmakelists_txt(bdef))
    print(f"[APX BoardGen] Generated: {cmake_path}")

    # 3. linker.ld
    ld_path = os.path.join(args.output_dir, "linker.ld")
    if not os.path.exists(ld_path):
        with open(ld_path, "w", encoding="utf-8") as f:
            f.write(generate_linker_ld(bdef))
        print(f"[APX BoardGen] Generated: {ld_path}")

    # 4. Copy template sources if missing (startup and system_init, main)
    ref_board_dir = os.path.join(os.path.dirname(__file__), "..", "platforms", "stm32h7", "boards", "apx_h743_v1")
    for tmpl_file in ("startup_stm32h743.s", "system_init.c", "apx_main_stm32h7.cpp"):
        dest_file = os.path.join(args.output_dir, tmpl_file)
        if not os.path.exists(dest_file) and os.path.exists(os.path.join(ref_board_dir, tmpl_file)):
            with open(os.path.join(ref_board_dir, tmpl_file), "r", encoding="utf-8") as src, \
                 open(dest_file, "w", encoding="utf-8") as dst:
                dst.write(src.read())
            print(f"[APX BoardGen] Staged: {dest_file}")

    # 5. Default parameters file
    parm_dir = os.path.join(os.path.dirname(args.output_dir), "..", "default_params")
    if os.path.exists(parm_dir):
        parm_path = os.path.join(parm_dir, f"{board_name}.parm")
        with open(parm_path, "w", encoding="utf-8") as f:
            f.write(generate_default_parm(bdef))
        print(f"[APX BoardGen] Generated parameters: {parm_path}")

    print(f"[APX BoardGen] Board '{board_name}' successfully generated in {args.output_dir}!")


if __name__ == "__main__":
    main()
