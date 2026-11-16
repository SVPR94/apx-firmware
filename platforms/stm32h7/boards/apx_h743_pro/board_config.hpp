// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once
#include <cstdint>

namespace apx::platform::stm32h7::apx_h743_pro {

// --- MCU Specifications -------------------------------------------------
constexpr const char* BOARD_NAME     = "apx_h743_pro";
constexpr const char* MCU_NAME       = "STM32H743xx";
constexpr uint32_t FLASH_SIZE_KB     = 2048;
constexpr uint32_t RAM_SIZE_KB       = 1024;
constexpr uint32_t OSCILLATOR_HZ     = 24000000;

// --- GPIO Port Definition -----------------------------------------------
enum class GpioPort : uint32_t { A = 0, B = 1, C = 2, D = 3, E = 4, F = 5, G = 6, H = 7, I = 8, J = 9, K = 10 };

constexpr uint32_t apx_pin(GpioPort port, uint8_t pin_number) {
    return static_cast<uint32_t>(port) * 16u + pin_number;
}

// --- IMU Sensor ---------------------------------------------------------
constexpr uint32_t IMU_SPI_BUS       = 1;
constexpr uint32_t IMU_CS_PIN        = apx_pin(GpioPort::C, 2);
constexpr uint32_t IMU_SPI_HZ        = 10000000;

// --- Magnetometer -------------------------------------------------------
constexpr uint32_t MAG_I2C_BUS       = 1;
constexpr uint32_t MAG_I2C_ADDR      = 0x0E;
constexpr uint32_t MAG_I2C_HZ        = 400000;

// --- Battery Monitor ----------------------------------------------------
constexpr uint32_t BATT_I2C_BUS      = 1;
constexpr uint32_t BATT_I2C_ADDR     = 0x40;
constexpr float    BATT_SHUNT_OHM    = 0.0005f;
constexpr uint8_t  BATT_CELL_COUNT_DEFAULT = 6;

// --- GPS UART -----------------------------------------------------------
constexpr uint32_t GPS_UART_ID       = 7;
constexpr uint32_t GPS_TX_PIN        = apx_pin(GpioPort::E, 8);
constexpr uint32_t GPS_RX_PIN        = apx_pin(GpioPort::E, 7);
constexpr uint32_t GPS_BAUD          = 115200;

// --- Telemetry UART -----------------------------------------------------
constexpr uint32_t TELEM_UART_ID     = 2;
constexpr uint32_t TELEM_TX_PIN      = apx_pin(GpioPort::D, 5);
constexpr uint32_t TELEM_RX_PIN      = apx_pin(GpioPort::D, 6);
constexpr uint32_t TELEM_BAUD        = 57600;

// --- RC Input UART ------------------------------------------------------
constexpr uint32_t RC_UART_ID        = 3;
constexpr uint32_t RC_TX_PIN         = apx_pin(GpioPort::D, 8);
constexpr uint32_t RC_RX_PIN         = apx_pin(GpioPort::D, 9);
constexpr uint32_t RC_SBUS_BAUD      = 57600;

// --- Motor Outputs ------------------------------------------------------
struct MotorPwmChannel {
    uint32_t timer_id;
    uint32_t channel;
    uint32_t gpio_pin;
};

constexpr MotorPwmChannel MOTOR1{1, 1, apx_pin(GpioPort::E, 9)};
constexpr MotorPwmChannel MOTOR2{1, 2, apx_pin(GpioPort::E, 11)};
constexpr MotorPwmChannel MOTOR3{1, 3, apx_pin(GpioPort::E, 13)};
constexpr MotorPwmChannel MOTOR4{1, 4, apx_pin(GpioPort::E, 14)};
constexpr MotorPwmChannel MOTOR5{4, 1, apx_pin(GpioPort::D, 12)};
constexpr MotorPwmChannel MOTOR6{4, 2, apx_pin(GpioPort::D, 13)};
constexpr MotorPwmChannel MOTOR7{4, 3, apx_pin(GpioPort::D, 14)};
constexpr MotorPwmChannel MOTOR8{4, 4, apx_pin(GpioPort::D, 15)};
constexpr MotorPwmChannel SERVO9{2, 1, apx_pin(GpioPort::A, 15)};
constexpr MotorPwmChannel SERVO10{2, 2, apx_pin(GpioPort::B, 3)};
constexpr MotorPwmChannel SERVO11{2, 3, apx_pin(GpioPort::B, 10)};
constexpr MotorPwmChannel SERVO12{2, 4, apx_pin(GpioPort::B, 11)};
constexpr uint32_t MOTOR_PWM_FREQ_HZ = 400;

// --- Status LEDs --------------------------------------------------------
constexpr uint32_t LED_ACTIVITY_PIN = apx_pin(GpioPort::B, 3);
constexpr bool     LED_ACTIVITY_ACTIVE_LOW = true;
constexpr uint32_t LED_BOOTLOADER_PIN = apx_pin(GpioPort::E, 12);
constexpr bool     LED_BOOTLOADER_ACTIVE_LOW = true;

// --- Relays & Actuators -------------------------------------------------

} // namespace apx::platform::stm32h7::apx_h743_pro
