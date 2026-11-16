// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once
#include <cstdint>

namespace apx::platform::stm32h7::cube_orange_plus {

// --- MCU Specifications -------------------------------------------------
constexpr const char* BOARD_NAME     = "cube_orange_plus";
constexpr const char* MCU_NAME       = "STM32H757xx";
constexpr uint32_t FLASH_SIZE_KB     = 2048;
constexpr uint32_t RAM_SIZE_KB       = 1024;
constexpr uint32_t OSCILLATOR_HZ     = 24000000;
constexpr uint32_t APJ_BOARD_ID      = 1063;

// --- GPIO Port Definition -----------------------------------------------
enum class GpioPort : uint32_t { A = 0, B = 1, C = 2, D = 3, E = 4, F = 5, G = 6, H = 7, I = 8, J = 9, K = 10 };

constexpr uint32_t apx_pin(GpioPort port, uint8_t pin_number) {
    return static_cast<uint32_t>(port) * 16u + pin_number;
}

// --- Power Rails --------------------------------------------------------
constexpr uint32_t SENSORS_PWR_PIN       = apx_pin(GpioPort::E, 3); // 3.3V sensor rail (active high)
constexpr bool     SENSORS_PWR_ACTIVE_HIGH = true;
constexpr uint32_t PERIPH_5V_PWR_PIN     = apx_pin(GpioPort::A, 8); // 5V peripheral rail (active low)
constexpr bool     PERIPH_5V_ACTIVE_LOW  = true;

// --- Sensor Lane Architecture (Exactly 3 Sensor Lanes Used) --------------
constexpr uint32_t NUM_SENSOR_LANES      = 3;

// Sensor Lane 1: Primary isolated IMU (ICM-42688-P) on SPI1
constexpr uint32_t IMU_LANE1_SPI_BUS     = 1;
constexpr uint32_t IMU_LANE1_CS_PIN      = apx_pin(GpioPort::E, 4);
constexpr uint32_t IMU_LANE1_EXT_CS_PIN  = apx_pin(GpioPort::C, 2);
constexpr uint32_t IMU_LANE1_SPI_HZ      = 10000000;

// Sensor Lane 2: Secondary isolated IMU (ICM-40605/42670) + Baro 1 on SPI4
constexpr uint32_t IMU_LANE2_SPI_BUS     = 4;
constexpr uint32_t IMU_LANE2_CS_PIN      = apx_pin(GpioPort::C, 13);
constexpr uint32_t BARO_LANE2_CS_PIN     = apx_pin(GpioPort::D, 7);
constexpr uint32_t IMU_LANE2_SPI_HZ      = 10000000;

// Sensor Lane 3: Backup isolated IMU (ICM-20649) + Baro 2 on SPI2
constexpr uint32_t IMU_LANE3_SPI_BUS     = 2;
constexpr uint32_t IMU_LANE3_CS_PIN      = apx_pin(GpioPort::C, 15);
constexpr uint32_t BARO_LANE3_CS_PIN     = apx_pin(GpioPort::C, 14);
constexpr uint32_t IMU_LANE3_SPI_HZ      = 10000000;

// Default alias for single-IMU legacy drivers
constexpr uint32_t IMU_SPI_BUS           = IMU_LANE1_SPI_BUS;
constexpr uint32_t IMU_CS_PIN            = IMU_LANE1_CS_PIN;
constexpr uint32_t IMU_SPI_HZ            = IMU_LANE1_SPI_HZ;

// --- Magnetometer (External I2C / DroneCAN) -----------------------------
constexpr uint32_t MAG_I2C_BUS           = 1;
constexpr uint32_t MAG_I2C_ADDR          = 0x0E;
constexpr uint32_t MAG_I2C_HZ            = 400000;

// --- Battery Monitor ----------------------------------------------------
constexpr uint32_t BATT_I2C_BUS          = 1;
constexpr uint32_t BATT_I2C_ADDR         = 0x40;
constexpr float    BATT_SHUNT_OHM        = 0.0005f;
constexpr uint8_t  BATT_CELL_COUNT_DEFAULT = 6;

// --- GPS UART (GPS 1 on UART7) -------------------------------------------
constexpr uint32_t GPS_UART_ID           = 7;
constexpr uint32_t GPS_TX_PIN            = apx_pin(GpioPort::E, 8);
constexpr uint32_t GPS_RX_PIN            = apx_pin(GpioPort::E, 7);
constexpr uint32_t GPS_BAUD              = 115200;

// --- Telemetry UARTs -----------------------------------------------------
constexpr uint32_t TELEM_UART_ID         = 2; // TELEM1 (USART2)
constexpr uint32_t TELEM_TX_PIN          = apx_pin(GpioPort::D, 5);
constexpr uint32_t TELEM_RX_PIN          = apx_pin(GpioPort::D, 6);
constexpr uint32_t TELEM_BAUD            = 57600;

constexpr uint32_t TELEM2_UART_ID        = 3; // TELEM2 (USART3)
constexpr uint32_t TELEM2_TX_PIN         = apx_pin(GpioPort::D, 8);
constexpr uint32_t TELEM2_RX_PIN         = apx_pin(GpioPort::D, 9);
constexpr uint32_t TELEM2_BAUD           = 57600;

// --- IOMCU Co-processor (USART6) -----------------------------------------
constexpr uint32_t IOMCU_UART_ID         = 6;
constexpr uint32_t IOMCU_TX_PIN          = apx_pin(GpioPort::G, 14);
constexpr uint32_t IOMCU_RX_PIN          = apx_pin(GpioPort::G, 9);
constexpr uint32_t IOMCU_BAUD            = 115200;

// --- Motor Outputs (Direct AUX Outputs 1-6 on TIM1 & TIM4) --------------
struct MotorPwmChannel {
    uint32_t timer_id;
    uint32_t channel;
    uint32_t gpio_pin;
};

constexpr MotorPwmChannel MOTOR1{1, 1, apx_pin(GpioPort::E, 9)};  // TIM1_CH1 (AUX1)
constexpr MotorPwmChannel MOTOR2{1, 2, apx_pin(GpioPort::E, 11)}; // TIM1_CH2 (AUX2)
constexpr MotorPwmChannel MOTOR3{1, 3, apx_pin(GpioPort::E, 13)}; // TIM1_CH3 (AUX3)
constexpr MotorPwmChannel MOTOR4{1, 4, apx_pin(GpioPort::E, 14)}; // TIM1_CH4 (AUX4)
constexpr MotorPwmChannel MOTOR5{4, 2, apx_pin(GpioPort::D, 13)}; // TIM4_CH2 (AUX5)
constexpr MotorPwmChannel MOTOR6{4, 3, apx_pin(GpioPort::D, 14)}; // TIM4_CH3 (AUX6)
constexpr uint32_t MOTOR_PWM_FREQ_HZ     = 400;

// --- Status LEDs --------------------------------------------------------
constexpr uint32_t LED_ACTIVITY_PIN      = apx_pin(GpioPort::B, 3);  // Red activity
constexpr bool     LED_ACTIVITY_ACTIVE_LOW = true;
constexpr uint32_t LED_BOOTLOADER_PIN    = apx_pin(GpioPort::E, 12); // Amber FMU LED
constexpr bool     LED_BOOTLOADER_ACTIVE_LOW = true;
constexpr uint32_t LED_FMU_AMBER_PIN     = apx_pin(GpioPort::E, 12);

} // namespace apx::platform::stm32h7::cube_orange_plus
