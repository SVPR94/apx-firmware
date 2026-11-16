// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
// other project's board-definition syntax. Logical peripheral roles
// (IMU_SPI, GPS_UART, MOTOR1, ...) are mapped to STM32H7 pin/peripheral
// identifiers here; a future stm32h7 HAL backend (hal/stm32h7/, currently
// empty -- see platforms/stm32h7/README.md) reads these constants to
// initialize its IHalSpi/IHalUart/IHalI2c/IHalPwm/IHalGpio implementations,
// the same abstract interfaces the SITL/Linux HAL backends already
// implement (see hal/include/apx/hal/hal_*.hpp).
//
// Physical layout (matches what drivers/ already supports for a
// quad-X multirotor):
//   - 1x SPI IMU            (ICM-42688, drivers/imu/icm42688)
//   - 1x I2C magnetometer   (IST8310,   drivers/mag/ist8310)
//   - 1x I2C battery monitor(INA226,    drivers/battery/ina226)
//   - 1x UART GPS           (u-blox,    drivers/gps/ublox)
//   - 1x UART telemetry     (MAVLink,   drivers/telemetry/uart_mavlink)
//   - 1x UART/SBUS RC input (drivers/rc_input/sbus)
//   - 4x PWM motor outputs  (drivers/servo, one TIM1 channel per motor)
#pragma once
#include <cstdint>

namespace apx::platform::stm32h7::apx_h743_v1 {

// --- GPIO pin identifiers ------------------------------------------------
// STM32H7 pins are named <port letter><pin number> (PA5, PC13, ...) on the
// silicon; APX packs that into a single uint32_t as (port_index * 16 +
// pin_number) so it can be passed directly as the `pin` argument of
// apx::hal::IHalGpio (hal/include/apx/hal/hal_gpio.hpp), which takes one
// flat id rather than a port+pin pair. This packing is APX's own
// convention, chosen to match the existing IHalGpio signature -- it is
// not tied to any register layout.
enum class GpioPort : uint32_t { A = 0, B = 1, C = 2, D = 3, E = 4, F = 5, G = 6, H = 7 };

constexpr uint32_t apx_pin(GpioPort port, uint8_t pin_number) {
    return static_cast<uint32_t>(port) * 16u + pin_number;
}

// --- IMU: ICM-42688-P on SPI1 --------------------------------------------
constexpr uint32_t IMU_SPI_BUS   = 1;   // STM32 SPI1 peripheral instance
constexpr uint32_t IMU_CS_PIN    = apx_pin(GpioPort::A, 4);   // PA4, active low
constexpr uint32_t IMU_SCK_PIN   = apx_pin(GpioPort::A, 5);   // PA5
constexpr uint32_t IMU_MISO_PIN  = apx_pin(GpioPort::A, 6);   // PA6
constexpr uint32_t IMU_MOSI_PIN  = apx_pin(GpioPort::A, 7);   // PA7
constexpr uint32_t IMU_DRDY_PIN  = apx_pin(GpioPort::C, 4);   // PC4, INT1, optional EXTI
constexpr uint32_t IMU_SPI_HZ    = 10'000'000;                // matches icm42688 typical max

// --- Magnetometer: IST8310 on I2C1 ---------------------------------------
constexpr uint32_t MAG_I2C_BUS   = 1;   // STM32 I2C1 peripheral instance
constexpr uint32_t MAG_I2C_ADDR  = 0x0E; // matches Ist8310Reg::DEFAULT_ADDR
constexpr uint32_t MAG_SCL_PIN   = apx_pin(GpioPort::B, 6);  // PB6
constexpr uint32_t MAG_SDA_PIN   = apx_pin(GpioPort::B, 7);  // PB7
constexpr uint32_t MAG_I2C_HZ    = 400'000;                  // fast mode

// --- Battery monitor: INA226 on I2C1 (shares MAG_I2C_BUS) ----------------
constexpr uint32_t BATT_I2C_BUS   = MAG_I2C_BUS;
constexpr uint32_t BATT_I2C_ADDR  = 0x40;      // INA226 default ADDR strap
constexpr float    BATT_SHUNT_OHM = 0.0005f;   // 0.5 mOhm shunt
constexpr uint8_t  BATT_CELL_COUNT_DEFAULT = 6;

// --- GPS: u-blox on USART1 ------------------------------------------------
constexpr uint32_t GPS_UART_ID   = 1;  // STM32 USART1 peripheral instance
constexpr uint32_t GPS_TX_PIN    = apx_pin(GpioPort::A, 9);   // PA9
constexpr uint32_t GPS_RX_PIN    = apx_pin(GpioPort::A, 10);  // PA10
constexpr uint32_t GPS_BAUD      = 115200;

// --- Telemetry: MAVLink over USART2 ---------------------------------------
constexpr uint32_t TELEM_UART_ID = 2;  // STM32 USART2 peripheral instance
constexpr uint32_t TELEM_TX_PIN  = apx_pin(GpioPort::A, 2);   // PA2
constexpr uint32_t TELEM_RX_PIN  = apx_pin(GpioPort::A, 3);   // PA3
constexpr uint32_t TELEM_BAUD    = 57600;

// --- RC input: SBUS over USART3 -------------------------------------------
// SBUS is inverted, single-wire-on-RX in practice; on real hardware this
// needs either a hardware inverter on the RX line or the H7 USART's
// built-in RX invert bit -- see the README's "what's still needed" list.
constexpr uint32_t RC_UART_ID    = 3;  // STM32 USART3 peripheral instance
constexpr uint32_t RC_RX_PIN     = apx_pin(GpioPort::B, 11);  // PB11
constexpr uint32_t RC_TX_PIN     = apx_pin(GpioPort::B, 10);  // PB10 (unused by SBUS, wired for symmetry)
constexpr uint32_t RC_SBUS_BAUD  = 100000; // 8E2, matches SbusDriver framing

// --- Motor outputs: 4x PWM on TIM1 ----------------------------------------
struct MotorPwmChannel {
    uint32_t timer_id;   // STM32 timer peripheral instance, e.g. 1 == TIM1
    uint32_t channel;    // timer output-compare channel, 1-4
    uint32_t gpio_pin;   // packed apx_pin() for the timer's AF pin
};

constexpr MotorPwmChannel MOTOR1{1, 1, apx_pin(GpioPort::E, 9)};   // TIM1_CH1, PE9
constexpr MotorPwmChannel MOTOR2{1, 2, apx_pin(GpioPort::E, 11)};  // TIM1_CH2, PE11
constexpr MotorPwmChannel MOTOR3{1, 3, apx_pin(GpioPort::E, 13)};  // TIM1_CH3, PE13
constexpr MotorPwmChannel MOTOR4{1, 4, apx_pin(GpioPort::E, 14)};  // TIM1_CH4, PE14
constexpr uint32_t MOTOR_PWM_FREQ_HZ = 400; // standard analog PWM ESC rate; DShot boards would override this

// --- Status LED ------------------------------------------------------------
constexpr uint32_t LED_STATUS_PIN = apx_pin(GpioPort::C, 13); // PC13, active low

// --- Clock configuration (informational; consumed by future SystemInit) --
// 8 MHz HSE crystal -> 400 MHz SYSCLK via the main PLL, AHB/2 = 200 MHz,
// APB1/APB2/APB3/APB4 /2 = 100 MHz each. This matches the PLL divider
// choice used elsewhere for this same silicon in this repo (see
// boards/apx_h7_v1/apx_h7_v1.dts, the earlier Zephyr-based exploration
// discussed in README.md) so both paths agree on the same run clock if
// they are ever reconciled.
constexpr uint32_t HSE_HZ    = 8'000'000;
constexpr uint32_t SYSCLK_HZ = 400'000'000;
constexpr uint32_t AHB_HZ    = 200'000'000;
constexpr uint32_t APB_HZ    = 100'000'000;

} // namespace apx::platform::stm32h7::apx_h743_v1
