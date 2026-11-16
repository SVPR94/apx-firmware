// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
// connect. There are no sensor drivers and no motor outputs yet, and the
// GCS is told so: no sensors present, no attitude or position, battery
// unknown, and arming is refused.

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>

#include "apx/kernel/timer.hpp"
#include "mavlink_handler.hpp"
#include "mode_manager.hpp"
#include "param_manager.hpp"

namespace {

constexpr uint32_t kLoopHz = 400;
constexpr uint32_t kLoopPeriodUs = 1000000 / kLoopHz;
constexpr uint32_t kNoticePeriodS = 60;

const gpio_dt_spec kLed = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
const gpio_dt_spec kSensorsPower = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), sensors_power_gpios);
const gpio_dt_spec kPeriph5vPower = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), periph_5v_power_gpios);
const device* const kUsb = DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart0));

RING_BUF_DECLARE(g_tx_ring, 8192);
RING_BUF_DECLARE(g_rx_ring, 2048);

void usb_isr(const device* dev, void*) {
    while (uart_irq_update(dev) > 0 && uart_irq_is_pending(dev) > 0) {
        if (uart_irq_rx_ready(dev) > 0) {
            uint8_t buf[64];
            const int n = uart_fifo_read(dev, buf, sizeof(buf));
            if (n > 0) {
                ring_buf_put(&g_rx_ring, buf, static_cast<uint32_t>(n));
            }
        }
        if (uart_irq_tx_ready(dev) > 0) {
            uint8_t* data = nullptr;
            const uint32_t n = ring_buf_get_claim(&g_tx_ring, &data, 64);
            if (n == 0) {
                uart_irq_tx_disable(dev);
            } else {
                const int sent = uart_fifo_fill(dev, data, static_cast<int>(n));
                ring_buf_get_finish(&g_tx_ring, sent > 0 ? static_cast<uint32_t>(sent) : 0U);
            }
        }
    }
}

// MavlinkHandler byte writer. A frame that does not fit (host not reading)
// is dropped whole rather than truncated.
void usb_write(const uint8_t* data, size_t len, void*) {
    const unsigned int key = irq_lock();
    if (ring_buf_space_get(&g_tx_ring) >= len) {
        ring_buf_put(&g_tx_ring, data, static_cast<uint32_t>(len));
    }
    irq_unlock(key);
    uart_irq_tx_enable(kUsb);
}

void power_rails_on() {
    // Sensors start unpowered; switch the rail on after the 5 V
    // peripheral rail, as a clean power-up for the IMUs and baros.
    gpio_pin_configure_dt(&kSensorsPower, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&kPeriph5vPower, GPIO_OUTPUT_ACTIVE);
    k_msleep(20);
    gpio_pin_set_dt(&kSensorsPower, 1);
}

} // namespace

int main() {
    power_rails_on();
    gpio_pin_configure_dt(&kLed, GPIO_OUTPUT_INACTIVE);

    static apx::gcs::MavlinkHandler mav(1, 1);
    mav.init(); // registers the parameter defaults
    static apx::modes::ModeManager modes;
    mav.set_mode_manager(&modes);
    mav.set_vehicle_type(MAV_TYPE_HEXAROTOR);

    // Report only what works on this build.
    mav.set_sensor_status(0, 0, 0);
    mav.set_estimates_valid(false, false);
    mav.set_arming_blocked("Arming refused: APX bring-up build has no sensors or motor outputs yet");
    mav.set_vehicle_state(0.0f, 0.0f, 0.0f, 0.0, 0.0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                          -1.0f, -1.0f, -1, false, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0);
    mav.set_raw_sensor_telemetry(apx::gcs::RawSensorTelemetry{}); // GPS_RAW_INT: no GPS

    mav.set_byte_writer(usb_write, nullptr);
    if (device_is_ready(kUsb)) {
        uart_irq_callback_user_data_set(kUsb, usb_isr, nullptr);
        uart_irq_rx_enable(kUsb);
    }

    static struct k_timer tick;
    k_timer_init(&tick, nullptr, nullptr);
    k_timer_start(&tick, K_USEC(kLoopPeriodUs), K_USEC(kLoopPeriodUs));

    uint32_t loops = 0;
    uint64_t next_notice_ms = 3000;
    while (true) {
        k_timer_status_sync(&tick);

        uint8_t buf[128];
        uint32_t got;
        while ((got = ring_buf_get(&g_rx_ring, buf, sizeof(buf))) > 0) {
            mav.receive_bytes(buf, got);
        }

        mav.set_custom_mode(apx::modes::ModeManager::to_custom_mode_number(modes.current_id()));
        mav.update();

        if (apx::kernel::now_ms() >= next_notice_ms) {
            mav.send_statustext(MAV_SEVERITY_WARNING,
                                "APX bring-up build: no sensors or motor outputs yet, arming disabled");
            next_notice_ms += kNoticePeriodS * 1000ULL;
        }

        if (++loops % (kLoopHz / 2) == 0) {
            gpio_pin_toggle_dt(&kLed); // 1 Hz blink: firmware running
        }
    }
    return 0;
}
