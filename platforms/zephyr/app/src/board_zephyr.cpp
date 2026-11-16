// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
// APX board interface (apx/firmware/board.hpp) for every Zephyr board.
//
// Nothing here is board-specific: each board describes its hardware in its
// devicetree, and this file reads it.
//
//   aliases  { led0 = &status_led; }             status LED (optional)
//   chosen   { apx,gcs-uart = &cdc_acm_uart0; }  GCS/MAVLink link (USB CDC or UART)
//   chosen   { apx,iomcu-uart = &usart6; }       IO coprocessor link (optional, 1.5 Mbaud)
//   zephyr,user {
//       power-enable-gpios = <...>, <...>;        rails switched on in this order (optional)
//       power-enable-delay-ms = <20>;             settle time after each rail
//       apj-board-id = <1063>;                    .apj bootloader board id (optional, CMake)
//   };
//   apx_boot_control: boot-control@...           RAM shared with the bootloader (optional)
//
// So adding a board from another manufacturer needs board files only
// (platforms/zephyr/boards/<vendor>/<board>/), no code.

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>

#include "apx/bootloader/boot_control.hpp"
#include "apx/firmware/board.hpp"
#include "uart_zephyr.hpp"

#define APX_USER_NODE DT_PATH(zephyr_user)

namespace apx::board {
namespace {

#if DT_NODE_EXISTS(DT_ALIAS(led0))
#define APX_HAS_STATUS_LED 1
const gpio_dt_spec kStatusLed = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
bool g_led_on = false;
#endif

#if DT_NODE_HAS_PROP(APX_USER_NODE, power_enable_gpios)
#define APX_HAS_POWER_RAILS 1
const gpio_dt_spec kPowerRails[] = {
    DT_FOREACH_PROP_ELEM_SEP(APX_USER_NODE, power_enable_gpios, GPIO_DT_SPEC_GET_BY_IDX, (, ))};
constexpr int32_t kPowerDelayMs = DT_PROP_OR(APX_USER_NODE, power_enable_delay_ms, 10);
#endif

const BoardInfo kInfo{CONFIG_BOARD, CONFIG_BOARD_TARGET, "zephyr"};

#if DT_HAS_CHOSEN(apx_gcs_uart)
apx::hal::zephyr::ZephyrUart& gcs_uart() {
    static apx::hal::zephyr::ZephyrUart uart(DEVICE_DT_GET(DT_CHOSEN(apx_gcs_uart)));
    return uart;
}
bool g_gcs_ok = false;
#endif

#if DT_HAS_CHOSEN(apx_iomcu_uart)
apx::hal::zephyr::ZephyrUart& iomcu_uart() {
    static apx::hal::zephyr::ZephyrUart uart(DEVICE_DT_GET(DT_CHOSEN(apx_iomcu_uart)));
    return uart;
}
bool g_iomcu_ok = false;
#endif

} // namespace

bool init() {
    bool ok = true;
#if defined(APX_HAS_POWER_RAILS)
    // All rails off first, then on in devicetree order with settle time,
    // so sensors see a clean power-up.
    for (const auto& rail : kPowerRails) {
        ok &= gpio_pin_configure_dt(&rail, GPIO_OUTPUT_INACTIVE) == 0;
    }
    for (const auto& rail : kPowerRails) {
        gpio_pin_set_dt(&rail, 1);
        k_msleep(kPowerDelayMs);
    }
#endif
#if defined(APX_HAS_STATUS_LED)
    ok &= gpio_pin_configure_dt(&kStatusLed, GPIO_OUTPUT_INACTIVE) == 0;
#endif
#if DT_HAS_CHOSEN(apx_gcs_uart)
    g_gcs_ok = gcs_uart().init(0); // 0: keep the devicetree baud rate
    ok &= g_gcs_ok;
#else
    ok = false; // no GCS link chosen in the devicetree
#endif
#if DT_HAS_CHOSEN(apx_iomcu_uart)
    g_iomcu_ok = iomcu_uart().init(0); // devicetree speed (1.5 Mbaud)
    ok &= g_iomcu_ok;
#endif
    return ok;
}

const BoardInfo& info() { return kInfo; }

void set_status_led(bool on) {
#if defined(APX_HAS_STATUS_LED)
    g_led_on = on;
    gpio_pin_set_dt(&kStatusLed, on ? 1 : 0);
#else
    (void)on;
#endif
}

void toggle_status_led() {
#if defined(APX_HAS_STATUS_LED)
    set_status_led(!g_led_on);
#endif
}

void reboot(bool to_bootloader) {
#if DT_NODE_EXISTS(DT_NODELABEL(apx_boot_control))
    auto* bc = reinterpret_cast<volatile apx::bootloader::BootControl*>(
        DT_REG_ADDR(DT_NODELABEL(apx_boot_control)));
    apx::bootloader::boot_control_set(bc, to_bootloader ? apx::bootloader::BootRequest::StayInBootloader
                                                        : apx::bootloader::BootRequest::None);
#else
    (void)to_bootloader; // no bootloader handshake on this board
#endif
    sys_reboot(SYS_REBOOT_WARM);
    while (true) {
    }
}

apx::hal::IHalUart* iomcu_link() {
#if DT_HAS_CHOSEN(apx_iomcu_uart)
    return g_iomcu_ok ? &iomcu_uart() : nullptr;
#else
    return nullptr;
#endif
}

apx::hal::IHalUart* gcs_link() {
#if DT_HAS_CHOSEN(apx_gcs_uart)
    return g_gcs_ok ? &gcs_uart() : nullptr;
#else
    return nullptr;
#endif
}

} // namespace apx::board
