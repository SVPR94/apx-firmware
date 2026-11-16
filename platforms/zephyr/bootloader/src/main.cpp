// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
// APX bootloader (Zephyr port) - boot policy.
//
//   power-up, valid app       listen CONFIG_APX_BL_BOOT_DELAY_MS for a ground
//                             station, then start the application
//   app asked for bootloader  (MAVLink reboot-to-bootloader) listen up to
//                             CONFIG_APX_BL_STAY_TIMEOUT_S
//   no valid app              stay in the bootloader until one is uploaded
//   upload finished (REBOOT)  start the new application
//
// The link is the board's chosen apx,bootloader-uart, else its apx,gcs-uart
// (USB CDC or a UART). The status LED blinks fast while in the bootloader.

#include <cstring>

#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>

#include "apx/bootloader/protocol.hpp"
#include "boot_jump.hpp"
#include "port_zephyr.hpp"
#include "uart_zephyr.hpp"

#if DT_HAS_CHOSEN(apx_bootloader_uart)
#define APX_BL_LINK DT_CHOSEN(apx_bootloader_uart)
#elif DT_HAS_CHOSEN(apx_gcs_uart)
#define APX_BL_LINK DT_CHOSEN(apx_gcs_uart)
#else
#error "APX bootloader: choose apx,bootloader-uart or apx,gcs-uart in the board devicetree"
#endif
#define APX_BL_LINK_IS_UART (!DT_NODE_HAS_COMPAT(APX_BL_LINK, zephyr_cdc_acm_uart))

#define APX_USER_NODE DT_PATH(zephyr_user)
#if !DT_NODE_HAS_PROP(APX_USER_NODE, apj_board_id)
#error "APX bootloader: set zephyr,user { apj-board-id = <N>; } in the board devicetree"
#endif

using namespace apx::bootloader;
using namespace apx::bootloader::zephyr;

namespace {

uint32_t chip_id() {
#if defined(CONFIG_SOC_SERIES_STM32H7X)
    return *reinterpret_cast<const volatile uint32_t*>(0x5C001000u); // DBGMCU_IDCODE
#else
    return 0;
#endif
}

const char* chip_description() {
    static char text[48];
    const char* rev = "?";
#if defined(CONFIG_SOC_SERIES_STM32H7X)
    switch (chip_id() >> 16) {
    case 0x1001: rev = "Z"; break;
    case 0x1003: rev = "Y"; break;
    case 0x2001: rev = "X"; break;
    case 0x2003: rev = "V"; break;
    default: break;
    }
#endif
    // "<soc>,<revision>" without pulling in printf
    std::strncpy(text, CONFIG_SOC, sizeof(text) - 3);
    text[sizeof(text) - 3] = '\0';
    std::strcat(text, ",");
    std::strcat(text, rev);
    return text;
}

} // namespace

int main() {
    static apx::hal::zephyr::ZephyrUart link(DEVICE_DT_GET(APX_BL_LINK));
    link.init(APX_BL_LINK_IS_UART ? CONFIG_APX_BL_UART_BAUD : 0);
    static ZephyrPort port(link, APX_BL_LINK_IS_UART);

    const Identity identity{
        DT_PROP(APX_USER_NODE, apj_board_id),
        DT_PROP_OR(APX_USER_NODE, board_revision, 0),
        app_size(),
        chip_id(),
        chip_description(),
        "APX bootloader " APX_BL_VERSION " (" CONFIG_BOARD_TARGET ")",
    };
    static Protocol protocol(port, identity);

    const bool stay = early_request() == BootRequest::StayInBootloader;
    uint32_t first_contact_ms = stay ? CONFIG_APX_BL_STAY_TIMEOUT_S * 1000U : CONFIG_APX_BL_BOOT_DELAY_MS;
    while (true) {
        const bool have_app = app_valid();
        protocol.run(have_app ? first_contact_ms : 0, have_app ? CONFIG_APX_BL_IDLE_TIMEOUT_S * 1000U : 0);
        if (app_valid()) {
            link.flush();
            start_application(); // does not return
        }
        first_contact_ms = 0; // no valid application: keep serving
    }
    return 0;
}
