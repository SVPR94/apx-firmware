// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
// APX bootloader (Zephyr port): early boot hook and jump to the application.

#include "boot_jump.hpp"

#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/util.h>

#include <cmsis_core.h>

#if defined(CONFIG_SOC_FAMILY_STM32)
#include <stm32_ll_rcc.h>
#endif

#define APX_FLASH_NODE DT_CHOSEN(zephyr_flash)
#define APX_APP_PART DT_NODELABEL(app_partition)
#define APX_BOOT_CTL DT_NODELABEL(apx_boot_control)

#if !DT_NODE_EXISTS(APX_APP_PART)
#error "APX bootloader: the board devicetree needs an app_partition flash partition"
#endif
#if !DT_NODE_EXISTS(APX_BOOT_CTL)
#error "APX bootloader: the board devicetree needs an apx_boot_control RAM node"
#endif

namespace apx::bootloader::zephyr {
namespace {

BootRequest g_early_request = BootRequest::None;

bool in_range(uint32_t v, uint32_t base, uint32_t size) { return v >= base && v <= base + size; }

// True after a power-on or brown-out reset: RAM (and its ECC) is not
// initialised, so the boot control block must not be read. Also saves and
// clears the reset flags so the next reset is classified correctly.
bool cold_reset_and_clear_flags(uint32_t& flags) {
#if defined(CONFIG_SOC_FAMILY_STM32) && defined(RCC_RSR_PORRSTF)   /* STM32H7 */
    flags = RCC->RSR;
    const bool cold = (flags & (RCC_RSR_PORRSTF | RCC_RSR_BORRSTF)) != 0;
    LL_RCC_ClearResetFlags();
    return cold;
#elif defined(CONFIG_SOC_FAMILY_STM32) && defined(RCC_CSR_PORRSTF)  /* STM32F4 */
    flags = RCC->CSR;
    const bool cold = (flags & (RCC_CSR_PORRSTF | RCC_CSR_BORRSTF)) != 0;
    LL_RCC_ClearResetFlags();
    return cold;
#else
    flags = 0;
    return true; // unknown reset cause: never trust RAM contents
#endif
}

[[noreturn]] void jump(uint32_t base) {
    const auto* vectors = reinterpret_cast<const volatile uint32_t*>(base);
    const uint32_t sp = vectors[0];
    const uint32_t entry = vectors[1];

    __disable_irq();
    SysTick->CTRL = 0;
    SysTick->VAL = 0;
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;
    for (size_t i = 0; i < ARRAY_SIZE(NVIC->ICER); ++i) {
        NVIC->ICER[i] = 0xFFFFFFFFu;
        NVIC->ICPR[i] = 0xFFFFFFFFu;
    }
#if defined(__MPU_PRESENT) && __MPU_PRESENT
    MPU->CTRL = 0;
#endif
#if defined(__DCACHE_PRESENT) && __DCACHE_PRESENT
    if (SCB->CCR & SCB_CCR_DC_Msk) {
        SCB_DisableDCache();
    }
#endif
#if defined(__ICACHE_PRESENT) && __ICACHE_PRESENT
    if (SCB->CCR & SCB_CCR_IC_Msk) {
        SCB_DisableICache();
    }
#endif
    SCB->VTOR = base;
    __DSB();
    __ISB();
    // Reset-like core state: MSP from the image, privileged thread mode on
    // MSP, no FPU context, interrupts unmasked (every source is disabled).
    __set_MSP(sp);
    __set_CONTROL(0);
    __set_BASEPRI(0);
    __ISB();
    __enable_irq();
    reinterpret_cast<void (*)()>(entry)();
    while (true) {
    }
}

// Runs before any driver: either jump straight to the application (clean
// peripheral state) or record the request for main().
int early_boot_hook() {
    volatile BootControl* bc = boot_control();
    uint32_t reset_flags = 0;
    if (cold_reset_and_clear_flags(reset_flags)) {
        // Initialise the whole block with word writes (sets RAM ECC).
        auto* words = reinterpret_cast<volatile uint32_t*>(bc);
        for (size_t i = 0; i < DT_REG_SIZE(APX_BOOT_CTL) / 4; ++i) {
            words[i] = 0;
        }
        g_early_request = BootRequest::None;
    } else {
        g_early_request = boot_control_take(bc);
    }
    bc->reserved = reset_flags; // reset cause, for the application
    if (g_early_request == BootRequest::StartApplication && app_valid()) {
        jump(app_base_address());
    }
    return 0;
}

} // namespace

uint32_t app_offset() { return DT_REG_ADDR(APX_APP_PART); }
uint32_t app_size() { return DT_REG_SIZE(APX_APP_PART); }
uint32_t app_base_address() { return DT_REG_ADDR(APX_FLASH_NODE) + app_offset(); }

bool app_valid() {
    const auto* vectors = reinterpret_cast<const volatile uint32_t*>(app_base_address());
    const uint32_t sp = vectors[0];
    const uint32_t entry = vectors[1];
    bool sp_ok = in_range(sp, DT_REG_ADDR(DT_CHOSEN(zephyr_sram)), DT_REG_SIZE(DT_CHOSEN(zephyr_sram)));
#if DT_NODE_EXISTS(DT_CHOSEN(zephyr_dtcm))
    sp_ok = sp_ok || in_range(sp, DT_REG_ADDR(DT_CHOSEN(zephyr_dtcm)), DT_REG_SIZE(DT_CHOSEN(zephyr_dtcm)));
#endif
    const bool entry_ok = (entry & 1u) != 0 && entry > app_base_address() &&
                          entry < app_base_address() + app_size();
    return sp_ok && (sp & 3u) == 0 && entry_ok;
}

volatile BootControl* boot_control() {
    return reinterpret_cast<volatile BootControl*>(DT_REG_ADDR(APX_BOOT_CTL));
}

BootRequest early_request() { return g_early_request; }

void start_application() {
    boot_control_set(boot_control(), BootRequest::StartApplication);
    sys_reboot(SYS_REBOOT_WARM);
    while (true) {
    }
}

} // namespace apx::bootloader::zephyr

namespace {
int apx_early_boot() { return apx::bootloader::zephyr::early_boot_hook(); }
} // namespace

SYS_INIT(apx_early_boot, PRE_KERNEL_1, 0);
