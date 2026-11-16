// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
 * so enabling the FPU coprocessor before *any* floating-point instruction
 * runs is not optional - without it the first float/double operation
 * anywhere in APX's flight-control math would UsageFault immediately.
 * That part is essential and safe to do blind.
 *
 * The PLL/clock-tree bring-up (HSI -> 400MHz over the PLL, with matching
 * flash wait-states and voltage-scaling register writes) is deliberately
 * NOT attempted here: getting that register sequence wrong on real H743
 * silicon (wrong wait-state count for the target voltage/frequency, wrong
 * PLL divider/multiplier combination) is a well-known way to hang or
 * brick the boot path, and there is no hardware in this environment to
 * verify it against. The core runs on its ~64MHz HSI reset clock until a
 * real clock-tree init is written and verified on actual apx_h743_v1
 * hardware - see README.md's "No SystemInit()/clock-tree init" item and
 * kernel/src/timer.cpp's kAssumedCoreClockHz, which assumes this same
 * default and must be updated together with a real clock-tree init.
 */

#include <stdint.h>

#define SCB_CPACR (*(volatile uint32_t*)0xE000ED88u)
#define SCB_VTOR  (*(volatile uint32_t*)0xE000ED08u)

/* Vector table base - must match linker.ld's .isr_vector placement
 * (FLASH origin, see linker.ld). Not strictly required at reset (the
 * hardware already loads the vector table from this address on power-up
 * per the BOOT pin configuration), but setting VTOR explicitly is cheap,
 * standard practice, and required if this image is ever entered from a
 * bootloader rather than directly from reset. */
#define APX_VECTOR_TABLE_BASE 0x08000000u

void SystemInit(void) {
    /* Enable full access (privileged + user) to CP10/CP11 (the FPU
     * coprocessor slots) - required before the hard-float ABI's
     * VFP/NEON instructions can execute without faulting. */
    SCB_CPACR |= (0xFu << 20);
    __asm volatile("dsb");
    __asm volatile("isb");

    SCB_VTOR = APX_VECTOR_TABLE_BASE;
}
