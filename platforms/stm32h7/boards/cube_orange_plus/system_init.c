// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include <stdint.h>

#define SCB_CPACR (*(volatile uint32_t*)0xE000ED88u)
#define SCB_VTOR  (*(volatile uint32_t*)0xE000ED08u)

/* Vector table base - matches linker.ld's FLASH origin */
#define APX_VECTOR_TABLE_BASE 0x08000000u

void SystemInit(void) {
    /* Enable full access to CP10/CP11 (FPU coprocessor slots) */
    SCB_CPACR |= (0xFu << 20);
    __asm volatile("dsb");
    __asm volatile("isb");

    SCB_VTOR = APX_VECTOR_TABLE_BASE;
}
