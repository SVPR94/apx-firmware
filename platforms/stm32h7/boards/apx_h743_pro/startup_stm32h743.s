/* SPDX-License-Identifier: CC-BY-NC-4.0
 * Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
 * Author: Savita Pawar (SVPR94)
 * Created: 2026
 * APX -- minimal Cortex-M7 reset/startup code for apx_h743_v1
 * (STM32H743). This follows the conventional Cortex-M startup pattern
 * used across the embedded-GCC ecosystem (vector table -> Reset_Handler
 * copies .data/zeroes .bss -> runs C++ static constructors -> calls
 * main()) -- generic boilerplate, not specific to any one project.
 *
 * The peripheral-interrupt vector table below covers the core ARMv7-M
 * system exceptions exactly (architecturally fixed) plus the STM32H743
 * IRQ numbers this board's drivers actually need (USART1/2/3, I2C1,
 * SPI1, TIM1, EXTI4 for the IMU DRDY line). The remaining STM32H743 IRQ
 * slots are reserved as Default_Handler placeholders -- this table has
 * NOT been exhaustively cross-checked against ST's RM0433 reference
 * manual table, and must be verified/extended before real use. See
 * README.md.
 */

    .syntax unified
    .cpu cortex-m7
    .fpu fpv5-d16
    .thumb

/* ------------------------------------------------------------------ */
/* Reset handler                                                       */
/* ------------------------------------------------------------------ */
    .section .text.Reset_Handler
    .weak Reset_Handler
    .type Reset_Handler, %function
Reset_Handler:
    ldr   sp, =_estack

    /* Copy .data initializers from FLASH (_sidata) to RAM (_sdata.._edata) */
    ldr   r0, =_sdata
    ldr   r1, =_edata
    ldr   r2, =_sidata
    movs  r3, #0
    b     LoopCopyDataInit

CopyDataInit:
    ldr   r4, [r2, r3]
    str   r4, [r0, r3]
    adds  r3, r3, #4

LoopCopyDataInit:
    adds  r4, r0, r3
    cmp   r4, r1
    bcc   CopyDataInit

    /* Zero .bss */
    ldr   r2, =_sbss
    ldr   r4, =_ebss
    movs  r3, #0
    b     LoopFillZerobss

FillZerobss:
    str   r3, [r2]
    adds  r2, r2, #4

LoopFillZerobss:
    cmp   r2, r4
    bcc   FillZerobss

    /* FPU-enable + VTOR setup (see system_init.c) - must run before
     * __libc_init_array, since C++ static constructors are free to
     * contain floating-point code (this project builds hard-float
     * throughout) and would fault without the FPU already enabled. */
    bl    SystemInit

    /* Run C++ global/static constructors (.init_array, see linker.ld)
     * before anything in main() can rely on them. */
    bl    __libc_init_array

    /* Hand off to the C++ entry point (see apx_main_stm32h7.cpp, not
     * main/apx_main.cpp - that one assumes a hosted SITL/Linux runtime
     * and is not linked into this board's firmware image). */
    bl    main

    /* main() should never return on an embedded target; if it does,
     * spin rather than run off into undefined memory. */
LoopForever:
    b     LoopForever

    .size Reset_Handler, .-Reset_Handler

/* ------------------------------------------------------------------ */
/* Default handler: catches any exception/IRQ without its own handler  */
/* ------------------------------------------------------------------ */
    .section .text.Default_Handler, "ax"
Default_Handler:
InfiniteLoop:
    b     InfiniteLoop
    .size Default_Handler, .-Default_Handler

/* Give every vector a weak alias to Default_Handler; a driver/board
 * translation unit can override any one of these by defining a strong
 * symbol with the same name. */
    .macro weak_alias name
    .weak \name
    .thumb_set \name, Default_Handler
    .endm

    weak_alias NMI_Handler
    weak_alias HardFault_Handler
    weak_alias MemManage_Handler
    weak_alias BusFault_Handler
    weak_alias UsageFault_Handler
    weak_alias SVC_Handler
    weak_alias DebugMon_Handler
    weak_alias PendSV_Handler
    weak_alias SysTick_Handler

    weak_alias WWDG1_IRQHandler
    weak_alias EXTI4_IRQHandler
    weak_alias EXTI9_5_IRQHandler
    weak_alias TIM1_CC_IRQHandler
    weak_alias I2C1_EV_IRQHandler
    weak_alias I2C1_ER_IRQHandler
    weak_alias SPI1_IRQHandler
    weak_alias USART1_IRQHandler
    weak_alias USART2_IRQHandler
    weak_alias USART3_IRQHandler

/* ------------------------------------------------------------------ */
/* Vector table                                                        */
/* ------------------------------------------------------------------ */
    .section .isr_vector, "a"
    .type g_pfnVectors, %object
g_pfnVectors:
    .word _estack                    /* 0:  initial stack pointer (DTCM top) */
    .word Reset_Handler               /* 1 */
    .word NMI_Handler                 /* 2 */
    .word HardFault_Handler           /* 3 */
    .word MemManage_Handler           /* 4 */
    .word BusFault_Handler            /* 5 */
    .word UsageFault_Handler          /* 6 */
    .word 0                           /* 7:  reserved */
    .word 0                           /* 8:  reserved */
    .word 0                           /* 9:  reserved */
    .word 0                           /* 10: reserved */
    .word SVC_Handler                 /* 11 */
    .word DebugMon_Handler            /* 12 */
    .word 0                           /* 13: reserved */
    .word PendSV_Handler              /* 14 */
    .word SysTick_Handler             /* 15 */

    /* STM32H743 peripheral IRQs (IRQ0 = WWDG1 per RM0433). Only the
     * vectors this board's drivers use are named; everything else in
     * this placeholder range is Default_Handler. Verify/extend against
     * RM0433 Table 132 before relying on any IRQ not listed here. */
    .word WWDG1_IRQHandler            /* IRQ0 */
    .word Default_Handler             /* IRQ1  PVD_PVM */
    .word Default_Handler             /* IRQ2  RTC_TAMP_STAMP_CSS_LSE */
    .word Default_Handler             /* IRQ3  RTC_WKUP */
    .word Default_Handler             /* IRQ4  FLASH */
    .word Default_Handler             /* IRQ5  RCC */
    .word Default_Handler             /* IRQ6  EXTI0 */
    .word Default_Handler             /* IRQ7  EXTI1 */
    .word Default_Handler             /* IRQ8  EXTI2 */
    .word Default_Handler             /* IRQ9  EXTI3 */
    .word EXTI4_IRQHandler            /* IRQ10 EXTI4 -- IMU_DRDY_PIN (PC4) */
    .word Default_Handler             /* IRQ11 DMA1_Stream0 */
    .word Default_Handler             /* IRQ12 DMA1_Stream1 */
    .word Default_Handler             /* IRQ13 DMA1_Stream2 */
    .word Default_Handler             /* IRQ14 DMA1_Stream3 */
    .word Default_Handler             /* IRQ15 DMA1_Stream4 */
    .word Default_Handler             /* IRQ16 DMA1_Stream5 */
    .word Default_Handler             /* IRQ17 DMA1_Stream6 */
    .word Default_Handler             /* IRQ18 ADC1_2 */
    .word Default_Handler             /* IRQ19 FDCAN1_IT0 */
    .word Default_Handler             /* IRQ20 FDCAN2_IT0 */
    .word Default_Handler             /* IRQ21 FDCAN1_IT1 */
    .word Default_Handler             /* IRQ22 FDCAN2_IT1 */
    .word EXTI9_5_IRQHandler          /* IRQ23 EXTI9_5 */
    .word Default_Handler             /* IRQ24 TIM1_BRK */
    .word Default_Handler             /* IRQ25 TIM1_UP */
    .word Default_Handler             /* IRQ26 TIM1_TRG_COM */
    .word TIM1_CC_IRQHandler          /* IRQ27 TIM1_CC -- MOTOR1..4 PWM timer */
    .word Default_Handler             /* IRQ28 TIM2 */
    .word Default_Handler             /* IRQ29 TIM3 */
    .word Default_Handler             /* IRQ30 TIM4 */
    .word I2C1_EV_IRQHandler          /* IRQ31 I2C1_EV -- MAG_I2C_BUS / BATT_I2C_BUS */
    .word I2C1_ER_IRQHandler          /* IRQ32 I2C1_ER */
    .word Default_Handler             /* IRQ33 I2C2_EV */
    .word Default_Handler             /* IRQ34 I2C2_ER */
    .word SPI1_IRQHandler             /* IRQ35 SPI1 -- IMU_SPI_BUS */
    .word Default_Handler             /* IRQ36 SPI2 */
    .word USART1_IRQHandler           /* IRQ37 USART1 -- GPS_UART_ID */
    .word USART2_IRQHandler           /* IRQ38 USART2 -- TELEM_UART_ID */
    .word USART3_IRQHandler           /* IRQ39 USART3 -- RC_UART_ID */

    /* Remaining STM32H743 IRQ slots (EXTI15_10 through the CM7-specific
     * IRQs in the 140s) are intentionally left as a reserved block of
     * Default_Handler placeholders rather than individually numbered --
     * add named vectors here as drivers that need them are ported. */
    .rept 110
    .word Default_Handler
    .endr

    .size g_pfnVectors, .-g_pfnVectors
