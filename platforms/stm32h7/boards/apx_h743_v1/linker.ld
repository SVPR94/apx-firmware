/* SPDX-License-Identifier: CC-BY-NC-4.0
 * Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
 * Author: Savita Pawar (SVPR94)
 * Created: 2026
 * APX -- linker script for apx_h743_v1 (STM32H743, 2MB flash / 128KB DTCM
 * / 512KB AXI SRAM D1). Standard Cortex-M GNU-ld layout: vector table
 * first in flash, .text/.rodata in flash, .data copied down to RAM at
 * startup, .bss zeroed at startup, main stack in DTCM for speed.
 *
 * This is a generic, original Cortex-M7/GCC linker script written for
 * this board's memory map -- the section layout below (isr_vector, text,
 * data, bss, heap/stack) is the standard pattern used by essentially
 * every bare-metal Cortex-M linker script, not specific to any one
 * project.
 */

ENTRY(Reset_Handler)
/* Reset_Handler is `.weak` in startup_stm32h743.s (so a board/driver TU
 * could override it) and only referenced from the vector table via a
 * `.word` (a data reference, not a call ld's GC pass necessarily follows
 * the same way it follows call-graph references) - ENTRY() alone did not
 * reliably keep it live under --gc-sections in practice (an actual build
 * warned "cannot find entry symbol Reset_Handler" and produced an
 * implausibly tiny image with most real code gc'd away). EXTERN() forces
 * it to be treated as an unresolved reference that must be pulled in and
 * rooted, which KEEP(*(.isr_vector)) alone did not guarantee. */
EXTERN(Reset_Handler)

/* Sizes are conservative defaults for the flight-control heap/stack;
 * tune per board once real stack-usage numbers exist (see README). */
_Min_Heap_Size  = 0x4000;  /* 16 KB */
_Min_Stack_Size = 0x4000;  /* 16 KB */

MEMORY
{
  /* STM32H743xI: 2 MB flash starting at the standard Cortex-M reset
   * vector location. */
  FLASH   (rx)  : ORIGIN = 0x08000000, LENGTH = 2048K

  /* DTCM: tightly-coupled, zero-wait-state RAM. Used here for the main
   * stack so the control-loop hot path never touches AXI SRAM. */
  DTCM    (rwx) : ORIGIN = 0x20000000, LENGTH = 128K

  /* AXI SRAM (D1 domain): general .data/.bss/heap RAM. */
  RAM     (rwx) : ORIGIN = 0x24000000, LENGTH = 512K
}

/* Top of DTCM = initial main stack pointer (stack grows down). */
_estack = ORIGIN(DTCM) + LENGTH(DTCM);

SECTIONS
{
  /* Cortex-M vector table: must be the very first thing in flash so the
   * boot ROM's reset sequence finds it at 0x08000000. */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)

    KEEP(*(.init))
    KEEP(*(.fini))

    . = ALIGN(4);
    _etext = .;
  } >FLASH

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  /* C++ static constructors: __libc_init_array() (called from
   * Reset_Handler, see startup_stm32h743.s) walks this table. */
  .preinit_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN(__preinit_array_start = .);
    KEEP(*(.preinit_array*))
    PROVIDE_HIDDEN(__preinit_array_end = .);
  } >FLASH

  .init_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN(__init_array_start = .);
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array*))
    PROVIDE_HIDDEN(__init_array_end = .);
  } >FLASH

  .fini_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN(__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*)))
    KEEP(*(.fini_array*))
    PROVIDE_HIDDEN(__fini_array_end = .);
  } >FLASH

  /* Load address in FLASH for .data; run address is RAM (see AT>). */
  _sidata = LOADADDR(.data);

  .data : ALIGN(4)
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } >RAM AT> FLASH

  .bss : ALIGN(4)
  {
    . = ALIGN(4);
    _sbss = .;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
  } >RAM

  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE(end = .);
    PROVIDE(_end = .);
    . = . + _Min_Heap_Size;
    . = ALIGN(8);
  } >RAM

  /* Only debug/vendor-metadata sections, NOT "libc.a(*)"/"libm.a(*)" -
   * an earlier version of this script discarded those wholesale, which
   * throws away the entire C/math library (memcpy, malloc, printf,
   * trig functions, everything __libc_init_array or any real driver
   * code would need), not just their unused parts - --gc-sections
   * above already drops whatever object code genuinely isn't
   * referenced; this block is for sections that exist but were never
   * meant to end up in the final image at all. Caught by an actual
   * build+link (text size was implausibly small - a handful of bytes
   * for real flight-control math - until this was fixed).
   */
  /DISCARD/ :
  {
    *(.comment)
    *(.ARM.attributes)
  }
}
