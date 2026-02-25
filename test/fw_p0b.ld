/* Linker script for P0-B firmware (PSRAM stack) */
/* _start at 0x00000000 (Flash XIP entry) */
/* Stack in PSRAM RAM_A */

ENTRY(_start)

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x00000000, LENGTH = 256K
    PSRAM (rw)  : ORIGIN = 0x01000000, LENGTH = 8K
}

SECTIONS
{
    .text : {
        *(.text._start)     /* _start MUST be first */
        *(.text .text.*)
        *(.rodata .rodata.*)
    } > FLASH

    /* No .data/.bss in flash — PSRAM used directly via pointers */
    /DISCARD/ : {
        *(.data .data.*)
        *(.bss .bss.*)
        *(.sbss .sbss.*)
        *(.sdata .sdata.*)
        *(.comment)
        *(.note*)
    }
}
