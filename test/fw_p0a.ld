/* Linker script for P0-A Flash-only firmware */
/* _start must be at address 0x00000000 (Flash XIP entry) */

ENTRY(_start)

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x00000000, LENGTH = 256K
    LMEM  (rw)  : ORIGIN = 0x04000000, LENGTH = 32
}

SECTIONS
{
    .text : {
        *(.text._start)     /* _start MUST be first */
        *(.text .text.*)
        *(.rodata .rodata.*)
    } > FLASH

    /* No .data/.bss — we don't use them in P0-A */
    /DISCARD/ : {
        *(.data .data.*)
        *(.bss .bss.*)
        *(.sbss .sbss.*)
        *(.sdata .sdata.*)
        *(.comment)
        *(.note*)
    }
}
