/* Linker script for IRQ timer test firmware */
/* Vector table: 0x0=reset, 0x4=trap, 0x8=interrupt */
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
        *(.text._vectors)   /* Vector table: 0x0, 0x4, 0x8 */
        *(.text .text.*)
        *(.rodata .rodata.*)
    } > FLASH

    /DISCARD/ : {
        *(.data .data.*)
        *(.bss .bss.*)
        *(.sbss .sbss.*)
        *(.sdata .sdata.*)
        *(.comment)
        *(.note*)
    }
}
