// ============================================================================
// Test C: Soft Reset — Write 0xA5 → Reset → CPU Restart
// ============================================================================
// Target: LoRa Edge SoC (TinyQV RV32EC @ 25MHz)
// Tests: Soft reset via SYSINFO write, reset_hold_counter, CPU reboot,
//        peripheral state cleared, PSRAM data survives.
//
// Build:
//   riscv64-elf-gcc -march=rv32ec_zicsr -mabi=ilp32e -nostdlib -Os \
//     -T fw_irq_timer.ld -o fw_soft_reset.elf fw_soft_reset.c
//   riscv64-elf-objcopy -O verilog fw_soft_reset.elf fw_soft_reset.hex
//
// Strategy:
//   Boot 1: Start timer, write magic to PSRAM, output "S1",
//           write 0xA5 to SYSINFO → soft reset.
//   Boot 2: Read PSRAM magic → present means PSRAM survived.
//           Check timer=0 (peripheral state cleared). Output "S2DN".
//
// Expected UART output: "S1S2DN" (6 chars)
// ============================================================================

#define PERI_BASE       0x08000000u
#define UART_DATA       (*(volatile unsigned int*)(PERI_BASE + 0x10))
#define UART_STATUS     (*(volatile unsigned int*)(PERI_BASE + 0x14))
#define TIMER_COUNTDOWN (*(volatile unsigned int*)(PERI_BASE + 0x30))
#define WDT_KICK        (*(volatile unsigned int*)(PERI_BASE + 0x34))
#define SYS_INFO        (*(volatile unsigned int*)(PERI_BASE + 0x3C))

#define UART_TX_BUSY    (1u << 0)

// PSRAM address for boot detection magic
#define PSRAM_MAGIC_ADDR  ((volatile unsigned int *)0x01000200)
#define BOOT_MAGIC        0x50F7CAFE

// ============================================================================
// Vector table
// ============================================================================
void __attribute__((naked, section(".text._vectors"))) _vectors(void) {
    __asm__ volatile (
        ".option push\n"
        ".option norvc\n"
        "j _reset_handler\n"       // 0x0: reset
        "j _trap_handler\n"        // 0x4: trap
        "j _trap_handler\n"        // 0x8: interrupt (not used)
        ".option pop\n"
    );
}

// TODO: Production firmware should trigger WDT reboot instead of infinite loop.
//       Fix: write non-zero to PERI_WDT (0x8000034) then loop until reset.
void __attribute__((naked)) _trap_handler(void) {
    __asm__ volatile ("j _trap_handler\n");
}

// ============================================================================
// UART helpers
// ============================================================================
static void uart_putc(unsigned char c) {
    while (UART_STATUS & UART_TX_BUSY);
    UART_DATA = c;
}

// ============================================================================
// Reset handler
// ============================================================================
void __attribute__((naked, noreturn)) _reset_handler(void) {
    __asm__ volatile (
        "li sp, 0x01000100\n"      // Stack in PSRAM (below magic addr)
        "j main\n"
    );
}

void __attribute__((noreturn)) main(void) {
    // Check if this is a warm boot (PSRAM magic survives soft reset)
    unsigned int magic = *PSRAM_MAGIC_ADDR;

    if (magic == BOOT_MAGIC) {
        // ---- Boot 2: Soft reset happened, we rebooted ----
        // Clear magic
        *PSRAM_MAGIC_ADDR = 0;

        // Verify peripheral state is cleared:
        // Timer should be 0 after reset
        unsigned int timer_val = TIMER_COUNTDOWN;

        // Output result
        uart_putc('S');
        if (timer_val == 0) {
            uart_putc('2');  // timer cleared = pass
        } else {
            uart_putc('0');  // timer not cleared = fail
        }

        uart_putc('D');
        uart_putc('N');

        while (1);
    }

    // ---- Boot 1: Cold boot — set up and trigger soft reset ----

    // Start a timer with a large value (won't expire during test)
    TIMER_COUNTDOWN = 50000;  // 50ms

    // Write magic to PSRAM for boot detection
    *PSRAM_MAGIC_ADDR = BOOT_MAGIC;

    // Verify PSRAM write
    unsigned int readback = *PSRAM_MAGIC_ADDR;
    if (readback != BOOT_MAGIC) {
        uart_putc('S');
        uart_putc('X');  // PSRAM write failed
        while (1);
    }

    uart_putc('S');
    uart_putc('1');

    // Wait for UART to finish transmitting
    // Use a register-only loop to avoid slow PSRAM access
    {
        unsigned int delay;
        for (delay = 0; delay < 5000; delay++)
            __asm__ volatile ("" ::: "memory");
    }

    // Trigger soft reset by writing 0xA5 to SYSINFO
    SYS_INFO = 0xA5;

    // Should never reach here
    uart_putc('E');
    uart_putc('R');
    while (1);
}
