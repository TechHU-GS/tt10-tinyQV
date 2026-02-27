// ============================================================================
// Test B: WDT Reboot — Watchdog Timeout → Reset → CPU Restart
// ============================================================================
// Target: LoRa Edge SoC (TinyQV RV32EC @ 25MHz)
// Tests: WDT expiry triggers wdt_reset, reset_hold_counter holds for
//        32 cycles, CPU reboots from address 0, PSRAM survives reset.
//
// Build:
//   riscv64-elf-gcc -march=rv32ec_zicsr -mabi=ilp32e -nostdlib -Os \
//     -T fw_irq_timer.ld -o fw_wdt_reboot.elf fw_wdt_reboot.c
//   riscv64-elf-objcopy -O verilog fw_wdt_reboot.elf fw_wdt_reboot.hex
//
// Strategy:
//   Boot 1: Write magic to PSRAM, output "B1", enable WDT with short
//           timeout, spin forever (no kick). WDT fires → reboot.
//   Boot 2: Read PSRAM magic, if present → output "B2DN".
//
// Expected UART output: "B1B2DN" (6 chars)
// ============================================================================

#define PERI_BASE       0x08000000u
#define UART_DATA       (*(volatile unsigned int*)(PERI_BASE + 0x10))
#define UART_STATUS     (*(volatile unsigned int*)(PERI_BASE + 0x14))
#define WDT_KICK        (*(volatile unsigned int*)(PERI_BASE + 0x34))

#define UART_TX_BUSY    (1u << 0)

// PSRAM address for boot detection magic
#define PSRAM_MAGIC_ADDR  ((volatile unsigned int *)0x01000200)
#define BOOT_MAGIC        0xB007CAFE

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
    // Check if this is a warm boot (PSRAM magic survives WDT reset)
    unsigned int magic = *PSRAM_MAGIC_ADDR;

    if (magic == BOOT_MAGIC) {
        // ---- Boot 2: WDT reset happened, we rebooted ----
        // Clear magic so next power-on doesn't false-positive
        *PSRAM_MAGIC_ADDR = 0;

        uart_putc('B');
        uart_putc('2');
        uart_putc('D');
        uart_putc('N');

        while (1);
    }

    // ---- Boot 1: Cold boot — set up WDT and let it fire ----

    // Write magic to PSRAM for boot detection
    *PSRAM_MAGIC_ADDR = BOOT_MAGIC;

    // Verify PSRAM write worked
    unsigned int readback = *PSRAM_MAGIC_ADDR;
    if (readback != BOOT_MAGIC) {
        // PSRAM write failed — can't proceed
        uart_putc('B');
        uart_putc('0');
        while (1);
    }

    uart_putc('B');
    uart_putc('1');

    // Wait for UART to finish transmitting before WDT fires
    // Each char = ~87us (10 bits * 8.68us/bit at 115200 baud)
    // 2 chars = ~174us. Add safety margin.
    // Use a register-only loop to avoid slow PSRAM access
    {
        unsigned int delay;
        for (delay = 0; delay < 5000; delay++)
            __asm__ volatile ("" ::: "memory");
    }

    // Enable WDT with 200us timeout (5000 clocks)
    // This is long enough for UART to finish but short enough for fast sim
    WDT_KICK = 200;

    // Spin forever — WDT will fire and reset the CPU
    while (1);
}
