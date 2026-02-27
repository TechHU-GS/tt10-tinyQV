// ============================================================================
// Test F: Timer Edge Cases
// ============================================================================
// Target: LoRa Edge SoC (TinyQV RV32EC @ 25MHz)
// Tests: Timer countdown=1 → IRQ, timer=0 no IRQ, timer reload.
//
// Build:
//   riscv64-elf-gcc -march=rv32ec_zicsr -mabi=ilp32e -nostdlib -Os \
//     -T fw_irq_timer.ld -o fw_timer_edge.elf fw_timer_edge.c
//   riscv64-elf-objcopy -O verilog fw_timer_edge.elf fw_timer_edge.hex
//
// Strategy:
//   Uses ISR from Test A pattern to detect timer interrupts.
//   F1: Timer=1 → should fire IRQ (minimum timeout)
//   F2: Timer=0 → should NOT fire (write 0 = clear/cancel)
//   F3: Timer reload after expired → fires again
//
// Expected UART output: "F1F2F3DN" (8 chars)
// ============================================================================

#define PERI_BASE       0x08000000u
#define UART_DATA       (*(volatile unsigned int*)(PERI_BASE + 0x10))
#define UART_STATUS     (*(volatile unsigned int*)(PERI_BASE + 0x14))
#define TIMER_COUNTDOWN (*(volatile unsigned int*)(PERI_BASE + 0x30))

#define UART_TX_BUSY    (1u << 0)

// Shared state in PSRAM
#define P_IRQ_COUNT ((volatile unsigned int *)0x01000084)
#define P_MCAUSE    ((volatile unsigned int *)0x01000080)

// ============================================================================
// Vector table + ISR
// ============================================================================
void __attribute__((naked, section(".text._vectors"))) _vectors(void) {
    __asm__ volatile (
        ".option push\n"
        ".option norvc\n"
        "j _reset_handler\n"
        "j _trap_handler\n"
        "j _irq_handler\n"
        ".option pop\n"
    );
}

// TODO: Production firmware should trigger WDT reboot instead of infinite loop.
//       Fix: write non-zero to PERI_WDT (0x8000034) then loop until reset.
void __attribute__((naked)) _trap_handler(void) {
    __asm__ volatile ("j _trap_handler\n");
}

void __attribute__((naked)) _irq_handler(void) {
    __asm__ volatile (
        "addi sp, sp, -24\n"
        "sw ra, 0(sp)\n"
        "sw t0, 4(sp)\n"
        "sw t1, 8(sp)\n"
        "sw a0, 12(sp)\n"
        "sw a1, 16(sp)\n"
        "sw a2, 20(sp)\n"
        // Read mcause
        "csrr t0, mcause\n"
        // Store mcause
        "lui t1, 0x01000\n"
        "sw t0, 0x80(t1)\n"
        // Clear timer_irq: write 0 to TIMER
        "sw zero, 0x30(tp)\n"
        // Clear mip_reg bit 17
        "lui t0, 0x20\n"
        "csrc 0x344, t0\n"
        // Increment irq_count
        "lui t1, 0x01000\n"
        "lw t0, 0x84(t1)\n"
        "addi t0, t0, 1\n"
        "sw t0, 0x84(t1)\n"
        // Restore
        "lw ra, 0(sp)\n"
        "lw t0, 4(sp)\n"
        "lw t1, 8(sp)\n"
        "lw a0, 12(sp)\n"
        "lw a1, 16(sp)\n"
        "lw a2, 20(sp)\n"
        "addi sp, sp, 24\n"
        "mret\n"
    );
}

// ============================================================================
// UART + helpers
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
        "li sp, 0x01000100\n"
        "j main\n"
    );
}

void __attribute__((noreturn)) main(void) {
    // Initialize shared state
    *P_IRQ_COUNT = 0;
    *P_MCAUSE = 0;

    // Enable IRQ17 (timer) in mie
    unsigned int mie_irq17 = (1u << 17);
    __asm__ volatile ("csrs 0x304, %0" : : "r"(mie_irq17));

    // Enable global interrupts
    unsigned int mstatus_mie = 8;
    __asm__ volatile ("csrs mstatus, %0" : : "r"(mstatus_mie));

    // ---- Test 1: Timer=1 (minimum timeout, should fire) ----
    *P_IRQ_COUNT = 0;
    TIMER_COUNTDOWN = 1;  // 1 microsecond

    {
        unsigned int timeout = 500000;
        while (*P_IRQ_COUNT == 0 && timeout > 0) timeout--;
        uart_putc('F');
        uart_putc((*P_IRQ_COUNT == 1) ? '1' : '0');
    }

    // ---- Test 2: Timer=0 (should NOT fire, just clear) ----
    *P_IRQ_COUNT = 0;
    TIMER_COUNTDOWN = 0;  // Write 0 = clear, should not trigger

    // Wait a bit to confirm no IRQ fires
    {
        unsigned int delay;
        for (delay = 0; delay < 2000; delay++)
            __asm__ volatile ("" ::: "memory");

        uart_putc('F');
        uart_putc((*P_IRQ_COUNT == 0) ? '2' : '0');
    }

    // ---- Test 3: Timer reload (fire again after previous expired) ----
    *P_IRQ_COUNT = 0;
    TIMER_COUNTDOWN = 50;  // 50 microseconds

    {
        unsigned int timeout = 500000;
        while (*P_IRQ_COUNT == 0 && timeout > 0) timeout--;
        uart_putc('F');
        uart_putc((*P_IRQ_COUNT == 1) ? '3' : '0');
    }

    // Disable interrupts
    __asm__ volatile ("csrc mstatus, %0" : : "r"(mstatus_mie));

    uart_putc('D');
    uart_putc('N');

    while (1);
}
