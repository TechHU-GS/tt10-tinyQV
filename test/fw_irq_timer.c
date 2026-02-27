// ============================================================================
// Test A: Timer IRQ17 — Interrupt Entry/Exit Verification
// ============================================================================
// Target: LoRa Edge SoC (TinyQV RV32EC @ 25MHz)
// Tests: ISR entry, mcause identification, timer_irq clearing,
//        mip_reg clearing via csrc, MRET return, double interrupt
//
// Build:
//   riscv64-elf-gcc -march=rv32ec_zicsr -mabi=ilp32e -nostdlib -Os \
//     -T fw_irq_timer.ld -o fw_irq_timer.elf fw_irq_timer.c
//   riscv64-elf-objcopy -O verilog fw_irq_timer.elf fw_irq_timer.hex
//
// Expected UART output: "I1I2DN" (two IRQs pass + done)
// ============================================================================

#define PERI_BASE       0x08000000u
#define UART_DATA       (*(volatile unsigned int*)(PERI_BASE + 0x10))
#define UART_STATUS     (*(volatile unsigned int*)(PERI_BASE + 0x14))
#define TIMER_COUNTDOWN (*(volatile unsigned int*)(PERI_BASE + 0x30))
#define SYS_INFO        (*(volatile unsigned int*)(PERI_BASE + 0x3C))

#define UART_TX_BUSY    (1u << 0)

// ============================================================================
// Shared state between ISR and main (volatile!)
// ============================================================================
static volatile unsigned int irq_count;
static volatile unsigned int last_mcause;

// ============================================================================
// Vector table — MUST be at addresses 0x0, 0x4, 0x8
// Each entry is a 4-byte JAL (compressed j is only 2 bytes but alignment
// requires exactly 4 bytes per slot to hit 0x4 and 0x8)
// ============================================================================
void __attribute__((naked, section(".text._vectors"))) _vectors(void) {
    __asm__ volatile (
        ".option push\n"
        ".option norvc\n"          // Force 4-byte instructions for alignment
        "j _reset_handler\n"       // 0x0: reset vector
        "j _trap_handler\n"        // 0x4: trap vector
        "j _irq_handler\n"         // 0x8: interrupt vector
        ".option pop\n"
    );
}

// ============================================================================
// Trap handler (should never fire in this test)
// TODO: Production firmware should trigger WDT reboot instead of infinite loop.
//       Dead loop = device permanently hung if trap fires in the field.
//       Fix: write non-zero to PERI_WDT (0x8000034) then loop until reset.
// ============================================================================
void __attribute__((naked)) _trap_handler(void) {
    __asm__ volatile (
        "j _trap_handler\n"        // infinite loop on unexpected trap
    );
}

// ============================================================================
// IRQ handler — must save/restore all registers used
// Hardware only saves mepc and mcause. We must save everything else.
// ============================================================================
void __attribute__((naked)) _irq_handler(void) {
    __asm__ volatile (
        // Save registers to stack (RV32E: x1,x5-x15 are caller-saved)
        // gp(x3) and tp(x4) are hardwired, no need to save
        "addi sp, sp, -24\n"
        "sw ra, 0(sp)\n"           // x1
        "sw t0, 4(sp)\n"           // x5
        "sw t1, 8(sp)\n"           // x6
        "sw a0, 12(sp)\n"          // x10
        "sw a1, 16(sp)\n"          // x11
        "sw a2, 20(sp)\n"          // x12

        // Read mcause
        "csrr t0, mcause\n"

        // Store mcause for main to inspect
        // last_mcause is a global — use a known PSRAM address
        // We'll use gp-relative or absolute address
        "lui t1, 0x01000\n"        // t1 = 0x01000000 (PSRAM base)
        "sw t0, 0x80(t1)\n"       // store mcause at PSRAM+0x80

        // Clear timer_irq: write 0 to TIMER register
        // TIMER is at PERI_BASE + 0x30 = tp + 0x30
        "sw zero, 0x30(tp)\n"     // TIMER_COUNTDOWN = 0, clears timer_irq

        // Clear mip_reg bit 1 (IRQ17): csrc mip, (1<<17)
        // mip CSR = 0x344, bit 17 = IRQ17
        // In TinyQV, mip_reg bits map to mip[1:0], so bit 1 = IRQ17
        // The CSR value has bit 17 at position 17 in the 32-bit CSR
        "lui t0, 0x20\n"          // t0 = 0x00020000 = (1 << 17)
        "csrc 0x344, t0\n"        // clear mip bit 17

        // Increment irq_count at PSRAM+0x84
        "lui t1, 0x01000\n"
        "lw t0, 0x84(t1)\n"
        "addi t0, t0, 1\n"
        "sw t0, 0x84(t1)\n"

        // Restore registers
        "lw ra, 0(sp)\n"
        "lw t0, 4(sp)\n"
        "lw t1, 8(sp)\n"
        "lw a0, 12(sp)\n"
        "lw a1, 16(sp)\n"
        "lw a2, 20(sp)\n"
        "addi sp, sp, 24\n"

        // Return from interrupt
        "mret\n"
    );
}

// ============================================================================
// UART helpers
// ============================================================================
static void uart_putc(unsigned char c) {
    while (UART_STATUS & UART_TX_BUSY);
    UART_DATA = c;
}

static void uart_result(unsigned char tag, int pass) {
    uart_putc(tag);
    uart_putc(pass ? '1' : '0');
}

// ============================================================================
// Reset handler (main entry after vector table)
// ============================================================================
void __attribute__((naked, noreturn)) _reset_handler(void) {
    __asm__ volatile (
        "li sp, 0x01000100\n"      // Stack = PSRAM + 256B
        "j main\n"
    );
}

void __attribute__((noreturn)) main(void) {
    // Use PSRAM for shared volatile state (flash is read-only)
    volatile unsigned int *p_irq_count = (volatile unsigned int *)0x01000084;
    volatile unsigned int *p_mcause   = (volatile unsigned int *)0x01000080;

    // Initialize shared state
    *p_irq_count = 0;
    *p_mcause = 0;

    // ---- Test 1: First timer interrupt ----

    // Enable IRQ17 in mie register
    // mie CSR = 0x304, IRQ17 = bit 17
    unsigned int mie_irq17 = (1u << 17);
    __asm__ volatile ("csrs 0x304, %0" : : "r"(mie_irq17));

    // Enable global interrupts (mstatus.MIE = bit 3)
    unsigned int mstatus_mie = 8;
    __asm__ volatile ("csrs mstatus, %0" : : "r"(mstatus_mie));

    // Start timer: 100 microseconds
    TIMER_COUNTDOWN = 100;

    // Poll for IRQ (no WFI support)
    {
        unsigned int timeout = 500000;
        while (*p_irq_count == 0 && timeout > 0) timeout--;

        // Check: irq fired, mcause = 0x80000011 (interrupt + cause 17)
        // mcause in TinyQV: bit[5]=interrupt, bits[4:0]=cause
        // IRQ17: mcause[5]=1, mcause[4:0]=10001 = 0x11
        // Full 32-bit: bit 31 + bits 4:0 = 0x80000011
        // But TinyQV stores in 6-bit mcause: {interrupt_flag, cause[4:0]}
        // CSR read reconstructs: mcause[5] at bit 31, mcause[4:0] at bits 4:0
        unsigned int mc = *p_mcause;
        int pass = (*p_irq_count == 1) &&
                   ((mc & 0x80000000) != 0) &&   // interrupt flag
                   ((mc & 0x1F) == 17);           // cause = 17

        uart_result('I', pass);
    }

    // ---- Test 2: Second timer interrupt (verify re-arm works) ----
    TIMER_COUNTDOWN = 50;  // shorter timeout

    {
        unsigned int timeout = 500000;
        while (*p_irq_count < 2 && timeout > 0) timeout--;

        // Output "I2" for second test pass, "I0" for fail
        uart_putc('I');
        uart_putc((*p_irq_count == 2) ? '2' : '0');
    }

    // ---- Done ----
    // Disable interrupts before finishing
    __asm__ volatile ("csrc mstatus, %0" : : "r"(mstatus_mie));

    uart_putc('D');
    uart_putc('N');

    while (1);
}
