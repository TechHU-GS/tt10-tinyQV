// ============================================================================
// Test: IRQ Priority — DIO1 (IRQ16) vs Timer (IRQ17) Priority Verification
// ============================================================================
// Target: LoRa Edge SoC (TinyQV RV32EC @ 25MHz)
// Tests:
//   P1: Timer IRQ17 alone         -> mcause=17
//   P2: DIO1  IRQ16 alone         -> mcause=16
//   P3: Both IRQ16+IRQ17 simul.   -> IRQ16 fires first (lower bit = higher priority)
//   P4: After ISR clears IRQ16    -> second ISR fires for IRQ17
//
// TinyQV priority encoder (core.v casez on mip & mie):
//   5'b0???1 -> IRQ16 (cause 16)  -- highest among interrupt_req
//   5'b0??10 -> IRQ17 (cause 17)
//
// IRQ16 and IRQ17 use edge-capture into mip_reg. The TB must produce
// a rising edge on ui_in[0] to trigger IRQ16. The firmware clears mip_reg
// via csrc 0x344. After clearing IRQ16's mip bit, the pending IRQ17
// fires on the next instruction boundary.
//
// Firmware signals "clear DIO1 now" to TB via GPIO_OUT bit 0 = 1.
// TB watches uo_out[7] (gpio_out_sel[7]=1, gpio_out[7]) for the signal.
//
// Build:
//   riscv64-elf-gcc -march=rv32ec_zicsr -mabi=ilp32e -nostdlib -Os \
//     -T fw_irq_priority.ld -o fw_irq_priority.elf fw_irq_priority.c
//   riscv64-elf-objcopy -O verilog fw_irq_priority.elf fw_irq_priority.hex
//
// Expected UART output: "P1P2P3P4DN" (10 chars)
// ============================================================================

#define PERI_BASE       0x08000000u
#define GPIO_OUT        (*(volatile unsigned int*)(PERI_BASE + 0x00))
#define GPIO_OUT_SEL    (*(volatile unsigned int*)(PERI_BASE + 0x0C))
#define UART_DATA       (*(volatile unsigned int*)(PERI_BASE + 0x10))
#define UART_STATUS     (*(volatile unsigned int*)(PERI_BASE + 0x14))
#define TIMER_COUNTDOWN (*(volatile unsigned int*)(PERI_BASE + 0x30))

#define UART_TX_BUSY    (1u << 0)

// PSRAM shared state addresses
// irq_count      @ 0x01000084
// mcause_log[0]  @ 0x01000088  (first ISR's mcause)
// mcause_log[1]  @ 0x0100008C  (second ISR's mcause)
// mcause_log[2]  @ 0x01000090
// mcause_log[3]  @ 0x01000094

// ============================================================================
// Vector table — 0x0, 0x4, 0x8
// ============================================================================
void __attribute__((naked, section(".text._vectors"))) _vectors(void) {
    __asm__ volatile (
        ".option push\n"
        ".option norvc\n"
        "j _reset_handler\n"       // 0x0: reset
        "j _trap_handler\n"        // 0x4: trap
        "j _irq_handler\n"         // 0x8: interrupt
        ".option pop\n"
    );
}

// ============================================================================
// Trap handler (should never fire)
// TODO: Production firmware should trigger WDT reboot instead of infinite loop.
//       Fix: write non-zero to PERI_WDT (0x8000034) then loop until reset.
// ============================================================================
void __attribute__((naked)) _trap_handler(void) {
    __asm__ volatile ("j _trap_handler\n");
}

// ============================================================================
// IRQ handler
// - Reads mcause
// - Stores mcause into mcause_log[irq_count] (PSRAM+0x88 + irq_count*4)
// - If cause==17 (timer): write 0 to TIMER to clear timer_irq
// - Clears the corresponding mip_reg bit via csrc 0x344
// - Increments irq_count
// ============================================================================
void __attribute__((naked)) _irq_handler(void) {
    __asm__ volatile (
        // Save caller-saved registers
        "addi sp, sp, -24\n"
        "sw ra, 0(sp)\n"
        "sw t0, 4(sp)\n"
        "sw t1, 8(sp)\n"
        "sw a0, 12(sp)\n"
        "sw a1, 16(sp)\n"
        "sw a2, 20(sp)\n"

        // Read mcause into t0
        "csrr t0, mcause\n"

        // Load irq_count from PSRAM+0x84
        "lui t1, 0x01000\n"        // t1 = 0x01000000
        "lw a0, 0x84(t1)\n"       // a0 = irq_count

        // Store mcause into mcause_log[irq_count]
        // address = 0x01000088 + irq_count * 4
        "slli a1, a0, 2\n"        // a1 = irq_count * 4
        "addi a1, a1, 0x88\n"     // a1 = 0x88 + irq_count*4 (offset)
        "add a1, a1, t1\n"        // a1 = 0x01000000 + offset
        "sw t0, 0(a1)\n"          // store mcause

        // Check cause: extract bits [4:0]
        "andi a2, t0, 0x1F\n"     // a2 = cause number

        // If cause == 17 (timer), clear timer_irq by writing 0 to TIMER
        "li a1, 17\n"
        "bne a2, a1, 1f\n"
        "sw zero, 0x30(tp)\n"     // TIMER_COUNTDOWN = 0
        "1:\n"

        // Clear mip_reg bit for this cause
        // IRQ16 -> bit 16, IRQ17 -> bit 17
        // Build mask: 1 << cause_number
        "li a1, 1\n"
        "sll a1, a1, a2\n"        // a1 = (1 << cause)
        "csrc 0x344, a1\n"        // clear mip bit

        // Increment irq_count
        "addi a0, a0, 1\n"
        "sw a0, 0x84(t1)\n"       // store updated irq_count

        // Restore registers
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
// UART helpers
// ============================================================================
static void uart_putc(unsigned char c) {
    while (UART_STATUS & UART_TX_BUSY);
    UART_DATA = c;
}

// Output "P<n>" on pass, "F<n>" on fail
static void uart_result(unsigned char test_num, int pass) {
    uart_putc(pass ? 'P' : 'F');
    uart_putc(test_num);
}

// ============================================================================
// Reset handler
// ============================================================================
void __attribute__((naked, noreturn)) _reset_handler(void) {
    __asm__ volatile (
        "li sp, 0x01000100\n"
        "lui tp, 0x08000\n"       // tp = PERI_BASE (0x08000000) for ISR use
        "j main\n"
    );
}

void __attribute__((noreturn)) main(void) {
    volatile unsigned int *p_irq_count  = (volatile unsigned int *)0x01000084;
    volatile unsigned int *p_mcause_log = (volatile unsigned int *)0x01000088;

    // Initialize shared state
    *p_irq_count = 0;
    p_mcause_log[0] = 0;
    p_mcause_log[1] = 0;
    p_mcause_log[2] = 0;
    p_mcause_log[3] = 0;

    // Enable IRQ16 and IRQ17 in mie
    // mie bits: bit 16 = IRQ16, bit 17 = IRQ17
    unsigned int mie_bits = (1u << 16) | (1u << 17);
    __asm__ volatile ("csrs 0x304, %0" : : "r"(mie_bits));

    // Enable global interrupts (mstatus.MIE = bit 3)
    unsigned int mstatus_mie = 8;
    __asm__ volatile ("csrs mstatus, %0" : : "r"(mstatus_mie));

    // ================================================================
    // Test P1: Timer IRQ17 alone -> verify mcause = 17
    // ================================================================
    {
        *p_irq_count = 0;
        TIMER_COUNTDOWN = 100;  // 100us

        unsigned int timeout = 500000;
        while (*p_irq_count == 0 && timeout > 0) timeout--;

        unsigned int mc = p_mcause_log[0];
        int pass = (*p_irq_count >= 1) &&
                   ((mc & 0x80000000) != 0) &&   // interrupt flag set
                   ((mc & 0x1F) == 17);           // cause = 17
        uart_result('1', pass);
    }

    // Small delay to ensure P1 is fully settled
    {
        volatile unsigned int d;
        for (d = 0; d < 1000; d++);
    }

    // ================================================================
    // Test P2: DIO1 IRQ16 alone -> verify mcause = 16
    // TB drives ui_in[0] = 1 (rising edge triggers mip_reg[16])
    // Firmware signals TB by setting GPIO_OUT bit 7 = 1
    // ================================================================
    {
        *p_irq_count = 0;

        // Signal TB: "ready for P2, please assert DIO1"
        // Set GPIO_OUT_SEL bit 7 = 1 (override uo_out[7] to gpio_out[7])
        GPIO_OUT_SEL = 0x80;
        GPIO_OUT = 0x80;  // gpio_out[7] = 1 -> TB sees uo_out[7] = 1

        unsigned int timeout = 500000;
        while (*p_irq_count == 0 && timeout > 0) timeout--;

        unsigned int mc = p_mcause_log[0];
        int pass = (*p_irq_count >= 1) &&
                   ((mc & 0x80000000) != 0) &&
                   ((mc & 0x1F) == 16);           // cause = 16
        uart_result('2', pass);

        // Clear the GPIO signal
        GPIO_OUT = 0x00;
    }

    // Small delay
    {
        volatile unsigned int d;
        for (d = 0; d < 1000; d++);
    }

    // ================================================================
    // Test P3 + P4: Both IRQ16 + IRQ17 simultaneously
    // Disable global interrupts → trigger both → wait for both mip
    // bits to be pending → re-enable → priority encoder must pick
    // IRQ16 (lower bit = higher priority per core.v casez).
    // After ISR handles IRQ16, IRQ17 should fire as second ISR.
    // ================================================================
    {
        *p_irq_count = 0;
        p_mcause_log[0] = 0;
        p_mcause_log[1] = 0;

        // Disable global interrupts so both IRQs accumulate in mip_reg
        __asm__ volatile ("csrc mstatus, %0" : : "r"(mstatus_mie));

        // Signal TB: "ready for P3, please assert DIO1"
        GPIO_OUT = 0x80;

        // Start timer (very short countdown)
        TIMER_COUNTDOWN = 5;  // 5us -- fires while interrupts off

        // Spin ~200 cycles for timer to expire + DIO1 edge to capture
        {
            volatile unsigned int d;
            for (d = 0; d < 50; d++);
        }

        // Now both mip_reg[16] (DIO1) and mip_reg[17] (timer) are set.
        // Re-enable global interrupts — priority encoder resolves.
        __asm__ volatile ("csrs mstatus, %0" : : "r"(mstatus_mie));

        // Wait for BOTH interrupts
        unsigned int timeout = 500000;
        while (*p_irq_count < 2 && timeout > 0) timeout--;

        // P3: First ISR should be IRQ16 (mcause = 16)
        {
            unsigned int mc = p_mcause_log[0];
            int pass = (*p_irq_count >= 1) &&
                       ((mc & 0x80000000) != 0) &&
                       ((mc & 0x1F) == 16);
            uart_result('3', pass);
        }

        // P4: Second ISR should be IRQ17 (mcause = 17)
        {
            unsigned int mc = p_mcause_log[1];
            int pass = (*p_irq_count >= 2) &&
                       ((mc & 0x80000000) != 0) &&
                       ((mc & 0x1F) == 17);
            uart_result('4', pass);
        }

        GPIO_OUT = 0x00;
    }

    // ================================================================
    // Done
    // ================================================================
    __asm__ volatile ("csrc mstatus, %0" : : "r"(mstatus_mie));

    uart_putc('D');
    uart_putc('N');

    while (1);
}
