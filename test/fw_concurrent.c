// ============================================================================
// Test H: Concurrent Operations — Timer IRQ + I2C + CRC
// ============================================================================
// Target: LoRa Edge SoC (TinyQV RV32EC @ 25MHz)
// Tests: Multiple peripherals operating simultaneously without interference.
//
// Build:
//   riscv64-elf-gcc -march=rv32ec_zicsr -mabi=ilp32e -nostdlib -Os \
//     -T fw_irq_timer.ld -o fw_concurrent.elf fw_concurrent.c
//   riscv64-elf-objcopy -O verilog fw_concurrent.elf fw_concurrent.hex
//
// Strategy:
//   H1: Start timer IRQ, then do I2C transaction — both succeed
//   H2: CRC computation while timer IRQ fires — both correct
//   H3: I2C + CRC simultaneously — both correct
//
// Expected UART output: "H1H2H3DN" (8 chars)
// ============================================================================

#define PERI_BASE       0x08000000u
#define UART_DATA       (*(volatile unsigned int*)(PERI_BASE + 0x10))
#define UART_STATUS     (*(volatile unsigned int*)(PERI_BASE + 0x14))
#define I2C_DATA        (*(volatile unsigned int*)(PERI_BASE + 0x18))
#define I2C_CONFIG      (*(volatile unsigned int*)(PERI_BASE + 0x1C))
#define CRC16_DATA      (*(volatile unsigned int*)(PERI_BASE + 0x08))
#define TIMER_COUNTDOWN (*(volatile unsigned int*)(PERI_BASE + 0x30))

#define UART_TX_BUSY    (1u << 0)
#define I2C_CMD_START   (1u << 8)
#define I2C_CMD_READ    (1u << 9)
#define I2C_CMD_WRITE   (1u << 10)
#define I2C_CMD_STOP    (1u << 12)
#define I2C_BUSY        (1u << 9)
#define I2C_RX_VALID    (1u << 10)
#define I2C_TX_PENDING  (1u << 11)
#define CRC16_BUSY      (1u << 16)
#define CRC16_INIT      (1u << 8)

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
// Helpers
// ============================================================================
static void uart_putc(unsigned char c) {
    while (UART_STATUS & UART_TX_BUSY);
    UART_DATA = c;
}

static int i2c_wait_tx(void) {
    unsigned int t = 200000;
    while ((I2C_DATA & I2C_TX_PENDING) && t > 0) t--;
    return t > 0;
}

static int i2c_wait_idle(void) {
    unsigned int t = 200000;
    while ((I2C_DATA & I2C_BUSY) && t > 0) t--;
    return t > 0;
}

static int i2c_wait_rx(void) {
    unsigned int t = 200000;
    unsigned int v;
    while (t > 0) {
        v = I2C_DATA;
        if (v & I2C_RX_VALID)
            return v & 0xFF;
        t--;
    }
    return -1;
}

static unsigned int crc16_compute(const unsigned char *data, int len) {
    CRC16_DATA = CRC16_INIT;
    while (CRC16_DATA & CRC16_BUSY);
    for (int i = 0; i < len; i++) {
        CRC16_DATA = data[i];
        while (CRC16_DATA & CRC16_BUSY);
    }
    return CRC16_DATA & 0xFFFF;
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
    // Initialize peripherals
    I2C_CONFIG = 63;
    *P_IRQ_COUNT = 0;
    *P_MCAUSE = 0;

    // Enable IRQ17 (timer) in mie
    unsigned int mie_irq17 = (1u << 17);
    __asm__ volatile ("csrs 0x304, %0" : : "r"(mie_irq17));

    // Enable global interrupts
    unsigned int mstatus_mie = 8;
    __asm__ volatile ("csrs mstatus, %0" : : "r"(mstatus_mie));

    // ---- Test 1: Timer IRQ during I2C transaction ----
    {
        *P_IRQ_COUNT = 0;
        // Start timer — will fire during I2C
        TIMER_COUNTDOWN = 200;  // 200 us

        // Do an I2C write+read transaction to addr 0x44
        I2C_DATA = I2C_CMD_START | I2C_CMD_WRITE | 0x44;
        i2c_wait_tx();

        I2C_DATA = I2C_CMD_WRITE | 0x24;
        i2c_wait_tx();
        I2C_DATA = I2C_CMD_WRITE | I2C_CMD_STOP | 0x00;
        i2c_wait_tx();
        i2c_wait_idle();

        // Read 1 byte
        I2C_DATA = I2C_CMD_START | I2C_CMD_READ | 0x44;
        int rx = i2c_wait_rx();

        // Read+STOP for 2nd byte
        I2C_DATA = I2C_CMD_READ | I2C_CMD_STOP | 0x44;
        i2c_wait_rx();
        i2c_wait_idle();

        // Wait for timer IRQ if not already fired
        {
            unsigned int timeout = 500000;
            while (*P_IRQ_COUNT == 0 && timeout > 0) timeout--;
        }

        int i2c_ok = (rx == 0x63);
        int timer_ok = (*P_IRQ_COUNT >= 1);

        uart_putc('H');
        uart_putc((i2c_ok && timer_ok) ? '1' : '0');
    }

    // ---- Test 2: CRC computation while timer IRQ fires ----
    {
        *P_IRQ_COUNT = 0;
        // Start timer — will fire during CRC
        TIMER_COUNTDOWN = 50;  // 50 us — fires quickly

        // Compute CRC — ISR will interrupt mid-computation
        const unsigned char test_data[3] = {0x01, 0x02, 0x03};
        unsigned int crc = crc16_compute(test_data, 3);

        // Wait for timer IRQ
        {
            unsigned int timeout = 500000;
            while (*P_IRQ_COUNT == 0 && timeout > 0) timeout--;
        }

        int crc_ok = (crc == 0x6161);
        int timer_ok = (*P_IRQ_COUNT >= 1);

        uart_putc('H');
        uart_putc((crc_ok && timer_ok) ? '2' : '0');
    }

    // ---- Test 3: I2C + CRC (no timer, verify both correct) ----
    {
        // Disable timer interrupts for clean test
        __asm__ volatile ("csrc mstatus, %0" : : "r"(mstatus_mie));

        // Start I2C write
        I2C_DATA = I2C_CMD_START | I2C_CMD_WRITE | 0x44;
        i2c_wait_tx();
        I2C_DATA = I2C_CMD_WRITE | 0x24;
        i2c_wait_tx();
        I2C_DATA = I2C_CMD_WRITE | I2C_CMD_STOP | 0x00;
        i2c_wait_tx();

        // While I2C is completing, compute CRC
        const unsigned char test_data[3] = {0x01, 0x02, 0x03};
        unsigned int crc = crc16_compute(test_data, 3);

        // Now finish I2C: read
        i2c_wait_idle();
        I2C_DATA = I2C_CMD_START | I2C_CMD_READ | 0x44;
        int rx = i2c_wait_rx();

        I2C_DATA = I2C_CMD_READ | I2C_CMD_STOP | 0x44;
        i2c_wait_rx();
        i2c_wait_idle();

        int i2c_ok = (rx == 0x63);
        int crc_ok = (crc == 0x6161);

        uart_putc('H');
        uart_putc((i2c_ok && crc_ok) ? '3' : '0');
    }

    // Disable interrupts
    __asm__ volatile ("csrc mstatus, %0" : : "r"(mstatus_mie));

    uart_putc('D');
    uart_putc('N');

    while (1);
}
