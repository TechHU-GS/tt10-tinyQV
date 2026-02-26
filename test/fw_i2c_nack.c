// ============================================================================
// Test G: I2C NACK — Error Detection and Recovery
// ============================================================================
// Target: LoRa Edge SoC (TinyQV RV32EC @ 25MHz)
// Tests: I2C NACK detection on invalid address, then successful transaction.
//
// Build:
//   riscv64-elf-gcc -march=rv32ec_zicsr -mabi=ilp32e -nostdlib -Os \
//     -T fw_irq_timer.ld -o fw_i2c_nack.elf fw_i2c_nack.c
//   riscv64-elf-objcopy -O verilog fw_i2c_nack.elf fw_i2c_nack.hex
//
// Strategy:
//   G1: Send I2C write to addr 0x7F (slave only responds to 0x44) → NACK
//   G2: Send I2C write to addr 0x44 → ACK + successful read
//
// Expected UART output: "G1G2DN" (6 chars)
// ============================================================================

#define PERI_BASE       0x08000000u
#define UART_DATA       (*(volatile unsigned int*)(PERI_BASE + 0x10))
#define UART_STATUS     (*(volatile unsigned int*)(PERI_BASE + 0x14))
#define I2C_DATA        (*(volatile unsigned int*)(PERI_BASE + 0x18))
#define I2C_CONFIG      (*(volatile unsigned int*)(PERI_BASE + 0x1C))

#define UART_TX_BUSY    (1u << 0)
#define I2C_CMD_START   (1u << 8)
#define I2C_CMD_READ    (1u << 9)
#define I2C_CMD_WRITE   (1u << 10)
#define I2C_CMD_STOP    (1u << 12)
#define I2C_BUSY        (1u << 9)
#define I2C_NACK        (1u << 8)
#define I2C_RX_VALID    (1u << 10)
#define I2C_TX_PENDING  (1u << 11)

// ============================================================================
// Vector table
// ============================================================================
void __attribute__((naked, section(".text._vectors"))) _vectors(void) {
    __asm__ volatile (
        ".option push\n"
        ".option norvc\n"
        "j _reset_handler\n"
        "j _trap_handler\n"
        "j _trap_handler\n"
        ".option pop\n"
    );
}

void __attribute__((naked)) _trap_handler(void) {
    __asm__ volatile ("j _trap_handler\n");
}

// ============================================================================
// UART + I2C helpers
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
    I2C_CONFIG = 63;

    // ---- Test 1: NACK on invalid address 0x7F ----
    {
        I2C_DATA = I2C_CMD_START | I2C_CMD_WRITE | 0x7F;
        i2c_wait_tx();

        // After START+WRITE, the Forencich master enters write_multiple mode
        // and waits in WRITE_1 for TX data even after NACK. We must send a
        // data byte with STOP to let the master finish: WRITE_1→WRITE_2→WRITE_3→STOP→IDLE
        I2C_DATA = I2C_CMD_WRITE | I2C_CMD_STOP | 0x00;
        i2c_wait_tx();
        i2c_wait_idle();

        // Check NACK bit — read BEFORE sending new command (which clears latch)
        unsigned int status = I2C_DATA;
        int got_nack = (status & I2C_NACK) != 0;

        uart_putc('G');
        uart_putc(got_nack ? '1' : '0');
    }

    // ---- Test 2: Successful transaction after NACK ----
    {
        // Write to valid address 0x44 — the new command clears missed_ack_latch
        I2C_DATA = I2C_CMD_START | I2C_CMD_WRITE | 0x44;
        i2c_wait_tx();

        // Send command bytes (SHT31 measurement: 0x24, 0x00)
        I2C_DATA = I2C_CMD_WRITE | 0x24;
        i2c_wait_tx();
        I2C_DATA = I2C_CMD_WRITE | I2C_CMD_STOP | 0x00;
        i2c_wait_tx();
        i2c_wait_idle();  // wait for write transaction to complete

        // Read phase: START+READ (first byte)
        I2C_DATA = I2C_CMD_START | I2C_CMD_READ | 0x44;
        int rx = i2c_wait_rx();

        // STOP: READ+STOP (second byte, NACKed to end)
        I2C_DATA = I2C_CMD_READ | I2C_CMD_STOP | 0x44;
        i2c_wait_rx();  // drain second byte

        i2c_wait_idle();

        // Check that NACK was cleared (no NACK on valid address)
        unsigned int status = I2C_DATA;
        int no_nack = !(status & I2C_NACK);

        uart_putc('G');
        uart_putc((no_nack && rx == 0x63) ? '2' : '0');
    }

    uart_putc('D');
    uart_putc('N');

    while (1);
}
