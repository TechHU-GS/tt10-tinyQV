// ============================================================================
// Test D: I2C Back-to-Back — Consecutive Multi-Byte Read
// ============================================================================
// Target: LoRa Edge SoC (TinyQV RV32EC @ 25MHz)
// Tests: Back-to-back I2C reads (6 bytes), rx_fire timing, rx_tready fix.
//
// Build:
//   riscv64-elf-gcc -march=rv32ec_zicsr -mabi=ilp32e -nostdlib -Os \
//     -T fw_irq_timer.ld -o fw_i2c_stress.elf fw_i2c_stress.c
//   riscv64-elf-objcopy -O verilog fw_i2c_stress.elf fw_i2c_stress.hex
//
// Strategy:
//   1. Configure I2C (prescaler=63 → ~200kHz)
//   2. SHT31 measurement command: START + W(0x44) + 0x24 + 0x00 + STOP
//   3. Read 6 bytes: START + R(0x44) + ACK×5 + NACK + STOP
//   4. Verify all 6 bytes match expected pattern
//
// Expected UART output: "D1D2DN" (6 chars)
//   D1 = I2C write command success
//   D2 = All 6 read bytes match
//   DN = Done
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

// Expected SHT31 read data from i2c_slave_model
static const unsigned char expected[6] = {0x63, 0x32, 0xA1, 0x8C, 0xA4, 0xDB};

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
// I2C helpers
// ============================================================================
static int i2c_wait_tx(void) {
    unsigned int t = 200000;
    while ((I2C_DATA & I2C_TX_PENDING) && t > 0) t--;
    return t > 0;
}

// Wait for RX data (returns data byte or -1 on timeout)
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
    // Configure I2C prescaler (200kHz @ 25MHz: prescaler=63)
    I2C_CONFIG = 63;

    // ---- Test 1: I2C write command (SHT31 measurement trigger) ----

    // START + write address (7-bit addr 0x44, I2C master adds R/W bit)
    I2C_DATA = I2C_CMD_START | I2C_CMD_WRITE | 0x44;
    if (!i2c_wait_tx()) { uart_putc('D'); uart_putc('T'); while(1); }

    // Check for NACK
    if (I2C_DATA & I2C_NACK) { uart_putc('D'); uart_putc('N'); while(1); }

    // Command byte 1
    I2C_DATA = I2C_CMD_WRITE | 0x24;
    if (!i2c_wait_tx()) { uart_putc('D'); uart_putc('T'); while(1); }

    // Command byte 2 + STOP
    I2C_DATA = I2C_CMD_WRITE | I2C_CMD_STOP | 0x00;
    if (!i2c_wait_tx()) { uart_putc('D'); uart_putc('T'); while(1); }

    uart_putc('D');
    uart_putc('1');  // Write command success

    // ---- Test 2: I2C read 6 bytes back-to-back ----

    // START + read address (7-bit addr 0x44, I2C master adds R/W bit)
    I2C_DATA = I2C_CMD_START | I2C_CMD_READ | 0x44;

    // Read 6 bytes: first 5 with ACK, last with NACK+STOP
    int all_match = 1;
    int i;
    for (i = 0; i < 6; i++) {
        int rx = i2c_wait_rx();
        if (rx < 0) {
            // Timeout
            uart_putc('D');
            uart_putc('X');
            while (1);
        }

        if ((unsigned char)rx != expected[i]) {
            all_match = 0;
        }

        // Queue next read command
        if (i < 4) {
            // More bytes to come: ACK + read
            I2C_DATA = I2C_CMD_READ | 0x44;
        } else if (i == 4) {
            // Last byte: NACK + STOP + read
            I2C_DATA = I2C_CMD_READ | I2C_CMD_STOP | 0x44;
        }
        // i == 5: no more commands
    }

    uart_putc('D');
    uart_putc(all_match ? '2' : '0');

    // ---- Done ----
    uart_putc('D');
    uart_putc('N');

    while (1);
}
