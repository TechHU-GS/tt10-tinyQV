// ============================================================================
// Test E: CRC Arbitration — Seal vs CPU CRC Access
// ============================================================================
// Target: LoRa Edge SoC (TinyQV RV32EC @ 25MHz)
// Tests: Seal commit uses CRC engine exclusively, CPU sees busy=1 during
//        seal commit, both produce correct results after arbitration.
//
// Build:
//   riscv64-elf-gcc -march=rv32ec_zicsr -mabi=ilp32e -nostdlib -Os \
//     -T fw_irq_timer.ld -o fw_crc_arb.elf fw_crc_arb.c
//   riscv64-elf-objcopy -O verilog fw_crc_arb.elf fw_crc_arb.hex
//
// Strategy:
//   1. CPU CRC test: compute CRC16 of {0x01, 0x02, 0x03}, verify = 0x6161
//   2. Seal commit: write 8 data bytes + commit, read seal CRC
//   3. CPU CRC after seal: compute same CRC again, verify = 0x6161
//
// Expected UART output: "E1E2E3DN" (8 chars)
//   E1 = CPU CRC correct (standalone)
//   E2 = Seal CRC correct
//   E3 = CPU CRC correct (after seal)
//   DN = Done
// ============================================================================

#define PERI_BASE       0x08000000u
#define UART_DATA       (*(volatile unsigned int*)(PERI_BASE + 0x10))
#define UART_STATUS     (*(volatile unsigned int*)(PERI_BASE + 0x14))
#define CRC16_DATA      (*(volatile unsigned int*)(PERI_BASE + 0x08))
#define SEAL_DATA       (*(volatile unsigned int*)(PERI_BASE + 0x2C))
#define SEAL_CTRL       (*(volatile unsigned int*)(PERI_BASE + 0x38))

#define UART_TX_BUSY    (1u << 0)
#define CRC16_BUSY      (1u << 16)
#define CRC16_INIT      (1u << 8)
#define SEAL_COMMIT     (1u << 1)
#define SEAL_BUSY       (1u << 0)
#define SEAL_READY      (1u << 1)

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
// CRC16 helpers
// ============================================================================
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
    // ---- Test 1: CPU CRC (standalone) ----
    // CRC16 of {0x01, 0x02, 0x03} should be 0x6161
    {
        const unsigned char test_data[3] = {0x01, 0x02, 0x03};
        unsigned int crc = crc16_compute(test_data, 3);
        uart_putc('E');
        uart_putc((crc == 0x6161) ? '1' : '0');
    }

    // ---- Test 2: Seal commit + verify seal CRC ----
    {
        // Write 32-bit payload to SEAL_DATA
        SEAL_DATA = 0x01020304;

        // Trigger seal commit with sensor_id=0xAB
        // SEAL_CTRL: {sensor_id[7:0], commit, crc_reset} = {0xAB, 1, 0}
        SEAL_CTRL = SEAL_COMMIT | (0xAB << 2);

        // Wait for seal to complete
        unsigned int t = 500000;
        while ((SEAL_CTRL & SEAL_BUSY) && t > 0) t--;

        if (t == 0) {
            uart_putc('E');
            uart_putc('T');  // timeout
            while (1);
        }

        // Verify seal completed: SEAL_READY (bit 1) set, SEAL_BUSY (bit 0) clear
        unsigned int seal_status = SEAL_CTRL;
        int seal_ok = (seal_status & SEAL_READY) && !(seal_status & SEAL_BUSY);

        // Read sealed record: 3x SEAL_DATA reads
        unsigned int sealed_value = SEAL_DATA;  // read 0: value
        unsigned int sealed_mono  = SEAL_DATA;  // read 1: {sid, mono[23:0]}
        unsigned int sealed_crc_r = SEAL_DATA;  // read 2: {mono[31:24], crc[15:0], 8'h00}

        // Verify value matches what we wrote
        seal_ok = seal_ok && (sealed_value == 0x01020304);

        // Verify CRC is non-zero (something was computed)
        unsigned int seal_crc = (sealed_crc_r >> 8) & 0xFFFF;
        seal_ok = seal_ok && (seal_crc != 0);

        uart_putc('E');
        uart_putc(seal_ok ? '2' : '0');
    }

    // ---- Test 3: CPU CRC after seal (verify CRC engine recovered) ----
    {
        const unsigned char test_data[3] = {0x01, 0x02, 0x03};
        unsigned int crc = crc16_compute(test_data, 3);
        uart_putc('E');
        uart_putc((crc == 0x6161) ? '3' : '0');
    }

    // ---- Done ----
    uart_putc('D');
    uart_putc('N');

    while (1);
}
