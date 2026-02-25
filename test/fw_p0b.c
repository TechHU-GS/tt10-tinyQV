// ============================================================================
// P0-B Integration Firmware — PSRAM Stack + Full Peripheral Test
// ============================================================================
// Target: LoRa Edge SoC (TinyQV RV32EC @ 25MHz)
// Stack: PSRAM RAM_A (sp=0x01000100, 256B stack)
// Interrupts: disabled (polling only)
//
// Build:
//   riscv64-elf-gcc -march=rv32ec_zicsr -mabi=ilp32e -nostdlib -Os \
//     -T fw_p0b.ld -o fw_p0b.elf fw_p0b.c
//   riscv64-elf-objcopy -O verilog fw_p0b.elf fw_p0b.hex
//
// Test sequence (each prints 2-char result: X1=pass, X0=fail):
//   1. "OK\n" — UART boot confirmation
//   2. "C1"   — CRC16: [0x01,0x02,0x03] → 0x6161
//   3. "S1"   — SYS_INFO: chip_id=0x01, version=0x10
//   4. "T1"   — Timer: countdown 100µs → poll to 0
//   5. "M1"   — Memory: PSRAM write/readback
//   6. "I1"   — I2C: write SHT31 cmd + read 2 bytes
//   7. "W1"   — WDT: write kick, verify remaining
//   8. "R1"   — RTC: read seconds, verify non-decreasing
//   9. "E1"   — Seal: commit + read 3x
//  10. "DN\n" — Done
// ============================================================================

#define PERI_BASE       0x08000000u

#define GPIO_OUT        (*(volatile unsigned int*)(PERI_BASE + 0x00))
#define GPIO_IN         (*(volatile unsigned int*)(PERI_BASE + 0x04))
#define CRC16_DATA      (*(volatile unsigned int*)(PERI_BASE + 0x08))
#define UART_DATA       (*(volatile unsigned int*)(PERI_BASE + 0x10))
#define UART_STATUS     (*(volatile unsigned int*)(PERI_BASE + 0x14))
#define I2C_DATA        (*(volatile unsigned int*)(PERI_BASE + 0x18))
#define I2C_CONFIG      (*(volatile unsigned int*)(PERI_BASE + 0x1C))
#define RTC_SECONDS     (*(volatile unsigned int*)(PERI_BASE + 0x28))
#define SEAL_DATA       (*(volatile unsigned int*)(PERI_BASE + 0x2C))
#define TIMER_COUNTDOWN (*(volatile unsigned int*)(PERI_BASE + 0x30))
#define WDT_KICK        (*(volatile unsigned int*)(PERI_BASE + 0x34))
#define SEAL_CTRL       (*(volatile unsigned int*)(PERI_BASE + 0x38))
#define SYS_INFO        (*(volatile unsigned int*)(PERI_BASE + 0x3C))

#define UART_TX_BUSY    (1u << 0)
#define CRC16_BUSY      (1u << 16)
#define CRC16_INIT      (1u << 8)
#define I2C_CMD_START   (1u << 8)
#define I2C_CMD_READ    (1u << 9)
#define I2C_CMD_WRITE   (1u << 10)
#define I2C_CMD_STOP    (1u << 12)
#define I2C_BUSY        (1u << 9)
#define I2C_NACK        (1u << 8)
#define I2C_RX_VALID    (1u << 10)
#define I2C_TX_PENDING  (1u << 11)
#define SEAL_COMMIT     (1u << 1)
#define SEAL_BUSY       (1u << 0)
#define SEAL_READY      (1u << 1)

void __attribute__((naked, noreturn, section(".text._start"))) _start(void) {
    __asm__ volatile (
        "csrci mstatus, 8\n"       // Disable interrupts (MIE=0)
        "li sp, 0x01000100\n"      // Stack = PSRAM RAM_A base + 256B
        "j main\n"
    );
}

static void uart_putc(unsigned char c) {
    while (UART_STATUS & UART_TX_BUSY);
    UART_DATA = c;
}

static void uart_result(unsigned char tag, int pass) {
    uart_putc(tag);
    uart_putc(pass ? '1' : '0');
}

// Wait for I2C not busy with timeout
static int i2c_wait(void) {
    unsigned int t = 200000;
    while ((I2C_DATA & I2C_BUSY) && t > 0) t--;
    return t > 0;
}

// Wait for I2C tx_pending to clear
static int i2c_wait_tx(void) {
    unsigned int t = 200000;
    while ((I2C_DATA & I2C_TX_PENDING) && t > 0) t--;
    return t > 0;
}

// Wait for I2C RX data available (returns data byte, or -1 on timeout)
// IMPORTANT: Do NOT call i2c_wait() before this — busy-polling reads
// I2C_DATA which triggers data_rd, clearing rx_has_data prematurely.
static int i2c_wait_rx(void) {
    unsigned int t = 200000;
    unsigned int v;
    while (t > 0) {
        v = I2C_DATA;
        if (v & I2C_RX_VALID)
            return v & 0xFF;  // rx_latch still valid even if rx_has_data cleared
        t--;
    }
    return -1;
}

void __attribute__((noreturn)) main(void) {
    int ok;

    // ---- 1. Boot confirmation ----
    uart_putc('O');
    uart_putc('K');
    uart_putc('\n');

    // ---- 2. CRC16 test ----
    CRC16_DATA = CRC16_INIT;
    while (CRC16_DATA & CRC16_BUSY);
    CRC16_DATA = 0x01;
    while (CRC16_DATA & CRC16_BUSY);
    CRC16_DATA = 0x02;
    while (CRC16_DATA & CRC16_BUSY);
    CRC16_DATA = 0x03;
    while (CRC16_DATA & CRC16_BUSY);
    unsigned int crc = CRC16_DATA & 0xFFFF;
    uart_result('C', crc == 0x6161);

    // ---- 3. SYS_INFO test ----
    unsigned int si = SYS_INFO;
    uart_result('S', ((si >> 8) & 0xFF) == 0x01 && (si & 0xFF) == 0x10);

    // ---- 4. Timer test ----
    TIMER_COUNTDOWN = 100;
    {
        unsigned int t = 100000;
        while (TIMER_COUNTDOWN != 0 && t > 0) t--;
        uart_result('T', t > 0);
    }

    // ---- 5. PSRAM memory test ----
    {
        volatile unsigned int *psram = (volatile unsigned int *)0x01000200;
        *psram = 0xDEADBEEF;
        unsigned int rb = *psram;
        uart_result('M', rb == 0xDEADBEEF);
    }

    // ---- 6. I2C test (SHT31 @ 0x44) ----
    {
        // Set prescale (default 63 is fine, but write it explicitly)
        I2C_CONFIG = 63;

        // START + WRITE address 0x44 (7-bit addr, Forencich handles R/W)
        // Bridge uses write_multiple mode: i2c_busy stays HIGH throughout.
        // Poll tx_pending (bit 11) for data flow, not i2c_busy.
        I2C_DATA = I2C_CMD_START | I2C_CMD_WRITE | 0x44;
        // Wait for cmd to be accepted (tx_pending=0 means ready for data)
        i2c_wait_tx();

        if (!(I2C_DATA & I2C_NACK)) {
            // Write command byte 0x24 (SHT31 measure)
            I2C_DATA = I2C_CMD_WRITE | 0x24;
            i2c_wait_tx();

            // Write second byte 0x00 + STOP (tlast=1 ends write_multiple)
            I2C_DATA = I2C_CMD_WRITE | I2C_CMD_STOP | 0x00;
            // After STOP, wait for i2c_busy to clear
            i2c_wait();

            // Read byte 1: START + READ + addr
            // Do NOT call i2c_wait() — it reads I2C_DATA to poll busy,
            // which triggers data_rd and clears rx_has_data prematurely.
            I2C_DATA = I2C_CMD_START | I2C_CMD_READ | 0x44;
            int rx1 = i2c_wait_rx();

            // Read byte 2 + STOP
            I2C_DATA = I2C_CMD_READ | I2C_CMD_STOP | 0x44;
            int rx2 = i2c_wait_rx();

            // SHT31 model returns 0x63, 0x32
            uart_result('I', rx1 == 0x63 && rx2 == 0x32);
        } else {
            uart_result('I', 0);  // NACK or timeout
        }
    }

    // ---- 7. WDT test ----
    {
        WDT_KICK = 50000;  // Enable WDT with 50ms timeout
        unsigned int rem = WDT_KICK;  // Read remaining
        // Should be counting down (less than initial value but nonzero)
        uart_result('W', rem > 0 && rem <= 50000);
    }

    // ---- 8. RTC test ----
    {
        RTC_SECONDS = 1000;  // Set to known value
        unsigned int s1 = RTC_SECONDS;
        // Small delay
        for (volatile int i = 0; i < 100; i++);
        unsigned int s2 = RTC_SECONDS;
        // Should be same or incremented (1 second = 1M µs ticks, won't increment in ~100 loops)
        uart_result('R', s1 >= 1000 && s2 >= s1);
    }

    // ---- 9. Seal test ----
    {
        // Wait for seal ready
        while (!(SEAL_CTRL & SEAL_READY));

        // Write value
        SEAL_DATA = 0xCAFE0001;

        // Commit with sensor_id=0x42
        SEAL_CTRL = (0x42 << 2) | SEAL_COMMIT;

        // Wait for completion
        unsigned int t = 100000;
        while ((SEAL_CTRL & SEAL_BUSY) && t > 0) t--;

        if (t > 0) {
            // Read 3x
            unsigned int r0 = SEAL_DATA;  // value
            unsigned int r1 = SEAL_DATA;  // {sid, mono[23:0]}
            unsigned int r2 = SEAL_DATA;  // {mono[31:24], crc, 0x00}

            // Verify: r0 = 0xCAFE0001, r1[31:24] should be session_id, r1[23:0]=mono=0
            ok = (r0 == 0xCAFE0001) && ((r1 & 0x00FFFFFF) == 0) && ((r2 & 0xFF) == 0);
            uart_result('E', ok);
        } else {
            uart_result('E', 0);
        }
    }

    // ---- 10. Done ----
    uart_putc('D');
    uart_putc('N');
    uart_putc('\n');

    while (1);
}
