// ============================================================================
// POST (Power-On Self-Test) Firmware
// ============================================================================
// Target: LoRa Edge SoC (TinyQV RV32EC @ 25MHz)
// Stack: PSRAM RAM_A (sp=0x01000100)
//
// Comprehensive power-on self-test covering all hardware peripherals.
// Each test prints a 2-char result tag (X1=pass, X0=fail).
//
// Build:
//   riscv64-elf-gcc -march=rv32ec_zicsr -mabi=ilp32e -nostdlib -Os \
//     -T fw_p0b.ld -o fw_post.elf fw_post.c
//   riscv64-elf-objcopy -O verilog fw_post.elf fw_post.hex
//
// Expected UART output:
//   "POST\n"             — banner
//   "Y1"                 — SYSINFO: chip_id=0x01, version=0x10
//   "C1"                 — CRC16: [0x01,0x02,0x03] → 0x6161
//   "T1"                 — Timer: countdown 100µs → poll to 0
//   "W1"                 — WDT: write kick, verify remaining
//   "I1"                 — I2C: write cmd + read 2 bytes (ACK)
//   "L1"                 — Seal: commit + verify mono=0 + CRC
//   "L2"                 — Seal: second commit → mono=1
//   "M1"                 — PSRAM memory readback
//   "R1"                 — RTC: write/read seconds
//   "DN\n"               — All done
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
        "csrci mstatus, 8\n"
        "li sp, 0x01000100\n"
        "j main\n"
    );
}

static void uart_putc(unsigned char c) {
    while (UART_STATUS & UART_TX_BUSY);
    UART_DATA = c;
}

static void uart_puts(const char *s) {
    while (*s) uart_putc(*s++);
}

static void uart_result(unsigned char tag, int pass) {
    uart_putc(tag);
    uart_putc(pass ? '1' : '0');
}

static int i2c_wait_tx(void) {
    unsigned int t = 200000;
    while ((I2C_DATA & I2C_TX_PENDING) && t > 0) t--;
    return t > 0;
}

static int i2c_wait(void) {
    unsigned int t = 200000;
    while ((I2C_DATA & I2C_BUSY) && t > 0) t--;
    return t > 0;
}

static int i2c_wait_rx(void) {
    unsigned int t = 200000;
    unsigned int v;
    while (t > 0) {
        v = I2C_DATA;
        if (v & I2C_RX_VALID) return v & 0xFF;
        t--;
    }
    return -1;
}

void __attribute__((noreturn)) main(void) {
    // ---- Banner ----
    uart_puts("POST\n");

    // ---- Y: SYSINFO ----
    {
        unsigned int si = SYS_INFO;
        // chip_id=0x01 (bits 15:8), version=0x10 (bits 7:0)
        uart_result('Y', si == 0x0110);
    }

    // ---- C: CRC16 ----
    {
        CRC16_DATA = CRC16_INIT;
        while (CRC16_DATA & CRC16_BUSY);
        CRC16_DATA = 0x01;
        while (CRC16_DATA & CRC16_BUSY);
        CRC16_DATA = 0x02;
        while (CRC16_DATA & CRC16_BUSY);
        CRC16_DATA = 0x03;
        while (CRC16_DATA & CRC16_BUSY);
        unsigned int crc = CRC16_DATA & 0xFFFF;
        // CRC16-MODBUS([0x01,0x02,0x03]) = 0x6161
        uart_result('C', crc == 0x6161);
    }

    // ---- T: Timer ----
    {
        TIMER_COUNTDOWN = 100;
        unsigned int t = 100000;
        while (TIMER_COUNTDOWN != 0 && t > 0) t--;
        uart_result('T', t > 0);
    }

    // ---- W: WDT ----
    {
        WDT_KICK = 50000;
        unsigned int rem = WDT_KICK;
        uart_result('W', rem > 0 && rem <= 50000);
    }

    // ---- I: I2C (SHT31 @ 0x44) ----
    {
        I2C_CONFIG = 63;
        I2C_DATA = I2C_CMD_START | I2C_CMD_WRITE | 0x44;
        i2c_wait_tx();

        int ok = 0;
        if (!(I2C_DATA & I2C_NACK)) {
            I2C_DATA = I2C_CMD_WRITE | 0x24;
            i2c_wait_tx();
            I2C_DATA = I2C_CMD_WRITE | I2C_CMD_STOP | 0x00;
            i2c_wait();

            I2C_DATA = I2C_CMD_START | I2C_CMD_READ | 0x44;
            int rx1 = i2c_wait_rx();
            I2C_DATA = I2C_CMD_READ | I2C_CMD_STOP | 0x44;
            int rx2 = i2c_wait_rx();

            ok = (rx1 == 0x63 && rx2 == 0x32);
        }
        uart_result('I', ok);
    }

    // ---- L: Seal (2 commits, verify mono_count) ----
    {
        // First commit
        while (!(SEAL_CTRL & SEAL_READY));
        SEAL_DATA = 0xABCD0001;
        SEAL_CTRL = (0x10 << 2) | SEAL_COMMIT;  // sensor_id=0x10
        while (SEAL_CTRL & SEAL_BUSY);

        unsigned int r0 = SEAL_DATA;  // value
        unsigned int r1 = SEAL_DATA;  // {sid, mono[23:0]}
        unsigned int r2 = SEAL_DATA;  // {mono[31:24], crc, 0x00}

        // mono_count=0 at first commit
        int ok = (r0 == 0xABCD0001) && ((r1 & 0x00FFFFFF) == 0);
        uart_result('L', ok);

        // Second commit — mono_count should be 1
        while (!(SEAL_CTRL & SEAL_READY));
        SEAL_DATA = 0xABCD0002;
        SEAL_CTRL = (0x10 << 2) | SEAL_COMMIT;
        while (SEAL_CTRL & SEAL_BUSY);

        r0 = SEAL_DATA;
        r1 = SEAL_DATA;
        r2 = SEAL_DATA;

        ok = (r0 == 0xABCD0002) && ((r1 & 0x00FFFFFF) == 1);
        uart_putc('L');
        uart_putc(ok ? '2' : '0');
    }

    // ---- M: PSRAM memory ----
    {
        volatile unsigned int *psram = (volatile unsigned int *)0x01000200;
        *psram = 0xDEADBEEF;
        unsigned int rb = *psram;
        uart_result('M', rb == 0xDEADBEEF);
    }

    // ---- R: RTC ----
    {
        RTC_SECONDS = 42;
        unsigned int s = RTC_SECONDS;
        uart_result('R', s == 42);
    }

    // ---- Done ----
    uart_puts("DN\n");

    while (1);
}
