// ============================================================================
// P0-A Flash-only Boot Firmware
// ============================================================================
// Target: LoRa Edge SoC (TinyQV RV32EC @ 25MHz)
// Constraints:
//   - Stack: latch_mem 32B (sp=0x04000020, full 32B usable)
//   - No PSRAM access (address 0x1000000+ forbidden)
//   - No interrupts (csrci mstatus,8 at entry)
//   - No standard library (-nostdlib)
//   - All polling, no function call nesting
//
// Build:
//   riscv64-elf-gcc -march=rv32ec_zicsr -mabi=ilp32e -nostdlib -Os \
//     -Ttext=0x00000000 -Wl,--no-dynamic-linker -o fw_p0a.elf fw_p0a.c
//   riscv64-elf-objcopy -O verilog fw_p0a.elf fw_p0a.hex
//
// Test sequence:
//   1. UART prints "OK\n"
//   2. CRC16 init → feed 3 bytes [0x01, 0x02, 0x03] → read CRC
//   3. Verify CRC == 0x6161 → UART "C1" (pass) or "C0" (fail)
//   4. Read SYS_INFO → verify CHIP_ID=0x01, VERSION=0x10
//   5. UART "SI" (pass) or "S0" (fail)
//   6. Timer test: write 100, poll until 0, UART "T1"/"T0"
//   7. UART "DN" (done) → infinite loop
// ============================================================================

#define PERI_BASE       0x08000000u

#define GPIO_OUT        (*(volatile unsigned int*)(PERI_BASE + 0x00))
#define GPIO_IN         (*(volatile unsigned int*)(PERI_BASE + 0x04))
#define CRC16_DATA      (*(volatile unsigned int*)(PERI_BASE + 0x08))
#define UART_DATA       (*(volatile unsigned int*)(PERI_BASE + 0x10))
#define UART_STATUS     (*(volatile unsigned int*)(PERI_BASE + 0x14))
#define TIMER_COUNTDOWN (*(volatile unsigned int*)(PERI_BASE + 0x30))
#define SYS_INFO        (*(volatile unsigned int*)(PERI_BASE + 0x3C))

#define UART_TX_BUSY    (1u << 0)
#define CRC16_BUSY      (1u << 16)
#define CRC16_INIT      (1u << 8)

void __attribute__((naked, noreturn, section(".text._start"))) _start(void) {
    __asm__ volatile (
        "csrci mstatus, 8\n"       // Disable interrupts (MIE=0)
        "li sp, 0x04000020\n"      // Stack = latch_mem top+1 (32B, 16-byte aligned)
        "j main\n"
    );
}

static void uart_putc(unsigned char c) {
    while (UART_STATUS & UART_TX_BUSY);
    UART_DATA = c;
}

static void crc16_init(void) {
    CRC16_DATA = CRC16_INIT;
}

static void crc16_feed(unsigned char b) {
    while (CRC16_DATA & CRC16_BUSY);
    CRC16_DATA = b;
}

static unsigned int crc16_result(void) {
    while (CRC16_DATA & CRC16_BUSY);
    return CRC16_DATA & 0xFFFFu;
}

void __attribute__((noreturn)) main(void) {
    // 1. UART "OK\n"
    uart_putc('O');
    uart_putc('K');
    uart_putc('\n');

    // 2. CRC16 test: [0x01, 0x02, 0x03] → expect 0x6161
    crc16_init();
    crc16_feed(0x01);
    crc16_feed(0x02);
    crc16_feed(0x03);
    unsigned int crc = crc16_result();
    uart_putc('C');
    uart_putc((crc == 0x6161u) ? '1' : '0');

    // 3. SYS_INFO: expect {pps_count[15:0], 0x01, 0x10}
    unsigned int si = SYS_INFO;
    unsigned char chip_id = (si >> 8) & 0xFF;
    unsigned char version = si & 0xFF;
    uart_putc('S');
    uart_putc((chip_id == 0x01 && version == 0x10) ? '1' : '0');

    // 4. Timer test: write 100µs, poll until 0
    TIMER_COUNTDOWN = 100;
    unsigned int timeout = 100000;
    while (TIMER_COUNTDOWN != 0 && timeout > 0) {
        timeout--;
    }
    uart_putc('T');
    uart_putc((timeout > 0) ? '1' : '0');

    // 5. Done
    uart_putc('D');
    uart_putc('N');
    uart_putc('\n');

    while (1);
}
