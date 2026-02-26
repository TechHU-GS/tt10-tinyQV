#include "Vcov_crc16_wrap.h"
#include "verilated.h"
#include "verilated_cov.h"
#include <cstdio>
#include <cstdlib>
#include <cstdint>

static Vcov_crc16_wrap *dut;
static vluint64_t sim_time = 0;

static void tick() {
    dut->clk = 0;
    dut->eval();
    sim_time++;
    dut->clk = 1;
    dut->eval();
    sim_time++;
}

static void reset() {
    dut->rst_n   = 0;
    dut->wr_en   = 0;
    dut->data_in = 0;
    for (int i = 0; i < 4; i++) tick();
    dut->rst_n = 1;
    tick();
}

// Write a value to the peripheral
static void peri_write(uint32_t val) {
    dut->data_in = val;
    dut->wr_en   = 1;
    tick();
    dut->wr_en   = 0;
    dut->data_in = 0;
}

// Read data_out
static uint32_t peri_read() {
    return dut->data_out;
}

// Wait for busy to clear, return cycles waited
static int wait_not_busy(int max_cycles = 100) {
    int cycles = 0;
    while ((peri_read() >> 16) & 1) {
        tick();
        cycles++;
        if (cycles > max_cycles) {
            printf("TIMEOUT: busy stuck high after %d cycles\n", max_cycles);
            return -1;
        }
    }
    return cycles;
}

// Init the CRC engine via peripheral (bit[8] = 1)
static void crc_init() {
    peri_write(0x100);
    tick();  // let init propagate
}

// Feed one byte, wait for completion
static void feed_byte(uint8_t b) {
    peri_write((uint32_t)b);
    wait_not_busy();
}

// Get CRC result (lower 16 bits)
static uint16_t get_crc() {
    return (uint16_t)(peri_read() & 0xFFFF);
}

// Check busy flag
static bool is_busy() {
    return (peri_read() >> 16) & 1;
}

static int test_count = 0;
static int pass_count = 0;
static int fail_count = 0;

static void check(const char *name, uint16_t got, uint16_t expected) {
    test_count++;
    if (got == expected) {
        pass_count++;
        printf("  PASS: %-40s got=0x%04X\n", name, got);
    } else {
        fail_count++;
        printf("  FAIL: %-40s got=0x%04X expected=0x%04X\n", name, got, expected);
    }
}

static void check_bool(const char *name, bool got, bool expected) {
    test_count++;
    if (got == expected) {
        pass_count++;
        printf("  PASS: %-40s got=%d\n", name, got);
    } else {
        fail_count++;
        printf("  FAIL: %-40s got=%d expected=%d\n", name, got, expected);
    }
}

// ============================================================================
// Software CRC16-MODBUS reference
// ============================================================================
static uint16_t sw_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xA001;
            else         crc >>= 1;
        }
    }
    return crc;
}

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vcov_crc16_wrap;

    printf("=== CRC16 Branch Coverage Testbench ===\n\n");

    // ------------------------------------------------------------------
    // Test 1: Basic reset state
    // ------------------------------------------------------------------
    printf("[Test 1] Reset state\n");
    reset();
    check("CRC after reset = 0xFFFF", get_crc(), 0xFFFF);
    check_bool("Not busy after reset", is_busy(), false);

    // ------------------------------------------------------------------
    // Test 2: Init via peripheral
    // ------------------------------------------------------------------
    printf("[Test 2] Init via peripheral\n");
    crc_init();
    check("CRC after init = 0xFFFF", get_crc(), 0xFFFF);
    check_bool("Not busy after init", is_busy(), false);

    // ------------------------------------------------------------------
    // Test 3: Feed {0x01, 0x02, 0x03} -> expected CRC = 0x6161
    // ------------------------------------------------------------------
    printf("[Test 3] Feed 0x01,0x02,0x03\n");
    crc_init();
    feed_byte(0x01);
    feed_byte(0x02);
    feed_byte(0x03);
    {
        uint8_t d[] = {0x01, 0x02, 0x03};
        uint16_t exp = sw_crc16(d, 3);
        check("CRC of {01,02,03}", get_crc(), exp);
    }

    // ------------------------------------------------------------------
    // Test 4: Feed longer sequence
    // ------------------------------------------------------------------
    printf("[Test 4] Feed 'Hello' (5 bytes)\n");
    crc_init();
    {
        uint8_t d[] = {'H', 'e', 'l', 'l', 'o'};
        for (int i = 0; i < 5; i++) feed_byte(d[i]);
        uint16_t exp = sw_crc16(d, 5);
        check("CRC of 'Hello'", get_crc(), exp);
    }

    // ------------------------------------------------------------------
    // Test 5: Accumulated CRC (no init between sequences)
    // ------------------------------------------------------------------
    printf("[Test 5] Accumulated CRC (no re-init)\n");
    crc_init();
    feed_byte(0xAA);
    feed_byte(0x55);
    uint16_t crc_after_first = get_crc();
    // continue feeding without init
    feed_byte(0xFF);
    feed_byte(0x00);
    {
        uint8_t d[] = {0xAA, 0x55, 0xFF, 0x00};
        uint16_t exp = sw_crc16(d, 4);
        check("Accumulated CRC {AA,55,FF,00}", get_crc(), exp);
    }
    // Verify first partial was different
    check_bool("Partial CRC differs from final", crc_after_first != get_crc(), true);

    // ------------------------------------------------------------------
    // Test 6: Busy flag goes high immediately after data write
    // ------------------------------------------------------------------
    printf("[Test 6] Busy timing\n");
    crc_init();
    // Write a byte and check busy on next read
    dut->data_in = 0x42;
    dut->wr_en   = 1;
    tick();
    dut->wr_en   = 0;
    dut->data_in = 0;
    // busy should be high now (bit_cnt loaded to 8 on same cycle)
    check_bool("Busy high immediately after feed", is_busy(), true);
    wait_not_busy();
    check_bool("Busy low after processing", is_busy(), false);

    // ------------------------------------------------------------------
    // Test 7: Feed while busy (should be ignored)
    // ------------------------------------------------------------------
    printf("[Test 7] Feed while busy (ignored)\n");
    crc_init();
    feed_byte(0x01);
    uint16_t crc1;
    {
        uint8_t d[] = {0x01};
        crc1 = sw_crc16(d, 1);
    }
    check("CRC after single 0x01", get_crc(), crc1);

    // Now feed 0x01 again, and while busy, try to feed 0x99
    dut->data_in = 0x01;
    dut->wr_en   = 1;
    tick();
    dut->wr_en = 0;
    // Engine is busy now. Try feeding 0x99 while busy.
    dut->data_in = 0x99;
    dut->wr_en   = 1;
    tick();
    dut->wr_en   = 0;
    dut->data_in = 0;
    wait_not_busy();
    // CRC should reflect only 0x01,0x01 (the 0x99 should be ignored)
    {
        uint8_t d[] = {0x01, 0x01};
        uint16_t exp = sw_crc16(d, 2);
        check("Feed-while-busy ignored", get_crc(), exp);
    }

    // ------------------------------------------------------------------
    // Test 8: Init while busy
    // ------------------------------------------------------------------
    printf("[Test 8] Init while busy\n");
    crc_init();
    // Feed a byte to make engine busy
    dut->data_in = 0xDE;
    dut->wr_en   = 1;
    tick();
    dut->wr_en   = 0;
    check_bool("Busy after feed 0xDE", is_busy(), true);
    // Now init while still busy
    crc_init();
    check("CRC reset by init-while-busy", get_crc(), 0xFFFF);
    check_bool("Not busy after init-while-busy", is_busy(), false);

    // ------------------------------------------------------------------
    // Test 9: CRC self-check (feed data + CRC bytes -> result 0x0000)
    // ------------------------------------------------------------------
    printf("[Test 9] CRC self-check (append CRC, verify 0x0000)\n");
    crc_init();
    {
        uint8_t d[] = {0x01, 0x02, 0x03};
        for (int i = 0; i < 3; i++) feed_byte(d[i]);
        uint16_t crc = get_crc();
        // Feed CRC bytes (little-endian) for self-check
        feed_byte(crc & 0xFF);
        feed_byte((crc >> 8) & 0xFF);
        check("Self-check = 0x0000", get_crc(), 0x0000);
    }

    // ------------------------------------------------------------------
    // Test 10: All-zero data
    // ------------------------------------------------------------------
    printf("[Test 10] All-zero data\n");
    crc_init();
    {
        uint8_t d[] = {0x00, 0x00, 0x00, 0x00};
        for (int i = 0; i < 4; i++) feed_byte(d[i]);
        uint16_t exp = sw_crc16(d, 4);
        check("CRC of {00,00,00,00}", get_crc(), exp);
    }

    // ------------------------------------------------------------------
    // Test 11: All-FF data
    // ------------------------------------------------------------------
    printf("[Test 11] All-FF data\n");
    crc_init();
    {
        uint8_t d[] = {0xFF, 0xFF, 0xFF, 0xFF};
        for (int i = 0; i < 4; i++) feed_byte(d[i]);
        uint16_t exp = sw_crc16(d, 4);
        check("CRC of {FF,FF,FF,FF}", get_crc(), exp);
    }

    // ------------------------------------------------------------------
    // Test 12: Single byte
    // ------------------------------------------------------------------
    printf("[Test 12] Single byte\n");
    crc_init();
    feed_byte(0x42);
    {
        uint8_t d[] = {0x42};
        uint16_t exp = sw_crc16(d, 1);
        check("CRC of single 0x42", get_crc(), exp);
    }

    // ------------------------------------------------------------------
    // Test 13: Multiple init/feed cycles
    // ------------------------------------------------------------------
    printf("[Test 13] Multiple init/feed cycles\n");
    for (int round = 0; round < 4; round++) {
        crc_init();
        uint8_t d = (uint8_t)(0x10 + round);
        feed_byte(d);
        uint16_t exp = sw_crc16(&d, 1);
        char msg[64];
        snprintf(msg, sizeof(msg), "Round %d: CRC of {0x%02X}", round, d);
        check(msg, get_crc(), exp);
    }

    // ------------------------------------------------------------------
    // Test 14: Data with bit[8]=0 explicitly (normal data path)
    // ------------------------------------------------------------------
    printf("[Test 14] Explicit data write (bit[8]=0)\n");
    crc_init();
    peri_write(0x0AB);  // bit[8]=0, data=0xAB
    wait_not_busy();
    {
        uint8_t d[] = {0xAB};
        uint16_t exp = sw_crc16(d, 1);
        check("CRC of data_in=0x0AB", get_crc(), exp);
    }

    // ------------------------------------------------------------------
    // Test 15: Write with upper bits set (should be ignored by peripheral)
    // ------------------------------------------------------------------
    printf("[Test 15] Write with upper bits set\n");
    crc_init();
    peri_write(0xFFFF0055);  // upper bits set, but peripheral uses [8:0] only
    wait_not_busy();
    {
        uint8_t d[] = {0x55};
        uint16_t exp = sw_crc16(d, 1);
        check("Upper bits ignored, data=0x55", get_crc(), exp);
    }

    // ------------------------------------------------------------------
    // Test 16: Rapid back-to-back feeds (wait between each)
    // ------------------------------------------------------------------
    printf("[Test 16] Rapid back-to-back 16-byte sequence\n");
    crc_init();
    {
        uint8_t d[16];
        for (int i = 0; i < 16; i++) {
            d[i] = (uint8_t)i;
            feed_byte(d[i]);
        }
        uint16_t exp = sw_crc16(d, 16);
        check("CRC of {00..0F}", get_crc(), exp);
    }

    // ------------------------------------------------------------------
    // Test 17: Reset mid-computation
    // ------------------------------------------------------------------
    printf("[Test 17] Hardware reset mid-computation\n");
    crc_init();
    feed_byte(0xAA);
    // Now do a hard reset
    dut->rst_n = 0;
    for (int i = 0; i < 4; i++) tick();
    dut->rst_n = 1;
    tick();
    check("CRC after hw reset = 0xFFFF", get_crc(), 0xFFFF);
    check_bool("Not busy after hw reset", is_busy(), false);

    // ------------------------------------------------------------------
    // Test 18: Reset while busy
    // ------------------------------------------------------------------
    printf("[Test 18] Hardware reset while busy\n");
    crc_init();
    dut->data_in = 0x77;
    dut->wr_en   = 1;
    tick();
    dut->wr_en   = 0;
    check_bool("Busy before hw reset", is_busy(), true);
    dut->rst_n = 0;
    for (int i = 0; i < 4; i++) tick();
    dut->rst_n = 1;
    tick();
    check("CRC after reset-while-busy = 0xFFFF", get_crc(), 0xFFFF);
    check_bool("Not busy after reset-while-busy", is_busy(), false);

    // ------------------------------------------------------------------
    // Test 19: crc_reg[0] branch coverage - ensure both 0 and 1 paths
    // Feed specific bytes that produce both LSB=0 and LSB=1 in crc_reg
    // ------------------------------------------------------------------
    printf("[Test 19] Exercise crc_reg[0] both paths\n");
    crc_init();
    // 0xFF will XOR with 0xFFFF -> 0xFF00, LSB=0 on first bit
    feed_byte(0xFF);
    {
        uint8_t d[] = {0xFF};
        check("CRC of {0xFF}", get_crc(), sw_crc16(d, 1));
    }
    // 0x01 after init -> XOR with 0xFFFF -> 0xFFFE, LSB=0
    // 0x00 after init -> XOR with 0xFFFF -> 0xFFFF, LSB=1
    crc_init();
    feed_byte(0x00);
    {
        uint8_t d[] = {0x00};
        check("CRC of {0x00}", get_crc(), sw_crc16(d, 1));
    }

    // ------------------------------------------------------------------
    // Test 20: Double init (init twice in a row)
    // ------------------------------------------------------------------
    printf("[Test 20] Double init\n");
    crc_init();
    feed_byte(0x42);
    crc_init();
    crc_init();  // second init
    check("CRC after double init = 0xFFFF", get_crc(), 0xFFFF);

    // ------------------------------------------------------------------
    // Summary
    // ------------------------------------------------------------------
    printf("\n=== Summary: %d/%d PASS, %d FAIL ===\n",
           pass_count, test_count, fail_count);

    // Write coverage data
    VerilatedCov::write("verify/obj_crc16/coverage.dat");

    dut->final();
    delete dut;

    return fail_count > 0 ? 1 : 0;
}
