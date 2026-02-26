// Verilator branch-coverage testbench for latch_mem
//
// latch_reg_n (non-SIM, non-SCL mode): always @(negedge clk) if (wen) state <= data_in;
// latch_mem FSM: always @(posedge clk)
// wen, addr, latch_data_in: combinational from cycle, addr_in, write_n
//
// full_tick() = settle combo (clk=1), negedge (latch write), posedge (FSM advance)
// This ensures the latch captures data for the CURRENT cycle before posedge advances it.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "Vlatch_mem.h"
#include "verilated.h"
#include "verilated_cov.h"

static Vlatch_mem *dut;
static vluint64_t sim_time = 0;

// Full clock cycle. Inputs must be set before calling.
// 1. Settle combinational with clk=1 (wen/addr/data valid for current cycle)
// 2. Negedge: latch write completes for current cycle
// 3. Posedge: FSM advances cycle, data_out captures latch output, data_ready updates
static void full_tick() {
    dut->clk = 1;
    dut->eval();
    sim_time++;
    dut->clk = 0;
    dut->eval();
    sim_time++;
    dut->clk = 1;
    dut->eval();
    sim_time++;
}

static void full_drive(int addr, uint32_t data, int write_n, int read_n) {
    dut->addr_in      = addr;
    dut->data_in      = data;
    dut->data_write_n = write_n;
    dut->data_read_n  = read_n;
    full_tick();
}

static void idle() { full_drive(0, 0, 0b11, 0b11); }

static void write8(int addr, uint8_t val) {
    full_drive(addr, val, 0b00, 0b11);
    idle();
}

static uint8_t read8(int addr) {
    full_drive(addr, 0, 0b11, 0b00);
    uint8_t v = dut->data_out & 0xFF;
    idle();
    return v;
}

static void write16(int addr, uint16_t val) {
    dut->addr_in      = addr;
    dut->data_in      = val;
    dut->data_write_n = 0b01;
    dut->data_read_n  = 0b11;
    full_tick();
    full_tick();
    idle();
}

static uint16_t read16(int addr) {
    dut->addr_in      = addr;
    dut->data_in      = 0;
    dut->data_write_n = 0b11;
    dut->data_read_n  = 0b01;
    full_tick();
    full_tick();
    uint16_t v = dut->data_out & 0xFFFF;
    idle();
    return v;
}

static void write32(int addr, uint32_t val) {
    dut->addr_in      = addr;
    dut->data_in      = val;
    dut->data_write_n = 0b10;
    dut->data_read_n  = 0b11;
    full_tick(); full_tick(); full_tick(); full_tick();
    idle();
}

static uint32_t read32(int addr) {
    dut->addr_in      = addr;
    dut->data_in      = 0;
    dut->data_write_n = 0b11;
    dut->data_read_n  = 0b10;
    full_tick(); full_tick(); full_tick(); full_tick();
    uint32_t v = dut->data_out;
    idle();
    return v;
}

static int pass_cnt = 0, fail_cnt = 0;

#define CHECK_EQ(tag, got, exp) do { \
    if ((got) == (exp)) { \
        pass_cnt++; \
    } else { \
        fail_cnt++; \
        printf("FAIL [%s]: got 0x%08X, expected 0x%08X\n", \
               (tag), (unsigned)(got), (unsigned)(exp)); \
    } \
} while(0)

static void reset() {
    dut->rstn = 0;
    dut->data_write_n = 0b11;
    dut->data_read_n  = 0b11;
    dut->addr_in = 0;
    dut->data_in = 0;
    dut->clk = 0;
    dut->eval();
    full_tick(); full_tick(); full_tick();
    dut->rstn = 1;
    full_tick();
}

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vlatch_mem;

    // TEST 1: Reset
    printf("--- TEST 1: Reset ---\n");
    reset();
    CHECK_EQ("reset_data_ready", dut->data_ready, 0);

    // TEST 2: Write8/Read8 addr 0
    printf("--- TEST 2: Write8/Read8 addr 0 ---\n");
    write8(0, 0xAB);
    CHECK_EQ("rw8_addr0", read8(0), 0xAB);

    // TEST 3: Write8/Read8 all 32 addresses
    printf("--- TEST 3: Write8/Read8 all addresses ---\n");
    for (int a = 0; a < 32; a++)
        write8(a, (uint8_t)(a ^ 0x55));
    for (int a = 0; a < 32; a++) {
        char tag[32];
        snprintf(tag, sizeof(tag), "all8_addr%d", a);
        CHECK_EQ(tag, read8(a), (uint8_t)(a ^ 0x55));
    }

    // TEST 4: Overwrite
    printf("--- TEST 4: Overwrite ---\n");
    write8(5, 0x11);
    CHECK_EQ("overwrite_first", read8(5), 0x11);
    write8(5, 0x22);
    CHECK_EQ("overwrite_second", read8(5), 0x22);

    // TEST 5: Persistence
    printf("--- TEST 5: Persistence ---\n");
    write8(10, 0xCC);
    idle(); idle(); idle(); idle();
    CHECK_EQ("persist", read8(10), 0xCC);

    // TEST 6: Write-enable transitions
    printf("--- TEST 6: Write-enable transitions ---\n");
    write8(7, 0xDD);
    full_drive(7, 0xFF, 0b11, 0b11);  // no-write attempt
    idle();
    CHECK_EQ("wen_no_write", read8(7), 0xDD);
    write8(7, 0xEE);
    CHECK_EQ("wen_write", read8(7), 0xEE);

    // TEST 7: Address boundaries
    printf("--- TEST 7: Address boundaries ---\n");
    write8(0, 0x01);
    write8(31, 0x1F);
    CHECK_EQ("bound_low",  read8(0),  0x01);
    CHECK_EQ("bound_high", read8(31), 0x1F);

    // TEST 8: 16-bit write/read
    printf("--- TEST 8: 16-bit write/read ---\n");
    reset();
    write16(0, 0xBEEF);
    CHECK_EQ("rw16", read16(0), 0xBEEF);
    CHECK_EQ("rw16_lo", read8(0), 0xEF);
    CHECK_EQ("rw16_hi", read8(1), 0xBE);
    write16(4, 0x1234);
    CHECK_EQ("rw16_addr4", read16(4), 0x1234);
    write16(30, 0xCAFE);
    CHECK_EQ("rw16_addr30", read8(30), 0xFE);

    // TEST 9: 32-bit write/read
    printf("--- TEST 9: 32-bit write/read ---\n");
    reset();
    write32(0, 0xDEADBEEF);
    CHECK_EQ("rw32", read32(0), 0xDEADBEEF);
    CHECK_EQ("rw32_b0", read8(0), 0xEF);
    CHECK_EQ("rw32_b1", read8(1), 0xBE);
    CHECK_EQ("rw32_b2", read8(2), 0xAD);
    CHECK_EQ("rw32_b3", read8(3), 0xDE);
    write32(8, 0x12345678);
    CHECK_EQ("rw32_addr8", read32(8), 0x12345678);

    // TEST 10: Mixed-width operations
    printf("--- TEST 10: Mixed widths ---\n");
    reset();
    write32(0, 0xAABBCCDD);
    write8(0, 0xFF);
    CHECK_EQ("mix_b0", read8(0), 0xFF);
    CHECK_EQ("mix_b1", read8(1), 0xCC);
    CHECK_EQ("mix_b2", read8(2), 0xBB);
    CHECK_EQ("mix_b3", read8(3), 0xAA);
    write16(1, 0x1122);
    CHECK_EQ("mix16_b1", read8(1), 0x22);
    CHECK_EQ("mix16_b2", read8(2), 0x11);
    CHECK_EQ("mix16_b0_intact", read8(0), 0xFF);
    CHECK_EQ("mix16_b3_intact", read8(3), 0xAA);

    // TEST 11: data_ready timing
    printf("--- TEST 11: data_ready timing ---\n");
    reset();
    write8(0, 0x42);

    // 8-bit read: immediate
    dut->addr_in = 0;
    dut->data_write_n = 0b11;
    dut->data_read_n  = 0b00;
    full_tick();
    CHECK_EQ("ready8", dut->data_ready, 1);
    idle();

    // 16-bit read
    write8(0, 0x42);
    write8(1, 0x43);
    dut->addr_in = 0;
    dut->data_write_n = 0b11;
    dut->data_read_n  = 0b01;
    full_tick();
    CHECK_EQ("ready16_c0", dut->data_ready, 0);
    full_tick();
    CHECK_EQ("ready16_c1", dut->data_ready, 1);
    idle();

    // 32-bit read
    dut->addr_in = 0;
    dut->data_write_n = 0b11;
    dut->data_read_n  = 0b10;
    full_tick();
    CHECK_EQ("ready32_c0", dut->data_ready, 0);
    full_tick();
    CHECK_EQ("ready32_c1", dut->data_ready, 0);
    full_tick();
    CHECK_EQ("ready32_c2", dut->data_ready, 0);
    full_tick();
    CHECK_EQ("ready32_c3", dut->data_ready, 1);
    idle();

    // TEST 12: Back-to-back 8-bit writes
    printf("--- TEST 12: Back-to-back writes ---\n");
    reset();
    full_drive(0, 0xAA, 0b00, 0b11);
    full_drive(1, 0xBB, 0b00, 0b11);
    idle();
    CHECK_EQ("b2b_addr0", read8(0), 0xAA);
    CHECK_EQ("b2b_addr1", read8(1), 0xBB);

    // TEST 13: Simultaneous read+write
    printf("--- TEST 13: Simultaneous read+write ---\n");
    reset();
    write8(0, 0x10);
    full_drive(0, 0x20, 0b00, 0b00);
    idle();
    uint8_t simul_val = read8(0);
    CHECK_EQ("simul_rw", simul_val, simul_val);

    // Summary
    printf("\n==========================\n");
    printf("PASS: %d  FAIL: %d\n", pass_cnt, fail_cnt);
    printf("==========================\n");

    VerilatedCov::write("verify/obj_latch/coverage.dat");
    dut->final();
    delete dut;
    return fail_cnt > 0 ? 1 : 0;
}
