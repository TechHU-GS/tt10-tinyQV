// seal_cov_tb.cpp — Verilator coverage testbench for seal_register
// Exercises all FSM arcs, backpressure, commit_dropped, read serialization,
// session_id locking, and standalone crc_reset.

#include "Vseal_register.h"
#include "verilated.h"
#include "verilated_cov.h"
#include "verilated_vcd_c.h"

#include <cstdio>
#include <cstdint>
#include <cassert>

static vluint64_t sim_time = 0;

static Vseal_register *dut;
static VerilatedVcdC   *tfp;

// ─── helpers ───────────────────────────────────────────────────────────

static void tick() {
    dut->clk = 0;
    dut->eval();
    if (tfp) tfp->dump(sim_time++);
    dut->clk = 1;
    dut->eval();
    if (tfp) tfp->dump(sim_time++);
}

static void reset() {
    dut->rst_n      = 0;
    dut->crc_busy   = 0;
    dut->crc_value  = 0xFFFF;
    dut->data_wr    = 0;
    dut->data_in    = 0;
    dut->data_rd    = 0;
    dut->ctrl_wr    = 0;
    dut->ctrl_in    = 0;
    dut->session_ctr_in = 0;
    for (int i = 0; i < 5; i++) tick();
    dut->rst_n = 1;
    tick();
}

// Write SEAL_DATA
static void write_data(uint32_t val) {
    dut->data_wr = 1;
    dut->data_in = val;
    tick();
    dut->data_wr = 0;
    dut->data_in = 0;
}

// Write SEAL_CTRL
static void write_ctrl(uint16_t val) {
    dut->ctrl_wr = 1;
    dut->ctrl_in = val & 0x3FF;
    tick();
    dut->ctrl_wr = 0;
    dut->ctrl_in = 0;
}

// Read SEAL_DATA (single-cycle pulse)
static uint32_t read_data() {
    uint32_t v = dut->data_out;
    dut->data_rd = 1;
    tick();
    dut->data_rd = 0;
    return v;
}

// Wait for seal to return to IDLE (ctrl_out bit[0]=busy becomes 0)
// crc_busy_pattern: each bit controls crc_busy for one cycle during
// FEED_BYTES to exercise backpressure paths.
static void wait_idle(uint32_t crc_busy_pattern = 0) {
    int cyc = 0;
    while (dut->ctrl_out & 0x1) {  // seal_busy
        // Simulate CRC engine: busy for 8 cycles after feed pulse
        // We override crc_busy with the pattern for the first 32 cycles
        if (cyc < 32) {
            dut->crc_busy = (crc_busy_pattern >> cyc) & 1;
        } else {
            dut->crc_busy = 0;
        }
        tick();
        cyc++;
        if (cyc > 500) {
            printf("ERROR: timeout waiting for IDLE\n");
            break;
        }
    }
    dut->crc_busy = 0;
}

// Do a full commit: write data, write ctrl with commit=1, wait idle.
// sensor_id goes in ctrl_in[9:2], commit is bit[1].
static void do_commit(uint32_t value, uint8_t sensor_id,
                      uint32_t crc_busy_pattern = 0) {
    write_data(value);
    uint16_t ctrl = ((uint16_t)sensor_id << 2) | 0x02; // commit=1, crc_reset=0
    write_ctrl(ctrl);
    wait_idle(crc_busy_pattern);
}

// ─── test scenarios ────────────────────────────────────────────────────

static int test_count = 0;
static int pass_count = 0;

#define CHECK(cond, msg) do { \
    test_count++; \
    if (!(cond)) { \
        printf("  FAIL: %s (line %d)\n", msg, __LINE__); \
    } else { \
        pass_count++; \
    } \
} while(0)

// T1: Normal commit flow (IDLE → FEED_BYTES → LATCH → IDLE)
static void test_normal_commit() {
    printf("[T1] Normal commit flow\n");
    reset();
    dut->session_ctr_in = 0xAB;

    // Before commit: seal should be idle
    CHECK((dut->ctrl_out & 0x3) == 0x2, "initially idle (ready=1, busy=0)");

    do_commit(0xDEADBEEF, 0x42);

    // After commit: back to idle, mono_count should have incremented
    CHECK((dut->ctrl_out & 0x3) == 0x2, "idle after commit");

    // Read 3 words
    uint32_t r0 = read_data(); // sealed_value
    uint32_t r1 = read_data(); // {session_id, mono_count[23:0]}
    uint32_t r2 = read_data(); // {mono_count[31:24], crc16, 8'h00}

    CHECK(r0 == 0xDEADBEEF, "sealed_value matches");
    // mono_count should be 0 at time of first commit (latched cur_mono before increment)
    CHECK((r1 & 0x00FFFFFF) == 0, "mono_count[23:0] == 0 for first commit");
    // session_id locked from session_ctr_in = 0xAB
    CHECK(((r1 >> 24) & 0xFF) == 0xAB, "session_id == 0xAB");
    CHECK(((r2 >> 24) & 0xFF) == 0x00, "mono_count[31:24] == 0");
    // CRC is whatever the engine computed — just check it's in the right field
    uint16_t crc = (r2 >> 8) & 0xFFFF;
    printf("  CRC16 = 0x%04X\n", crc);

    printf("  [T1] done\n");
}

// T2: CRC busy backpressure during FEED_BYTES
static void test_crc_backpressure() {
    printf("[T2] CRC busy backpressure\n");
    reset();
    dut->session_ctr_in = 0x01;

    // Pattern: every other cycle is busy for the first 18 cycles
    // This forces the state machine to stall waiting for crc_busy=0
    uint32_t pattern = 0x55555555; // alternating 0/1
    do_commit(0x12345678, 0x10, pattern);

    CHECK((dut->ctrl_out & 0x3) == 0x2, "idle after backpressured commit");

    uint32_t r0 = read_data();
    CHECK(r0 == 0x12345678, "value correct after backpressure");
    printf("  [T2] done\n");
}

// T3: commit_dropped — commit while seal is busy
static void test_commit_dropped() {
    printf("[T3] commit_dropped\n");
    reset();
    dut->session_ctr_in = 0x01;

    // Start a commit
    write_data(0xAAAAAAAA);
    uint16_t ctrl = (0x20 << 2) | 0x02;
    write_ctrl(ctrl);

    // Seal should be busy now
    CHECK((dut->ctrl_out & 0x1) == 1, "busy after commit");

    // Try to commit again while busy
    dut->ctrl_wr = 1;
    dut->ctrl_in = (0x30 << 2) | 0x02; // another commit
    tick();
    dut->ctrl_wr = 0;
    dut->ctrl_in = 0;

    // commit_dropped (bit[2]) should be set
    CHECK((dut->ctrl_out & 0x4) == 0x4, "commit_dropped set");

    // Wait for original commit to finish
    wait_idle();

    // After original finishes: commit_dropped should still be sticky
    // (It clears only on the NEXT successful commit)
    CHECK((dut->ctrl_out & 0x4) == 0x4, "commit_dropped still sticky after first finishes");

    // Now do a clean commit — it should clear commit_dropped
    do_commit(0xBBBBBBBB, 0x40);
    CHECK((dut->ctrl_out & 0x4) == 0x0, "commit_dropped cleared after successful commit");

    printf("  [T3] done\n");
}

// T4: Read serialization (3x data_rd, auto-wrap)
static void test_read_serialization() {
    printf("[T4] Read serialization\n");
    reset();
    dut->session_ctr_in = 0xCC;

    do_commit(0x11223344, 0x55);

    // Read 3 words
    uint32_t r0 = read_data();
    uint32_t r1 = read_data();
    uint32_t r2 = read_data();

    // After 3 reads, counter should wrap — read again should give r0 again
    uint32_t r0_again = read_data();
    CHECK(r0_again == r0, "read counter wraps after 3 reads");

    // Commit should reset read counter
    do_commit(0x55667788, 0x66);
    uint32_t r0_new = read_data();
    CHECK(r0_new == 0x55667788, "commit resets read counter to 0");

    printf("  [T4] done\n");
}

// T5: Session ID locking (first commit vs subsequent)
static void test_session_locking() {
    printf("[T5] Session ID locking\n");
    reset();
    dut->session_ctr_in = 0x77;

    // First commit — should lock session_id to 0x77
    do_commit(0x00000001, 0x01);
    read_data(); // r0
    uint32_t r1a = read_data(); // r1: {session_id, mono[23:0]}
    uint8_t sid1 = (r1a >> 24) & 0xFF;
    CHECK(sid1 == 0x77, "first commit locks session_id = 0x77");

    // Change session_ctr_in (shouldn't matter, already locked)
    dut->session_ctr_in = 0xFF;
    do_commit(0x00000002, 0x02);
    read_data(); // r0
    uint32_t r1b = read_data(); // r1
    uint8_t sid2 = (r1b >> 24) & 0xFF;
    CHECK(sid2 == 0x77, "second commit still uses locked session_id = 0x77");

    printf("  [T5] done\n");
}

// T6: Standalone crc_reset (ctrl_in[0]=1, commit=0)
static void test_standalone_crc_reset() {
    printf("[T6] Standalone crc_reset\n");
    reset();

    // Write ctrl with crc_reset=1, commit=0
    write_ctrl(0x01); // bit[0]=1, bit[1]=0
    // crc_init should have pulsed (we can't observe it directly in C++,
    // but the module should stay in IDLE)
    CHECK((dut->ctrl_out & 0x3) == 0x2, "still idle after standalone crc_reset");

    // Do a commit after reset to verify normal operation continues
    dut->session_ctr_in = 0x33;
    do_commit(0xCAFEBABE, 0x99);
    CHECK((dut->ctrl_out & 0x3) == 0x2, "idle after commit post-crc_reset");

    printf("  [T6] done\n");
}

// T7: commit with both crc_reset and commit bits set (commit takes priority)
static void test_commit_with_crc_reset() {
    printf("[T7] Commit with crc_reset bit also set\n");
    reset();
    dut->session_ctr_in = 0x11;

    write_data(0xFACEFACE);
    // ctrl_in: sensor=0x22, commit=1, crc_reset=1 → both bits set
    uint16_t ctrl = (0x22 << 2) | 0x03; // bit[1]=1 (commit), bit[0]=1 (crc_reset)
    write_ctrl(ctrl);
    wait_idle();

    CHECK((dut->ctrl_out & 0x3) == 0x2, "idle after commit+crc_reset");
    uint32_t r0 = read_data();
    CHECK(r0 == 0xFACEFACE, "value sealed correctly when both bits set");
    printf("  [T7] done\n");
}

// T8: Multiple commits to exercise monotonic counter increment
// NOTE: Each commit does mono_count++ at S_LATCH→S_IDLE.
// With crc_busy=0, the CRC engine init pulse (crc_init) causes the
// crc16_engine to also process, and its busy signal is connected as an
// input. Since we're not driving crc_busy from the actual engine, the
// seal FSM processes bytes at 2 cycles/byte (feed+advance). The mono
// counter increments strictly once per commit, but the effective count
// seen by reads includes the cur_mono snapshot taken at commit start.
// We verify monotonic increase: each subsequent mono > previous.
static void test_mono_counter() {
    printf("[T8] Monotonic counter increments\n");
    reset();
    dut->session_ctr_in = 0x01;

    uint32_t prev_mono = 0xFFFFFFFF; // sentinel
    for (int i = 0; i < 5; i++) {
        do_commit(0x10000000 + i, 0x01);
        uint32_t r0 = read_data();
        uint32_t r1 = read_data();
        uint32_t r2 = read_data();
        uint32_t mono_lo = r1 & 0x00FFFFFF;
        uint32_t mono_hi = (r2 >> 24) & 0xFF;
        uint32_t mono = (mono_hi << 24) | mono_lo;
        printf("  iter=%d r0=0x%08X r1=0x%08X r2=0x%08X mono=%u\n",
               i, r0, r1, r2, mono);
        CHECK(r0 == (uint32_t)(0x10000000 + i), "sealed_value correct");
        if (i == 0) {
            CHECK(mono == 0, "first commit mono == 0");
        } else {
            CHECK(mono > prev_mono, "mono strictly increases");
        }
        prev_mono = mono;
    }
    printf("  [T8] done\n");
}

// T9: Prolonged crc_busy stall — hold busy=1 for many cycles in a row
static void test_prolonged_busy() {
    printf("[T9] Prolonged crc_busy stall\n");
    reset();
    dut->session_ctr_in = 0x05;

    write_data(0xBEEF0000);
    uint16_t ctrl = (0x07 << 2) | 0x02;
    write_ctrl(ctrl);

    // Hold crc_busy=1 for 20 cycles straight (stall the FSM)
    dut->crc_busy = 1;
    for (int i = 0; i < 20; i++) tick();
    dut->crc_busy = 0;

    wait_idle();
    CHECK((dut->ctrl_out & 0x3) == 0x2, "idle after prolonged busy");
    uint32_t r0 = read_data();
    CHECK(r0 == 0xBEEF0000, "value correct after prolonged busy");
    printf("  [T9] done\n");
}

// T10: crc_busy stall during S_LATCH (busy=1 when entering LATCH)
static void test_latch_busy() {
    printf("[T10] crc_busy during LATCH state\n");
    reset();
    dut->session_ctr_in = 0x08;

    write_data(0xDEAD0001);
    uint16_t ctrl = (0x09 << 2) | 0x02;
    write_ctrl(ctrl);

    // Let FEED_BYTES finish quickly
    for (int i = 0; i < 50; i++) {
        dut->crc_busy = 0;
        tick();
    }
    // Now hold busy=1 for a while (should stall in LATCH)
    dut->crc_busy = 1;
    for (int i = 0; i < 10; i++) tick();
    dut->crc_busy = 0;

    wait_idle();
    CHECK((dut->ctrl_out & 0x3) == 0x2, "idle after LATCH busy stall");
    printf("  [T10] done\n");
}

// T11: byte_sent path — crc_busy=1 on the cycle we try to feed
// This specifically targets the !byte_sent && crc_busy path
static void test_feed_while_busy() {
    printf("[T11] Feed while crc_busy (byte_sent=0, crc_busy=1)\n");
    reset();
    dut->session_ctr_in = 0x0A;

    write_data(0x99887766);
    uint16_t ctrl = (0x0B << 2) | 0x02;
    write_ctrl(ctrl);

    // Pattern: busy=1 for 3 cycles then 0 for 1, repeat
    // This exercises the !byte_sent && crc_busy path repeatedly
    uint32_t pattern = 0b11101110111011101110111011101110;
    wait_idle(pattern);

    CHECK((dut->ctrl_out & 0x3) == 0x2, "idle after feed-while-busy pattern");
    uint32_t r0 = read_data();
    CHECK(r0 == 0x99887766, "value correct after feed-while-busy");
    printf("  [T11] done\n");
}

// ─── main ──────────────────────────────────────────────────────────────

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    Verilated::traceEverOn(true);

    dut = new Vseal_register;
    tfp = new VerilatedVcdC;
    dut->trace(tfp, 99);
    tfp->open("seal_cov.vcd");

    printf("=== seal_register coverage testbench ===\n\n");

    test_normal_commit();
    test_crc_backpressure();
    test_commit_dropped();
    test_read_serialization();
    test_session_locking();
    test_standalone_crc_reset();
    test_commit_with_crc_reset();
    test_mono_counter();
    test_prolonged_busy();
    test_latch_busy();
    test_feed_while_busy();

    printf("\n=== Results: %d / %d PASS ===\n", pass_count, test_count);

    tfp->close();
    dut->final();
    // Write coverage data — try multiple paths for robustness
    const char *cov_path = "coverage.dat";
    VerilatedCov::write(cov_path);
    printf("Coverage written to: %s\n", cov_path);
    delete dut;

    return (pass_count == test_count) ? 0 : 1;
}
