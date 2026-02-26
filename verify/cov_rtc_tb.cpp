// ============================================================================
// Verilator branch-coverage testbench for rtc_counter
// ============================================================================
// Scenarios:
//   1. Reset: seconds=0, us_count=0
//   2. Tick 1,000,000 times → seconds=1
//   3. Write seconds=42 → read back 42, us_count resets
//   4. Write + tick_1us same cycle → write wins
//   5. seconds overflow: 0xFFFFFFFF + 1 tick rollover → 0
//   6. Multiple second boundaries (tick through 3 seconds)
//   7. Continuous ticking without writes
// ============================================================================

#include "Vrtc_counter.h"
#include "verilated.h"
#include "verilated_cov.h"
#include <cstdio>
#include <cstdlib>

static Vrtc_counter *dut;
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
    dut->rst_n    = 0;
    dut->tick_1us = 0;
    dut->wr_en    = 0;
    dut->data_in  = 0;
    for (int i = 0; i < 4; i++) tick();
    dut->rst_n = 1;
    tick();
}

static void write_seconds(uint32_t val) {
    dut->wr_en   = 1;
    dut->data_in = val;
    tick();
    dut->wr_en   = 0;
    dut->data_in = 0;
}

// Pulse tick_1us for one clock
static void pulse_tick() {
    dut->tick_1us = 1;
    tick();
    dut->tick_1us = 0;
}

static int fail_count = 0;

#define CHECK(cond, msg) do { \
    if (!(cond)) { \
        printf("FAIL: %s  (seconds_out=%u)\n", msg, dut->seconds_out); \
        fail_count++; \
    } \
} while(0)

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vrtc_counter;

    // ---- Test 1: Reset ----
    printf("[T1] Reset\n");
    reset();
    CHECK(dut->seconds_out == 0, "T1: seconds_out should be 0 after reset");

    // ---- Test 2: Tick 1,000,000 times → seconds=1 ----
    printf("[T2] Tick 1M → seconds=1\n");
    for (int i = 0; i < 1000000; i++) {
        pulse_tick();
    }
    CHECK(dut->seconds_out == 1, "T2: seconds_out should be 1 after 1M ticks");

    // ---- Test 3: Write seconds=42 → read back, us_count resets ----
    printf("[T3] Write seconds=42\n");
    // First tick a few times to make us_count non-zero
    for (int i = 0; i < 500; i++) pulse_tick();
    write_seconds(42);
    CHECK(dut->seconds_out == 42, "T3: seconds_out should be 42 after write");
    // Tick 1M to verify us_count was reset (should go to 43 exactly)
    for (int i = 0; i < 1000000; i++) pulse_tick();
    CHECK(dut->seconds_out == 43, "T3: seconds_out should be 43 after 1M more ticks");

    // ---- Test 4: Write + tick_1us same cycle → write wins ----
    printf("[T4] Write + tick same cycle → write wins\n");
    // Tick some to get us_count near rollover
    reset();
    for (int i = 0; i < 999999; i++) pulse_tick();
    // Now us_count=999999. Write + tick on same cycle:
    dut->wr_en    = 1;
    dut->data_in  = 100;
    dut->tick_1us = 1;
    tick();
    dut->wr_en    = 0;
    dut->data_in  = 0;
    dut->tick_1us = 0;
    CHECK(dut->seconds_out == 100, "T4: write should win over tick (seconds=100)");
    // After write, us_count should be 0. Tick 1M → seconds=101
    for (int i = 0; i < 1000000; i++) pulse_tick();
    CHECK(dut->seconds_out == 101, "T4: seconds should be 101 after 1M ticks (us_count was reset)");

    // ---- Test 5: seconds overflow 0xFFFFFFFF → 0 ----
    printf("[T5] Overflow 0xFFFFFFFF → 0\n");
    reset();
    write_seconds(0xFFFFFFFF);
    CHECK(dut->seconds_out == 0xFFFFFFFF, "T5: seconds should be 0xFFFFFFFF after write");
    for (int i = 0; i < 1000000; i++) pulse_tick();
    CHECK(dut->seconds_out == 0, "T5: seconds should be 0 after overflow");

    // ---- Test 6: Multiple second boundaries ----
    printf("[T6] Multiple second boundaries (3 seconds)\n");
    reset();
    for (int s = 0; s < 3; s++) {
        for (int i = 0; i < 1000000; i++) pulse_tick();
    }
    CHECK(dut->seconds_out == 3, "T6: seconds should be 3 after 3M ticks");

    // ---- Test 7: Continuous ticking without writes ----
    printf("[T7] Continuous ticking (no writes)\n");
    // Already at 3 from T6, tick 2 more seconds
    for (int s = 0; s < 2; s++) {
        for (int i = 0; i < 1000000; i++) pulse_tick();
    }
    CHECK(dut->seconds_out == 5, "T7: seconds should be 5 after 2 more seconds");

    // ---- Test 8: Idle cycles (no tick, no write) ----
    printf("[T8] Idle cycles\n");
    uint32_t before = dut->seconds_out;
    for (int i = 0; i < 100; i++) tick();  // plain ticks, no tick_1us
    CHECK(dut->seconds_out == before, "T8: seconds should not change during idle");

    // ---- Done ----
    dut->final();

    VerilatedCov::write("verify/obj_rtc/coverage.dat");

    if (fail_count == 0) {
        printf("\n=== ALL RTC TESTS PASSED ===\n");
    } else {
        printf("\n=== %d TEST(S) FAILED ===\n", fail_count);
    }

    delete dut;
    return fail_count;
}
