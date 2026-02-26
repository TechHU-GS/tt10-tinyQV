// ============================================================================
// Verilator branch-coverage testbench for watchdog.v
// ============================================================================
// Exercises all paths:
//   1. Reset state
//   2. Kick with non-zero value -> enable + load
//   3. Tick countdown
//   4. Counter reaches 1 -> next tick -> wdt_reset pulse
//   5. Counter reaches 0 -> wdt_reset fires then clears
//   6. Write zero while enabled -> ignored
//   7. Re-kick with new value while enabled
//   8. Kick and tick_1us same cycle -> kick takes precedence
//   9. Multiple consecutive kicks
//  10. Very large counter value
// ============================================================================

#include "Vwatchdog.h"
#include "verilated.h"
#include "verilated_cov.h"
#include <cstdio>
#include <cstdlib>
#include <cassert>

static vluint64_t sim_time = 0;

static void tick(Vwatchdog *dut) {
    dut->clk = 0;
    dut->eval();
    sim_time++;
    dut->clk = 1;
    dut->eval();
    sim_time++;
}

// Helper: drive one clock with specified inputs
static void drive(Vwatchdog *dut, int kick, uint32_t kick_value,
                  int tick_1us) {
    dut->kick       = kick;
    dut->kick_value = kick_value;
    dut->tick_1us   = tick_1us;
    tick(dut);
}

// Helper: idle one clock (no kick, no tick)
static void idle(Vwatchdog *dut) {
    drive(dut, 0, 0, 0);
}

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    Vwatchdog *dut = new Vwatchdog;

    int pass = 0;
    int fail = 0;

    auto check = [&](bool cond, const char *msg) {
        if (cond) {
            pass++;
        } else {
            printf("  FAIL: %s\n", msg);
            fail++;
        }
    };

    // ---- Test 1: Reset state ------------------------------------------------
    printf("Test 1: Reset state\n");
    dut->rst_n     = 0;
    dut->kick      = 0;
    dut->kick_value = 0;
    dut->tick_1us  = 0;
    tick(dut);
    tick(dut);  // two reset cycles for good measure
    check(dut->remaining == 0,  "counter == 0 after reset");
    check(dut->wdt_reset == 0,  "wdt_reset == 0 after reset");

    // Release reset
    dut->rst_n = 1;
    idle(dut);
    check(dut->remaining == 0,  "counter still 0 after reset release");
    check(dut->wdt_reset == 0,  "wdt_reset still 0 after reset release");

    // ---- Test 2: Kick with non-zero value -> enable + load ------------------
    printf("Test 2: Kick with non-zero value\n");
    drive(dut, 1, 5, 0);  // kick=1, value=5, no tick
    check(dut->remaining == 5,  "counter loaded to 5");
    check(dut->wdt_reset == 0,  "no reset on kick");

    // ---- Test 3: Tick countdown ---------------------------------------------
    printf("Test 3: Tick countdown\n");
    drive(dut, 0, 0, 1);  // tick_1us
    check(dut->remaining == 4,  "counter decremented to 4");
    check(dut->wdt_reset == 0,  "no reset yet");

    drive(dut, 0, 0, 1);  // tick_1us
    check(dut->remaining == 3,  "counter decremented to 3");

    drive(dut, 0, 0, 1);  // tick_1us
    check(dut->remaining == 2,  "counter decremented to 2");

    // ---- Test 4 & 5: Counter reaches 1 -> wdt_reset pulse on next tick ------
    printf("Test 4/5: Counter 1->0, wdt_reset pulse\n");
    check(dut->remaining == 2, "counter is 2 before final ticks");

    drive(dut, 0, 0, 1);  // counter 2->1
    check(dut->remaining == 1,  "counter decremented to 1");
    check(dut->wdt_reset == 0,  "no reset at counter==1 (fires when 1->0)");

    drive(dut, 0, 0, 1);  // counter 1->0 -> wdt_reset pulse
    check(dut->remaining == 0,  "counter reached 0");
    check(dut->wdt_reset == 1,  "wdt_reset fires when counter transitions 1->0");

    // Next cycle: wdt_reset should clear (1-cycle pulse)
    idle(dut);
    check(dut->wdt_reset == 0,  "wdt_reset clears after 1 cycle");
    check(dut->remaining == 0,  "counter stays at 0");

    // Additional tick with counter==0: should stay 0, no reset
    drive(dut, 0, 0, 1);
    check(dut->remaining == 0,  "counter stays 0 when already 0");
    check(dut->wdt_reset == 0,  "no reset when counter already 0");

    // ---- Test 6: Write zero while enabled -> ignored ------------------------
    printf("Test 6: Write zero while enabled -> ignored\n");
    // First re-arm the watchdog
    drive(dut, 1, 10, 0);
    check(dut->remaining == 10, "re-armed to 10");

    // Now try to write zero
    drive(dut, 1, 0, 0);  // kick with value=0
    check(dut->remaining == 10, "counter unchanged (zero kick ignored)");
    check(dut->wdt_reset == 0,  "no reset on zero kick");

    // ---- Test 7: Re-kick with new value while enabled -----------------------
    printf("Test 7: Re-kick with new value\n");
    drive(dut, 1, 20, 0);
    check(dut->remaining == 20, "counter reloaded to 20");
    check(dut->wdt_reset == 0,  "no reset on re-kick");

    // Verify it counts down from new value
    drive(dut, 0, 0, 1);
    check(dut->remaining == 19, "counting down from reloaded value");

    // ---- Test 8: Kick and tick_1us same cycle -> kick takes precedence ------
    printf("Test 8: Kick + tick same cycle -> kick wins\n");
    // Currently at 19
    drive(dut, 1, 50, 1);  // kick=1, value=50, tick_1us=1 simultaneously
    check(dut->remaining == 50, "kick takes precedence over tick (if-else structure)");
    check(dut->wdt_reset == 0,  "no reset");

    // ---- Test 9: Multiple consecutive kicks ---------------------------------
    printf("Test 9: Multiple consecutive kicks\n");
    drive(dut, 1, 100, 0);
    check(dut->remaining == 100, "first consecutive kick: 100");
    drive(dut, 1, 200, 0);
    check(dut->remaining == 200, "second consecutive kick: 200");
    drive(dut, 1, 300, 0);
    check(dut->remaining == 300, "third consecutive kick: 300");

    // ---- Test 10: Very large counter value ----------------------------------
    printf("Test 10: Very large counter value\n");
    drive(dut, 1, 0xFFFFFFFF, 0);
    check(dut->remaining == 0xFFFFFFFF, "loaded max value");
    drive(dut, 0, 0, 1);
    check(dut->remaining == 0xFFFFFFFE, "decremented from max value");
    check(dut->wdt_reset == 0, "no reset at large value");

    // ---- Bonus: Cover the "not enabled" + tick path (no decrement) ----------
    printf("Bonus: tick_1us while disabled -> no effect\n");
    // Full reset to return to disabled state
    dut->rst_n = 0;
    tick(dut);
    tick(dut);
    dut->rst_n = 1;
    idle(dut);
    check(dut->remaining == 0, "counter 0 after fresh reset");

    // tick while disabled: should have no effect
    drive(dut, 0, 0, 1);
    check(dut->remaining == 0, "tick while disabled: counter stays 0");
    check(dut->wdt_reset == 0, "tick while disabled: no reset");

    // ---- Bonus: kick with zero while disabled -> ignored --------------------
    printf("Bonus: kick zero while disabled -> no effect\n");
    drive(dut, 1, 0, 0);
    check(dut->remaining == 0, "kick zero while disabled: counter stays 0");

    // ---- Bonus: tick + kick(0) same cycle while enabled ---------------------
    printf("Bonus: tick + kick(0) same cycle while enabled\n");
    drive(dut, 1, 3, 0);  // enable with value 3
    check(dut->remaining == 3, "enabled with 3");
    drive(dut, 1, 0, 1);  // kick(0) + tick same cycle
    // kick_value==0 so kick branch not taken, tick branch taken
    check(dut->remaining == 2, "kick(0)+tick: tick wins since kick(0) falls through");

    // ---- Summary -----------------------------------------------------------
    printf("\n========================================\n");
    printf("  PASS: %d / %d\n", pass, pass + fail);
    printf("  FAIL: %d\n", fail);
    printf("========================================\n");

    // Write coverage data
    VerilatedCov::write("verify/obj_wdt/coverage.dat");

    dut->final();
    delete dut;

    return (fail > 0) ? 1 : 0;
}
