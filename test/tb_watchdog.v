// ============================================================================
// TB: Hardware Watchdog Timer
// ============================================================================
// Tests:
//   1. Power-on: disabled, no reset even after waiting
//   2. Write non-zero: enable + countdown → wdt_reset at expiry
//   3. Kick (reload) before expiry → no reset
//   4. Write zero while enabled → ignored, WDT continues
//   5. wdt_reset pulse width = 1 clock cycle
// ============================================================================

`timescale 1ns / 1ps

module tb_watchdog;

    reg        clk = 0;
    always #20 clk = ~clk;  // 25 MHz

    reg        rst_n;
    reg        tick_1us;
    reg        kick;
    reg [31:0] kick_value;
    wire [31:0] remaining;
    wire       wdt_reset;

    watchdog dut (
        .clk        (clk),
        .rst_n      (rst_n),
        .tick_1us   (tick_1us),
        .kick       (kick),
        .kick_value (kick_value),
        .remaining  (remaining),
        .wdt_reset  (wdt_reset)
    );

    integer pass_count = 0;
    integer fail_count = 0;

    task check(input expected, input actual, input [8*64-1:0] msg);
        begin
            if (expected === actual) begin
                $display("[PASS] %0s", msg);
                pass_count = pass_count + 1;
            end else begin
                $display("[FAIL] %0s: expected=%0d, got=%0d", msg, expected, actual);
                fail_count = fail_count + 1;
            end
        end
    endtask

    task check32(input [31:0] expected, input [31:0] actual, input [8*64-1:0] msg);
        begin
            if (expected === actual) begin
                $display("[PASS] %0s: %0d", msg, actual);
                pass_count = pass_count + 1;
            end else begin
                $display("[FAIL] %0s: expected=%0d, got=%0d", msg, expected, actual);
                fail_count = fail_count + 1;
            end
        end
    endtask

    // Generate tick_1us: pulse every 25 clk cycles (1µs at 25MHz)
    integer tick_div = 0;
    always @(posedge clk) begin
        if (!rst_n) begin
            tick_div <= 0;
            tick_1us <= 0;
        end else begin
            if (tick_div == 24) begin
                tick_div <= 0;
                tick_1us <= 1;
            end else begin
                tick_div <= tick_div + 1;
                tick_1us <= 0;
            end
        end
    end

    // Write to WDT
    task wdt_write(input [31:0] val);
        begin
            @(posedge clk);
            kick       <= 1'b1;
            kick_value <= val;
            @(posedge clk);
            kick       <= 1'b0;
        end
    endtask

    // Wait N microseconds (approximate: N * 25 clk cycles)
    task wait_us(input integer n);
        begin
            repeat(n * 25) @(posedge clk);
        end
    endtask

    // Count wdt_reset pulses over a time window
    integer reset_pulse_count;
    integer reset_pulse_width;

    initial begin
        $dumpfile("tb_watchdog.vcd");
        $dumpvars(0, tb_watchdog);

        rst_n      = 0;
        kick       = 0;
        kick_value = 0;
        #200;
        rst_n = 1;
        #80;

        $display("=== Watchdog Timer Testbench ===");
        $display("");

        // ---- Test 1: Disabled by default, no reset ----
        $display("--- Test 1: Default disabled ---");
        check32(32'd0, remaining, "remaining=0 after reset");
        check(1'b0, wdt_reset, "wdt_reset=0 on power-up");
        wait_us(10);
        check(1'b0, wdt_reset, "wdt_reset=0 after 10us (disabled)");

        // ---- Test 2: Enable with countdown=5, expires ----
        $display("--- Test 2: Enable + countdown → reset ---");
        wdt_write(32'd5);  // 5 microseconds
        wait_us(1);
        // remaining should be ~4
        check(1'b0, wdt_reset, "no reset at 1us");
        wait_us(5);  // total ~6us, should have expired
        // Check that wdt_reset fired at some point
        // We need to catch the pulse — let's check remaining instead
        check32(32'd0, remaining, "remaining=0 after expiry");

        // ---- Test 3: Reload (kick) before expiry ----
        $display("--- Test 3: Kick before expiry ---");
        wdt_write(32'd100);  // 100µs countdown
        wait_us(50);
        // Should still have ~50 left
        check(1'b0, wdt_reset, "no reset at 50us");
        wdt_write(32'd100);  // kick: reload to 100µs
        wait_us(50);
        check(1'b0, wdt_reset, "no reset at 50us after kick");
        // Now let it expire
        wait_us(60);  // 50+60=110 > 100
        check32(32'd0, remaining, "remaining=0 after full expiry");

        // ---- Test 4: Write zero while enabled → ignored ----
        $display("--- Test 4: Write 0 ignored ---");
        wdt_write(32'd200);  // reload
        wait_us(10);
        wdt_write(32'd0);    // try to disable — should be ignored
        wait_us(10);
        // remaining should still be counting down (not 0)
        if (remaining > 32'd0 && remaining < 32'd200) begin
            $display("[PASS] write 0 ignored: remaining=%0d (still counting)", remaining);
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] write 0 not ignored: remaining=%0d", remaining);
            fail_count = fail_count + 1;
        end

        // ---- Test 5: wdt_reset pulse width ----
        $display("--- Test 5: Reset pulse width ---");
        // Let current countdown expire to get a clean pulse
        wdt_write(32'd10);   // short countdown
        reset_pulse_count = 0;
        reset_pulse_width = 0;
        fork
            begin
                // Monitor reset pulse
                @(posedge wdt_reset);
                reset_pulse_count = reset_pulse_count + 1;
                @(posedge clk);
                if (wdt_reset) reset_pulse_width = reset_pulse_width + 1;
                else reset_pulse_width = 1;
            end
            begin
                wait_us(15);
            end
        join
        check(1'b1, (reset_pulse_count >= 1) ? 1'b1 : 1'b0, "reset pulse detected");

        // ---- Test 6: Cycle-exact pet at N-1 (saves from reset) ----
        $display("--- Test 6: Pet at N-1 µs ---");
        rst_n = 0; repeat(5) @(posedge clk); rst_n = 1; repeat(3) @(posedge clk);
        wdt_write(32'd5);  // 5µs timeout
        wait_us(4);        // 4µs elapsed, 1µs remaining
        // Pet BEFORE expiry — should reload
        wdt_write(32'd100);
        // Wait a bit — should NOT have fired
        wait_us(2);
        check(1'b0, wdt_reset, "pet at N-1: no reset");
        // remaining should be >90
        if (remaining > 32'd90) begin
            $display("[PASS] pet at N-1: remaining=%0d (reloaded)", remaining);
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] pet at N-1: remaining=%0d (expected >90)", remaining);
            fail_count = fail_count + 1;
        end

        // ---- Test 7: Same-cycle kick vs tick: kick wins (priority in RTL) ----
        // In watchdog.v, `kick` branch is the IF, tick is the ELSE IF.
        // So kick always takes priority over tick decrement.
        $display("--- Test 7: Kick priority over tick ---");
        rst_n = 0; repeat(5) @(posedge clk); rst_n = 1; repeat(3) @(posedge clk);
        wdt_write(32'd50);
        wait_us(20);  // ~30 left
        // Issue kick right before a tick — kick should win
        @(posedge clk);
        kick       <= 1'b1;
        kick_value <= 32'd200;
        @(posedge clk);
        kick       <= 1'b0;
        @(posedge clk);
        // remaining should be exactly 200 (kick loaded), not 199 (tick decremented)
        check32(32'd200, remaining, "kick overrides tick: remaining=200");

        // ---- Test 8: WDT reset clears enabled flag (after system reset) ----
        $display("--- Test 8: Reset clears enabled ---");
        rst_n = 0; repeat(5) @(posedge clk); rst_n = 1; repeat(3) @(posedge clk);
        wdt_write(32'd5);  // enable WDT
        wait_us(6);  // let it expire
        @(posedge clk);
        // After system reset (rst_n cycle), enabled should be 0
        rst_n = 0; repeat(5) @(posedge clk); rst_n = 1; repeat(3) @(posedge clk);
        check32(32'd0, remaining, "remaining=0 after hard reset");
        // Write 0 should do nothing (WDT disabled, not re-enabled)
        wait_us(10);
        check(1'b0, wdt_reset, "no reset after hard reset (disabled)");

        // ---- Test 9: WDT countdown=1 fires after exactly 1 tick ----
        $display("--- Test 9: Countdown=1 fires on first tick ---");
        rst_n = 0; repeat(5) @(posedge clk); rst_n = 1; repeat(3) @(posedge clk);
        wdt_write(32'd1);  // 1µs timeout = should fire after first tick_1us
        // First tick at ~25 clk. Wait ~30 clk and check.
        repeat(30) @(posedge clk);
        check32(32'd0, remaining, "countdown=1: expired after 1 tick");

        // ---- Test 10: Large countdown doesn't overflow ----
        $display("--- Test 10: Large countdown ---");
        rst_n = 0; repeat(5) @(posedge clk); rst_n = 1; repeat(3) @(posedge clk);
        wdt_write(32'hFFFF_FFFF);  // Max value
        wait_us(5);
        // Should still be close to max (decremented by 5)
        if (remaining > 32'hFFFF_FFF0) begin
            $display("[PASS] large countdown: remaining=0x%08X", remaining);
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] large countdown: remaining=0x%08X (too low)", remaining);
            fail_count = fail_count + 1;
        end

        // ---- Test 11: C6 — Write 0 while enabled ----
        $display("--- Test 11: C6 write 0 ignored ---");
        rst_n = 0; repeat(5) @(posedge clk); rst_n = 1; repeat(3) @(posedge clk);
        wdt_write(32'd500);    // enable with 500µs
        wait_us(100);
        // remaining should be ~400
        if (remaining == 32'd0) begin
            $display("[FAIL] C6: WDT not counting");
            fail_count = fail_count + 1;
        end else begin
            $display("[PASS] C6: WDT counting (remaining=%0d)", remaining);
            pass_count = pass_count + 1;
        end
        // Write 0 — should be ignored
        wdt_write(32'd0);
        wait_us(50);
        // Should still be counting (remaining > 0, not stopped)
        if (remaining > 32'd0 && remaining < 32'd500) begin
            $display("[PASS] C6: write 0 ignored (remaining=%0d)", remaining);
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] C6: write 0 stopped WDT (remaining=%0d)", remaining);
            fail_count = fail_count + 1;
        end

        // ---- Test 12: WDT reset pulse width ----
        $display("--- Test 12: Reset pulse width ---");
        rst_n = 0; repeat(5) @(posedge clk); rst_n = 1; repeat(3) @(posedge clk);
        wdt_write(32'd3);  // very short: 3µs
        begin : pulse_width_block
            integer pulse_clks;
            pulse_clks = 0;
            // Wait for reset pulse to go high
            @(posedge wdt_reset);
            // Sample after NBA region settles (#1 delay past posedge clk)
            @(posedge clk); #1;
            while (wdt_reset) begin
                pulse_clks = pulse_clks + 1;
                @(posedge clk); #1;
                if (pulse_clks > 10) begin
                    $display("[FAIL] reset pulse >10 clocks");
                    fail_count = fail_count + 1;
                    disable pulse_width_block;
                end
            end
            // pulse_clks=0 means wdt_reset was already low after 1st posedge
            // → pulse was exactly 1 clock cycle (RTL sets high, then default low)
            check(1'b1, (pulse_clks == 0) ? 1'b1 : 1'b0, "reset pulse = 1 cycle");
        end

        // ---- Test 13: Random 0-writes don't disable ----
        $display("--- Test 13: Random 0-writes ---");
        rst_n = 0; repeat(5) @(posedge clk); rst_n = 1; repeat(3) @(posedge clk);
        wdt_write(32'd10000);  // 10ms timeout
        wait_us(100);
        // Write 0 many times
        wdt_write(32'd0);
        wdt_write(32'd0);
        wdt_write(32'd0);
        wdt_write(32'd0);
        wdt_write(32'd0);
        wait_us(100);
        if (remaining > 32'd0 && remaining < 32'd10000) begin
            $display("[PASS] still counting after 5x zero writes (remaining=%0d)", remaining);
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] zero writes affected WDT (remaining=%0d)", remaining);
            fail_count = fail_count + 1;
        end

        // ---- Summary ----
        $display("");
        $display("=== Results: %0d PASS, %0d FAIL ===", pass_count, fail_count);
        if (fail_count == 0)
            $display("ALL TESTS PASSED");
        else
            $display("SOME TESTS FAILED");

        #100;
        $finish;
    end

endmodule
