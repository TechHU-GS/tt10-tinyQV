// ============================================================================
// TB: rtc_counter.v — White-Box Unit Test
// ============================================================================
// Uses hierarchical force/release to preset us_count near rollover,
// avoiding 1M tick_1us cycles per second.
// ============================================================================

`timescale 1ns / 1ps

module tb_rtc;

    reg clk = 0;
    always #20 clk = ~clk;  // 25 MHz

    reg rst_n;
    reg tick_1us;
    reg wr_en;
    reg [31:0] data_in;
    wire [31:0] seconds_out;

    rtc_counter dut (
        .clk(clk),
        .rst_n(rst_n),
        .tick_1us(tick_1us),
        .wr_en(wr_en),
        .data_in(data_in),
        .seconds_out(seconds_out)
    );

    integer pass_count = 0;
    integer fail_count = 0;

    task check(input [511:0] name, input condition);
    begin
        if (condition) begin
            $display("[PASS] %0s", name);
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] %0s", name);
            fail_count = fail_count + 1;
        end
    end
    endtask

    // Pulse tick_1us for exactly 1 clock cycle.
    // Set on negedge (setup), sample on posedge (RTL), clear on next negedge.
    task pulse_tick;
    begin
        @(negedge clk);
        tick_1us = 1;
        @(negedge clk); // after posedge where RTL sampled tick=1
        tick_1us = 0;
    end
    endtask

    task pulse_ticks(input integer n);
        integer i;
    begin
        for (i = 0; i < n; i = i + 1) begin
            @(negedge clk);
            tick_1us = 1;
            @(negedge clk);
            tick_1us = 0;
        end
    end
    endtask

    initial begin
        $dumpfile("tb_rtc.vcd");
        $dumpvars(0, tb_rtc);

        rst_n = 0;
        tick_1us = 0;
        wr_en = 0;
        data_in = 0;
        repeat(4) @(posedge clk);
        rst_n = 1;
        @(posedge clk);

        // ============================================================
        // T1: Reset state
        // ============================================================
        $display("--- T1: Reset ---");
        check("T1: seconds=0 after reset", seconds_out === 32'd0);
        check("T1: us_count=0 after reset", dut.us_count === 20'd0);

        // ============================================================
        // T2: White-box — preset us_count near rollover
        // Force us_count = 999_998, then 2 ticks → seconds=1
        // ============================================================
        $display(""); $display("--- T2: Preset Rollover ---");
        force dut.us_count = 20'd999_998;
        @(posedge clk);
        release dut.us_count;
        check("T2: us_count preset to 999998", dut.us_count === 20'd999_998);
        // tick 1: us_count = 999_999
        pulse_tick;
        check("T2: us_count=999999 after 1 tick", dut.us_count === 20'd999_999);
        check("T2: seconds still 0", seconds_out === 32'd0);
        // tick 2: us_count rolls over → seconds = 1
        pulse_tick;
        check("T2: us_count=0 after rollover", dut.us_count === 20'd0);
        check("T2: seconds=1 after rollover", seconds_out === 32'd1);

        // ============================================================
        // T3: White-box — multiple second increments
        // Preset again to get seconds=2, then seconds=3
        // ============================================================
        $display(""); $display("--- T3: Multi-Second ---");
        force dut.us_count = 20'd999_999;
        @(posedge clk);
        release dut.us_count;
        pulse_tick;  // rollover → seconds=2
        check("T3: seconds=2", seconds_out === 32'd2);
        force dut.us_count = 20'd999_999;
        @(posedge clk);
        release dut.us_count;
        pulse_tick;  // rollover → seconds=3
        check("T3: seconds=3", seconds_out === 32'd3);

        // ============================================================
        // T4: Write — set seconds value
        // ============================================================
        $display(""); $display("--- T4: Write ---");
        @(posedge clk);
        data_in = 32'd100;
        wr_en = 1;
        @(posedge clk);
        wr_en = 0;
        @(posedge clk);
        check("T4: seconds=100 after write", seconds_out === 32'd100);

        // ============================================================
        // T5: Write resets us_count to 0
        // ============================================================
        $display(""); $display("--- T5: Write Resets us_count ---");
        // First accumulate some us_count
        pulse_ticks(50);
        check("T5: us_count=50 after 50 ticks", dut.us_count === 20'd50);
        // Write — should reset us_count
        @(posedge clk);
        data_in = 32'd200;
        wr_en = 1;
        @(posedge clk);
        wr_en = 0;
        @(posedge clk);
        check("T5: seconds=200 after write", seconds_out === 32'd200);
        check("T5: us_count=0 after write", dut.us_count === 20'd0);

        // ============================================================
        // T6: Write priority — write and tick_1us same cycle
        // RTL: wr_en branch before tick_1us → write wins
        // ============================================================
        $display(""); $display("--- T6: Write Priority ---");
        // Preset us_count near rollover
        force dut.us_count = 20'd999_999;
        @(posedge clk);
        release dut.us_count;
        // Same-cycle: wr_en=1 AND tick_1us=1 — set on negedge before posedge
        @(negedge clk);
        data_in = 32'd500;
        wr_en = 1;
        tick_1us = 1;
        @(negedge clk); // posedge between: RTL samples both
        wr_en = 0;
        tick_1us = 0;
        @(posedge clk); // settle
        // Write should have won: seconds=500, us_count=0 (not seconds=201, us_count=0)
        check("T6: write priority, seconds=500", seconds_out === 32'd500);
        check("T6: write priority, us_count=0", dut.us_count === 20'd0);

        // ============================================================
        // T7: Seconds 0xFFFFFFFF → 0x00000000 overflow
        // ============================================================
        $display(""); $display("--- T7: Overflow ---");
        @(posedge clk);
        data_in = 32'hFFFF_FFFF;
        wr_en = 1;
        @(posedge clk);
        wr_en = 0;
        @(posedge clk);
        check("T7: seconds=0xFFFFFFFF", seconds_out === 32'hFFFF_FFFF);
        // Preset us_count near rollover and tick
        force dut.us_count = 20'd999_999;
        @(posedge clk);
        release dut.us_count;
        pulse_tick;
        check("T7: seconds wrapped to 0", seconds_out === 32'd0);
        check("T7: us_count=0 after wrap", dut.us_count === 20'd0);

        // ============================================================
        // T8: No tick → no increment (black-box sanity)
        // ============================================================
        $display(""); $display("--- T8: No Tick ---");
        @(posedge clk);
        data_in = 32'd42;
        wr_en = 1;
        @(posedge clk);
        wr_en = 0;
        repeat(100) @(posedge clk);  // 100 clocks with no tick
        check("T8: seconds=42, no change without tick", seconds_out === 32'd42);
        check("T8: us_count=0, no change without tick", dut.us_count === 20'd0);

        // ============================================================
        $display("");
        $display("=== Results: %0d PASS, %0d FAIL ===", pass_count, fail_count);
        if (fail_count == 0) $display("ALL TESTS PASSED");
        #100; $finish;
    end

endmodule
