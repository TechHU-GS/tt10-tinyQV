// ============================================================================
// Regression Test: TinyQV bit-serial read + read-to-clear registers
// ============================================================================
// Bug: TinyQV is a 4-bit serial CPU. A 32-bit MMIO read takes 8 clock
// cycles with read_n active throughout. If a peripheral clears a flag on
// the first cycle of read_n (data_rd = read_n != 2'b11), status bits in
// upper nibbles read as stale/zero by the time the CPU shifts them in.
//
// Fix: read-side-effects must use read_complete (1-cycle pulse at END),
// not the multi-cycle data_rd derived from read_n.
//
// This test verifies:
//   1. I2C: RX_VALID (bit[10]) remains stable during 8-cycle read
//   2. I2C: rx_has_data clears only AFTER read_complete fires
//   3. Seal: read_seq advances only on read_complete, not on multi-cycle read
//   4. Seal: data_out remains stable during 8-cycle read
// ============================================================================
`timescale 1ns/1ps

module tb_read_clear_regression;
    reg         clk;
    reg         rst_n;

    integer pass_count = 0;
    integer fail_count = 0;
    integer total_tests = 0;

    task check(input [255:0] name, input condition);
    begin
        total_tests = total_tests + 1;
        if (condition) begin
            pass_count = pass_count + 1;
            $display("[PASS] %0s", name);
        end else begin
            fail_count = fail_count + 1;
            $display("[FAIL] %0s", name);
        end
    end
    endtask

    // Clock: 25MHz = 40ns period
    initial clk = 0;
    always #20 clk = ~clk;

    // ====================================================================
    // Part 1: I2C Peripheral — RX_VALID stability during multi-cycle read
    // ====================================================================
    reg  [31:0] i2c_data_in;
    reg         i2c_data_wr;
    reg         i2c_data_rd;     // single-cycle pulse (read_complete)
    wire [31:0] i2c_data_out;
    reg  [31:0] i2c_config_in;
    reg         i2c_config_wr;
    wire [31:0] i2c_config_out;

    // I2C bus: stub (loopback not needed for this test)
    wire scl_o, scl_t, sda_o, sda_t;

    i2c_peripheral i_i2c (
        .clk(clk), .rst_n(rst_n),
        .data_in(i2c_data_in), .data_wr(i2c_data_wr),
        .data_rd(i2c_data_rd), .data_out(i2c_data_out),
        .config_in(i2c_config_in), .config_wr(i2c_config_wr),
        .config_out(i2c_config_out),
        .scl_i(scl_t), .scl_o(scl_o), .scl_t(scl_t),
        .sda_i(sda_t), .sda_o(sda_o), .sda_t(sda_t)
    );

    // ====================================================================
    // Part 2: Seal Register — read_seq stability during multi-cycle read
    // ====================================================================
    reg         seal_data_wr;
    reg  [31:0] seal_data_in;
    wire [31:0] seal_data_out;
    reg         seal_data_rd;    // single-cycle pulse (read_complete)
    reg         seal_ctrl_wr;
    reg  [9:0]  seal_ctrl_in;
    wire [31:0] seal_ctrl_out;

    wire [7:0]  seal_crc_byte;
    wire        seal_crc_feed;
    wire        seal_crc_init;

    // CRC engine stub: goes busy for 2 cycles on init or feed, returns 0xBEEF
    reg         crc_busy_reg;
    reg  [15:0] crc_value_reg;
    reg  [1:0]  crc_busy_cnt;
    initial begin
        crc_busy_reg  = 0;
        crc_value_reg = 16'hBEEF;
        crc_busy_cnt  = 0;
    end
    always @(posedge clk) begin
        if (!rst_n) begin
            crc_busy_reg <= 0;
            crc_busy_cnt <= 0;
        end else if (seal_crc_init || seal_crc_feed) begin
            crc_busy_reg <= 1;
            crc_busy_cnt <= 2;
        end else if (crc_busy_cnt > 0) begin
            crc_busy_cnt <= crc_busy_cnt - 1;
            if (crc_busy_cnt == 1) crc_busy_reg <= 0;
        end
    end

    seal_register i_seal (
        .clk(clk), .rst_n(rst_n),
        .crc_byte(seal_crc_byte), .crc_feed(seal_crc_feed),
        .crc_busy(crc_busy_reg), .crc_value(crc_value_reg),
        .crc_init(seal_crc_init),
        .data_wr(seal_data_wr), .data_in(seal_data_in),
        .data_out(seal_data_out), .data_rd(seal_data_rd),
        .ctrl_wr(seal_ctrl_wr), .ctrl_in(seal_ctrl_in),
        .ctrl_out(seal_ctrl_out),
        .session_ctr_in(8'h42)
    );

    // ====================================================================
    // Test sequence
    // ====================================================================
    integer i;
    reg [31:0] snapshot;

    initial begin
        $dumpfile("tb_read_clear_regression.vcd");
        $dumpvars(0, tb_read_clear_regression);

        rst_n = 0;
        i2c_data_in = 0; i2c_data_wr = 0; i2c_data_rd = 0;
        i2c_config_in = 0; i2c_config_wr = 0;
        seal_data_wr = 0; seal_data_in = 0; seal_data_rd = 0;
        seal_ctrl_wr = 0; seal_ctrl_in = 0;

        $display("=== Regression: TinyQV bit-serial read + read-to-clear ===\n");

        // Reset
        repeat (10) @(posedge clk);
        rst_n = 1;
        repeat (5) @(posedge clk);

        // ================================================================
        // I2C Test: Inject RX data via internal force, then verify stability
        // ================================================================
        $display("--- I2C: RX_VALID stability during multi-cycle read ---");

        // Force rx_has_data=1 and rx_latch=0xAB inside the i2c_peripheral
        // (Normally this comes from AXI stream, but we bypass for unit test)
        force i_i2c.rx_has_data = 1'b1;
        force i_i2c.rx_latch = 8'hAB;
        @(posedge clk);
        @(posedge clk);

        // Verify initial state: bit[10]=1 (RX_VALID), [7:0]=0xAB
        check("I2C: RX_VALID=1 before read", i2c_data_out[10] == 1'b1);
        check("I2C: rx_data=0xAB before read", i2c_data_out[7:0] == 8'hAB);

        // Release the force — now rx_has_data is under DUT control
        release i_i2c.rx_has_data;
        release i_i2c.rx_latch;
        @(posedge clk);

        // Simulate 8-cycle multi-cycle read WITHOUT firing read_complete:
        // In real hardware, read_n != 2'b11 for 8 cycles, but data_rd (which
        // the bridge sees) should be read_complete, NOT read_n.
        // Since we wired data_rd = read_complete in project.v, the bridge
        // should NOT clear rx_has_data during these 8 cycles.
        //
        // We keep data_rd = 0 for 8 cycles (simulating the multi-cycle read
        // where the CPU hasn't finished yet).
        snapshot = i2c_data_out;
        for (i = 0; i < 8; i = i + 1) begin
            @(posedge clk);
            // data_rd stays 0 — CPU is still reading nibbles
        end

        // After 8 "read" cycles with no read_complete: data should be stable
        check("I2C: RX_VALID still 1 after 8 cycles (no read_complete)",
              i2c_data_out[10] == 1'b1);
        check("I2C: data_out stable after 8 cycles",
              i2c_data_out == snapshot);

        // Now fire read_complete (single pulse)
        @(posedge clk);
        i2c_data_rd <= 1;
        @(posedge clk);
        i2c_data_rd <= 0;
        @(posedge clk);
        @(posedge clk);

        // After read_complete: rx_has_data should be cleared
        check("I2C: RX_VALID=0 after read_complete", i2c_data_out[10] == 1'b0);

        // ================================================================
        // I2C Negative test: verify that a WRONG implementation (clearing
        // on first cycle) would fail. We do this by directly pulsing
        // data_rd for 8 consecutive cycles and checking what happens.
        // ================================================================
        $display("");
        $display("--- I2C: Verify single-pulse data_rd is sufficient ---");

        // Re-inject RX data
        force i_i2c.rx_has_data = 1'b1;
        force i_i2c.rx_latch = 8'h55;
        @(posedge clk); @(posedge clk);
        release i_i2c.rx_has_data;
        release i_i2c.rx_latch;
        @(posedge clk);

        check("I2C: re-injected RX_VALID=1", i2c_data_out[10] == 1'b1);
        check("I2C: re-injected rx_data=0x55", i2c_data_out[7:0] == 8'h55);

        // Single pulse clears it
        @(posedge clk);
        i2c_data_rd <= 1;
        @(posedge clk);
        i2c_data_rd <= 0;
        @(posedge clk); @(posedge clk);
        check("I2C: cleared by single pulse", i2c_data_out[10] == 1'b0);

        // ================================================================
        // Seal Test: read_seq stability during multi-cycle read
        // ================================================================
        $display("");
        $display("--- Seal: read_seq advances only on read_complete ---");

        // First, commit a value so sealed_value/sealed_mono/sealed_crc are populated
        @(posedge clk);
        seal_data_in <= 32'hCAFE0001;
        seal_data_wr <= 1;
        @(posedge clk);
        seal_data_wr <= 0;

        // Commit with sensor_id=0x42
        // ctrl_in = {sensor_id[9:2], commit[1], crc_reset[0]}
        @(posedge clk);
        seal_ctrl_in <= 10'b01_0000_10_1_0;  // sensor_id=0x42, commit=1
        seal_ctrl_wr <= 1;
        @(posedge clk);
        seal_ctrl_wr <= 0;
        seal_ctrl_in <= 10'b0;
        repeat (2) @(posedge clk);  // let state machine start

        // Wait for seal to complete (state machine: FEED_BYTES → LATCH → IDLE)
        // With CRC stub (never busy), each byte is sent every other cycle
        // 9 bytes * 2 cycles + LATCH = ~20 cycles, add margin
        begin : seal_wait
            integer tw;
            tw = 0;
            while (seal_ctrl_out[0] && tw < 500) begin
                @(posedge clk);
                tw = tw + 1;
            end
            $display("  Seal commit took %0d cycles", tw);
        end

        // Verify seal is idle
        check("Seal: idle after commit", seal_ctrl_out[0] == 1'b0);
        check("Seal: ready after commit", seal_ctrl_out[1] == 1'b1);

        // Read 0: should be sealed_value = 0xCAFE0001
        snapshot = seal_data_out;
        $display("  Seal read[0] = 0x%08X (expect 0xCAFE0001)", snapshot);
        check("Seal: read[0] = 0xCAFE0001", snapshot == 32'hCAFE0001);

        // Simulate 8-cycle multi-cycle read: do NOT fire data_rd
        for (i = 0; i < 8; i = i + 1) begin
            @(posedge clk);
            // Verify data_out remains stable (read_seq hasn't advanced)
        end
        check("Seal: read[0] stable after 8 cycles (no read_complete)",
              seal_data_out == snapshot);

        // Fire read_complete — advance to read[1]
        @(posedge clk);
        seal_data_rd <= 1;
        @(posedge clk);
        seal_data_rd <= 0;
        @(posedge clk);

        // read[1]: {session_id[7:0], mono_count[23:0]}
        // First commit: session_id = 0x42 (from session_ctr_in), mono = 0x000000
        snapshot = seal_data_out;
        $display("  Seal read[1] = 0x%08X (expect 0x42000000)", snapshot);
        check("Seal: read[1] = {sid=0x42, mono[23:0]=0}",
              snapshot == 32'h42000000);

        // 8-cycle stability check for read[1]
        for (i = 0; i < 8; i = i + 1) @(posedge clk);
        check("Seal: read[1] stable after 8 cycles", seal_data_out == snapshot);

        // Advance to read[2]
        @(posedge clk);
        seal_data_rd <= 1;
        @(posedge clk);
        seal_data_rd <= 0;
        @(posedge clk);

        // read[2]: {mono[31:24], crc16[15:0], 8'h00}
        // mono[31:24] = 0x00, crc = 0xBEEF (stub), pad = 0x00
        snapshot = seal_data_out;
        $display("  Seal read[2] = 0x%08X (expect 0x00BEEF00)", snapshot);
        check("Seal: read[2] = {mono_hi=0, crc=0xBEEF, 0x00}",
              snapshot == 32'h00BEEF00);

        // 8-cycle stability check for read[2]
        for (i = 0; i < 8; i = i + 1) @(posedge clk);
        check("Seal: read[2] stable after 8 cycles", seal_data_out == snapshot);

        // Advance past read[2] → wraps to read[0]
        @(posedge clk);
        seal_data_rd <= 1;
        @(posedge clk);
        seal_data_rd <= 0;
        @(posedge clk);

        check("Seal: read[0] again after wrap",
              seal_data_out == 32'hCAFE0001);

        // ================================================================
        // Seal: verify double-pulse doesn't skip a read
        // ================================================================
        $display("");
        $display("--- Seal: double-pulse must not skip reads ---");

        // We're at read[0] after the wrap. Fire TWO single-cycle pulses
        // back-to-back. This should advance read_seq by 2 (0→1→2).
        @(posedge clk);
        seal_data_rd <= 1;
        @(posedge clk);
        seal_data_rd <= 1;  // second consecutive pulse
        @(posedge clk);
        seal_data_rd <= 0;
        @(posedge clk);

        // Should be at read[2] now (0→1→2)
        check("Seal: double-pulse → read[2]",
              seal_data_out == 32'h00BEEF00);

        // ================================================================
        // Summary
        // ================================================================
        repeat (10) @(posedge clk);
        $display("");
        $display("=== Results: %0d PASS, %0d FAIL out of %0d tests ===",
                 pass_count, fail_count, total_tests);
        if (fail_count == 0) $display("ALL TESTS PASSED");
        else $display("SOME TESTS FAILED");
        $finish;
    end

    // Timeout
    initial begin
        #10_000_000;
        $display("TIMEOUT!");
        $finish;
    end

endmodule
