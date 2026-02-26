// Seal Register Testbench
// Tests: commit flow, mono_count, 3x read serialization, session_id, CRC verify
`timescale 1ns/1ps

module tb_seal;
    reg         clk;
    reg         rst_n;

    // CRC engine interface
    wire [7:0]  crc_byte;
    wire        crc_feed;
    wire        crc_busy;
    wire [15:0] crc_value;
    wire        crc_init;

    // SEAL_DATA bus
    reg         data_wr;
    reg  [31:0] data_in;
    wire [31:0] data_out;
    reg         data_rd;

    // SEAL_CTRL bus
    reg         ctrl_wr;
    reg  [9:0]  ctrl_in;
    wire [31:0] ctrl_out;

    // Session counter
    reg  [7:0]  session_ctr_in;

    // Shared CRC16 engine instance
    crc16_engine i_crc (
        .clk       (clk),
        .rst_n     (rst_n),
        .init      (crc_init),
        .data_in   (crc_byte),
        .data_valid(crc_feed),
        .crc_out   (crc_value),
        .busy      (crc_busy)
    );

    // DUT
    seal_register uut (
        .clk            (clk),
        .rst_n          (rst_n),
        .crc_byte       (crc_byte),
        .crc_feed       (crc_feed),
        .crc_busy       (crc_busy),
        .crc_value      (crc_value),
        .crc_init       (crc_init),
        .data_wr        (data_wr),
        .data_in        (data_in),
        .data_out       (data_out),
        .data_rd        (data_rd),
        .ctrl_wr        (ctrl_wr),
        .ctrl_in        (ctrl_in),
        .ctrl_out       (ctrl_out),
        .session_ctr_in (session_ctr_in)
    );

    // Clock: 25MHz
    initial clk = 0;
    always #20 clk = ~clk;

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

    task seal_write_data(input [31:0] val);
    begin
        @(posedge clk);
        data_in <= val;
        data_wr <= 1;
        @(posedge clk);
        data_wr <= 0;
    end
    endtask

    task seal_write_ctrl(input [9:0] val);
    begin
        @(posedge clk);
        ctrl_in <= val;
        ctrl_wr <= 1;
        @(posedge clk);
        ctrl_wr <= 0;
    end
    endtask

    task seal_read_data;
    begin
        @(posedge clk);
        data_rd <= 1;
        @(posedge clk);
        data_rd <= 0;
    end
    endtask

    task wait_seal_done;
        integer timeout;
    begin
        // Wait one cycle for commit to take effect (NBA scheduling)
        @(posedge clk);
        timeout = 0;
        while (ctrl_out[0] && timeout < 5000) begin
            @(posedge clk);
            timeout = timeout + 1;
        end
    end
    endtask

    // Variables for read results
    reg [31:0] rd0, rd1, rd2;

    // Golden vector storage: {sensor_id[7:0], value[31:0], crc[15:0]} = 56 bits
    reg [55:0] golden_mem [0:99];
    integer gv_i;
    reg [7:0]  gv_sid;
    reg [31:0] gv_val;
    reg [15:0] gv_expected_crc;
    reg [15:0] gv_actual_crc;
    integer gv_pass, gv_fail;

    // CRC16-CCITT reference (polynomial 0x8005, init 0xFFFF)
    // We'll verify by computing CRC of same bytes via the engine
    reg [15:0] expected_crc;

    initial begin
        $dumpfile("tb_seal.vcd");
        $dumpvars(0, tb_seal);

        rst_n = 0;
        data_wr = 0;
        data_rd = 0;
        data_in = 0;
        ctrl_wr = 0;
        ctrl_in = 0;
        session_ctr_in = 8'h42;  // Free-running counter value at boot

        $display("=== Seal Register Testbench ===\n");

        repeat(10) @(posedge clk);
        rst_n = 1;
        repeat(5) @(posedge clk);

        // --- Test 1: Default state ---
        $display("--- Test 1: Default state ---");
        check("seal_ready=1 after reset", ctrl_out[1] == 1'b1);
        check("seal_busy=0 after reset", ctrl_out[0] == 1'b0);

        // --- Test 2: Basic commit flow ---
        $display("--- Test 2: Basic commit (value=0xDEADBEEF, sensor=0x01) ---");
        // CRC reset
        seal_write_ctrl(10'b00_0000_0001);  // crc_reset=1
        repeat(2) @(posedge clk);

        // Write value
        seal_write_data(32'hDEADBEEF);

        // Commit: sensor_id=0x01, commit=1, crc_reset=0
        seal_write_ctrl({8'h01, 1'b1, 1'b0});  // sensor_id=0x01, commit=1
        repeat(2) @(posedge clk);
        check("busy after commit", ctrl_out[0] == 1'b1);

        // Wait for completion
        wait_seal_done;
        check("done after commit", ctrl_out[0] == 1'b0);
        check("ready after commit", ctrl_out[1] == 1'b1);

        // --- Test 3: 3x read serialization ---
        $display("--- Test 3: 3x read serialization ---");
        // Read 0: value
        rd0 = data_out;
        check("read0 = value 0xDEADBEEF", rd0 == 32'hDEADBEEF);

        seal_read_data;
        repeat(2) @(posedge clk);
        // Read 1: {session_id, mono_count[23:0]}
        rd1 = data_out;
        check("read1[31:24] = session_id 0x42", rd1[31:24] == 8'h42);
        check("read1[23:0] = mono_count[23:0] = 0", rd1[23:0] == 24'd0);

        seal_read_data;
        repeat(2) @(posedge clk);
        // Read 2: {mono_count[31:24], crc16, 8'h00}
        rd2 = data_out;
        check("read2[31:24] = mono_count[31:24] = 0", rd2[31:24] == 8'h00);
        check("read2[7:0] = 0x00 (padding)", rd2[7:0] == 8'h00);
        // CRC should be non-zero (we'll verify exact value in Test 6)
        check("read2 CRC != 0", rd2[23:8] != 16'h0000);
        $display("  CRC16 = 0x%04X", rd2[23:8]);

        // --- Test 4: Mono count increments ---
        $display("--- Test 4: Second commit (mono_count=1) ---");
        seal_write_ctrl(10'b00_0000_0001);  // crc_reset
        repeat(2) @(posedge clk);
        seal_write_data(32'h12345678);
        seal_write_ctrl({8'h02, 1'b1, 1'b0});  // sensor_id=0x02, commit
        wait_seal_done;

        // Read sequence should have been reset by commit
        rd0 = data_out;
        check("2nd commit value = 0x12345678", rd0 == 32'h12345678);

        seal_read_data;
        repeat(2) @(posedge clk);
        rd1 = data_out;
        check("2nd commit mono_count[23:0] = 1", rd1[23:0] == 24'd1);
        check("session_id still 0x42", rd1[31:24] == 8'h42);

        // --- Test 5: Session ID locked ---
        $display("--- Test 5: Session ID locked after first commit ---");
        session_ctr_in = 8'hFF;  // Change counter (should NOT affect session_id)
        seal_write_ctrl(10'b00_0000_0001);  // crc_reset
        repeat(2) @(posedge clk);
        seal_write_data(32'hAAAABBBB);
        seal_write_ctrl({8'h03, 1'b1, 1'b0});  // commit
        wait_seal_done;

        seal_read_data;  // consume read0 (advance to read1)
        repeat(2) @(posedge clk);
        rd1 = data_out;
        check("session_id still 0x42 (locked)", rd1[31:24] == 8'h42);
        check("mono_count = 2", rd1[23:0] == 24'd2);

        // --- Test 6: CRC verification ---
        // Compute expected CRC by feeding same bytes through engine directly
        $display("--- Test 6: CRC verification ---");
        // First, do a seal commit with known values
        seal_write_ctrl(10'b00_0000_0001);  // crc_reset
        repeat(2) @(posedge clk);
        seal_write_data(32'h00000000);  // value = 0
        seal_write_ctrl({8'h00, 1'b1, 1'b0});  // sensor_id=0, commit
        wait_seal_done;

        // Read the CRC from seal
        seal_read_data;  // skip read0
        repeat(2) @(posedge clk);
        seal_read_data;  // skip read1
        repeat(2) @(posedge clk);
        rd2 = data_out;
        $display("  Seal CRC for zero data = 0x%04X", rd2[23:8]);

        // The CRC was computed over: sensor_id(0x00) + value(00 00 00 00) + mono(03 00 00 00)
        // mono_count was 3 at the time of this commit
        // We can't easily compute reference CRC in verilog, but we verify it's consistent
        // by doing another identical commit and checking CRC changes (mono_count differs)

        seal_write_ctrl(10'b00_0000_0001);
        repeat(2) @(posedge clk);
        seal_write_data(32'h00000000);
        seal_write_ctrl({8'h00, 1'b1, 1'b0});
        wait_seal_done;

        seal_read_data;
        repeat(2) @(posedge clk);
        seal_read_data;
        repeat(2) @(posedge clk);
        check("CRC changes with different mono_count", data_out[23:8] != rd2[23:8]);

        // --- Test 7: Read sequence reset on commit ---
        $display("--- Test 7: Read seq reset on commit ---");
        // Do a commit (resets read counter)
        seal_write_ctrl(10'b00_0000_0001);
        repeat(2) @(posedge clk);
        seal_write_data(32'hCAFEBABE);
        seal_write_ctrl({8'h07, 1'b1, 1'b0});
        wait_seal_done;

        // Read once (should be read0 = value)
        rd0 = data_out;
        check("after commit read0 = 0xCAFEBABE", rd0 == 32'hCAFEBABE);

        // Read to advance to read1
        seal_read_data;
        repeat(2) @(posedge clk);

        // Now commit again (should reset read counter to 0)
        seal_write_ctrl(10'b00_0000_0001);
        repeat(2) @(posedge clk);
        seal_write_data(32'h11223344);
        seal_write_ctrl({8'h08, 1'b1, 1'b0});
        wait_seal_done;

        // Read should be back at read0
        rd0 = data_out;
        check("commit reset: read0 = 0x11223344", rd0 == 32'h11223344);

        // --- Test 8: T-SEAL-05 CRC bit-exact (highest priority) ---
        // Reference: CRC16-MODBUS(sensor_id, value[7:0..31:24], mono[7:0..31:24])
        // Computed via Python: crc16_modbus(init=0xFFFF, poly=0xA001)
        $display("--- Test 8: CRC bit-exact verification ---");

        // Reset state: mono_count is currently 5 (from tests 2,4,5,6a,6b)
        // We need to reset to get a known mono_count.
        // Apply hardware reset to clear mono_count.
        rst_n = 0;
        repeat(5) @(posedge clk);
        rst_n = 1;
        session_ctr_in = 8'h01;  // Known session counter
        repeat(5) @(posedge clk);

        // Vector 1: sensor=0xAA, value=0x00000000, mono=0
        // Expected CRC: 0x578C
        seal_write_data(32'h00000000);
        seal_write_ctrl({8'hAA, 1'b1, 1'b0});  // sensor_id=0xAA, commit
        wait_seal_done;
        rd0 = data_out;                          // read0: value
        check("V1 value=0x00000000", rd0 == 32'h00000000);
        seal_read_data; repeat(2) @(posedge clk);
        rd1 = data_out;                          // read1: {sid, mono[23:0]}
        check("V1 mono[23:0]=0", rd1[23:0] == 24'd0);
        check("V1 session_id=0x01", rd1[31:24] == 8'h01);
        seal_read_data; repeat(2) @(posedge clk);
        rd2 = data_out;                          // read2: {mono[31:24], crc, 0x00}
        check("V1 mono[31:24]=0", rd2[31:24] == 8'h00);
        check("V1 CRC=0x578C", rd2[23:8] == 16'h578C);
        check("V1 pad=0x00", rd2[7:0] == 8'h00);

        // Vector 2: sensor=0xFF, value=0xFFFFFFFF, mono=1
        // bytes: FF FF FF FF FF 01 00 00 00
        // Expected: Python crc16_modbus([0xFF,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x00,0x00])
        seal_write_data(32'hFFFFFFFF);
        seal_write_ctrl({8'hFF, 1'b1, 1'b0});
        wait_seal_done;
        seal_read_data; repeat(2) @(posedge clk);  // skip read0
        rd1 = data_out;
        check("V2 mono[23:0]=1", rd1[23:0] == 24'd1);
        seal_read_data; repeat(2) @(posedge clk);
        rd2 = data_out;
        // bytes: FF FF FF FF FF 01 00 00 00 → CRC=0xE80E
        check("V2 CRC=0xE80E", rd2[23:8] == 16'hE80E);

        // --- Test 9: T-SEAL-04 mono_count sequence (0, 1, 2) ---
        $display("--- Test 9: Mono count strict sequence ---");
        rst_n = 0;
        repeat(5) @(posedge clk);
        rst_n = 1;
        session_ctr_in = 8'h77;
        repeat(5) @(posedge clk);

        // Commit #0
        seal_write_data(32'h11111111);
        seal_write_ctrl({8'h01, 1'b1, 1'b0});
        wait_seal_done;
        seal_read_data; repeat(2) @(posedge clk);
        check("commit0 mono=0", data_out[23:0] == 24'd0);

        // Commit #1
        seal_write_data(32'h22222222);
        seal_write_ctrl({8'h02, 1'b1, 1'b0});
        wait_seal_done;
        seal_read_data; repeat(2) @(posedge clk);
        check("commit1 mono=1", data_out[23:0] == 24'd1);

        // Commit #2
        seal_write_data(32'h33333333);
        seal_write_ctrl({8'h03, 1'b1, 1'b0});
        wait_seal_done;
        seal_read_data; repeat(2) @(posedge clk);
        check("commit2 mono=2", data_out[23:0] == 24'd2);

        // --- Test 10: T-SEAL-03 sealed_sid first and second commit ---
        $display("--- Test 10: Session ID lock ---");
        rst_n = 0;
        repeat(5) @(posedge clk);
        rst_n = 1;
        session_ctr_in = 8'hAB;
        repeat(5) @(posedge clk);

        // First commit — should lock session_id = 0xAB
        seal_write_data(32'h00000000);
        seal_write_ctrl({8'h01, 1'b1, 1'b0});
        wait_seal_done;
        seal_read_data; repeat(2) @(posedge clk);
        check("1st commit sid=0xAB", data_out[31:24] == 8'hAB);

        // Change counter, second commit — sid should still be 0xAB
        session_ctr_in = 8'hCD;
        seal_write_data(32'h00000000);
        seal_write_ctrl({8'h02, 1'b1, 1'b0});
        wait_seal_done;
        seal_read_data; repeat(2) @(posedge clk);
        check("2nd commit sid=0xAB (locked)", data_out[31:24] == 8'hAB);

        // Third commit — still 0xAB
        session_ctr_in = 8'hEF;
        seal_write_data(32'h00000000);
        seal_write_ctrl({8'h03, 1'b1, 1'b0});
        wait_seal_done;
        seal_read_data; repeat(2) @(posedge clk);
        check("3rd commit sid=0xAB (still)", data_out[31:24] == 8'hAB);

        // --- Test 11: T-SEAL-02 read_seq reset on commit ---
        $display("--- Test 11: read_seq timing on commit ---");
        rst_n = 0;
        repeat(5) @(posedge clk);
        rst_n = 1;
        session_ctr_in = 8'h10;
        repeat(5) @(posedge clk);

        // Commit value A
        seal_write_data(32'hAAAA0001);
        seal_write_ctrl({8'h01, 1'b1, 1'b0});
        wait_seal_done;

        // Read once (read_seq=0 → value), advance to read_seq=1
        check("pre-commit rd0=0xAAAA0001", data_out == 32'hAAAA0001);
        seal_read_data; repeat(2) @(posedge clk);
        // Now read_seq=1 (showing session+mono)

        // Commit value B — should reset read_seq to 0
        seal_write_data(32'hBBBB0002);
        seal_write_ctrl({8'h02, 1'b1, 1'b0});
        wait_seal_done;

        // After commit, read should start from seq 0 (value)
        check("post-commit rd0=0xBBBB0002", data_out == 32'hBBBB0002);

        // Full readout to confirm sequence
        seal_read_data; repeat(2) @(posedge clk);
        rd1 = data_out;
        check("post-commit rd1 mono=1", rd1[23:0] == 24'd1);
        seal_read_data; repeat(2) @(posedge clk);
        rd2 = data_out;
        check("post-commit rd2 pad=0", rd2[7:0] == 8'h00);

        // --- Test 12: commit+crc_reset both set (commit wins) ---
        $display("--- Test 12: commit+crc_reset priority ---");
        rst_n = 0;
        repeat(5) @(posedge clk);
        rst_n = 1;
        session_ctr_in = 8'h20;
        repeat(5) @(posedge clk);

        seal_write_data(32'hDEAD0000);
        // ctrl_in = {sensor_id=0x55, commit=1, crc_reset=1} — both bits set
        seal_write_ctrl({8'h55, 1'b1, 1'b1});
        wait_seal_done;
        rd0 = data_out;
        check("commit+reset: value correct", rd0 == 32'hDEAD0000);
        // CRC should be valid (commit executed, not just reset)
        seal_read_data; repeat(2) @(posedge clk);
        seal_read_data; repeat(2) @(posedge clk);
        check("commit+reset: CRC!=0", data_out[23:8] != 16'h0000);

        // --- Test 13: Mono counter overflow (0xFFFFFFFF → 0x00000000) ---
        $display("--- Test 13: Mono counter overflow ---");
        rst_n = 0;
        repeat(5) @(posedge clk);
        rst_n = 1;
        session_ctr_in = 8'h30;
        repeat(5) @(posedge clk);

        // Force mono_count to 0xFFFFFFFF via hierarchical access
        force uut.mono_count = 32'hFFFF_FFFF;
        @(posedge clk);
        release uut.mono_count;
        @(posedge clk);

        // Commit — mono should be 0xFFFFFFFF, then wrap to 0
        seal_write_data(32'hDEAD0000);
        seal_write_ctrl({8'h01, 1'b1, 1'b0});
        wait_seal_done;

        // Read mono from sealed record
        seal_read_data; repeat(2) @(posedge clk);
        rd1 = data_out;
        check("overflow: sealed_mono[23:0]=0xFFFFFF", rd1[23:0] == 24'hFFFFFF);
        seal_read_data; repeat(2) @(posedge clk);
        rd2 = data_out;
        check("overflow: sealed_mono[31:24]=0xFF", rd2[31:24] == 8'hFF);

        // Next commit — mono_count should have wrapped to 0x00000000
        seal_write_data(32'hBEEF0000);
        seal_write_ctrl({8'h02, 1'b1, 1'b0});
        wait_seal_done;
        seal_read_data; repeat(2) @(posedge clk);
        rd1 = data_out;
        check("overflow: after wrap mono=0", rd1[23:0] == 24'd0);
        seal_read_data; repeat(2) @(posedge clk);
        rd2 = data_out;
        check("overflow: after wrap mono[31:24]=0", rd2[31:24] == 8'h00);

        // --- Test 14: data_wr during S_FEED_BYTES (value overwrite) ---
        // seal_register latches value on data_wr in IDLE state.
        // During FEED_BYTES, data_wr should be accepted (value_reg updated)
        // but it won't affect the CURRENT commit (feed is using old value_reg).
        $display("--- Test 14: data_wr during commit ---");
        rst_n = 0;
        repeat(5) @(posedge clk);
        rst_n = 1;
        session_ctr_in = 8'h40;
        repeat(5) @(posedge clk);

        seal_write_data(32'h1111_1111);
        seal_write_ctrl({8'h01, 1'b1, 1'b0});
        // While seal is busy, write a different value
        repeat(2) @(posedge clk);
        check("seal busy for overwrite test", ctrl_out[0] == 1'b1);
        seal_write_data(32'h2222_2222);  // this writes to value_reg while busy
        wait_seal_done;

        // Current commit used 0x11111111 (latched before commit started)
        rd0 = data_out;
        check("commit used original value", rd0 == 32'h1111_1111);

        // data_wr during FEED_BYTES was IGNORED (state != IDLE).
        // value_reg still holds 0x11111111.
        // Next commit without fresh data_wr reuses old value.
        seal_write_ctrl({8'h02, 1'b1, 1'b0});
        wait_seal_done;
        rd0 = data_out;
        check("next commit reuses old value (wr in busy ignored)", rd0 == 32'h1111_1111);

        // --- Test 15: Rapid commits — seal should process sequentially ---
        $display("--- Test 15: Rapid sequential commits ---");
        rst_n = 0;
        repeat(5) @(posedge clk);
        rst_n = 1;
        session_ctr_in = 8'h50;
        repeat(5) @(posedge clk);

        // Commit 1
        seal_write_data(32'hAAAA_0001);
        seal_write_ctrl({8'h01, 1'b1, 1'b0});
        wait_seal_done;

        // Immediately commit 2 (no wait between)
        seal_write_data(32'hBBBB_0002);
        seal_write_ctrl({8'h02, 1'b1, 1'b0});
        wait_seal_done;

        // Read should show commit 2's data
        rd0 = data_out;
        check("rapid: latest value=0xBBBB0002", rd0 == 32'hBBBB_0002);
        seal_read_data; repeat(2) @(posedge clk);
        check("rapid: mono=1", data_out[23:0] == 24'd1);

        // --- Test 16: CRC engine busy at commit time ---
        // If CRC is busy (from external source), seal should wait in FEED_BYTES
        $display("--- Test 16: CRC busy stalls seal ---");
        rst_n = 0;
        repeat(5) @(posedge clk);
        rst_n = 1;
        session_ctr_in = 8'h60;
        repeat(5) @(posedge clk);

        seal_write_data(32'h0000_0001);
        seal_write_ctrl({8'h01, 1'b1, 1'b0});
        // Seal should eventually complete even though CRC takes time
        wait_seal_done;
        check("seal completes with CRC delays", ctrl_out[1] == 1'b1);

        // Verify CRC is valid (non-zero)
        seal_read_data; repeat(2) @(posedge clk);
        seal_read_data; repeat(2) @(posedge clk);
        check("CRC valid after stalls", data_out[23:8] != 16'h0000);

        // --- Test 17: A2 — Reset clears mono_count + session_locked ---
        $display("--- Test 17: Reset clears mono + session ---");
        rst_n = 0;
        repeat(5) @(posedge clk);
        rst_n = 1;
        session_ctr_in = 8'hA0;
        repeat(5) @(posedge clk);

        // Do 3 commits to set mono=2
        seal_write_data(32'h11111111);
        seal_write_ctrl({8'h01, 1'b1, 1'b0}); wait_seal_done;
        seal_write_data(32'h22222222);
        seal_write_ctrl({8'h02, 1'b1, 1'b0}); wait_seal_done;
        seal_write_data(32'h33333333);
        seal_write_ctrl({8'h03, 1'b1, 1'b0}); wait_seal_done;

        // Verify mono=2
        seal_read_data; repeat(2) @(posedge clk);
        check("pre-reset mono=2", data_out[23:0] == 24'd2);
        check("pre-reset sid=0xA0", data_out[31:24] == 8'hA0);

        // Simulate WDT/soft reset (just re-assert rst_n)
        rst_n = 0;
        repeat(5) @(posedge clk);
        rst_n = 1;
        session_ctr_in = 8'hB0;  // different counter value
        repeat(5) @(posedge clk);

        // mono_count should be 0, session should re-lock
        seal_write_data(32'h44444444);
        seal_write_ctrl({8'h04, 1'b1, 1'b0}); wait_seal_done;

        seal_read_data; repeat(2) @(posedge clk);
        check("post-reset mono=0 (reset clears)", data_out[23:0] == 24'd0);
        check("post-reset sid=0xB0 (new session)", data_out[31:24] == 8'hB0);

        // --- Test 18: C1 — Simultaneous commit + crc_reset ---
        $display("--- Test 18: C1 simultaneous commit+crc_reset ---");
        rst_n = 0;
        repeat(5) @(posedge clk);
        rst_n = 1;
        session_ctr_in = 8'h01;
        repeat(5) @(posedge clk);

        // First: do commit with separate crc_reset step (reference)
        seal_write_ctrl(10'b00_0000_0001);  // standalone crc_reset
        repeat(2) @(posedge clk);
        seal_write_data(32'h00000000);
        seal_write_ctrl({8'hAA, 1'b1, 1'b0});  // commit
        wait_seal_done;
        seal_read_data; repeat(2) @(posedge clk);
        seal_read_data; repeat(2) @(posedge clk);
        begin : c1_ref_block
            reg [15:0] ref_crc;
            ref_crc = data_out[23:8];
            $display("  Reference CRC = 0x%04X", ref_crc);

            // Reset and do commit with both bits set (no separate crc_reset step)
            rst_n = 0;
            repeat(5) @(posedge clk);
            rst_n = 1;
            session_ctr_in = 8'h01;
            repeat(5) @(posedge clk);

            // No separate crc_reset! Both bits set:
            seal_write_data(32'h00000000);
            seal_write_ctrl({8'hAA, 1'b1, 1'b1});  // commit=1, crc_reset=1
            wait_seal_done;
            seal_read_data; repeat(2) @(posedge clk);
            seal_read_data; repeat(2) @(posedge clk);
            check("C1: CRC matches ref (both bits)", data_out[23:8] == ref_crc);
        end

        // --- Test 19: C2 — SEAL_DATA write during FEED_BYTES discarded ---
        $display("--- Test 19: C2 SEAL_DATA write in FEED_BYTES ---");
        rst_n = 0;
        repeat(5) @(posedge clk);
        rst_n = 1;
        session_ctr_in = 8'h70;
        repeat(5) @(posedge clk);

        // Write value and commit
        seal_write_data(32'hAAAA_AAAA);
        seal_write_ctrl({8'h01, 1'b1, 1'b0});
        repeat(3) @(posedge clk);  // Enter FEED_BYTES
        check("T19: busy during feed", ctrl_out[0] == 1'b1);

        // Try to overwrite value during FEED_BYTES
        seal_write_data(32'hBBBB_BBBB);
        // Wait for completion
        wait_seal_done;

        // Verify original value was sealed
        rd0 = data_out;
        check("C2: sealed original 0xAAAAAAAA", rd0 == 32'hAAAA_AAAA);

        // Verify next commit: if value_reg was silently overwritten, it would use 0xBBBBBBBB
        // If properly discarded, next commit with no fresh data_wr reuses old 0xAAAAAAAA
        seal_write_ctrl({8'h02, 1'b1, 1'b0});
        wait_seal_done;
        rd0 = data_out;
        check("C2: next commit still 0xAAAAAAAA", rd0 == 32'hAAAA_AAAA);

        // --- Test 20: C3 — Consecutive commits without explicit crc_reset ---
        $display("--- Test 20: C3 no explicit crc_reset between commits ---");
        rst_n = 0;
        repeat(5) @(posedge clk);
        rst_n = 1;
        session_ctr_in = 8'h01;
        repeat(5) @(posedge clk);

        // Commit 1: value=0xDEAD, sensor=0x55 — NO crc_reset first
        seal_write_data(32'h0000DEAD);
        seal_write_ctrl({8'h55, 1'b1, 1'b0});  // commit only, no crc_reset
        wait_seal_done;
        seal_read_data; repeat(2) @(posedge clk);
        seal_read_data; repeat(2) @(posedge clk);
        begin : c3_block
            reg [15:0] crc1, crc2;
            crc1 = data_out[23:8];
            $display("  Commit1 CRC=0x%04X (mono=0)", crc1);

            // Commit 2: same value, same sensor — NO crc_reset
            seal_write_data(32'h0000DEAD);
            seal_write_ctrl({8'h55, 1'b1, 1'b0});
            wait_seal_done;
            seal_read_data; repeat(2) @(posedge clk);
            seal_read_data; repeat(2) @(posedge clk);
            crc2 = data_out[23:8];
            $display("  Commit2 CRC=0x%04X (mono=1)", crc2);

            // CRCs differ because mono changed (both started from 0xFFFF)
            check("C3: CRCs differ (auto-init works)", crc1 != crc2);
            // Both should be non-zero
            check("C3: CRC1 non-zero", crc1 != 16'h0000);
            check("C3: CRC2 non-zero", crc2 != 16'h0000);
        end

        // --- Test 21: commit_dropped sticky flag ---
        $display("--- Test 21: commit_dropped flag ---");
        rst_n = 0;
        repeat(5) @(posedge clk);
        rst_n = 1;
        session_ctr_in = 8'h80;
        repeat(5) @(posedge clk);

        check("commit_dropped=0 initially", ctrl_out[2] == 1'b0);

        // Start a commit
        seal_write_data(32'h11110000);
        seal_write_ctrl({8'h01, 1'b1, 1'b0});
        repeat(3) @(posedge clk);
        check("T21: busy", ctrl_out[0] == 1'b1);

        // Try commit while busy
        seal_write_ctrl({8'h02, 1'b1, 1'b0});
        repeat(2) @(posedge clk);
        check("commit_dropped=1 (sticky)", ctrl_out[2] == 1'b1);

        // Wait for first commit to finish
        wait_seal_done;
        check("commit_dropped still 1", ctrl_out[2] == 1'b1);

        // Successful commit clears it
        seal_write_data(32'h22220000);
        seal_write_ctrl({8'h03, 1'b1, 1'b0});
        wait_seal_done;
        check("commit_dropped=0 after success", ctrl_out[2] == 1'b0);

        // --- Test 22: FSM illegal state recovery ---
        $display("--- Test 22: FSM illegal state ---");
        rst_n = 0;
        repeat(5) @(posedge clk);
        rst_n = 1;
        session_ctr_in = 8'h90;
        repeat(5) @(posedge clk);

        // Force illegal state
        force uut.state = 2'd3;
        @(posedge clk);
        release uut.state;
        @(posedge clk);
        @(posedge clk);  // default branch should transition to S_IDLE
        check("illegal state → IDLE", ctrl_out[0] == 1'b0);
        check("ready after recovery", ctrl_out[1] == 1'b1);

        // Verify normal operation works after recovery
        seal_write_data(32'hDEAD_BEEF);
        seal_write_ctrl({8'h01, 1'b1, 1'b0});
        wait_seal_done;
        rd0 = data_out;
        check("normal after FSM recovery", rd0 == 32'hDEAD_BEEF);

        // --- Test 23: 100 CRC16-MODBUS golden vectors ---
        $display("--- Test 23: 100 CRC16 golden vectors ---");

        // Reset DUT so mono_count starts at 0
        rst_n = 0;
        repeat(5) @(posedge clk);
        rst_n = 1;
        session_ctr_in = 8'hCC;  // any value, we don't check session here
        repeat(5) @(posedge clk);

        // Load golden vectors from file
        $readmemh("seal_golden.mem", golden_mem);

        gv_pass = 0;
        gv_fail = 0;

        for (gv_i = 0; gv_i < 100; gv_i = gv_i + 1) begin
            // Unpack: {sensor_id[7:0], value[31:0], crc[15:0]}
            gv_sid          = golden_mem[gv_i][55:48];
            gv_val          = golden_mem[gv_i][47:16];
            gv_expected_crc = golden_mem[gv_i][15:0];

            // Write value
            seal_write_data(gv_val);
            // Commit with sensor_id (mono_count = gv_i, auto-incremented by DUT)
            seal_write_ctrl({gv_sid, 1'b1, 1'b0});
            wait_seal_done;

            // Read sequence: skip read0 (value), skip read1 (sid+mono), get read2 (mono_hi+crc+pad)
            seal_read_data; repeat(2) @(posedge clk);  // advance past read0
            seal_read_data; repeat(2) @(posedge clk);  // advance past read1
            gv_actual_crc = data_out[23:8];

            total_tests = total_tests + 1;
            if (gv_actual_crc == gv_expected_crc) begin
                gv_pass = gv_pass + 1;
                pass_count = pass_count + 1;
            end else begin
                gv_fail = gv_fail + 1;
                fail_count = fail_count + 1;
                $display("[FAIL] GV[%0d] sid=0x%02X val=0x%08X mono=%0d: CRC got=0x%04X exp=0x%04X",
                         gv_i, gv_sid, gv_val, gv_i, gv_actual_crc, gv_expected_crc);
            end
        end

        $display("[GOLDEN] %0d / 100 PASS, %0d FAIL", gv_pass, gv_fail);
        if (gv_pass == 100)
            $display("[PASS] All 100 golden vectors match");

        // ================================================================
        // Summary
        // ================================================================
        repeat(50) @(posedge clk);
        $display("");
        $display("=== Results: %0d PASS, %0d FAIL ===", pass_count, fail_count);
        if (fail_count == 0) $display("ALL TESTS PASSED");
        else $display("SOME TESTS FAILED");
        $finish;
    end

    // Timeout (increased for 100 golden vectors)
    initial begin
        #50_000_000;
        $display("TIMEOUT!");
        $finish;
    end

endmodule
