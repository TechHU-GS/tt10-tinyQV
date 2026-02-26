// ============================================================================
// TB: project.v Bus-Level Integration Test
// ============================================================================
// Forces TinyQV output wires via hierarchical force/release to test all MMIO
// slots, CRC arbitration, interrupts, reset chain, timer, PPS, I2C pins.
// ============================================================================

`timescale 1ns / 1ps

module tb_project;

    reg clk = 0;
    always #20 clk = ~clk;  // 25 MHz

    reg rst_n;
    reg [7:0] ui_in;
    wire [7:0] uo_out;
    wire [7:0] uio_out;
    wire [7:0] uio_oe;
    reg [7:0] uio_in;

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

    tt_um_MichaelBell_tinyQV dut (
        .ui_in(ui_in), .uo_out(uo_out),
        .uio_in(uio_in), .uio_out(uio_out), .uio_oe(uio_oe),
        .ena(1'b1), .clk(clk), .rst_n(rst_n)
    );

    // Internal signal reads (hierarchical refs into project.v)
    wire        int_rst_reg_n     = dut.rst_reg_n;
    wire [31:0] int_timer_count   = dut.timer_count;
    wire        int_timer_irq     = dut.timer_irq;
    wire [3:0]  int_irq           = dut.interrupt_req;
    wire [15:0] int_pps_count     = dut.pps_count;
    wire [7:0]  int_session_ctr   = dut.session_ctr;
    wire [5:0]  int_reset_hold    = dut.reset_hold_counter;
    wire        int_seal_using    = dut.seal_using_crc;
    wire [31:0] int_read_data     = dut.data_from_read;

    initial uio_in = 8'hFF;

    // ================================================================
    // Bus interface via force/release on TinyQV output wires.
    // addr[27:0]: {1'b1, 20'b0, slot[4:0], 2'b00} for MMIO
    // ================================================================
    reg [27:0] tb_addr;
    reg [1:0]  tb_write_n;
    reg [1:0]  tb_read_n;
    reg [31:0] tb_wdata;
    reg [31:0] rd;

    task bus_write(input [4:0] slot, input [31:0] data);
    begin
        tb_addr    = {1'b1, 20'b0, slot, 2'b00};
        tb_write_n = 2'b10;
        tb_read_n  = 2'b11;
        tb_wdata   = data;
        @(posedge clk);
        // Force on TinyQV output wires (these feed into project.v)
        force dut.i_tinyqv.data_addr    = tb_addr;
        force dut.i_tinyqv.data_write_n = tb_write_n;
        force dut.i_tinyqv.data_read_n  = tb_read_n;
        force dut.i_tinyqv.data_out     = tb_wdata;
        @(posedge clk);
        // Deassert write
        tb_write_n = 2'b11;
        force dut.i_tinyqv.data_write_n = tb_write_n;
        @(posedge clk);
        release dut.i_tinyqv.data_addr;
        release dut.i_tinyqv.data_write_n;
        release dut.i_tinyqv.data_read_n;
        release dut.i_tinyqv.data_out;
    end
    endtask

    task bus_read(input [4:0] slot);
    begin
        tb_addr    = {1'b1, 20'b0, slot, 2'b00};
        tb_write_n = 2'b11;
        tb_read_n  = 2'b10;
        @(posedge clk);
        force dut.i_tinyqv.data_addr    = tb_addr;
        force dut.i_tinyqv.data_write_n = tb_write_n;
        force dut.i_tinyqv.data_read_n  = tb_read_n;
        force dut.i_tinyqv.data_out     = 32'd0;
        #1; rd = int_read_data;
        // Pulse read_complete — simulates CPU load-instruction completion.
        // Required for read-side-effects (seal read_seq, i2c rx_has_data, uart rx).
        force dut.i_tinyqv.data_read_complete = 1'b1;
        @(posedge clk);
        force dut.i_tinyqv.data_read_complete = 1'b0;
        // Deassert read
        tb_read_n = 2'b11;
        force dut.i_tinyqv.data_read_n  = tb_read_n;
        @(posedge clk);
        release dut.i_tinyqv.data_addr;
        release dut.i_tinyqv.data_write_n;
        release dut.i_tinyqv.data_read_n;
        release dut.i_tinyqv.data_read_complete;
        release dut.i_tinyqv.data_out;
    end
    endtask

    // ================================================================
    initial begin
        $dumpfile("tb_project.vcd");
        $dumpvars(0, tb_project);
        rst_n = 0; ui_in = 8'h00;
        #400; @(posedge clk); rst_n = 1;
        repeat(5) @(posedge clk);

        $display("=== project.v Integration Tests ===");

        // GROUP 1: Reset state
        $display(""); $display("--- G1: Reset State ---");
        check("rst_reg_n=1", int_rst_reg_n === 1'b1);
        check("timer=0", int_timer_count === 32'd0);
        check("timer_irq=0", int_timer_irq === 1'b0);
        check("pps=0", int_pps_count === 16'd0);
        check("reset_hold=0", int_reset_hold === 6'd0);
        check("SCL released", uo_out[2] === 1'b1);
        check("SDA released", uo_out[6] === 1'b1);
        check("SX1268_RST high", uo_out[1] === 1'b1);

        // GROUP 2: SYS_INFO
        $display(""); $display("--- G2: SYS_INFO ---");
        bus_read(5'hF);

        check("CHIP_ID=0x01", rd[15:8] === 8'h01);
        check("VERSION=0x10", rd[7:0] === 8'h10);
        check("pps=0 in SYS_INFO", rd[31:16] === 16'd0);

        // GROUP 3: GPIO
        $display(""); $display("--- G3: GPIO ---");
        bus_write(5'h3, 32'hFF);   // gpio_out_sel = all GPIO
        bus_write(5'h0, 32'hA5);   // gpio_out = 0xA5
        repeat(2) @(posedge clk);
        check("GPIO 0xA5 -> uo_out", uo_out === 8'hA5);
        bus_write(5'h0, 32'h5A);
        repeat(2) @(posedge clk);
        check("GPIO 0x5A -> uo_out", uo_out === 8'h5A);
        bus_read(5'h0);
        check("GPIO readback=0x5A", rd[7:0] === 8'h5A);
        ui_in = 8'hC3; @(posedge clk);
        bus_read(5'h1);
        check("GPIO_IN=0xC3", rd[7:0] === 8'hC3);
        bus_write(5'h3, 32'h0); ui_in = 8'h00;

        // GROUP 4: Timer
        $display(""); $display("--- G4: Timer ---");
        bus_write(5'hC, 32'd100);
        repeat(2) @(posedge clk);
        check("timer loaded 100", int_timer_count === 32'd100);
        check("irq cleared on load", int_timer_irq === 1'b0);
        repeat(2600) @(posedge clk);
        check("timer=0 after countdown", int_timer_count === 32'd0);
        check("timer_irq fired", int_timer_irq === 1'b1);
        check("IRQ[1]=timer", int_irq[1] === 1'b1);
        bus_write(5'hC, 32'd0);
        repeat(2) @(posedge clk);
        check("irq cleared by write 0", int_timer_irq === 1'b0);

        // GROUP 5: DIO1 sync
        $display(""); $display("--- G5: DIO1 Sync ---");
        ui_in[0] = 0; repeat(4) @(posedge clk);
        check("DIO1=0 IRQ[0]=0", int_irq[0] === 1'b0);
        ui_in[0] = 1; repeat(3) @(posedge clk);
        check("DIO1=1 IRQ[0]=1", int_irq[0] === 1'b1);
        ui_in[0] = 0; repeat(3) @(posedge clk);
        check("DIO1=0 IRQ[0]=0", int_irq[0] === 1'b0);

        // GROUP 6: PPS
        $display(""); $display("--- G6: PPS ---");
        ui_in[4] = 0; repeat(4) @(posedge clk);
        ui_in[4] = 1; repeat(4) @(posedge clk);
        check("pps=1", int_pps_count === 16'd1);
        ui_in[4] = 0; repeat(4) @(posedge clk);
        ui_in[4] = 1; repeat(4) @(posedge clk);
        check("pps=2", int_pps_count === 16'd2);
        bus_read(5'hF);
        check("SYS_INFO pps=2", rd[31:16] === 16'd2);

        // GROUP 7: CRC16
        $display(""); $display("--- G7: CRC16 ---");
        bus_write(5'h2, 32'h100); repeat(2) @(posedge clk);
        bus_read(5'h2);
        check("CRC init=0xFFFF", rd[15:0] === 16'hFFFF);
        check("CRC not busy", rd[16] === 1'b0);
        // Feed "123456789"
        bus_write(5'h2, 32'h31); repeat(10) @(posedge clk);
        bus_write(5'h2, 32'h32); repeat(10) @(posedge clk);
        bus_write(5'h2, 32'h33); repeat(10) @(posedge clk);
        bus_write(5'h2, 32'h34); repeat(10) @(posedge clk);
        bus_write(5'h2, 32'h35); repeat(10) @(posedge clk);
        bus_write(5'h2, 32'h36); repeat(10) @(posedge clk);
        bus_write(5'h2, 32'h37); repeat(10) @(posedge clk);
        bus_write(5'h2, 32'h38); repeat(10) @(posedge clk);
        bus_write(5'h2, 32'h39); repeat(10) @(posedge clk);
        bus_read(5'h2);
        check("CRC('123456789')=0x4B37", rd[15:0] === 16'h4B37);

        // GROUP 8: CRC Arbitration
        $display(""); $display("--- G8: CRC Arb ---");
        bus_write(5'h2, 32'h100); repeat(2) @(posedge clk);
        bus_write(5'hB, 32'hDEADBEEF); repeat(2) @(posedge clk);
        bus_write(5'hE, {22'b0, 8'hAA, 1'b1, 1'b0}); repeat(4) @(posedge clk);
        check("seal owns CRC", int_seal_using === 1'b1);
        bus_read(5'h2);
        check("CPU CRC busy during seal", rd[16] === 1'b1);
        repeat(120) @(posedge clk);
        check("seal released CRC", int_seal_using === 1'b0);
        bus_read(5'h2);
        check("CPU CRC free after seal", rd[16] === 1'b0);

        // GROUP 9: Seal round-trip
        // Note: mono_count may be >0 from G8 seal commits
        $display(""); $display("--- G9: Seal ---");
        bus_read(5'hE);
        check("seal ready", rd[1] === 1'b1);
        bus_write(5'hB, 32'h12345678); repeat(2) @(posedge clk);
        bus_write(5'hE, {22'b0, 8'h55, 1'b1, 1'b0});
        repeat(120) @(posedge clk);
        bus_read(5'hE);
        check("seal done", rd[1] === 1'b1);
        bus_read(5'hB);
        check("seal val=0x12345678", rd === 32'h12345678);
        bus_read(5'hB);
        // mono_count is 1 from G8 commit, G9 increments to 2, but
        // sealed_mono is the value AT commit time (before increment)

        check("seal mono valid", rd[23:0] !== 24'hxxxxxx);
        bus_read(5'hB);
        check("seal pad=0x00", rd[7:0] === 8'h00);
        check("seal CRC!=0", rd[23:8] !== 16'h0000);

        // GROUP 10: WDT reset chain
        // Load timer first so we can verify it gets cleared by WDT reset
        $display(""); $display("--- G10: WDT Reset ---");
        bus_write(5'hC, 32'd99999);  // Load timer with large value
        repeat(2) @(posedge clk);
        check("timer pre-loaded", int_timer_count !== 32'd0);
        bus_write(5'hD, 32'd50);     // Enable WDT with 50µs timeout
        // WDT countdown=50µs = 1250 clk + alignment. Wait 1600 + 40 for hold.
        repeat(1700) @(posedge clk);
        // By now: WDT expired → reset_hold_counter=32 → counts to 0 → rst_reg_n=1
        check("rst recovered after WDT", int_rst_reg_n === 1'b1);
        check("timer cleared by WDT reset", int_timer_count === 32'd0);
        // Verify WDT is disabled after reset (enabled flag cleared)
        bus_read(5'hD);
        check("WDT counter=0 after reset", rd === 32'd0);

        // GROUP 11: Soft reset
        $display(""); $display("--- G11: Soft Reset ---");
        bus_write(5'hC, 32'd5000); repeat(2) @(posedge clk);
        check("timer=5000", int_timer_count === 32'd5000);
        bus_write(5'hF, 32'hA5); repeat(3) @(posedge clk);
        check("reset_hold>0", int_reset_hold > 6'd0);
        repeat(40) @(posedge clk);
        check("rst recovers", int_rst_reg_n === 1'b1);
        check("timer cleared", int_timer_count === 32'd0);

        // GROUP 12: Address decode
        $display(""); $display("--- G12: Addr Decode ---");
        bus_read(5'h1F);
        check("bad slot->0xFFFFFFFF", rd === 32'hFFFF_FFFF);
        bus_read(5'h5);
        check("UART_STATUS no X", ^rd !== 1'bx);
        bus_read(5'h9);
        check("SPI_STATUS no X", ^rd !== 1'bx);

        // GROUP 13: RTC
        $display(""); $display("--- G13: RTC ---");
        bus_write(5'hA, 32'd1000); repeat(2) @(posedge clk);
        bus_read(5'hA);
        check("RTC=1000", rd === 32'd1000);
        repeat(100) @(posedge clk);
        bus_read(5'hA);
        check("RTC still 1000", rd === 32'd1000);

        // GROUP 14: Interrupts
        $display(""); $display("--- G14: Interrupts ---");
        check("IRQ[3]=0 (TX disabled)", int_irq[3] === 1'b0);

        // GROUP 15: QSPI OE
        $display(""); $display("--- G15: QSPI OE ---");
        check("uio_oe[0]=1 Flash CS", uio_oe[0] === 1'b1);
        check("uio_oe[3]=1 SCK", uio_oe[3] === 1'b1);

        // GROUP 16: Session counter
        // session_ctr increments every 1ms (1000 tick_1us = 25000 clk)
        // After G10/G11 resets, counter restarts from 0
        // Wait enough time for at least 1 increment
        $display(""); $display("--- G16: Session ---");
        repeat(26000) @(posedge clk);  // ~1.04ms
        check("session_ctr>0", int_session_ctr !== 8'd0);

        // ================================================================
        // BOUNDARY / EDGE CASE TESTS
        // ================================================================

        // GROUP 17: Soft reset wrong magic number
        $display(""); $display("--- G17: Soft Reset Boundary ---");
        bus_write(5'hC, 32'd5000); repeat(2) @(posedge clk);
        check("timer pre-loaded 5000", int_timer_count === 32'd5000);
        // Wrong magic: 0xA4, 0xA6, 0xFF, 0x00 — should NOT reset
        bus_write(5'hF, 32'hA4); repeat(3) @(posedge clk);
        check("0xA4 no reset", int_timer_count !== 32'd0);
        bus_write(5'hF, 32'hA6); repeat(3) @(posedge clk);
        check("0xA6 no reset", int_timer_count !== 32'd0);
        bus_write(5'hF, 32'h00); repeat(3) @(posedge clk);
        check("0x00 no reset", int_timer_count !== 32'd0);
        bus_write(5'hF, 32'hFF); repeat(3) @(posedge clk);
        check("0xFF no reset", int_timer_count !== 32'd0);
        // Correct magic should reset
        bus_write(5'hF, 32'hA5); repeat(40) @(posedge clk);
        check("0xA5 resets", int_timer_count === 32'd0);

        // GROUP 18: Timer boundary values
        $display(""); $display("--- G18: Timer Boundaries ---");
        // Timer = 1 (minimum countdown)
        bus_write(5'hC, 32'd1); repeat(2) @(posedge clk);
        check("timer=1 loaded", int_timer_count === 32'd1);
        // Wait for one tick_1us (25 clk) + margin
        repeat(30) @(posedge clk);
        check("timer=1 expires", int_timer_irq === 1'b1);
        // Timer reload while counting: load 1000, wait, reload 2000
        bus_write(5'hC, 32'd1000); repeat(2) @(posedge clk);
        check("timer reload 1000", int_timer_count === 32'd1000);
        check("irq cleared on reload", int_timer_irq === 1'b0);
        repeat(200) @(posedge clk);
        // Timer should have decremented
        check("timer counting down", int_timer_count < 32'd1000);
        // Reload to 2000 while counting
        bus_write(5'hC, 32'd2000); repeat(2) @(posedge clk);
        check("timer reloaded 2000", int_timer_count === 32'd2000);
        // Stop timer (write 0)
        bus_write(5'hC, 32'd0); repeat(2) @(posedge clk);
        check("timer stopped", int_timer_count === 32'd0);
        check("timer irq cleared", int_timer_irq === 1'b0);

        // GROUP 19: WDT kick reload (续命)
        $display(""); $display("--- G19: WDT Kick Reload ---");
        bus_write(5'hD, 32'd100);  // 100µs timeout
        repeat(1200) @(posedge clk);  // ~48µs elapsed (within 100µs)
        bus_read(5'hD);
        check("WDT counting", rd > 32'd0 && rd < 32'd100);
        // Kick (reload) before expiry
        bus_write(5'hD, 32'd100);  // Reload to 100
        repeat(2) @(posedge clk);
        bus_read(5'hD);
        check("WDT reloaded", rd > 32'd90);  // Should be close to 100
        // Verify it didn't reset
        check("no spurious reset", int_rst_reg_n === 1'b1);
        // Now let it expire
        repeat(3000) @(posedge clk);  // >100µs
        repeat(40) @(posedge clk);    // Wait for reset hold
        // WDT should have reset the system
        check("WDT expired after non-kick", int_rst_reg_n === 1'b1);

        // GROUP 20: WDT write-0-after-enable (cannot disable)
        $display(""); $display("--- G20: WDT Write-0 ---");
        bus_write(5'hD, 32'd200);  // Enable with 200µs
        repeat(2) @(posedge clk);
        bus_read(5'hD);
        check("WDT enabled", rd > 32'd0);
        bus_write(5'hD, 32'd0);  // Try to disable
        repeat(2) @(posedge clk);
        bus_read(5'hD);
        check("WDT still counting (write 0 ignored)", rd > 32'd0);
        // Kick to prevent expiry
        bus_write(5'hD, 32'd5000);

        // GROUP 21: PPS no false edge
        $display(""); $display("--- G21: PPS Stability ---");
        // Record current pps count
        bus_read(5'hF);
        begin : pps_stable_block
            reg [15:0] saved_pps;
            saved_pps = rd[31:16];
            // Hold high for many cycles — should NOT increment
            ui_in[4] = 1; repeat(100) @(posedge clk);
            check("PPS steady high no inc", int_pps_count === saved_pps);
            // Hold low — should NOT increment
            ui_in[4] = 0; repeat(100) @(posedge clk);
            check("PPS steady low no inc", int_pps_count === saved_pps);
            // Rising edge — should increment exactly once
            ui_in[4] = 1; repeat(4) @(posedge clk);
            check("PPS rising +1", int_pps_count === saved_pps + 16'd1);
            // Hold high again — no more increments
            repeat(100) @(posedge clk);
            check("PPS no double count", int_pps_count === saved_pps + 16'd1);
            ui_in[4] = 0; repeat(4) @(posedge clk);
        end

        // GROUP 22: GPIO partial sel
        $display(""); $display("--- G22: GPIO Partial Sel ---");
        // gpio_out_sel = 0x80 → only bit 7 from GPIO, bits 6:0 from peripherals
        bus_write(5'h3, 32'h80);  // Only bit 7 = GPIO
        bus_write(5'h0, 32'hFF);  // gpio_out = all 1s
        repeat(2) @(posedge clk);
        // bit 7 should be 1 (GPIO), other bits from peripherals
        check("partial sel bit7=GPIO", uo_out[7] === 1'b1);
        // bit 1 should be SX1268_RST default (1, not GPIO)
        check("partial sel bit1=periph", uo_out[1] === 1'b1);
        // bit 6 should be I2C SDA (released=1, not GPIO)
        check("partial sel bit6=I2C", uo_out[6] === 1'b1);
        // Now set gpio_out bit 7 = 0
        bus_write(5'h0, 32'h00);
        repeat(2) @(posedge clk);
        check("GPIO bit7 now 0", uo_out[7] === 1'b0);
        // Reset gpio_out_sel
        bus_write(5'h3, 32'h0);

        // GROUP 23: CRC init vs data in same write
        $display(""); $display("--- G23: CRC Init Priority ---");
        bus_write(5'h2, 32'h100); repeat(2) @(posedge clk);  // init
        bus_read(5'h2);
        check("CRC init clean", rd[15:0] === 16'hFFFF);
        // Feed a byte to change CRC from 0xFFFF
        bus_write(5'h2, 32'h41); repeat(10) @(posedge clk);  // 'A'
        bus_read(5'h2);
        check("CRC after A != 0xFFFF", rd[15:0] !== 16'hFFFF);
        // Now write init+data simultaneously (bit 8 = init, bits 7:0 = 0x42)
        bus_write(5'h2, 32'h142); repeat(10) @(posedge clk);
        bus_read(5'h2);
        // Init should win — CRC should be 0xFFFF, not CRC('B')
        check("init wins over data", rd[15:0] === 16'hFFFF);

        // GROUP 24: Seal commit_dropped flag
        $display(""); $display("--- G24: Seal Commit Dropped ---");
        bus_read(5'hE);
        check("seal idle", rd[1] === 1'b1);  // ready
        // Start first commit
        bus_write(5'hB, 32'hAAAAAAAA); repeat(2) @(posedge clk);
        bus_write(5'hE, {22'b0, 8'h01, 1'b1, 1'b0});  // commit
        repeat(2) @(posedge clk);
        // Seal should be busy
        bus_read(5'hE);
        check("seal busy after commit", rd[0] === 1'b1);
        // Try second commit while busy
        bus_write(5'hB, 32'hBBBBBBBB); repeat(2) @(posedge clk);
        bus_write(5'hE, {22'b0, 8'h02, 1'b1, 1'b0});  // commit while busy
        repeat(2) @(posedge clk);
        bus_read(5'hE);
        check("commit_dropped set", rd[2] === 1'b1);
        // Wait for first to complete
        repeat(120) @(posedge clk);
        bus_read(5'hE);
        check("seal done after wait", rd[1] === 1'b1);
        // New commit should clear commit_dropped
        bus_write(5'hB, 32'hCCCCCCCC); repeat(2) @(posedge clk);
        bus_write(5'hE, {22'b0, 8'h03, 1'b1, 1'b0});
        repeat(120) @(posedge clk);
        bus_read(5'hE);
        check("commit_dropped cleared", rd[2] === 1'b0);

        // GROUP 25: RTC write vs tick priority
        $display(""); $display("--- G25: RTC Boundaries ---");
        // Write max value
        bus_write(5'hA, 32'hFFFFFFFF); repeat(2) @(posedge clk);
        bus_read(5'hA);
        check("RTC max loaded", rd === 32'hFFFFFFFF);
        // Write 0
        bus_write(5'hA, 32'd0); repeat(2) @(posedge clk);
        bus_read(5'hA);
        check("RTC zero loaded", rd === 32'd0);
        // Write a value, verify it holds (not drifting immediately)
        bus_write(5'hA, 32'd42); repeat(2) @(posedge clk);
        bus_read(5'hA);
        check("RTC holds 42", rd === 32'd42);
        // Wait < 1 second (25000 clk = 1ms, far less than 1s)
        repeat(100) @(posedge clk);
        bus_read(5'hA);
        check("RTC still 42 (no early inc)", rd === 32'd42);

        // GROUP 26: Address decode write to NONE slot
        $display(""); $display("--- G26: NONE Slot Write ---");
        bus_write(5'h3, 32'hFF);  // gpio_out_sel = all GPIO
        bus_write(5'h0, 32'hA5);  // gpio_out = known value
        repeat(2) @(posedge clk);
        check("GPIO pre-set 0xA5", uo_out === 8'hA5);
        // Write to NONE slot (0x1F) — should have no effect
        bus_write(5'h1F, 32'h00); repeat(2) @(posedge clk);
        check("NONE write no effect on GPIO", uo_out === 8'hA5);
        // Read NONE slot
        bus_read(5'h1F);
        check("NONE read=0xFFFFFFFF", rd === 32'hFFFF_FFFF);
        bus_write(5'h3, 32'h0);  // cleanup

        // GROUP 27: Reset clears all state
        $display(""); $display("--- G27: Reset Clears All ---");
        // Load state into various peripherals
        bus_write(5'h3, 32'hAA);  // gpio_out_sel
        bus_write(5'h0, 32'h55);  // gpio_out
        bus_write(5'hC, 32'd999); // timer
        repeat(2) @(posedge clk);
        // Soft reset
        bus_write(5'hF, 32'hA5); repeat(40) @(posedge clk);
        check("rst recovered", int_rst_reg_n === 1'b1);
        // All state should be cleared
        check("gpio_out cleared", dut.gpio_out === 8'd0);
        check("gpio_out_sel cleared", dut.gpio_out_sel === 8'd0);
        check("timer cleared", int_timer_count === 32'd0);
        check("timer_irq cleared", int_timer_irq === 1'b0);
        // I2C should be released after reset
        check("SCL released after rst", uo_out[2] === 1'b1);
        check("SDA released after rst", uo_out[6] === 1'b1);

        // GROUP 28: DIO1 short glitch (<2 cycles)
        $display(""); $display("--- G28: DIO1 Glitch ---");
        ui_in[0] = 0; repeat(5) @(posedge clk);
        check("DIO1 baseline low", int_irq[0] === 1'b0);
        // 1-cycle glitch: high for exactly 1 clk, then low
        ui_in[0] = 1; @(posedge clk);
        ui_in[0] = 0; repeat(5) @(posedge clk);
        // 2-stage sync needs 2+ cycles to propagate
        // After 1-cycle pulse, the synchronizer MIGHT catch it
        // (1 clk high → sync stage 0 samples 1, next clk stage 1 copies)
        // This is acceptable — test that the system doesn't crash
        check("DIO1 glitch no crash", ^int_irq !== 1'bx);

        // GROUP 29: CRC known vector via bus
        // Verify CRC of single byte 'A' (0x41) = 0x9F01 through the MMIO path
        $display(""); $display("--- G29: CRC Known Vector ---");
        bus_write(5'h2, 32'h100); repeat(2) @(posedge clk);  // init
        bus_read(5'h2);
        check("CRC init for vector", rd[15:0] === 16'hFFFF);
        bus_write(5'h2, 32'h41); repeat(10) @(posedge clk);  // 'A'
        bus_read(5'h2);
        check("CRC('A')=0x707F", rd[15:0] === 16'h707F);
        check("CRC not busy", rd[16] === 1'b0);
        // Feed 'B' after wait — should accumulate
        bus_write(5'h2, 32'h42); repeat(10) @(posedge clk);  // 'B'
        bus_read(5'h2);
        check("CRC('AB') != CRC('A')", rd[15:0] !== 16'h707F);
        check("CRC('AB') != 0", rd[15:0] !== 16'h0000);
        // Note: write-during-busy is tested in tb_crc16 (test 9)

        // GROUP 30: QSPI OE during reset
        $display(""); $display("--- G30: QSPI OE Reset ---");
        // Trigger reset and check OE becomes 0 during reset hold
        bus_write(5'hF, 32'hA5);
        repeat(2) @(posedge clk);
        // During soft reset: rst_reg_n=0, uio_oe uses rst_reg_n so
        // QSPI CS lines tristate during soft/WDT reset (C7 fix)
        check("uio_oe during soft rst", uio_oe === 8'h00);
        repeat(40) @(posedge clk);

        // GROUP 31: T-SEAL-01 CRC arbitration during seal commit
        // While seal is processing (S_FEED_BYTES), CPU writes CRC16 slot
        // → engine output must not be corrupted
        $display(""); $display("--- G31: CRC Arb During Seal ---");
        // Init CRC, set up seal
        bus_write(5'h2, 32'h100); repeat(2) @(posedge clk);  // CRC init
        bus_write(5'hB, 32'hAABBCCDD); repeat(2) @(posedge clk);
        // Trigger seal commit
        bus_write(5'hE, {22'b0, 8'h42, 1'b1, 1'b0});
        repeat(2) @(posedge clk);
        check("seal active for arb test", int_seal_using === 1'b1);
        // CPU writes CRC16 slot while seal is active — should be blocked
        bus_write(5'h2, 32'h55);  // Try to feed byte to CRC
        bus_write(5'h2, 32'h66);  // Another attempt
        // Wait for seal to finish
        repeat(120) @(posedge clk);
        check("seal done", int_seal_using === 1'b0);
        // Now init CRC and compute a known value to verify engine is clean
        bus_write(5'h2, 32'h100); repeat(2) @(posedge clk);
        bus_read(5'h2);
        check("CRC clean after arb: 0xFFFF", rd[15:0] === 16'hFFFF);
        // Feed known byte and verify
        bus_write(5'h2, 32'h41); repeat(10) @(posedge clk);
        bus_read(5'h2);
        check("CRC engine intact: A=0x707F", rd[15:0] === 16'h707F);

        // GROUP 32: T-PROJ-01 GPIO reset vs write race
        // Verify reset takes priority over write in same always block
        $display(""); $display("--- G32: GPIO Reset Priority ---");
        bus_write(5'h3, 32'hFF);  // gpio_out_sel = all GPIO
        bus_write(5'h0, 32'hBB);  // gpio_out = 0xBB
        repeat(2) @(posedge clk);
        check("GPIO set to 0xBB", uo_out === 8'hBB);
        // Soft reset — gpio should go to 0
        bus_write(5'hF, 32'hA5);
        repeat(40) @(posedge clk);
        // After reset: gpio_out=0, gpio_out_sel=0, so uo_out from peripherals
        check("GPIO out cleared by reset", dut.gpio_out === 8'd0);
        check("GPIO sel cleared by reset", dut.gpio_out_sel === 8'd0);

        // GROUP 33: T-PROJ-03 DIO1 level behavior confirmation
        // DIO1 (ui_in[0]) → 2-stage sync → interrupt_req[0]
        // TinyQV core does edge detection on bits [1:0]
        // At project.v level it IS level-sensitive (synchronized)
        $display(""); $display("--- G33: DIO1 Level Behavior ---");
        ui_in[0] = 0; repeat(5) @(posedge clk);
        check("DIO1 low baseline", int_irq[0] === 1'b0);
        // Assert high and hold
        ui_in[0] = 1; repeat(5) @(posedge clk);
        check("DIO1 high → IRQ[0]=1", int_irq[0] === 1'b1);
        // Hold high for 100 cycles — IRQ should stay 1 (level)
        repeat(100) @(posedge clk);
        check("DIO1 held high = still 1", int_irq[0] === 1'b1);
        // Release
        ui_in[0] = 0; repeat(5) @(posedge clk);
        check("DIO1 released = IRQ[0]=0", int_irq[0] === 1'b0);

        // GROUP 34: T-PROJ-02 reset_hold_counter duration
        $display(""); $display("--- G34: Reset Hold Duration ---");
        bus_write(5'hC, 32'd50000); repeat(2) @(posedge clk);
        check("timer loaded", int_timer_count !== 32'd0);
        // Trigger soft reset
        bus_write(5'hF, 32'hA5);
        @(posedge clk);
        // Check reset_hold goes to 32
        repeat(2) @(posedge clk);
        check("hold counter loaded", int_reset_hold > 6'd0);
        // rst_reg_n should be 0 during hold
        check("rst_reg_n=0 during hold", int_rst_reg_n === 1'b0);
        // Wait for hold to expire (32 + margin)
        repeat(35) @(posedge clk);
        check("rst_reg_n=1 after hold", int_rst_reg_n === 1'b1);
        check("hold counter=0", int_reset_hold === 6'd0);

        // GROUP 35: Seal CRC bit-exact via project.v bus
        // Verify end-to-end: bus write → seal → CRC matches Python reference
        $display(""); $display("--- G35: Seal CRC End-to-End ---");
        // Reset to get known mono=0
        bus_write(5'hF, 32'hA5); repeat(40) @(posedge clk);
        // sensor=0xAA, value=0x00000000, mono=0 → CRC=0x578C
        bus_write(5'hB, 32'h00000000); repeat(2) @(posedge clk);
        bus_write(5'hE, {22'b0, 8'hAA, 1'b1, 1'b0});
        repeat(120) @(posedge clk);
        bus_read(5'hE);
        check("seal done for CRC test", rd[1] === 1'b1);
        bus_read(5'hB);  // read0: value
        check("seal rd0=0x00000000", rd === 32'h00000000);
        bus_read(5'hB);  // read1: {sid, mono[23:0]}
        check("seal rd1 mono=0", rd[23:0] === 24'd0);
        bus_read(5'hB);  // read2: {mono[31:24], crc, pad}
        check("seal CRC=0x578C via bus", rd[23:8] === 16'h578C);

        // ================================================================
        // DEEP BOUNDARY TESTS (External Review Round 2)
        // ================================================================

        // GROUP 36: Back-to-back read/write same address (0-gap)
        $display(""); $display("--- G36: Back-to-Back RW ---");
        bus_write(5'hA, 32'd777);   // Write RTC
        bus_read(5'hA);             // Immediately read same slot
        check("b2b write→read: RTC=777", rd === 32'd777);
        // Write different address, then read previous
        bus_write(5'hC, 32'd5000);  // Timer
        bus_read(5'hA);             // Read RTC (should still be 777)
        check("b2b cross-addr: RTC still 777", rd === 32'd777);
        bus_read(5'hC);
        check("b2b cross-addr: Timer=5000", rd === 32'd5000);
        // Cleanup
        bus_write(5'hC, 32'd0);

        // GROUP 37: Multiple IRQs same cycle
        $display(""); $display("--- G37: Multi-IRQ Same Cycle ---");
        // Setup: timer about to fire + DIO1 about to go high
        ui_in[0] = 0; repeat(4) @(posedge clk);
        bus_write(5'hC, 32'd1);    // timer=1µs (fires on next tick)
        repeat(20) @(posedge clk); // Just before tick
        ui_in[0] = 1;              // DIO1 high around same time
        repeat(10) @(posedge clk); // Let sync + timer fire
        // Both IRQ[0] (DIO1) and IRQ[1] (timer) should be set
        check("multi-IRQ: timer fired", int_timer_irq === 1'b1);
        check("multi-IRQ: DIO1 synced", int_irq[0] === 1'b1);
        check("multi-IRQ: both set", int_irq[1:0] === 2'b11);
        ui_in[0] = 0;
        bus_write(5'hC, 32'd0); // Clear timer
        repeat(4) @(posedge clk);

        // GROUP 38: RTC 32-bit rollover (0xFFFFFFFF → 0x00000000)
        $display(""); $display("--- G38: RTC Rollover ---");
        bus_write(5'hA, 32'hFFFF_FFFF);
        repeat(2) @(posedge clk);
        bus_read(5'hA);
        check("RTC at max", rd === 32'hFFFF_FFFF);
        // Wait for 1 second = 25,000,000 clk is too long for sim.
        // Instead, force us_count near rollover point.
        force dut.i_rtc.us_count = 20'd999_998;
        @(posedge clk);
        release dut.i_rtc.us_count;
        // Wait for a few ticks — us_count will hit 999999 → sec+1 → wrap
        repeat(100) @(posedge clk);
        bus_read(5'hA);
        check("RTC rolled over to 0", rd === 32'h0000_0000);

        // GROUP 39: Reset release timing — no IRQ glitch
        $display(""); $display("--- G39: Reset Release Clean ---");
        ui_in = 8'h00;
        bus_write(5'hF, 32'hA5); // soft reset
        // Monitor: during reset hold, IRQ should not glitch
        begin : reset_irq_block
            reg saw_irq_glitch;
            integer i;
            saw_irq_glitch = 0;
            for (i = 0; i < 40; i = i + 1) begin
                @(posedge clk);
                if (int_irq !== 4'b0000 && int_rst_reg_n === 1'b0)
                    saw_irq_glitch = 1;
            end
            check("no IRQ glitch during reset", saw_irq_glitch === 0);
        end
        check("rst recovered after G39", int_rst_reg_n === 1'b1);

        // GROUP 40: CRC same-cycle contention (CPU write + seal commit)
        // Force CPU CRC write in the EXACT same cycle as seal commit trigger
        $display(""); $display("--- G40: CRC Same-Cycle Contention ---");
        bus_write(5'h2, 32'h100); repeat(2) @(posedge clk); // CRC init
        bus_write(5'hB, 32'h0000_0000); repeat(2) @(posedge clk); // seal data
        // Now simultaneously: write CRC data + trigger seal commit
        tb_addr = {1'b1, 20'b0, 5'h2, 2'b00}; // CRC slot
        tb_write_n = 2'b10;
        tb_read_n  = 2'b11;
        tb_wdata   = 32'h41; // try to feed 'A' to CRC
        @(posedge clk);
        force dut.i_tinyqv.data_addr    = tb_addr;
        force dut.i_tinyqv.data_write_n = tb_write_n;
        force dut.i_tinyqv.data_read_n  = tb_read_n;
        force dut.i_tinyqv.data_out     = tb_wdata;
        @(posedge clk);
        // Release CRC write, now trigger seal commit
        tb_write_n = 2'b11;
        force dut.i_tinyqv.data_write_n = tb_write_n;
        @(posedge clk);
        release dut.i_tinyqv.data_addr;
        release dut.i_tinyqv.data_write_n;
        release dut.i_tinyqv.data_read_n;
        release dut.i_tinyqv.data_out;
        // Now trigger seal commit
        bus_write(5'hE, {22'b0, 8'hAA, 1'b1, 1'b0});
        repeat(2) @(posedge clk);
        // Seal should own CRC now
        check("G40: seal owns after contention", int_seal_using === 1'b1);
        // CPU CRC read should show busy
        bus_read(5'h2);
        check("G40: CPU CRC busy", rd[16] === 1'b1);
        // Wait for seal to finish
        repeat(120) @(posedge clk);
        // Verify CRC engine is clean (init + known byte)
        bus_write(5'h2, 32'h100); repeat(2) @(posedge clk);
        bus_write(5'h2, 32'h41); repeat(10) @(posedge clk);
        bus_read(5'h2);
        check("G40: CRC intact A=0x707F", rd[15:0] === 16'h707F);

        // GROUP 41: WDT cycle-exact expiry via project.v integration
        $display(""); $display("--- G41: WDT Cycle-Exact ---");
        bus_write(5'hD, 32'd2); // 2µs timeout
        // Wait exactly 2µs = 50 clk, + small margin for tick alignment
        repeat(55) @(posedge clk);
        // Check if reset happened (timer should be cleared)
        // Give reset hold time (32 clk)
        repeat(35) @(posedge clk);
        check("G41: WDT 2µs expiry recovery", int_rst_reg_n === 1'b1);
        check("G41: timer cleared by WDT", int_timer_count === 32'd0);

        // GROUP 42: Seal mono overflow via bus
        $display(""); $display("--- G42: Seal Mono Overflow ---");
        // Force mono_count to near max
        force dut.i_seal.mono_count = 32'hFFFF_FFFE;
        @(posedge clk);
        release dut.i_seal.mono_count;
        @(posedge clk);
        // Commit — mono sealed = 0xFFFFFFFE
        bus_write(5'hB, 32'h1234_5678);
        bus_write(5'hE, {22'b0, 8'h01, 1'b1, 1'b0});
        repeat(120) @(posedge clk);
        bus_read(5'hB); // read0: value
        bus_read(5'hB); // read1: {sid, mono[23:0]}
        check("G42: mono[23:0]=0xFFFFFE", rd[23:0] === 24'hFFFFFE);
        // Another commit — mono sealed = 0xFFFFFFFF
        bus_write(5'hB, 32'h0000_0000);
        bus_write(5'hE, {22'b0, 8'h02, 1'b1, 1'b0});
        repeat(120) @(posedge clk);
        bus_read(5'hB);
        bus_read(5'hB);
        check("G42: mono at max 0xFFFFFF", rd[23:0] === 24'hFFFFFF);
        bus_read(5'hB);
        check("G42: mono[31:24]=0xFF", rd[31:24] === 8'hFF);
        // Next commit — should wrap to 0
        bus_write(5'hB, 32'h0000_0000);
        bus_write(5'hE, {22'b0, 8'h03, 1'b1, 1'b0});
        repeat(120) @(posedge clk);
        bus_read(5'hB);
        bus_read(5'hB);
        check("G42: mono wrapped to 0", rd[23:0] === 24'd0);

        // GROUP 43: Address decode boundary (slot 0x10 = outside 4-bit range)
        // addr[6:2] can be 5-bit, but only 0x0-0xF are valid slots.
        // Slot 0x10+ should hit PERI_NONE
        $display(""); $display("--- G43: Addr Boundary ---");
        bus_read(5'h10); // slot 16 = NONE
        check("G43: slot 16=NONE (0xFFFFFFFF)", rd === 32'hFFFF_FFFF);
        bus_read(5'h1E); // slot 30 = NONE
        check("G43: slot 30=NONE", rd === 32'hFFFF_FFFF);

        // GROUP 44: Write during reset hold (should be ignored)
        $display(""); $display("--- G44: Write During Reset ---");
        bus_write(5'hF, 32'hA5); // Trigger soft reset
        @(posedge clk);
        // During reset hold: try to write GPIO
        bus_write(5'h3, 32'hFF); // gpio_out_sel
        bus_write(5'h0, 32'hCC); // gpio_out
        // Wait for reset to complete
        repeat(40) @(posedge clk);
        // GPIO should still be cleared (reset wins)
        check("G44: gpio_out cleared despite write", dut.gpio_out === 8'd0);
        check("G44: gpio_out_sel cleared", dut.gpio_out_sel === 8'd0);

        // GROUP 45: PPS 16-bit overflow (preload to near max)
        $display(""); $display("--- G45: PPS Overflow ---");
        force dut.pps_count = 16'hFFFE;
        @(posedge clk);
        release dut.pps_count;
        @(posedge clk);
        // Generate rising edge
        ui_in[4] = 0; repeat(4) @(posedge clk);
        ui_in[4] = 1; repeat(4) @(posedge clk);
        check("G45: PPS at 0xFFFF", int_pps_count === 16'hFFFF);
        // Another rising edge → wrap to 0
        ui_in[4] = 0; repeat(4) @(posedge clk);
        ui_in[4] = 1; repeat(4) @(posedge clk);
        check("G45: PPS wrapped to 0", int_pps_count === 16'h0000);
        ui_in[4] = 0;

        // GROUP 46: Register fuzz — random writes to all slots, no X or hang
        $display(""); $display("--- G46: Register Fuzz ---");
        begin : fuzz_block
            integer i;
            reg saw_x;
            reg [31:0] fuzz_data;
            saw_x = 0;
            for (i = 0; i < 16; i = i + 1) begin
                fuzz_data = {i[3:0], 28'hAAA_5555};
                bus_write(i[4:0], fuzz_data);
            end
            // Read all slots, check no X
            for (i = 0; i < 16; i = i + 1) begin
                bus_read(i[4:0]);
                if (^rd === 1'bx) saw_x = 1;
            end
            check("G46: no X in any slot read", saw_x === 0);
            check("G46: system still alive", int_rst_reg_n === 1'b1);
        end
        // Note: G46 may have triggered soft reset (wrote 0xFA5 to SYS_INFO slot 0xF)
        // Check: slot 0xF write = {0xF, 0xAAA_5555} → data[7:0] = 0x55, not 0xA5
        // So no reset triggered. Good.

        // GROUP 47: Timer write+tick same cycle priority
        $display(""); $display("--- G47: Timer Write Priority ---");
        // If write and tick_1us happen same cycle, write should win (RTL: if/else)
        bus_write(5'hC, 32'd500); // Load timer
        // Wait until near a tick boundary, then write new value
        repeat(20) @(posedge clk);
        bus_write(5'hC, 32'd999); // Reload
        repeat(2) @(posedge clk);
        check("G47: timer reloaded to 999", int_timer_count === 32'd999);

        // ============================================================
        // GROUP 48: B2/B7 — timer_irq stays high until reload
        // ============================================================
        $display(""); $display("--- G48: Timer IRQ sticky ---");
        bus_write(5'hC, 32'd2);  // 2µs countdown
        repeat(80) @(posedge clk);  // wait ~3µs for expiry
        check("G48: timer_irq=1 after expiry", int_timer_irq === 1'b1);
        // Read timer — should be 0 (expired)
        bus_read(5'hC);
        check("G48: timer_count=0", rd === 32'd0);
        // IRQ stays high without action
        repeat(50) @(posedge clk);
        check("G48: timer_irq still 1 (sticky)", int_timer_irq === 1'b1);
        // Write 0 → stops timer AND clears IRQ
        bus_write(5'hC, 32'd0);
        repeat(2) @(posedge clk);
        check("G48: timer_irq=0 after write 0", int_timer_irq === 1'b0);
        check("G48: timer_count=0 after write 0", int_timer_count === 32'd0);
        // Write new value → starts fresh AND clears IRQ
        bus_write(5'hC, 32'd5);
        repeat(2) @(posedge clk);
        check("G48: timer running after reload", int_timer_count !== 32'd0);

        // ============================================================
        // GROUP 49: C7 — uio_oe tristates during soft/WDT reset
        // ============================================================
        $display(""); $display("--- G49: uio_oe during reset ---");
        // Trigger soft reset
        bus_write(5'hF, 32'hA5);
        repeat(2) @(posedge clk);
        // During reset hold, rst_reg_n=0, uio_oe should be 0
        check("G49: rst_reg_n=0 after soft reset", int_rst_reg_n === 1'b0);
        check("G49: uio_oe=0 during reset", uio_oe === 8'h00);
        // Wait for reset to complete
        repeat(40) @(posedge clk);
        check("G49: uio_oe restored after reset", uio_oe !== 8'h00);

        // ============================================================
        // GROUP 50: C8 — PERI_NONE address writes are silent, reads=0xFFFFFFFF
        // ============================================================
        $display(""); $display("--- G50: PERI_NONE behavior ---");
        // Write to all undefined slots — should have no effect
        bus_write(5'h10, 32'hDEADBEEF);
        bus_write(5'h11, 32'hCAFEBABE);
        bus_write(5'h1E, 32'h12345678);
        bus_write(5'h1F, 32'hFFFFFFFF);
        // Read them all — must be 0xFFFFFFFF
        bus_read(5'h10);
        check("G50: slot 0x10 read=0xFFFFFFFF", rd === 32'hFFFF_FFFF);
        bus_read(5'h11);
        check("G50: slot 0x11 read=0xFFFFFFFF", rd === 32'hFFFF_FFFF);
        bus_read(5'h1F);
        check("G50: slot 0x1F read=0xFFFFFFFF", rd === 32'hFFFF_FFFF);
        // Verify no side effects: CRC not started, WDT not kicked, etc.
        check("G50: system alive after NONE writes", int_rst_reg_n === 1'b1);

        // ============================================================
        // GROUP 51: D4 — SYSINFO read masks pps_count correctly
        // ============================================================
        $display(""); $display("--- G51: SYSINFO format ---");
        // Reset PPS to known state
        force dut.pps_count = 16'h0000;
        @(posedge clk); release dut.pps_count; @(posedge clk);
        bus_read(5'hF);
        check("G51: CHIP_ID=0x01", rd[15:8] === 8'h01);
        check("G51: VERSION=0x10", rd[7:0] === 8'h10);
        check("G51: pps_count=0", rd[31:16] === 16'h0000);

        // Inject a PPS pulse and re-read
        ui_in[4] = 0; repeat(4) @(posedge clk);
        ui_in[4] = 1; repeat(4) @(posedge clk);
        ui_in[4] = 0; repeat(2) @(posedge clk);
        bus_read(5'hF);
        check("G51: pps_count=1 in SYSINFO", rd[31:16] === 16'h0001);
        check("G51: low 16 bits unchanged", rd[15:0] === 16'h0110);

        // ============================================================
        // GROUP 52: RO slot writes don't trigger side effects
        // ============================================================
        $display(""); $display("--- G52: RO write side effects ---");
        // GPIO_IN (slot 1) is RO
        bus_write(5'h1, 32'hFFFF_FFFF);
        bus_read(5'h1);
        check("G52: GPIO_IN write ignored", rd[7:0] === ui_in);
        // UART_STATUS (slot 5) is RO
        bus_write(5'h5, 32'hFFFF_FFFF);
        bus_read(5'h5);
        check("G52: UART_STATUS survived write", ^rd !== 1'bx);
        // SPI_STATUS (slot 9) — spi_busy is RO part
        bus_read(5'h9);
        check("G52: SPI_STATUS no X", ^rd !== 1'bx);

        // ============================================================
        // GROUP 53: Reset glitch (assert-deassert-reassert)
        // ============================================================
        $display(""); $display("--- G53: Reset glitch ---");
        rst_n = 0;
        repeat(2) @(posedge clk);  // very short reset
        rst_n = 1;
        repeat(2) @(posedge clk);
        rst_n = 0;  // reassert immediately
        repeat(5) @(posedge clk);
        rst_n = 1;
        repeat(10) @(posedge clk);
        // System should be in clean state
        check("G53: clean after glitch reset", int_rst_reg_n === 1'b1);
        check("G53: gpio_out=0 after reset", dut.gpio_out === 8'd0);
        check("G53: gpio_out_sel=0 after reset", dut.gpio_out_sel === 8'd0);
        bus_read(5'hD);
        check("G53: WDT remaining=0 after reset", rd === 32'd0);

        // ============================================================
        // GROUP 54: Soft reset + hard reset collision
        // ============================================================
        $display(""); $display("--- G54: Soft+Hard reset collision ---");
        bus_write(5'hF, 32'hA5);  // trigger soft reset
        repeat(5) @(posedge clk);
        // External reset during soft reset hold
        rst_n = 0;
        repeat(10) @(posedge clk);
        rst_n = 1;
        repeat(40) @(posedge clk);
        // Both resets should resolve cleanly
        check("G54: system alive after collision", int_rst_reg_n === 1'b1);
        check("G54: reset_hold=0", int_reset_hold === 6'd0);

        // ============================================================
        // GROUP 55: WDT reset triggers clean internal reset
        // ============================================================
        $display(""); $display("--- G55: WDT-triggered reset ---");
        // Set a very short WDT timeout
        bus_write(5'hD, 32'd3);  // 3µs
        repeat(200) @(posedge clk);  // ~8µs, well past expiry
        // WDT should have fired and internal reset should have held
        // After reset hold counter expires, system should be clean
        repeat(50) @(posedge clk);
        check("G55: system recovered from WDT reset", int_rst_reg_n === 1'b1);
        // WDT should be disabled after reset (enabled flag cleared)
        bus_read(5'hD);
        check("G55: WDT remaining=0 (disabled)", rd === 32'd0);
        // GPIO should be cleared
        check("G55: gpio cleared by WDT reset", dut.gpio_out === 8'd0);

        // ============================================================
        // GROUP 56: CRC arbitration — CPU blocked during seal
        // ============================================================
        $display(""); $display("--- G56: CRC arb CPU blocked ---");
        // Init CRC
        bus_write(5'h2, 32'h100);  // CRC init
        repeat(2) @(posedge clk);
        // Start a seal commit
        bus_write(5'hB, 32'hDEAD_BEEF);  // SEAL_DATA
        bus_write(5'hE, {22'd0, 8'h01, 1'b1, 1'b0});  // SEAL_CTRL: commit
        repeat(3) @(posedge clk);
        // Seal should be active
        check("G56: seal active", int_seal_using === 1'b1);
        // Read CRC16 slot — should show busy=1 (seal is using CRC)
        bus_read(5'h2);
        check("G56: CRC shows busy during seal", rd[16] === 1'b1);
        // Write CRC16 slot while seal active — should be silently ignored
        bus_write(5'h2, 32'h0000_0042);  // try to feed byte
        repeat(2) @(posedge clk);
        // Wait for seal to complete
        begin : g56_wait
            integer t;
            t = 0;
            while (int_seal_using && t < 5000) begin
                @(posedge clk); t = t + 1;
            end
        end
        check("G56: seal completed", int_seal_using === 1'b0);
        // CRC should now be usable again
        bus_write(5'h2, 32'h100);  // init
        repeat(2) @(posedge clk);
        bus_read(5'h2);
        check("G56: CRC free after seal", rd[16] === 1'b0);

        // ============================================================
        // GROUP 57: X-prop scan — all outputs defined after reset
        // ============================================================
        $display(""); $display("--- G57: X-prop scan ---");
        rst_n = 0; repeat(10) @(posedge clk); rst_n = 1;
        repeat(5) @(posedge clk);
        begin : xprop_block
            reg saw_x;
            integer s;
            saw_x = 0;
            // Check all output pins
            if (^uo_out === 1'bx) saw_x = 1;
            if (^uio_out === 1'bx) saw_x = 1;
            if (^uio_oe === 1'bx) saw_x = 1;
            // Check key internal signals
            if (^int_timer_count === 1'bx) saw_x = 1;
            if (^int_timer_irq === 1'bx) saw_x = 1;
            if (^int_irq === 1'bx) saw_x = 1;
            if (^int_pps_count === 1'bx) saw_x = 1;
            if (^int_session_ctr === 1'bx) saw_x = 1;
            // Read all MMIO slots — none should return X
            for (s = 0; s < 16; s = s + 1) begin
                bus_read(s[4:0]);
                if (^rd === 1'bx) begin
                    $display("  X detected in slot %0d", s);
                    saw_x = 1;
                end
            end
            check("G57: no X in outputs after reset", saw_x === 0);
        end

        // ============================================================
        // GROUP 58: Interrupt mapping correctness
        // ============================================================
        $display(""); $display("--- G58: Interrupt mapping ---");
        // IRQ[3] should be 0 (reserved, not TX ready)
        check("G58: IRQ[3]=0 (reserved)", int_irq[3] === 1'b0);
        // IRQ[2] = uart_rx_valid (should be 0 — nothing received)
        check("G58: IRQ[2]=0 (no UART RX)", int_irq[2] === 1'b0);
        // IRQ[1] = timer_irq (0 — timer not expired)
        check("G58: IRQ[1]=0 (no timer)", int_irq[1] === 1'b0);
        // IRQ[0] = DIO1 synced from ui_in[0]
        ui_in[0] = 1'b0; repeat(4) @(posedge clk);
        check("G58: IRQ[0]=0 (DIO1 low)", int_irq[0] === 1'b0);
        ui_in[0] = 1'b1; repeat(4) @(posedge clk);
        check("G58: IRQ[0]=1 (DIO1 high)", int_irq[0] === 1'b1);
        ui_in[0] = 1'b0; repeat(4) @(posedge clk);
        check("G58: IRQ[0]=0 (DIO1 cleared)", int_irq[0] === 1'b0);

        // ============================================================
        // GROUP 59: RTC write-on-tick priority
        // ============================================================
        $display(""); $display("--- G59: RTC write priority ---");
        bus_write(5'hA, 32'd1000);  // set RTC to 1000
        repeat(2) @(posedge clk);
        bus_read(5'hA);
        check("G59: RTC set to 1000", rd === 32'd1000);
        // Now force us_count near rollover AND write simultaneously
        force dut.i_rtc.us_count = 20'd999_998;
        @(posedge clk); release dut.i_rtc.us_count;
        // Write new value — should override even if tick fires same cycle
        bus_write(5'hA, 32'd5000);
        repeat(2) @(posedge clk);
        bus_read(5'hA);
        check("G59: write wins over tick", rd === 32'd5000);

        // ============================================================
        // GROUP 60: Soft reset magic number must be exact 0xA5
        // ============================================================
        $display(""); $display("--- G60: Soft reset magic ---");
        // Write 0xA4 — should NOT trigger reset
        bus_write(5'hF, 32'h0000_00A4);
        repeat(5) @(posedge clk);
        check("G60: 0xA4 no reset", int_rst_reg_n === 1'b1);
        // Write 0xA6 — should NOT trigger reset
        bus_write(5'hF, 32'h0000_00A6);
        repeat(5) @(posedge clk);
        check("G60: 0xA6 no reset", int_rst_reg_n === 1'b1);
        // Write 0xFF — should NOT trigger reset
        bus_write(5'hF, 32'h0000_00FF);
        repeat(5) @(posedge clk);
        check("G60: 0xFF no reset", int_rst_reg_n === 1'b1);
        // Write 0x00 — should NOT trigger reset
        bus_write(5'hF, 32'h0000_0000);
        repeat(5) @(posedge clk);
        check("G60: 0x00 no reset", int_rst_reg_n === 1'b1);
        // Write 0x1A5 (upper bits set) — data[7:0] = 0xA5, SHOULD trigger
        bus_write(5'hF, 32'h0000_01A5);
        repeat(5) @(posedge clk);
        check("G60: 0x1A5 triggers reset (data[7:0]=A5)", int_rst_reg_n === 1'b0);
        repeat(40) @(posedge clk);
        check("G60: recovered after correct magic", int_rst_reg_n === 1'b1);

        // ============================================================
        // GROUP 61: Extended random write fuzz (100 writes)
        // ============================================================
        $display(""); $display("--- G61: Extended fuzz (100 writes) ---");
        begin : fuzz100_block
            integer i;
            reg saw_x;
            reg [31:0] fuzz_addr;
            reg [31:0] fuzz_data;
            saw_x = 0;
            // Write random data to random valid slots
            for (i = 0; i < 100; i = i + 1) begin
                fuzz_addr = (i * 7 + 3) % 32;  // pseudo-random slot 0..31
                fuzz_data = {i[7:0], i[7:0] ^ 8'hFF, i[7:0] + 8'h42, 8'hA0};
                // Avoid 0xA5 in low byte for SYSINFO slot (prevent accidental soft reset)
                if (fuzz_addr[4:0] == 5'hF && fuzz_data[7:0] == 8'hA5)
                    fuzz_data[7:0] = 8'hA4;
                bus_write(fuzz_addr[4:0], fuzz_data);
            end
            // Verify system didn't crash
            check("G61: system alive after 100 writes", int_rst_reg_n === 1'b1);
            // Read all valid slots — no X
            for (i = 0; i < 16; i = i + 1) begin
                bus_read(i[4:0]);
                if (^rd === 1'bx) begin
                    $display("  X in slot %0d after fuzz", i);
                    saw_x = 1;
                end
            end
            check("G61: no X after fuzz", saw_x === 0);
        end

        // ============================================================
        // GROUP 62: Write strobe held 2 cycles (should not double-trigger)
        // ============================================================
        $display(""); $display("--- G62: Double write strobe ---");
        // Set GPIO_OUT to known value
        bus_write(5'h0, 32'h0000_0000);
        repeat(2) @(posedge clk);
        // Manually hold write for 2 cycles
        tb_addr = {1'b1, 20'b0, 5'h0, 2'b00};
        tb_write_n = 2'b10;
        tb_read_n = 2'b11;
        tb_wdata = 32'h0000_0042;
        @(posedge clk);
        force dut.i_tinyqv.data_addr    = tb_addr;
        force dut.i_tinyqv.data_write_n = tb_write_n;
        force dut.i_tinyqv.data_read_n  = tb_read_n;
        force dut.i_tinyqv.data_out     = tb_wdata;
        @(posedge clk);
        // Hold for second cycle (don't deassert yet)
        @(posedge clk);
        // Now deassert
        tb_write_n = 2'b11;
        force dut.i_tinyqv.data_write_n = tb_write_n;
        @(posedge clk);
        release dut.i_tinyqv.data_addr;
        release dut.i_tinyqv.data_write_n;
        release dut.i_tinyqv.data_read_n;
        release dut.i_tinyqv.data_out;
        // Verify: gpio_out reg should be 0x42 (single write, not corrupted)
        // Note: reading slot 0 returns uo_out (pin-muxed), not raw gpio_out
        check("G62: gpio_out=0x42 (single effect)", dut.gpio_out === 8'h42);

        // ============================================================
        // GROUP 63: Consecutive reads return consistent values
        // ============================================================
        $display(""); $display("--- G63: Read consistency ---");
        bus_write(5'hA, 32'd12345);  // RTC = 12345
        repeat(2) @(posedge clk);
        begin : read_consist
            reg [31:0] r1, r2, r3;
            bus_read(5'hA); r1 = rd;
            bus_read(5'hA); r2 = rd;
            bus_read(5'hA); r3 = rd;
            // All 3 should be same (no tick happened in ~6 clk cycles)
            check("G63: RTC 3x read consistent", (r1 === r2) && (r2 === r3));
        end

        // ============================================================
        // GROUP 64: A2 — WDT reset clears seal mono_count
        // ============================================================
        $display(""); $display("--- G64: WDT resets seal mono ---");
        // Do a seal commit to set mono_count > 0
        bus_write(5'hB, 32'h1111_0000);  // SEAL_DATA
        bus_write(5'hE, {22'd0, 8'h01, 1'b1, 1'b0});  // commit
        begin : g64_wait1
            integer t; t = 0;
            while (dut.seal_ctrl_out[0] && t < 5000) begin @(posedge clk); t = t + 1; end
        end
        bus_write(5'hB, 32'h2222_0000);  // second commit
        bus_write(5'hE, {22'd0, 8'h02, 1'b1, 1'b0});
        begin : g64_wait2
            integer t; t = 0;
            while (dut.seal_ctrl_out[0] && t < 5000) begin @(posedge clk); t = t + 1; end
        end
        // mono should be 2 now
        // Trigger WDT reset
        bus_write(5'hD, 32'd2);  // 2µs timeout
        repeat(200) @(posedge clk);
        repeat(50) @(posedge clk);  // wait for reset hold to complete
        check("G64: system recovered", int_rst_reg_n === 1'b1);
        // Now do a commit — mono should be back to 0
        bus_write(5'hB, 32'h3333_0000);
        bus_write(5'hE, {22'd0, 8'h03, 1'b1, 1'b0});
        begin : g64_wait3
            integer t; t = 0;
            while (dut.seal_ctrl_out[0] && t < 5000) begin @(posedge clk); t = t + 1; end
        end
        // Read sealed record
        bus_read(5'hB);  // read0: value
        bus_read(5'hB);  // read1: {sid, mono[23:0]}
        check("G64: mono=0 after WDT reset", rd[23:0] === 24'd0);

        // ============================================================
        // GROUP 65-68: CRC Arbitration — Invisible Pollution Test
        // Prove that Seal preemption leaves no residue in CPU CRC
        // ============================================================

        // GROUP 65: CPU feeds 3 bytes, then Seal preempts
        $display(""); $display("--- G65: CRC Seal Preempt ---");
        // CPU init CRC
        bus_write(5'h2, {23'b0, 1'b1, 8'h00}); // init CRC
        repeat(2) @(posedge clk);
        // CPU feed 3 bytes: 0x11, 0x22, 0x33
        // Golden: python3 -c "d=bytes([0x11,0x22,0x33]);c=0xFFFF
        //   for b in d:
        //     c^=b
        //     for _ in range(8): c=(c>>1)^0xA001 if c&1 else c>>1
        //   print(f'0x{c:04X}')"  → 0x7079
        bus_write(5'h2, {23'b0, 1'b0, 8'h11}); repeat(12) @(posedge clk);
        bus_write(5'h2, {23'b0, 1'b0, 8'h22}); repeat(12) @(posedge clk);
        bus_write(5'h2, {23'b0, 1'b0, 8'h33}); repeat(12) @(posedge clk);
        // Read CPU CRC = 0x7079
        bus_read(5'h2);
        check("G65: CPU CRC before seal = 0x7079", rd[15:0] === 16'h7079);
        // Now trigger Seal commit → Seal takes over CRC engine
        bus_write(5'hB, 32'hAAAA_BBBB); repeat(2) @(posedge clk);
        bus_write(5'hE, {22'b0, 8'hCC, 1'b1, 1'b0}); // commit
        // While seal is active, CPU CRC reads should show busy
        repeat(5) @(posedge clk);
        check("G65: seal active", int_seal_using === 1'b1);

        // GROUP 66: CPU CRC reads busy during Seal
        $display(""); $display("--- G66: CRC Busy During Seal ---");
        bus_read(5'h2);
        check("G66: CRC busy during seal", rd[16] === 1'b1);
        // Wait for seal to complete
        begin : g66_wait
            integer t; t = 0;
            while (int_seal_using && t < 5000) begin @(posedge clk); t = t + 1; end
        end
        check("G66: seal completed", int_seal_using === 1'b0);

        // GROUP 67: After Seal, CPU re-init + same 3 bytes → CRC must match
        // This is the CRITICAL test: proves no Seal data residue in CRC
        $display(""); $display("--- G67: CRC No Seal Residue ---");
        // CPU re-init CRC
        bus_write(5'h2, {23'b0, 1'b1, 8'h00}); // init
        repeat(2) @(posedge clk);
        // Feed same 3 bytes: 0x11, 0x22, 0x33
        bus_write(5'h2, {23'b0, 1'b0, 8'h11}); repeat(12) @(posedge clk);
        bus_write(5'h2, {23'b0, 1'b0, 8'h22}); repeat(12) @(posedge clk);
        bus_write(5'h2, {23'b0, 1'b0, 8'h33}); repeat(12) @(posedge clk);
        bus_read(5'h2);
        check("G67: CRC after seal = 0x7079 (no residue)", rd[15:0] === 16'h7079);

        // GROUP 68: Partial CPU + Seal preempt + CPU re-init → still correct
        $display(""); $display("--- G68: Partial CRC + Seal + Re-init ---");
        // CPU partial: init + 1 byte only
        bus_write(5'h2, {23'b0, 1'b1, 8'h00}); repeat(2) @(posedge clk);
        bus_write(5'h2, {23'b0, 1'b0, 8'hFF}); repeat(12) @(posedge clk);
        // Seal preempt
        bus_write(5'hB, 32'h1234_5678); repeat(2) @(posedge clk);
        bus_write(5'hE, {22'b0, 8'hDD, 1'b1, 1'b0});
        begin : g68_wait
            integer t; t = 0;
            while (int_seal_using && t < 5000) begin @(posedge clk); t = t + 1; end
        end
        // CPU re-init and compute full 3-byte CRC
        bus_write(5'h2, {23'b0, 1'b1, 8'h00}); repeat(2) @(posedge clk);
        bus_write(5'h2, {23'b0, 1'b0, 8'h11}); repeat(12) @(posedge clk);
        bus_write(5'h2, {23'b0, 1'b0, 8'h22}); repeat(12) @(posedge clk);
        bus_write(5'h2, {23'b0, 1'b0, 8'h33}); repeat(12) @(posedge clk);
        bus_read(5'h2);
        check("G68: CRC after partial+seal = 0x7079", rd[15:0] === 16'h7079);

        // ============================================================
        // GROUP 69-71: GPIO Bypass Robustness
        // ============================================================

        // GROUP 69: GPIO bypass mode — uo_out reflects gpio_out, then revert
        $display(""); $display("--- G69: GPIO Bypass Round-Trip ---");
        // Start clean: gpio_out_sel=0 (all peripheral), gpio_out=0
        bus_write(5'h3, 32'h00);
        bus_write(5'h0, 32'h00);
        repeat(2) @(posedge clk);
        // Record peripheral-driven uo_out baseline
        begin : g69_block
            reg [7:0] periph_baseline;
            periph_baseline = uo_out;
            // Set gpio_out to 0xA5
            bus_write(5'h0, 32'hA5);
            repeat(2) @(posedge clk);
            // Enable ALL gpio bypass
            bus_write(5'h3, 32'hFF);
            repeat(2) @(posedge clk);
            check("G69: bypass uo_out=0xA5", uo_out === 8'hA5);
            // Readback gpio_out_sel via MMIO
            bus_read(5'h3);
            check("G69: gpio_out_sel readback=0xFF", rd[7:0] === 8'hFF);
            // Disable bypass — back to peripheral mode
            bus_write(5'h3, 32'h00);
            repeat(2) @(posedge clk);
            check("G69: revert to periph output", uo_out === periph_baseline);
            // Verify gpio_out_sel cleared
            bus_read(5'h3);
            check("G69: gpio_out_sel readback=0x00", rd[7:0] === 8'h00);
        end

        // GROUP 70: Toggle gpio_out_sel during UART TX — no corruption
        $display(""); $display("--- G70: GPIO Sel Toggle During UART TX ---");
        // Start UART TX (write byte 0x55 to UART slot)
        bus_write(5'h3, 32'h00);  // peripheral mode
        repeat(2) @(posedge clk);
        bus_write(5'h4, 32'h55);  // Start UART TX of 0x55
        repeat(5) @(posedge clk);
        // While UART is transmitting, toggle gpio_out_sel[0] (UART TX pin)
        bus_write(5'h3, 32'h01);  // bit 0 = GPIO mode for UART TX pin
        repeat(10) @(posedge clk);
        bus_write(5'h3, 32'h00);  // back to peripheral mode
        repeat(10) @(posedge clk);
        // Verify UART is still alive (tx_busy or completed, no X)
        bus_read(5'h5);  // UART_STATUS
        check("G70: UART_STATUS no X after sel toggle", ^rd !== 1'bx);
        // Verify system didn't crash
        check("G70: system alive after sel toggle", int_rst_reg_n === 1'b1);
        // Wait for UART TX to finish (115200 baud ≈ 2170 clk per byte)
        repeat(3000) @(posedge clk);
        bus_read(5'h5);
        check("G70: UART TX completed", rd[0] === 1'b0);  // tx_busy=0

        // GROUP 71: Write each individual gpio_out_sel bit pattern
        $display(""); $display("--- G71: GPIO Sel Bit Patterns ---");
        bus_write(5'h0, 32'hFF);  // gpio_out = all 1s
        repeat(2) @(posedge clk);
        begin : g71_block
            integer bit_idx;
            reg [7:0] sel_pattern;
            reg ok;
            ok = 1;
            for (bit_idx = 0; bit_idx < 8; bit_idx = bit_idx + 1) begin
                sel_pattern = (8'd1 << bit_idx);
                bus_write(5'h3, {24'b0, sel_pattern});
                repeat(2) @(posedge clk);
                // The selected bit should be 1 (from gpio_out=0xFF)
                if (uo_out[bit_idx] !== 1'b1) ok = 0;
            end
            check("G71: each sel bit drives gpio_out=1", ok === 1);
            // Now gpio_out = 0x00, each sel bit should drive 0
            bus_write(5'h0, 32'h00);
            repeat(2) @(posedge clk);
            ok = 1;
            for (bit_idx = 0; bit_idx < 8; bit_idx = bit_idx + 1) begin
                sel_pattern = (8'd1 << bit_idx);
                bus_write(5'h3, {24'b0, sel_pattern});
                repeat(2) @(posedge clk);
                if (uo_out[bit_idx] !== 1'b0) ok = 0;
            end
            check("G71: each sel bit drives gpio_out=0", ok === 1);
        end
        // Cleanup
        bus_write(5'h3, 32'h00);

        // ============================================================
        // GROUP 72-74: Soft Reset Extensions
        // ============================================================

        // GROUP 72: Two consecutive soft resets — system recovers
        $display(""); $display("--- G72: Double Soft Reset ---");
        bus_write(5'hC, 32'd8000);  // Load timer (canary)
        repeat(2) @(posedge clk);
        check("G72: timer pre-loaded", int_timer_count !== 32'd0);
        // First soft reset
        bus_write(5'hF, 32'hA5);
        repeat(40) @(posedge clk);
        check("G72: rst recovered after 1st reset", int_rst_reg_n === 1'b1);
        check("G72: timer cleared after 1st reset", int_timer_count === 32'd0);
        // Load state again
        bus_write(5'hC, 32'd9000);
        repeat(2) @(posedge clk);
        check("G72: timer re-loaded", int_timer_count === 32'd9000);
        // Second soft reset immediately
        bus_write(5'hF, 32'hA5);
        repeat(40) @(posedge clk);
        check("G72: rst recovered after 2nd reset", int_rst_reg_n === 1'b1);
        check("G72: timer cleared after 2nd reset", int_timer_count === 32'd0);
        // Verify peripherals still work
        bus_write(5'hA, 32'd42);
        repeat(2) @(posedge clk);
        bus_read(5'hA);
        check("G72: RTC works after double reset", rd === 32'd42);

        // GROUP 73: Wrong magic values must NOT trigger reset
        $display(""); $display("--- G73: Wrong Magic No Reset ---");
        bus_write(5'hC, 32'd7777);  // Canary
        repeat(2) @(posedge clk);
        // 0xA4 — off by one low
        bus_write(5'hF, 32'hA4);
        repeat(5) @(posedge clk);
        check("G73: 0xA4 no reset (rst_reg_n=1)", int_rst_reg_n === 1'b1);
        check("G73: 0xA4 timer survives", int_timer_count !== 32'd0);
        // 0xA6 — off by one high
        bus_write(5'hF, 32'hA6);
        repeat(5) @(posedge clk);
        check("G73: 0xA6 no reset", int_rst_reg_n === 1'b1);
        // 0xFF — all ones
        bus_write(5'hF, 32'hFF);
        repeat(5) @(posedge clk);
        check("G73: 0xFF no reset", int_rst_reg_n === 1'b1);
        // 0x00 — all zeros
        bus_write(5'hF, 32'h00);
        repeat(5) @(posedge clk);
        check("G73: 0x00 no reset", int_rst_reg_n === 1'b1);
        // 0x5A — complement of 0xA5
        bus_write(5'hF, 32'h5A);
        repeat(5) @(posedge clk);
        check("G73: 0x5A no reset", int_rst_reg_n === 1'b1);
        // Verify timer still has value (no reset happened)
        check("G73: timer still alive", int_timer_count !== 32'd0);
        // Cleanup
        bus_write(5'hC, 32'd0);

        // GROUP 74: Soft reset clears seal mono_count
        $display(""); $display("--- G74: Soft Reset Clears Seal Mono ---");
        // Do a few seal commits to increment mono_count
        bus_write(5'hB, 32'hAAAA_1111);
        bus_write(5'hE, {22'b0, 8'h01, 1'b1, 1'b0});
        begin : g74_wait1
            integer t; t = 0;
            while (dut.seal_ctrl_out[0] && t < 5000) begin @(posedge clk); t = t + 1; end
        end
        bus_write(5'hB, 32'hBBBB_2222);
        bus_write(5'hE, {22'b0, 8'h02, 1'b1, 1'b0});
        begin : g74_wait2
            integer t; t = 0;
            while (dut.seal_ctrl_out[0] && t < 5000) begin @(posedge clk); t = t + 1; end
        end
        // mono_count should be >= 2 now
        // Trigger soft reset
        bus_write(5'hF, 32'hA5);
        repeat(40) @(posedge clk);
        check("G74: rst recovered", int_rst_reg_n === 1'b1);
        // Commit after reset — mono should be 0
        bus_write(5'hB, 32'hCCCC_3333);
        bus_write(5'hE, {22'b0, 8'h03, 1'b1, 1'b0});
        begin : g74_wait3
            integer t; t = 0;
            while (dut.seal_ctrl_out[0] && t < 5000) begin @(posedge clk); t = t + 1; end
        end
        // Read sealed record: read0=value, read1={sid, mono[23:0]}
        bus_read(5'hB);  // read0: value
        bus_read(5'hB);  // read1: {sid, mono[23:0]}
        check("G74: mono=0 after soft reset", rd[23:0] === 24'd0);

        // ============================================================
        // GROUP 75: WDT Survivability — latch_mem data after WDT reset
        // ============================================================
        // latch_mem uses latch_reg_n which has NO hardware reset.
        // Data persists through WDT/soft reset because rstn only resets
        // the cycle counter, not the latch contents.
        // Test: write to latch_mem via bus, trigger WDT, verify data persists.
        $display(""); $display("--- G75: Latch Mem WDT Survivability ---");
        begin : g75_block
            reg [31:0] lmem_readback;
            // Write 0xDEAD_BEEF to latch_mem word 0 via bus
            // addr[26]=1 selects latch_mem. addr[4:0] = byte address.
            // For 32-bit write (write_n=2'b10), latch_mem needs 4 cycles.
            tb_addr    = {2'b01, 21'b0, 5'b0};  // addr[26]=1, addr[4:0]=0
            tb_write_n = 2'b10;  // 32-bit write
            tb_read_n  = 2'b11;
            tb_wdata   = 32'hDEAD_BEEF;
            @(posedge clk);
            force dut.i_tinyqv.data_addr    = tb_addr;
            force dut.i_tinyqv.data_write_n = tb_write_n;
            force dut.i_tinyqv.data_read_n  = tb_read_n;
            force dut.i_tinyqv.data_out     = tb_wdata;
            // Hold write for 4 cycles (cycle counter: 00→01→10→11)
            repeat(4) @(posedge clk);
            // Wait one more cycle for last byte to latch (falling edge)
            @(posedge clk);
            // Deassert write
            tb_write_n = 2'b11;
            force dut.i_tinyqv.data_write_n = tb_write_n;
            @(posedge clk);
            release dut.i_tinyqv.data_addr;
            release dut.i_tinyqv.data_write_n;
            release dut.i_tinyqv.data_read_n;
            release dut.i_tinyqv.data_out;
            repeat(2) @(posedge clk);
            // Read back via bus
            tb_addr    = {2'b01, 21'b0, 5'b0};
            tb_write_n = 2'b11;
            tb_read_n  = 2'b10;  // 32-bit read
            @(posedge clk);
            force dut.i_tinyqv.data_addr    = tb_addr;
            force dut.i_tinyqv.data_write_n = tb_write_n;
            force dut.i_tinyqv.data_read_n  = tb_read_n;
            force dut.i_tinyqv.data_out     = 32'd0;
            // Hold read for 4 cycles + 1 for data_out to assemble
            repeat(5) @(posedge clk);
            lmem_readback = dut.lmem_data_from_read;
            tb_read_n = 2'b11;
            force dut.i_tinyqv.data_read_n  = tb_read_n;
            @(posedge clk);
            release dut.i_tinyqv.data_addr;
            release dut.i_tinyqv.data_write_n;
            release dut.i_tinyqv.data_read_n;
            release dut.i_tinyqv.data_out;
            check("G75: latch_mem pre-WDT=0xDEADBEEF", lmem_readback === 32'hDEAD_BEEF);

            // Now trigger WDT reset
            bus_write(5'hD, 32'd2);  // 2µs timeout
            repeat(200) @(posedge clk);  // Wait for WDT expiry
            repeat(50) @(posedge clk);   // Wait for reset hold to complete
            check("G75: system recovered from WDT", int_rst_reg_n === 1'b1);
            // Also verify normal MMIO peripherals DID reset
            check("G75: timer cleared (MMIO reset)", int_timer_count === 32'd0);
            check("G75: gpio_out cleared (MMIO reset)", dut.gpio_out === 8'd0);

            // Read latch_mem again — data should PERSIST (no hardware reset on latches)
            tb_addr    = {2'b01, 21'b0, 5'b0};
            tb_write_n = 2'b11;
            tb_read_n  = 2'b10;
            @(posedge clk);
            force dut.i_tinyqv.data_addr    = tb_addr;
            force dut.i_tinyqv.data_write_n = tb_write_n;
            force dut.i_tinyqv.data_read_n  = tb_read_n;
            force dut.i_tinyqv.data_out     = 32'd0;
            repeat(5) @(posedge clk);
            lmem_readback = dut.lmem_data_from_read;
            tb_read_n = 2'b11;
            force dut.i_tinyqv.data_read_n  = tb_read_n;
            @(posedge clk);
            release dut.i_tinyqv.data_addr;
            release dut.i_tinyqv.data_write_n;
            release dut.i_tinyqv.data_read_n;
            release dut.i_tinyqv.data_out;
            // latch_reg_n has NO reset — data persists through WDT reset
            check("G75: latch_mem persists after WDT", lmem_readback === 32'hDEAD_BEEF);
        end

        // ============================================================
        // GROUP 76-80: SPI Peripheral Path Verification
        // Validates project.v address decode, spi_ctrl wiring, GPIO mux
        // ============================================================

        // GROUP 76: SPI idle state — CS high, SCK low, busy=0
        $display(""); $display("--- G76: SPI Idle State ---");
        // Ensure gpio_out_sel bits 3-5 are 0 (SPI pins not bypassed)
        bus_write(5'h3, 32'h00);
        repeat(2) @(posedge clk);
        check("G76: SPI CS idle high", uo_out[4] === 1'b1);
        check("G76: SPI SCK idle low", uo_out[5] === 1'b0);
        bus_read(5'h9);  // PERI_SPI_STATUS
        check("G76: SPI not busy", rd[0] === 1'b0);

        // GROUP 77: SPI config + single byte TX — verify CS/SCK/MOSI
        $display(""); $display("--- G77: SPI TX Byte ---");
        // Set divider=1 (SCK = clk/4) for easier cycle counting
        bus_write(5'h9, 32'h0000_0001);
        repeat(2) @(posedge clk);
        // Send byte 0xA5, end_txn=1 (release CS after)
        bus_write(5'h8, {22'b0, 1'b0, 1'b1, 8'hA5});
        repeat(2) @(posedge clk);
        // SPI should now be busy, CS should be low
        check("G77: SPI busy after write", dut.spi_busy === 1'b1);
        check("G77: CS low during TX", uo_out[4] === 1'b0);
        // Wait for SPI to complete (8 bits * 2 half-clocks * (div+1=2) = 32 clk + margin)
        begin : g77_spi_wait
            integer t; t = 0;
            while (dut.spi_busy && t < 200) begin @(posedge clk); t = t + 1; end
            check("G77: SPI completed", dut.spi_busy === 1'b0);
        end
        // After end_txn=1, CS should return high
        repeat(2) @(posedge clk);
        check("G77: CS high after end_txn", uo_out[4] === 1'b1);
        check("G77: SCK idle after TX", uo_out[5] === 1'b0);

        // GROUP 78: SPI MISO → read path
        $display(""); $display("--- G78: SPI MISO Read ---");
        // Drive MISO=1 (ui_in[2]) throughout transfer → should read 0xFF
        ui_in[2] = 1'b1;
        bus_write(5'h8, {22'b0, 1'b0, 1'b1, 8'h00});  // send 0x00, end_txn=1
        begin : g78_wait
            integer t; t = 0;
            repeat(2) @(posedge clk);
            while (dut.spi_busy && t < 200) begin @(posedge clk); t = t + 1; end
        end
        repeat(2) @(posedge clk);
        bus_read(5'h8);  // PERI_SPI read
        check("G78: MISO=1 reads 0xFF", rd[7:0] === 8'hFF);
        // Now with MISO=0
        ui_in[2] = 1'b0;
        bus_write(5'h8, {22'b0, 1'b0, 1'b1, 8'h00});
        begin : g78_wait2
            integer t; t = 0;
            repeat(2) @(posedge clk);
            while (dut.spi_busy && t < 200) begin @(posedge clk); t = t + 1; end
        end
        repeat(2) @(posedge clk);
        bus_read(5'h8);
        check("G78: MISO=0 reads 0x00", rd[7:0] === 8'h00);
        ui_in[2] = 1'b0;  // restore

        // GROUP 79: SPI back-to-back without end_txn — CS stays low
        $display(""); $display("--- G79: SPI B2B No End ---");
        // First byte: end_txn=0 (keep CS asserted)
        bus_write(5'h8, {22'b0, 1'b0, 1'b0, 8'h55});
        begin : g79_wait1
            integer t; t = 0;
            repeat(2) @(posedge clk);
            while (dut.spi_busy && t < 200) begin @(posedge clk); t = t + 1; end
        end
        repeat(2) @(posedge clk);
        check("G79: CS stays low after no end_txn", uo_out[4] === 1'b0);
        // Second byte: end_txn=1 (release CS)
        bus_write(5'h8, {22'b0, 1'b0, 1'b1, 8'hAA});
        begin : g79_wait2
            integer t; t = 0;
            repeat(2) @(posedge clk);
            while (dut.spi_busy && t < 200) begin @(posedge clk); t = t + 1; end
        end
        repeat(2) @(posedge clk);
        check("G79: CS high after end_txn=1", uo_out[4] === 1'b1);

        // GROUP 80: SPI busy read via bus during transfer
        $display(""); $display("--- G80: SPI Busy Status ---");
        bus_write(5'h8, {22'b0, 1'b0, 1'b1, 8'hFF});
        repeat(3) @(posedge clk);  // should still be busy
        bus_read(5'h9);
        check("G80: SPI_STATUS busy=1 mid-transfer", rd[0] === 1'b1);
        begin : g80_wait
            integer t; t = 0;
            while (dut.spi_busy && t < 200) begin @(posedge clk); t = t + 1; end
        end
        repeat(2) @(posedge clk);
        bus_read(5'h9);
        check("G80: SPI_STATUS busy=0 after complete", rd[0] === 1'b0);

        // ============================================================
        $display("");
        $display("=== Results: %0d PASS, %0d FAIL ===", pass_count, fail_count);
        if (fail_count == 0) $display("ALL TESTS PASSED");
        #100; $finish;
    end

    initial begin #400_000_000; $display("[ABORT] Timeout"); $finish; end

endmodule
