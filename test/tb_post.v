// ============================================================================
// TB: POST (Power-On Self-Test) Firmware Verification
// ============================================================================
// Verifies: CPU boot → PSRAM stack → Full peripheral POST sequence
//
// Environment:
//   - QSPI Flash model (fw_post.hex preloaded)
//   - QSPI PSRAM model (RAM_A, 8KB)
//   - I2C slave model (SHT31 @ 0x44)
//   - UART monitor captures output bytes
//
// Expected UART output:
//   "POST\n"             — banner (5 chars, idx 0-4)
//   "Y1"                 — SYSINFO  (idx 5-6)
//   "C1"                 — CRC16    (idx 7-8)
//   "T1"                 — Timer    (idx 9-10)
//   "W1"                 — WDT      (idx 11-12)
//   "I1"                 — I2C      (idx 13-14)
//   "L1"                 — Seal #1  (idx 15-16)
//   "L2"                 — Seal #2  (idx 17-18)
//   "M1"                 — PSRAM    (idx 19-20)
//   "R1"                 — RTC      (idx 21-22)
//   "DN\n"               — Done     (idx 23-25)
//   Total: 26 chars
// ============================================================================

`timescale 1ns / 1ps

module tb_post;

    // 25 MHz clock (40ns period)
    reg clk = 0;
    always #20 clk = ~clk;

    reg rst_n;

    // TT interface
    reg  [7:0] ui_in;
    wire [7:0] uo_out;
    wire [7:0] uio_out;
    wire [7:0] uio_oe;
    reg  [7:0] uio_in;

    // DUT
    tt_um_techhu_rv32_trial dut (
        .ui_in  (ui_in),
        .uo_out (uo_out),
        .uio_in (uio_in),
        .uio_out(uio_out),
        .uio_oe (uio_oe),
        .ena    (1'b1),
        .clk    (clk),
        .rst_n  (rst_n)
    );

    // ================================================================
    // QSPI Flash Model
    // ================================================================
    wire flash_cs_n = uio_out[0];
    wire spi_clk    = uio_out[3];

    wire [3:0] qspi_data_to_flash = {uio_out[5], uio_out[4], uio_out[2], uio_out[1]};
    wire [3:0] qspi_data_from_flash;
    wire [3:0] qspi_oe = {uio_oe[5], uio_oe[4], uio_oe[2], uio_oe[1]};

    qspi_flash_model #(.HEX_FILE("fw_post.hex")) i_flash (
        .spi_clk     (spi_clk),
        .spi_cs_n    (flash_cs_n),
        .spi_data_in (qspi_data_to_flash),
        .spi_data_out(qspi_data_from_flash),
        .spi_data_oe (qspi_oe)
    );

    // ================================================================
    // QSPI PSRAM Model (RAM_A only — address bit[24:23]=10)
    // ================================================================
    wire ram_a_cs_n = uio_out[6];

    wire [3:0] qspi_data_to_psram = {uio_out[5], uio_out[4], uio_out[2], uio_out[1]};
    wire [3:0] qspi_data_from_psram;

    qspi_psram_model i_psram (
        .spi_clk     (spi_clk),
        .spi_cs_n    (ram_a_cs_n),
        .spi_data_in (qspi_data_to_psram),
        .spi_data_out(qspi_data_from_psram),
        .spi_data_oe (qspi_oe)
    );

    // ================================================================
    // I2C Slave Model (SHT31 @ 0x44)
    // ================================================================
    wire i2c_scl = uo_out[2];
    wire i2c_sda_master = uo_out[6];

    wire slave_sda_o;
    wire sda_bus_value = i2c_sda_master & slave_sda_o;

    i2c_slave_model #(.SLAVE_ADDR(7'h44)) i_sht31 (
        .scl(i2c_scl),
        .sda_i(sda_bus_value),
        .sda_o(slave_sda_o)
    );

    // ================================================================
    // QSPI Data Bus Mux (Flash vs PSRAM readback)
    // ================================================================
    wire [3:0] ext_data_to_dut;
    assign ext_data_to_dut = (!flash_cs_n) ? qspi_data_from_flash :
                              (!ram_a_cs_n) ? qspi_data_from_psram :
                              4'hF;

    // ================================================================
    // Pin Connection — Latency Config + Flash/PSRAM Mux
    // ================================================================
    reg latency_config_done;
    always @(posedge clk) begin
        if (rst_n) latency_config_done <= 1;
        else latency_config_done <= 0;
    end

    always @(*) begin
        uio_in[0] = 1'b1;
        uio_in[3] = 1'b1;
        uio_in[6] = 1'b1;
        uio_in[7] = 1'b1;

        if (!latency_config_done) begin
            uio_in[1] = 1'b1;
            uio_in[2] = 1'b0;
            uio_in[4] = 1'b0;
            uio_in[5] = 1'b0;
        end else begin
            uio_in[1] = uio_oe[1] ? uio_out[1] : ext_data_to_dut[0];
            uio_in[2] = uio_oe[2] ? uio_out[2] : ext_data_to_dut[1];
            uio_in[4] = uio_oe[4] ? uio_out[4] : ext_data_to_dut[2];
            uio_in[5] = uio_oe[5] ? uio_out[5] : ext_data_to_dut[3];
        end

        // I2C SDA readback
        ui_in[3] = sda_bus_value;
    end

    // ================================================================
    // UART Monitor (115200 baud @ 25MHz = ~217 clocks per bit)
    // ================================================================
    wire uart_txd = uo_out[0];
    reg [7:0] uart_buf [0:127];
    integer uart_idx = 0;
    integer uart_bit_cnt;
    reg [7:0] uart_shift;
    integer uart_clk_cnt;
    localparam UART_BIT_CLKS = 217;

    reg uart_txd_prev;
    always @(posedge clk) uart_txd_prev <= uart_txd;
    wire uart_start_edge = uart_txd_prev && !uart_txd;

    always @(posedge clk) begin
        if (!rst_n) begin
            uart_bit_cnt <= -1;
            uart_clk_cnt <= 0;
        end else begin
            if (uart_bit_cnt == -1) begin
                if (uart_start_edge) begin
                    uart_bit_cnt <= 0;
                    uart_clk_cnt <= UART_BIT_CLKS + (UART_BIT_CLKS / 2);
                end
            end else begin
                if (uart_clk_cnt > 0) begin
                    uart_clk_cnt <= uart_clk_cnt - 1;
                end else begin
                    uart_clk_cnt <= UART_BIT_CLKS;
                    if (uart_bit_cnt < 8) begin
                        uart_shift <= {uart_txd, uart_shift[7:1]};
                        uart_bit_cnt <= uart_bit_cnt + 1;
                    end else begin
                        if (uart_idx < 128) begin
                            uart_buf[uart_idx] = uart_shift;
                            $display("[UART] byte %0d: 0x%02X '%c' @ %0t ns",
                                     uart_idx, uart_shift, uart_shift, $time);
                            uart_idx = uart_idx + 1;
                        end
                        uart_bit_cnt <= -1;
                    end
                end
            end
        end
    end

    // ================================================================
    // Test Result Checking
    // ================================================================
    // Expected: "POST\nY1C1T1W1I1L1L2M1R1DN\n" = 26 chars
    localparam EXPECTED_CHARS = 26;
    integer pass_count = 0;
    integer fail_count = 0;

    task check_2char(input integer idx, input [7:0] tag, input [7:0] val, input [8*8-1:0] name);
        begin
            if (uart_idx > idx + 1) begin
                if (uart_buf[idx] == tag && uart_buf[idx+1] == val) begin
                    $display("[PASS] %0s: %c%c", name, tag, val);
                    pass_count = pass_count + 1;
                end else begin
                    $display("[FAIL] %0s: expected %c%c, got 0x%02X 0x%02X",
                             name, tag, val, uart_buf[idx], uart_buf[idx+1]);
                    fail_count = fail_count + 1;
                end
            end else begin
                $display("[FAIL] %0s: not enough UART bytes (need idx %0d)", name, idx+1);
                fail_count = fail_count + 1;
            end
        end
    endtask

    initial begin
        $dumpfile("tb_post.vcd");
        $dumpvars(0, tb_post);

        // ---- Reset ----
        rst_n = 0;
        ui_in = 8'h00;
        #400;

        @(posedge clk);
        @(posedge clk);
        rst_n = 1;

        $display("=== POST Firmware Verification ===");
        $display("Waiting for firmware to complete...");

        // Wait for expected UART chars or timeout (300ms for Seal + I2C)
        begin : wait_loop
            integer wt;
            for (wt = 0; wt < 7500; wt = wt + 1) begin
                #40000;  // 40us steps, 7500 × 40us = 300ms
                if (uart_idx >= EXPECTED_CHARS) disable wait_loop;
            end
            if (uart_idx < EXPECTED_CHARS)
                $display("[TIMEOUT] Only received %0d UART bytes after 300ms", uart_idx);
        end

        #100000;  // margin

        $display("");
        $display("--- Received %0d UART bytes ---", uart_idx);

        // Check "POST\n" banner (idx 0-4)
        if (uart_idx >= 5
            && uart_buf[0] == "P" && uart_buf[1] == "O"
            && uart_buf[2] == "S" && uart_buf[3] == "T"
            && uart_buf[4] == 8'h0A) begin
            $display("[PASS] Banner: 'POST\\n'");
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] Banner: expected 'POST\\n'");
            fail_count = fail_count + 1;
        end

        // Check test results (2-char pairs starting at idx 5)
        check_2char(5,  "Y", "1", "SYSINFO");
        check_2char(7,  "C", "1", "CRC16");
        check_2char(9,  "T", "1", "Timer");
        check_2char(11, "W", "1", "WDT");
        check_2char(13, "I", "1", "I2C");
        check_2char(15, "L", "1", "Seal_1");
        check_2char(17, "L", "2", "Seal_2");
        check_2char(19, "M", "1", "PSRAM");
        check_2char(21, "R", "1", "RTC");

        // Check "DN\n" (idx 23-25)
        if (uart_idx >= 26
            && uart_buf[23] == "D" && uart_buf[24] == "N"
            && uart_buf[25] == 8'h0A) begin
            $display("[PASS] Firmware complete: DN");
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] Firmware did not reach completion (DN\\n)");
            fail_count = fail_count + 1;
        end

        $display("");
        $display("=== Results: %0d PASS, %0d FAIL ===", pass_count, fail_count);
        if (fail_count == 0)
            $display("ALL TESTS PASSED — POST firmware verified!");
        else
            $display("SOME TESTS FAILED");

        #100;
        $finish;
    end

    // Global watchdog: 400ms
    initial begin
        #400_000_000;
        $display("[ABORT] Simulation timeout at 400ms");
        $display("  UART bytes received: %0d", uart_idx);
        $finish;
    end

endmodule
