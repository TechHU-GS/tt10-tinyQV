// ============================================================================
// TB: Gate-Level Simulation (GLS) — Post-Route Netlist
// ============================================================================
// Same test as tb_integration.v (P0-A Flash-only Boot), but runs on the
// gate-level netlist instead of RTL. Verifies synthesis + PnR correctness.
//
// Key differences from RTL simulation:
//   - DUT is the post-route netlist (sg13g2 cells)
//   - Longer propagation delays (gate-level, even without SDF)
//   - Timeout extended for gate-level simulation speed
//
// Pass criteria: Same as RTL — UART outputs "OK\nC1S1T1DN\n"
// ============================================================================

`timescale 1ns / 1ps

module tb_gls;

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

    // DUT — gate-level netlist (same port names as RTL)
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
    // QSPI Flash Model (same as RTL TB)
    // ================================================================
    wire flash_cs_n = uio_out[0];
    wire spi_clk    = uio_out[3];

    wire [3:0] qspi_data_to_flash = {uio_out[5], uio_out[4], uio_out[2], uio_out[1]};
    wire [3:0] qspi_data_from_flash;
    wire [3:0] qspi_oe = {uio_oe[5], uio_oe[4], uio_oe[2], uio_oe[1]};

    qspi_flash_model i_flash (
        .spi_clk     (spi_clk),
        .spi_cs_n    (flash_cs_n),
        .spi_data_in (qspi_data_to_flash),
        .spi_data_out(qspi_data_from_flash),
        .spi_data_oe (qspi_oe)
    );

    // ================================================================
    // QSPI Data Mux — same latency config logic as RTL TB
    // ================================================================
    reg latency_config_done;
    always @(posedge clk) begin
        if (rst_n) latency_config_done <= 1;
        else latency_config_done <= 0;
    end

    always @(*) begin
        uio_in[0] = 1'b1;  // Flash CS readback
        uio_in[3] = 1'b1;  // SCK readback
        uio_in[6] = 1'b1;  // RAM_A CS (no PSRAM)
        uio_in[7] = 1'b1;  // RAM_B CS (no PSRAM)

        if (!latency_config_done) begin
            uio_in[1] = 1'b1;  // spi_data_in[0] = 1 (latency config)
            uio_in[2] = 1'b0;
            uio_in[4] = 1'b0;
            uio_in[5] = 1'b0;
        end else begin
            uio_in[1] = uio_oe[1] ? uio_out[1] : qspi_data_from_flash[0];
            uio_in[2] = uio_oe[2] ? uio_out[2] : qspi_data_from_flash[1];
            uio_in[4] = uio_oe[4] ? uio_out[4] : qspi_data_from_flash[2];
            uio_in[5] = uio_oe[5] ? uio_out[5] : qspi_data_from_flash[3];
        end
    end

    // ================================================================
    // UART Monitor (115200 baud @ 25MHz = ~217 clocks per bit)
    // ================================================================
    wire uart_txd = uo_out[0];
    reg [7:0] uart_buf [0:63];
    integer uart_idx = 0;
    integer uart_bit_cnt;
    reg [7:0] uart_shift;
    integer uart_clk_cnt;
    localparam UART_BIT_CLKS = 217;

    reg uart_txd_prev;
    always @(posedge clk) begin
        uart_txd_prev <= uart_txd;
    end
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
                        if (uart_idx < 64) begin
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
    // Test Sequence
    // ================================================================
    integer pass_count = 0;
    integer fail_count = 0;

    initial begin
        // VCD disabled by default (2.4GB for 20ms GLS)
        // Uncomment to debug: $dumpfile("tb_gls.vcd"); $dumpvars(0, tb_gls);

        rst_n = 0;
        ui_in = 8'h00;
        #400;

        @(posedge clk);
        @(posedge clk);
        rst_n = 1;

        $display("=== GLS: Gate-Level Simulation - Post-Route Netlist ===");
        $display("Waiting for firmware to complete...");

        // Wait for 11 UART chars or timeout (20ms = 500000 clk @ 25MHz)
        begin : wait_loop
            integer i;
            for (i = 0; i < 500000; i = i + 1) begin
                @(posedge clk);
                if (uart_idx >= 11) disable wait_loop;
            end
        end

        if (uart_idx < 11)
            $display("[TIMEOUT] Only received %0d UART bytes after 20ms", uart_idx);

        #50000;

        $display("");
        $display("--- Received %0d UART bytes ---", uart_idx);

        // Check "OK\n"
        if (uart_idx >= 3) begin
            if (uart_buf[0] == "O" && uart_buf[1] == "K" && uart_buf[2] == 8'h0A) begin
                $display("[PASS] UART: 'OK\\n'");
                pass_count = pass_count + 1;
            end else begin
                $display("[FAIL] UART: expected 'OK\\n', got 0x%02X 0x%02X 0x%02X",
                         uart_buf[0], uart_buf[1], uart_buf[2]);
                fail_count = fail_count + 1;
            end
        end else begin
            $display("[FAIL] Not enough UART bytes (got %0d, need 3)", uart_idx);
            fail_count = fail_count + 1;
        end

        // CRC16 "C1"
        if (uart_idx >= 5) begin
            if (uart_buf[3] == "C" && uart_buf[4] == "1") begin
                $display("[PASS] CRC16 test: C1");
                pass_count = pass_count + 1;
            end else begin
                $display("[FAIL] CRC16 test: got '%c%c'", uart_buf[3], uart_buf[4]);
                fail_count = fail_count + 1;
            end
        end else begin
            $display("[FAIL] Missing CRC test result");
            fail_count = fail_count + 1;
        end

        // SYS_INFO "S1"
        if (uart_idx >= 7) begin
            if (uart_buf[5] == "S" && uart_buf[6] == "1") begin
                $display("[PASS] SYS_INFO test: S1");
                pass_count = pass_count + 1;
            end else begin
                $display("[FAIL] SYS_INFO test: got '%c%c'", uart_buf[5], uart_buf[6]);
                fail_count = fail_count + 1;
            end
        end else begin
            $display("[FAIL] Missing SYS_INFO test result");
            fail_count = fail_count + 1;
        end

        // Timer "T1"
        if (uart_idx >= 9) begin
            if (uart_buf[7] == "T" && uart_buf[8] == "1") begin
                $display("[PASS] Timer test: T1");
                pass_count = pass_count + 1;
            end else begin
                $display("[FAIL] Timer test: got '%c%c'", uart_buf[7], uart_buf[8]);
                fail_count = fail_count + 1;
            end
        end else begin
            $display("[FAIL] Missing Timer test result");
            fail_count = fail_count + 1;
        end

        // "DN"
        if (uart_idx >= 11) begin
            if (uart_buf[9] == "D" && uart_buf[10] == "N") begin
                $display("[PASS] Firmware complete: DN");
                pass_count = pass_count + 1;
            end else begin
                $display("[FAIL] Firmware end: expected 'DN', got '%c%c'",
                         uart_buf[9], uart_buf[10]);
                fail_count = fail_count + 1;
            end
        end else begin
            $display("[FAIL] Firmware did not reach completion");
            fail_count = fail_count + 1;
        end

        $display("");
        $display("=== GLS Results: %0d PASS, %0d FAIL ===", pass_count, fail_count);
        if (fail_count == 0)
            $display("ALL TESTS PASSED - Gate-level netlist matches RTL behavior!");
        else
            $display("SOME TESTS FAILED - Synthesis/PnR may have introduced errors");

        #100;
        $finish;
    end

    // Watchdog: 100ms absolute timeout
    initial begin
        #100000000;
        $display("[ABORT] GLS simulation timeout at 100ms");
        $display("  UART bytes received: %0d", uart_idx);
        $finish;
    end

endmodule
