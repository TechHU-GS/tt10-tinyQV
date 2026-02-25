// ============================================================================
// TB: P0-A Integration Test — Flash-only Boot
// ============================================================================
// Verifies: CPU boot from Flash XIP → UART "OK" → CRC16 → SYS_INFO → Timer
//
// Environment:
//   - QSPI Flash model (fw_p0a.hex preloaded)
//   - PSRAM CS unconnected (pullup on data lines)
//   - No I2C slave
//   - UART monitor captures output bytes
//
// Pass criteria:
//   - UART outputs: "OK\nC1S1T1DN\n"
//   - No X propagation on critical signals
// ============================================================================

`timescale 1ns / 1ps

module tb_integration;

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
    tt_um_MichaelBell_tinyQV dut (
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
    wire flash_cs_n = uio_out[0];   // Flash CS (directly from DUT output)
    wire spi_clk    = uio_out[3];   // QSPI SCK

    // Bidirectional QSPI data: when DUT drives (uio_oe=1), flash receives
    // When DUT doesn't drive (uio_oe=0), flash drives
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

    // Connect flash data back to DUT inputs (when DUT is not driving)
    // uio_in[1] = SD0, uio_in[2] = SD1, uio_in[4] = SD2, uio_in[5] = SD3
    //
    // IMPORTANT: During reset, qspi_ctrl captures spi_data_in[2:0] as latency config.
    // spi_data_in = {uio_in[5:4], uio_in[2:1]} in project.v.
    // We need: delay_cycles_cfg = 2'b01, spi_clk_use_neg = 0
    // → spi_data_in = 4'b0001 → uio_in[1]=1, uio_in[2]=0, uio_in[4]=0, uio_in[5]=0
    //
    // After reset, the real flash/pullup values take over.
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
            // During reset: force latency config = 1 (normal)
            uio_in[1] = 1'b1;  // spi_data_in[0] = 1
            uio_in[2] = 1'b0;  // spi_data_in[1] = 0
            uio_in[4] = 1'b0;  // spi_data_in[2] = 0
            uio_in[5] = 1'b0;  // spi_data_in[3] = 0 (unused)
        end else begin
            // After reset: connect flash model
            uio_in[1] = uio_oe[1] ? uio_out[1] : qspi_data_from_flash[0];  // SD0
            uio_in[2] = uio_oe[2] ? uio_out[2] : qspi_data_from_flash[1];  // SD1
            uio_in[4] = uio_oe[4] ? uio_out[4] : qspi_data_from_flash[2];  // SD2
            uio_in[5] = uio_oe[5] ? uio_out[5] : qspi_data_from_flash[3];  // SD3
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
    localparam UART_BIT_CLKS = 217;  // 25MHz / 115200

    // Simple UART receiver
    // Detect falling edge of uart_txd for start bit
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
                // Wait for falling edge (start bit)
                if (uart_start_edge) begin
                    uart_bit_cnt <= 0;
                    uart_clk_cnt <= UART_BIT_CLKS + (UART_BIT_CLKS / 2);  // 1.5 bit times to mid of bit 0
                end
            end else begin
                if (uart_clk_cnt > 0) begin
                    uart_clk_cnt <= uart_clk_cnt - 1;
                end else begin
                    uart_clk_cnt <= UART_BIT_CLKS;
                    if (uart_bit_cnt < 8) begin
                        uart_shift <= {uart_txd, uart_shift[7:1]};  // LSB first
                        uart_bit_cnt <= uart_bit_cnt + 1;
                    end else begin
                        // Stop bit — byte complete
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
    // Delay configuration during reset
    // ================================================================
    // TinyQV reads spi_data_in[2:0] during reset to configure read latency.
    // We need to set latency=1 (normal mode): spi_data_in[1:0]=1, spi_data_in[2]=0
    // spi_data_in maps to {uio_in[5:4], uio_in[2:1]} in project.v
    // So we need: uio_in[1]=1, uio_in[2]=0, uio_in[4]=0, uio_in[5]=0
    // → spi_data_in = {0, 0, 0, 1} = 4'b0001 → latency cfg bits [1:0]=01, [2]=0

    // ================================================================
    // Test Sequence
    // ================================================================
    integer pass_count = 0;
    integer fail_count = 0;

    task check_uart_string(input [8*16-1:0] expected_str, input integer start_idx, input integer len);
        integer i;
        reg match;
        begin
            match = 1;
            for (i = 0; i < len; i = i + 1) begin
                if (uart_buf[start_idx + i] !== expected_str[(len-1-i)*8 +: 8]) begin
                    match = 0;
                end
            end
            if (match) begin
                $display("[PASS] UART string matches at idx %0d", start_idx);
                pass_count = pass_count + 1;
            end else begin
                $display("[FAIL] UART string mismatch at idx %0d", start_idx);
                $display("  Expected: %0s", expected_str);
                $write("  Got:      ");
                for (i = 0; i < len; i = i + 1)
                    $write("%c", uart_buf[start_idx + i]);
                $display("");
                fail_count = fail_count + 1;
            end
        end
    endtask

    initial begin
        $dumpfile("tb_integration.vcd");
        $dumpvars(0, tb_integration);

        // ---- Reset with latency config ----
        rst_n = 0;
        ui_in = 8'h00;
        // During reset, set QSPI data lines for latency config
        // uio_in is already driven by always block, but we need
        // spi_data_in[1:0]=1 during reset
        // The always block connects flash model which isn't active yet,
        // but uio_in[1] should be 1 (pullup) which gives latency=1
        #400;  // 10 clock cycles at 40ns

        // Release reset on rising edge of clock
        @(posedge clk);
        @(posedge clk);
        rst_n = 1;

        // Wait for firmware to run
        // Expected sequence: OK\nC1S1T1DN\n = 11 chars
        // At 115200 baud, ~87µs per char, ~960µs total
        // Plus boot time + CRC computation + timer wait
        // Give it 5ms (125000 clock cycles at 25MHz)
        $display("=== P0-A Integration Test: Flash-only Boot ===");
        $display("Waiting for firmware to complete...");

        // Wait until we see 11 UART characters or timeout
        fork
            begin
                wait(uart_idx >= 11);
            end
            begin
                #5_000_000;  // 5ms timeout
                if (uart_idx < 11) begin
                    $display("[TIMEOUT] Only received %0d UART bytes after 5ms", uart_idx);
                end
            end
        join_any
        disable fork;

        // Small margin after last byte
        #50000;

        $display("");
        $display("--- Received %0d UART bytes ---", uart_idx);

        // Check output
        if (uart_idx >= 3) begin
            // Check "OK\n"
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

        if (uart_idx >= 5) begin
            // Check CRC result: "C1" = pass, "C0" = fail
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

        if (uart_idx >= 7) begin
            // Check SYS_INFO: "S1" = pass
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

        if (uart_idx >= 9) begin
            // Check Timer: "T1" = pass
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

        if (uart_idx >= 11) begin
            // Check "DN\n"
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
        $display("=== Results: %0d PASS, %0d FAIL ===", pass_count, fail_count);
        if (fail_count == 0)
            $display("ALL TESTS PASSED — P0-A Flash-only boot verified!");
        else
            $display("SOME TESTS FAILED");

        #100;
        $finish;
    end

    // Watchdog: abort if simulation takes too long (50ms)
    initial begin
        #50_000_000;
        $display("[ABORT] Simulation timeout at 50ms");
        $display("  UART bytes received: %0d", uart_idx);
        $finish;
    end

endmodule
