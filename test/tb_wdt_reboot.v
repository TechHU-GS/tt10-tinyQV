// ============================================================================
// TB: Test B — WDT Reboot (Watchdog Timeout → Reset → CPU Restart)
// ============================================================================
// Verifies: WDT expiry, reset_hold_counter, CPU reboot from 0x0,
//           PSRAM data survives reset.
// Expected UART: "B1B2DN" (6 chars)
// ============================================================================

`timescale 1ns / 1ps

module tb_wdt_reboot;

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
    wire flash_cs_n = uio_out[0];
    wire spi_clk    = uio_out[3];

    wire [3:0] qspi_data_to_flash = {uio_out[5], uio_out[4], uio_out[2], uio_out[1]};
    wire [3:0] qspi_data_from_flash;
    wire [3:0] qspi_oe = {uio_oe[5], uio_oe[4], uio_oe[2], uio_oe[1]};

    qspi_flash_model #(.HEX_FILE("fw_wdt_reboot.hex")) i_flash (
        .spi_clk     (spi_clk),
        .spi_cs_n    (flash_cs_n),
        .spi_data_in (qspi_data_to_flash),
        .spi_data_out(qspi_data_from_flash),
        .spi_data_oe (qspi_oe)
    );

    // ================================================================
    // QSPI PSRAM Model (RAM_A) — must survive soft/WDT reset
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
    // QSPI Data Bus Mux
    // ================================================================
    wire [3:0] ext_data_to_dut;
    assign ext_data_to_dut = (!flash_cs_n) ? qspi_data_from_flash :
                              (!ram_a_cs_n) ? qspi_data_from_psram :
                              4'hF;

    // ================================================================
    // Pin Connection — Latency Config + Flash/PSRAM Mux
    // ================================================================
    // For WDT/soft reset, the internal rst_reg_n goes low while
    // external rst_n stays high. We must re-present latency config
    // on spi_data_in pins DURING internal reset so the QSPI controller
    // samples the correct delay_cycles_cfg.
    //
    // We tap the DUT's internal rst_reg_n to detect internal reset.
    // When rst_reg_n is low, latency config must be present on pins.
    wire internal_rst_n = dut.rst_reg_n;

    reg latency_config_done;
    always @(posedge clk) begin
        if (!rst_n || !internal_rst_n) begin
            // Re-arm latency config whenever internal reset is active
            latency_config_done <= 0;
        end else begin
            if (!flash_cs_n || !ram_a_cs_n) begin
                // Once any CS goes low, config phase is over
                if (!latency_config_done)
                    latency_config_done <= 1;
            end
        end
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
    localparam EXPECTED_CHARS = 6;  // "B1B2DN"
    integer pass_count = 0;
    integer fail_count = 0;

    task check_2char(input integer idx, input [7:0] tag, input [7:0] val, input [8*16-1:0] name);
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
        rst_n = 0;
        ui_in = 8'h00;
        #400;

        @(posedge clk);
        @(posedge clk);
        rst_n = 1;

        $display("=== Test B: WDT Reboot ===");
        $display("Waiting for firmware (boot 1 + WDT reset + boot 2)...");

        // Wait for expected UART chars or timeout (200ms)
        // WDT timeout=200us, but reboot + reflash takes time
        begin : wait_loop
            integer wt;
            for (wt = 0; wt < 20000; wt = wt + 1) begin
                #10000;
                if (uart_idx >= EXPECTED_CHARS) disable wait_loop;
            end
            if (uart_idx < EXPECTED_CHARS)
                $display("[TIMEOUT] Only received %0d UART bytes after 200ms", uart_idx);
        end

        #200000;

        $display("");
        $display("--- Received %0d UART bytes ---", uart_idx);

        // Check "B1" — first boot
        check_2char(0, "B", "1", "Boot 1");

        // Check "B2" — second boot after WDT reset
        check_2char(2, "B", "2", "Boot 2 (WDT)");

        // Check "DN" — firmware completed
        if (uart_idx >= 6 && uart_buf[4] == "D" && uart_buf[5] == "N") begin
            $display("[PASS] Firmware complete: DN");
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] Firmware did not reach completion");
            fail_count = fail_count + 1;
        end

        $display("");
        $display("=== Test B Results: %0d PASS, %0d FAIL ===", pass_count, fail_count);
        if (fail_count == 0)
            $display("ALL TESTS PASSED");
        else
            $display("SOME TESTS FAILED");

        #100;
        $finish;
    end

    // Global watchdog: 500ms
    initial begin
        #500000000;
        $display("[ABORT] Simulation timeout at 500ms");
        $display("  UART bytes received: %0d", uart_idx);
        $finish;
    end

endmodule
