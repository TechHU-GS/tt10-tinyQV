// ============================================================================
// TB: IRQ Priority — DIO1 (IRQ16) vs Timer (IRQ17) Priority Verification
// ============================================================================
// Tests:
//   P1: Timer IRQ17 alone         -> mcause=17
//   P2: DIO1  IRQ16 alone         -> mcause=16
//   P3: Both IRQ16+IRQ17 simul.   -> IRQ16 fires first
//   P4: After ISR clears IRQ16    -> second ISR fires for IRQ17
//
// Expected UART: "P1P2P3P4DN" (10 chars)
//
// TB controls ui_in[0] (DIO1) to generate IRQ16 rising edges.
// Firmware signals "ready for DIO1" via uo_out[7] (gpio_out[7] with sel=1).
// ============================================================================

`timescale 1ns / 1ps

module tb_irq_priority;

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

    qspi_flash_model #(.HEX_FILE("fw_irq_priority.hex")) i_flash (
        .spi_clk     (spi_clk),
        .spi_cs_n    (flash_cs_n),
        .spi_data_in (qspi_data_to_flash),
        .spi_data_out(qspi_data_from_flash),
        .spi_data_oe (qspi_oe)
    );

    // ================================================================
    // QSPI PSRAM Model (RAM_A)
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
    // DIO1 Control
    // ================================================================
    // TB drives ui_in[0] to produce rising edges for IRQ16.
    // Firmware signals "ready for DIO1" by setting uo_out[7] = 1
    // (via GPIO_OUT_SEL[7]=1, GPIO_OUT[7]=1).
    //
    // For P2: When firmware sets uo_out[7]=1, TB asserts ui_in[0]=1
    //         (rising edge -> mip_reg[16] captured). After firmware
    //         handles ISR and clears gpio signal, TB clears ui_in[0]=0.
    //
    // For P3: Same but timer is also running simultaneously.
    // ================================================================

    // Watch for firmware signal on uo_out[7]
    reg uo7_prev = 0;
    always @(posedge clk) begin
        if (!rst_n)
            uo7_prev <= 0;
        else
            uo7_prev <= uo_out[7];
    end
    wire uo7_rising = uo_out[7] && !uo7_prev;

    // When firmware raises uo_out[7], assert DIO1 (ui_in[0]) after rising edge.
    // When firmware clears uo_out[7], deassert DIO1.
    always @(posedge clk) begin
        if (!rst_n) begin
            ui_in <= 8'h00;
        end else if (uo7_rising) begin
            ui_in[0] <= 1'b1;
        end else if (!uo_out[7] && ui_in[0]) begin
            ui_in[0] <= 1'b0;
        end
    end

    // ================================================================
    // Test Sequence
    // ================================================================
    localparam EXPECTED_CHARS = 10;  // "P1P2P3P4DN"
    integer pass_count = 0;
    integer fail_count = 0;

    task check_2char(input integer idx, input [7:0] tag, input [7:0] val, input [8*24-1:0] name);
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
                $display("[FAIL] %0s: not enough UART bytes (need idx %0d, have %0d)",
                         name, idx+1, uart_idx);
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

        $display("=== Test: IRQ Priority — DIO1(IRQ16) vs Timer(IRQ17) ===");
        $display("Waiting for firmware to complete...");

        // Wait for expected UART chars or timeout (80ms sim time)
        begin : wait_loop
            integer wt;
            for (wt = 0; wt < 8000; wt = wt + 1) begin
                #10000;
                if (uart_idx >= EXPECTED_CHARS) disable wait_loop;
            end
            if (uart_idx < EXPECTED_CHARS)
                $display("[TIMEOUT] Only received %0d of %0d UART bytes after 80ms",
                         uart_idx, EXPECTED_CHARS);
        end

        #200000;  // Extra settle time

        $display("");
        $display("--- Received %0d UART bytes ---", uart_idx);

        // P1: Timer IRQ17 alone
        check_2char(0, "P", "1", "P1 Timer IRQ17 alone");

        // P2: DIO1 IRQ16 alone
        check_2char(2, "P", "2", "P2 DIO1 IRQ16 alone");

        // P3: Priority — IRQ16 fires first
        check_2char(4, "P", "3", "P3 IRQ16 higher priority");

        // P4: IRQ17 fires second
        check_2char(6, "P", "4", "P4 IRQ17 fires second");

        // DN: Firmware complete
        if (uart_idx >= 10 && uart_buf[8] == "D" && uart_buf[9] == "N") begin
            $display("[PASS] Firmware complete: DN");
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] Firmware did not reach completion (DN)");
            fail_count = fail_count + 1;
        end

        $display("");
        $display("=== IRQ Priority Results: %0d PASS, %0d FAIL ===", pass_count, fail_count);
        if (fail_count == 0)
            $display("ALL TESTS PASSED");
        else
            $display("SOME TESTS FAILED");

        #100;
        $finish;
    end

    // Global watchdog: 150ms
    initial begin
        #150000000;
        $display("[ABORT] Simulation timeout at 150ms");
        $display("  UART bytes received: %0d", uart_idx);
        begin : dump_buf
            integer i;
            for (i = 0; i < uart_idx; i = i + 1)
                $display("  uart_buf[%0d] = 0x%02X '%c'", i, uart_buf[i], uart_buf[i]);
        end
        $finish;
    end

endmodule
