// ============================================================================
// TB: P0-B Integration Test — PSRAM Boot + Full Peripheral Sweep
// ============================================================================
// Verifies: CPU boot → PSRAM stack → UART + CRC16 + SYS_INFO + Timer
//           + PSRAM R/W + I2C (SHT31) + WDT + RTC + Seal
//
// Environment:
//   - QSPI Flash model (fw_p0b.hex preloaded)
//   - QSPI PSRAM model (RAM_A, 8KB)
//   - I2C slave model (SHT31 @ 0x44)
//   - UART monitor captures output bytes
//
// Pass criteria:
//   - UART outputs: "OK\nC1S1T1M1I1W1R1E1DN\n" (21 chars)
// ============================================================================

`timescale 1ns / 1ps

module tb_integration_b;

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

    qspi_flash_model #(.HEX_FILE("fw_p0b.hex")) i_flash (
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
    // Push-pull pseudo open-drain bus model:
    //   master: uo_out[6]=0 → pulling low, uo_out[6]=1 → released
    //   slave:  sda_o=0 → pulling low, sda_o=1 → released
    //   bus = master AND slave (wired-AND, either can pull low)
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
    // When Flash CS active, flash drives data; when PSRAM CS active, PSRAM drives
    wire [3:0] ext_data_to_dut;
    assign ext_data_to_dut = (!flash_cs_n) ? qspi_data_from_flash :
                              (!ram_a_cs_n) ? qspi_data_from_psram :
                              4'hF;  // pullup when nothing selected

    // ================================================================
    // Pin Connection — Latency Config + Flash/PSRAM Mux
    // ================================================================
    reg latency_config_done;
    always @(posedge clk) begin
        if (rst_n) latency_config_done <= 1;
        else latency_config_done <= 0;
    end

    always @(*) begin
        uio_in[0] = 1'b1;  // Flash CS readback
        uio_in[3] = 1'b1;  // SCK readback
        uio_in[6] = 1'b1;  // RAM_A CS readback
        uio_in[7] = 1'b1;  // RAM_B CS (no PSRAM B)

        if (!latency_config_done) begin
            // During reset: force latency config = 1 (normal)
            uio_in[1] = 1'b1;  // spi_data_in[0] = 1
            uio_in[2] = 1'b0;  // spi_data_in[1] = 0
            uio_in[4] = 1'b0;  // spi_data_in[2] = 0
            uio_in[5] = 1'b0;
        end else begin
            // After reset: connect external data bus
            uio_in[1] = uio_oe[1] ? uio_out[1] : ext_data_to_dut[0];  // SD0
            uio_in[2] = uio_oe[2] ? uio_out[2] : ext_data_to_dut[1];  // SD1
            uio_in[4] = uio_oe[4] ? uio_out[4] : ext_data_to_dut[2];  // SD2
            uio_in[5] = uio_oe[5] ? uio_out[5] : ext_data_to_dut[3];  // SD3
        end

        // I2C SDA readback — wired-AND of master and slave
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
    // Expected: "OK\nC1S1T1M1I1W1R1E1DN\n" = 21 chars
    localparam EXPECTED_CHARS = 21;
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
        $dumpfile("tb_integration_b.vcd");
        $dumpvars(0, tb_integration_b);

        // ---- Reset ----
        rst_n = 0;
        ui_in = 8'h00;
        #400;

        @(posedge clk);
        @(posedge clk);
        rst_n = 1;

        $display("=== P0-B Integration Test: PSRAM Boot + Full Peripheral Sweep ===");
        $display("Waiting for firmware to complete...");

        // Wait for expected UART chars or timeout (200ms for slow I2C)
        begin : wait_loop
            integer wt;
            for (wt = 0; wt < 5000; wt = wt + 1) begin
                #40000;  // 40us steps, 5000 × 40us = 200ms
                if (uart_idx >= EXPECTED_CHARS) disable wait_loop;
            end
            if (uart_idx < EXPECTED_CHARS)
                $display("[TIMEOUT] Only received %0d UART bytes after 200ms", uart_idx);
        end

        #100000;  // margin

        $display("");
        $display("--- Received %0d UART bytes ---", uart_idx);

        // Check "OK\n" (idx 0,1,2)
        if (uart_idx >= 3 && uart_buf[0] == "O" && uart_buf[1] == "K" && uart_buf[2] == 8'h0A) begin
            $display("[PASS] UART: 'OK\\n'");
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] UART: expected 'OK\\n'");
            fail_count = fail_count + 1;
        end

        // Check test results (2-char pairs starting at idx 3)
        check_2char(3,  "C", "1", "CRC16");
        check_2char(5,  "S", "1", "SYS_INFO");
        check_2char(7,  "T", "1", "Timer");
        check_2char(9,  "M", "1", "PSRAM");
        check_2char(11, "I", "1", "I2C");
        check_2char(13, "W", "1", "WDT");
        check_2char(15, "R", "1", "RTC");
        check_2char(17, "E", "1", "Seal");

        // Check "DN\n" (idx 19,20,21)
        if (uart_idx >= 21 && uart_buf[19] == "D" && uart_buf[20] == "N") begin
            $display("[PASS] Firmware complete: DN");
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] Firmware did not reach completion");
            fail_count = fail_count + 1;
        end

        $display("");
        $display("=== Results: %0d PASS, %0d FAIL ===", pass_count, fail_count);
        if (fail_count == 0)
            $display("ALL TESTS PASSED — P0-B PSRAM boot + full peripheral verified!");
        else
            $display("SOME TESTS FAILED");

        #100;
        $finish;
    end

    // Global watchdog: 200ms (I2C transactions are slow)
    initial begin
        #300_000_000;
        $display("[ABORT] Simulation timeout at 300ms");
        $display("  UART bytes received: %0d", uart_idx);
        $finish;
    end

    // I2C master internal state debug monitor
    reg [3:0] i2c_prev_state;
    reg [3:0] i2c_prev_phy;
    initial begin
        i2c_prev_state = 0;
        i2c_prev_phy = 0;
    end
    always @(posedge clk) begin
        if (dut.i_i2c_peri.i_i2c.state_reg !== i2c_prev_state) begin
            $display("[I2C_MASTER] state %0d->%0d phy=%0d busy=%b sda_i=%b cmd_rdy=%b tvalid=%b missed_ack=%b @ %0t",
                i2c_prev_state,
                dut.i_i2c_peri.i_i2c.state_reg,
                dut.i_i2c_peri.i_i2c.phy_state_reg,
                dut.i_i2c_peri.i_i2c.busy_reg,
                dut.i_i2c_peri.i_i2c.sda_i_reg,
                dut.i_i2c_peri.i_i2c.s_axis_cmd_ready_reg,
                dut.i_i2c_peri.i_i2c.m_axis_data_tvalid_reg,
                dut.i_i2c_peri.i_i2c.missed_ack_reg,
                $time);
            i2c_prev_state <= dut.i_i2c_peri.i_i2c.state_reg;
        end
    end

endmodule
