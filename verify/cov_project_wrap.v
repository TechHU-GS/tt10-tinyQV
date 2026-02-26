// ============================================================================
// cov_project_wrap.v — Verilator coverage wrapper for full SoC
// ============================================================================
// Instantiates tt_um_MichaelBell_tinyQV (DUT) + synchronous flash/PSRAM/I2C
// models. Uses sys_clk edge detection to avoid Verilator derived-clock issues.
// ============================================================================

`timescale 1ns / 1ps
`default_nettype none

module cov_project_wrap (
    input  wire       clk,
    input  wire       rst_n,
    output wire [7:0] uo_out,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe
);

    // TT interface signals
    reg  [7:0] ui_in;
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
    // QSPI Flash Model (synchronous — uses sys_clk edge detection)
    // ================================================================
    wire flash_cs_n = uio_out[0];
    wire spi_clk    = uio_out[3];

    wire [3:0] qspi_data_to_flash = {uio_out[5], uio_out[4], uio_out[2], uio_out[1]};
    wire [3:0] qspi_data_from_flash;
    wire [3:0] qspi_oe = {uio_oe[5], uio_oe[4], uio_oe[2], uio_oe[1]};

    qspi_flash_model #(.HEX_FILE("fw_post.hex")) i_flash (
        .sys_clk     (clk),
        .spi_clk     (spi_clk),
        .spi_cs_n    (flash_cs_n),
        .spi_data_in (qspi_data_to_flash),
        .spi_data_out(qspi_data_from_flash),
        .spi_data_oe (qspi_oe)
    );

    // ================================================================
    // QSPI PSRAM Model (synchronous)
    // ================================================================
    wire ram_a_cs_n = uio_out[6];

    wire [3:0] qspi_data_to_psram = {uio_out[5], uio_out[4], uio_out[2], uio_out[1]};
    wire [3:0] qspi_data_from_psram;

    qspi_psram_model i_psram (
        .sys_clk     (clk),
        .spi_clk     (spi_clk),
        .spi_cs_n    (ram_a_cs_n),
        .spi_data_in (qspi_data_to_psram),
        .spi_data_out(qspi_data_from_psram),
        .spi_data_oe (qspi_oe)
    );

    // ================================================================
    // I2C Slave Model (synchronous)
    // ================================================================
    wire i2c_scl = uo_out[2];
    wire i2c_sda_master = uo_out[6];

    wire slave_sda_o;
    wire sda_bus_value = i2c_sda_master & slave_sda_o;

    i2c_slave_model #(.SLAVE_ADDR(7'h44)) i_sht31 (
        .sys_clk(clk),
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
    // 1PPS Generator — inject periodic pulses for coverage
    // ================================================================
    reg [19:0] pps_timer;
    reg        pps_gen;
    always @(posedge clk) begin
        if (!rst_n) begin
            pps_timer <= 0;
            pps_gen   <= 0;
        end else begin
            pps_timer <= pps_timer + 1;
            // Generate a 500-cycle pulse every 100,000 cycles (~4ms @ 25MHz)
            // This gives several edges during 80M cycle POST run
            if (pps_timer == 20'd99_999)
                pps_timer <= 0;
            pps_gen <= (pps_timer < 20'd500);
        end
    end

    // ================================================================
    // Pin Connection — Latency Config + Flash/PSRAM Mux
    // ================================================================
    reg latency_config_done;
    always @(posedge clk) begin
        if (rst_n) latency_config_done <= 1;
        else latency_config_done <= 0;
    end

    always @(*) begin
        // uio_in: QSPI data bus + latency config
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

        // ui_in: dedicated input pins
        ui_in[0] = 1'b0;       // DIO1 (IRQ) - tie low
        ui_in[1] = 1'b0;       // SX1268 BUSY - tie low
        ui_in[2] = 1'b1;       // SPI MISO - tie high
        ui_in[3] = sda_bus_value; // I2C SDA readback
        ui_in[4] = pps_gen;    // 1PPS - generated pulses for coverage
        ui_in[5] = 1'b0;       // spare GPIO
        ui_in[6] = 1'b0;       // spare GPIO
        ui_in[7] = 1'b1;       // UART RX - tie high (idle)
    end

endmodule
