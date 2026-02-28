/*
 * FPGA Wrapper — Alchitry Cu V2 (iCE40HX8K-CB132)
 * LoRa Edge SoC FPGA verification target
 *
 * PLL: 100MHz board oscillator -> 25MHz system clock
 * SB_IO: QSPI bidirectional pin handling
 * Reset: button debounce + 2FF synchronizer
 *
 * Copyright (c) 2026 TechHU-GS
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module fpga_top (
    input  wire       clk_100m,     // 100MHz board oscillator (P7)
    input  wire       rst_n_btn,    // Reset button, active low (P8)

    // On-board LEDs
    output wire [7:0] led,          // Active high LEDs

    // On-board FTDI UART
    output wire       usb_tx,       // UART TX -> FTDI -> USB (M9)
    input  wire       usb_rx,       // UART RX <- FTDI <- USB (P14)

    // QSPI Flash + PSRAM (directly to SB_IO, active low CS)
    inout  wire       flash_cs,     // QSPI Flash chip select
    inout  wire [3:0] sd,           // QSPI data lines
    inout  wire       qspi_sck,     // QSPI clock
    inout  wire       ram_a_cs,     // PSRAM A chip select
    inout  wire       ram_b_cs,     // PSRAM B chip select

    // SX1268 LoRa transceiver (directly to breakout/IO board)
    input  wire       sx_dio1,      // SX1268 DIO1 (IRQ)
    input  wire       sx_busy,      // SX1268 BUSY
    output wire       sx_reset,     // SX1268 RESET
    input  wire       sx_miso,      // SPI MISO from SX1268
    output wire       sx_mosi,      // SPI MOSI to SX1268
    output wire       sx_cs,        // SPI CS to SX1268
    output wire       sx_sck,       // SPI SCK to SX1268

    // I2C (directly to breakout/IO board)
    input  wire       i2c_sda_i,    // SDA input (open-drain readback)
    output wire       i2c_sda_o,    // SDA output (0=pull low, 1=release)
    output wire       i2c_scl_o,    // SCL output (0=pull low, 1=release)

    // Misc
    input  wire       pps_in,       // 1PPS from GPS
    input  wire       gpio_in5,     // Spare GPIO / DIP switch
    input  wire       gpio_in6      // Spare GPIO / DIP switch
);

    // ================================================================
    // PLL: 100MHz -> 25MHz
    // VCO = 100 * (DIVF+1) / (DIVR+1) = 100 * 8 / 1 = 800MHz
    // F_out = VCO / 2^DIVQ = 800 / 32 = 25MHz
    // ================================================================
    wire clk_25m;
    wire pll_locked;

    SB_PLL40_CORE #(
        .FEEDBACK_PATH("SIMPLE"),
        .DIVR(4'b0000),            // DIVR = 0
        .DIVF(7'b0000111),         // DIVF = 7
        .DIVQ(3'b101),             // DIVQ = 5 -> /32
        .FILTER_RANGE(3'b101)      // For 100MHz input
    ) i_pll (
        .REFERENCECLK(clk_100m),
        .PLLOUTCORE(clk_25m),
        .LOCK(pll_locked),
        .RESETB(1'b1),
        .BYPASS(1'b0)
    );

    // ================================================================
    // Reset synchronizer: button debounce + PLL lock
    // ================================================================
    reg [2:0] rst_sync;
    wire rst_n_internal = rst_sync[2];

    always @(posedge clk_25m or negedge pll_locked) begin
        if (!pll_locked)
            rst_sync <= 3'b000;
        else
            rst_sync <= {rst_sync[1:0], rst_n_btn};
    end

    // ================================================================
    // QSPI bidirectional pins via SB_IO
    // PIN_TYPE = 6'b1010_01: registered output, registered input
    // ================================================================
    wire [3:0] qspi_data_in;
    wire [3:0] qspi_data_out;
    wire [3:0] qspi_data_oe;

    // Data pins: sd[3:0] — dynamically switched between input/output
    SB_IO #(
        .PIN_TYPE(6'b1010_01),
        .PULLUP(1'b0)
    ) qspi_data_io [3:0] (
        .PACKAGE_PIN(sd),
        .OUTPUT_CLK(clk_25m),
        .INPUT_CLK(clk_25m),
        .OUTPUT_ENABLE(qspi_data_oe),
        .D_OUT_0(qspi_data_out),
        .D_IN_0(qspi_data_in)
    );

    // Control pins: flash_cs, qspi_sck, ram_a_cs, ram_b_cs — always output (when not reset)
    wire       ctrl_flash_cs;
    wire       ctrl_sck;
    wire       ctrl_ram_a_cs;
    wire       ctrl_ram_b_cs;

    SB_IO #(
        .PIN_TYPE(6'b1010_01),
        .PULLUP(1'b0)
    ) qspi_ctrl_io [3:0] (
        .PACKAGE_PIN({flash_cs, qspi_sck, ram_a_cs, ram_b_cs}),
        .OUTPUT_CLK(clk_25m),
        .OUTPUT_ENABLE({4{rst_n_internal}}),
        .D_OUT_0({ctrl_flash_cs, ctrl_sck, ctrl_ram_a_cs, ctrl_ram_b_cs})
    );

    // ================================================================
    // TT interface wires
    // ================================================================
    wire [7:0] tt_ui_in;
    wire [7:0] tt_uo_out;
    wire [7:0] tt_uio_in;
    wire [7:0] tt_uio_out;
    wire [7:0] tt_uio_oe;

    // ================================================================
    // Input mapping: physical pins -> tt_ui_in
    // ================================================================
    assign tt_ui_in[0] = sx_dio1;       // SX1268 DIO1 (IRQ)
    assign tt_ui_in[1] = sx_busy;       // SX1268 BUSY
    assign tt_ui_in[2] = sx_miso;       // SPI MISO
    assign tt_ui_in[3] = i2c_sda_i;    // I2C SDA input
    assign tt_ui_in[4] = pps_in;        // 1PPS input
    assign tt_ui_in[5] = gpio_in5;      // DIP switch / spare
    assign tt_ui_in[6] = gpio_in6;      // DIP switch / spare
    assign tt_ui_in[7] = usb_rx;        // UART RX from FTDI

    // ================================================================
    // Output mapping: tt_uo_out -> physical pins
    // ================================================================
    assign usb_tx    = tt_uo_out[0];    // UART TX -> FTDI
    assign sx_reset  = tt_uo_out[1];    // SX1268 RESET
    assign i2c_scl_o = tt_uo_out[2];    // I2C SCL
    assign sx_mosi   = tt_uo_out[3];    // SPI MOSI
    assign sx_cs     = tt_uo_out[4];    // SPI CS
    assign sx_sck    = tt_uo_out[5];    // SPI SCK
    assign i2c_sda_o = tt_uo_out[6];    // I2C SDA
    assign led[0]    = tt_uo_out[7];    // LED GPIO

    // Remaining LEDs unused (active low = off for active-high LEDs -> drive 0)
    assign led[7:1] = 7'b0;

    // ================================================================
    // Bidirectional mapping: uio <-> SB_IO QSPI pins
    //
    // project.v uio_out = {ram_b_cs, ram_a_cs, sd[3:2], sck, sd[1:0], flash_cs}
    // project.v uio_oe  = {1,1, oe[3:2], 1, oe[1:0], 1} (when rst_reg_n=1)
    //
    // We feed the SB_IO-registered inputs back through uio_in,
    // and connect the SB_IO outputs from uio_out.
    // ================================================================

    // uio_out -> QSPI control signals
    assign ctrl_flash_cs = tt_uio_out[0];
    assign ctrl_sck      = tt_uio_out[3];
    assign ctrl_ram_a_cs = tt_uio_out[6];
    assign ctrl_ram_b_cs = tt_uio_out[7];

    // uio_out -> QSPI data signals
    assign qspi_data_out = {tt_uio_out[5:4], tt_uio_out[2:1]};

    // uio_oe -> QSPI data output enables (control pins always driven by SB_IO OE)
    assign qspi_data_oe  = {tt_uio_oe[5:4], tt_uio_oe[2:1]};

    // QSPI data inputs -> uio_in (control pins don't need readback)
    assign tt_uio_in[0] = 1'b0;                // flash_cs — output only
    assign tt_uio_in[1] = qspi_data_in[0];     // SD0
    assign tt_uio_in[2] = qspi_data_in[1];     // SD1
    assign tt_uio_in[3] = 1'b0;                // SCK — output only
    assign tt_uio_in[4] = qspi_data_in[2];     // SD2
    assign tt_uio_in[5] = qspi_data_in[3];     // SD3
    assign tt_uio_in[6] = 1'b0;                // RAM A CS — output only
    assign tt_uio_in[7] = 1'b0;                // RAM B CS — output only

    // ================================================================
    // Design under test: LoRa Edge SoC
    // ================================================================
    tt_um_techhu_rv32_trial i_dut (
        .ui_in   (tt_ui_in),
        .uo_out  (tt_uo_out),
        .uio_in  (tt_uio_in),
        .uio_out (tt_uio_out),
        .uio_oe  (tt_uio_oe),
        .ena     (1'b1),
        .clk     (clk_25m),
        .rst_n   (rst_n_internal)
    );

endmodule
