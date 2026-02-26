// cov_i2c_wrap.v — Verilator coverage wrapper for i2c_peripheral + i2c_master
// Exposes MMIO interface + I2C physical pins for C++ testbench.
`timescale 1ns/1ps

module cov_i2c_wrap (
    input  wire        clk,
    input  wire        rst_n,

    // MMIO — Slot 0x6: I2C_DATA
    input  wire [31:0] data_in,
    input  wire        data_wr,
    input  wire        data_rd,
    output wire [31:0] data_out,

    // MMIO — Slot 0x7: I2C_CONFIG
    input  wire [31:0] config_in,
    input  wire        config_wr,
    output wire [31:0] config_out,

    // I2C physical pins (directly exposed)
    input  wire        scl_i,
    output wire        scl_o,
    output wire        scl_t,
    input  wire        sda_i,
    output wire        sda_o,
    output wire        sda_t
);

    i2c_peripheral i_peri (
        .clk        (clk),
        .rst_n      (rst_n),
        .data_in    (data_in),
        .data_wr    (data_wr),
        .data_rd    (data_rd),
        .data_out   (data_out),
        .config_in  (config_in),
        .config_wr  (config_wr),
        .config_out (config_out),
        .scl_i      (scl_i),
        .scl_o      (scl_o),
        .scl_t      (scl_t),
        .sda_i      (sda_i),
        .sda_o      (sda_o),
        .sda_t      (sda_t)
    );

endmodule
