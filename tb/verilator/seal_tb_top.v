// Seal cross-validation top: seal_register + crc16_engine (shared instance)
// Mirrors tb_seal.v wiring for co-simulation with C++.

/* verilator lint_off UNUSEDSIGNAL */

`timescale 1ns/1ps

module seal_tb_top (
    input         clk,
    input         rst_n,

    // SEAL_DATA bus
    input         data_wr,
    input  [31:0] data_in,
    output [31:0] data_out,
    input         data_rd,

    // SEAL_CTRL bus
    input         ctrl_wr,
    input  [9:0]  ctrl_in,
    output [31:0] ctrl_out,

    // Session counter
    input  [7:0]  session_ctr_in
);

    // CRC16 engine interface (internal wires)
    wire [7:0]  crc_byte;
    wire        crc_feed;
    wire        crc_busy;
    wire [15:0] crc_value;
    wire        crc_init;

    crc16_engine i_crc (
        .clk       (clk),
        .rst_n     (rst_n),
        .init      (crc_init),
        .data_in   (crc_byte),
        .data_valid(crc_feed),
        .crc_out   (crc_value),
        .busy      (crc_busy)
    );

    seal_register i_seal (
        .clk            (clk),
        .rst_n          (rst_n),
        .crc_byte       (crc_byte),
        .crc_feed       (crc_feed),
        .crc_busy       (crc_busy),
        .crc_value      (crc_value),
        .crc_init       (crc_init),
        .data_wr        (data_wr),
        .data_in        (data_in),
        .data_out       (data_out),
        .data_rd        (data_rd),
        .ctrl_wr        (ctrl_wr),
        .ctrl_in        (ctrl_in),
        .ctrl_out       (ctrl_out),
        .session_ctr_in (session_ctr_in)
    );

endmodule
