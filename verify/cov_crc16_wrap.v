`timescale 1ns/1ps
module cov_crc16_wrap(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        wr_en,
    input  wire [31:0] data_in,
    output wire [31:0] data_out
);

    wire        crc_init;
    wire [7:0]  crc_data;
    wire        crc_data_valid;
    wire [15:0] crc_value;
    wire        crc_busy;

    crc16_peripheral peri (
        .clk           (clk),
        .rst_n         (rst_n),
        .data_in       (data_in),
        .wr_en         (wr_en),
        .data_out      (data_out),
        .crc_init      (crc_init),
        .crc_data      (crc_data),
        .crc_data_valid(crc_data_valid),
        .crc_value     (crc_value),
        .crc_busy      (crc_busy)
    );

    crc16_engine eng (
        .clk        (clk),
        .rst_n      (rst_n),
        .init       (crc_init),
        .data_in    (crc_data),
        .data_valid (crc_data_valid),
        .crc_out    (crc_value),
        .busy       (crc_busy)
    );

endmodule
