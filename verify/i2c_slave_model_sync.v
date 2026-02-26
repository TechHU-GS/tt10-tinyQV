// ============================================================================
// I2C Slave Model — Synchronous version for Verilator
// ============================================================================
// Same functionality as test/i2c_slave_model.v but uses system clk +
// edge detection. Avoids Verilator derived-clock issues.
// ============================================================================

`timescale 1ns / 1ps
`default_nettype none

module i2c_slave_model #(
    parameter [6:0] SLAVE_ADDR = 7'h44
) (
    input  wire sys_clk,    // System clock (added for sync model)
    input  wire scl,
    input  wire sda_i,
    output reg  sda_o
);

    // Preset read data (SHT31 temperature + humidity measurement)
    reg [7:0] read_data [0:5];
    initial begin
        read_data[0] = 8'h63;  // T MSB
        read_data[1] = 8'h32;  // T LSB
        read_data[2] = 8'hA1;  // T CRC
        read_data[3] = 8'h8C;  // H MSB
        read_data[4] = 8'hA4;  // H LSB
        read_data[5] = 8'hDB;  // H CRC
        sda_o = 1'b1;
    end

    localparam ST_IDLE     = 0;
    localparam ST_ADDR     = 1;
    localparam ST_ADDR_ACK = 2;
    localparam ST_WR_DATA  = 3;
    localparam ST_WR_ACK   = 4;
    localparam ST_RD_DATA  = 5;
    localparam ST_RD_ACK   = 6;

    reg [3:0] state;
    reg [3:0] bit_count;
    reg [7:0] shift_reg;
    reg       is_read;
    reg [2:0] read_idx;

    // Edge detection
    reg scl_prev, sda_prev;
    wire scl_posedge = scl && !scl_prev;
    wire scl_negedge = !scl && scl_prev;
    wire sda_negedge = !sda_i && sda_prev;
    wire sda_posedge = sda_i && !sda_prev;

    initial begin
        state = ST_IDLE;
        bit_count = 0;
        shift_reg = 0;
        is_read = 0;
        read_idx = 0;
        scl_prev = 1;
        sda_prev = 1;
    end

    always @(posedge sys_clk) begin
        scl_prev <= scl;
        sda_prev <= sda_i;

        // START: SDA falls while SCL high
        if (sda_negedge && scl) begin
            state <= ST_ADDR;
            bit_count <= 0;
            shift_reg <= 0;
            sda_o <= 1'b1;
        end

        // STOP: SDA rises while SCL high
        if (sda_posedge && scl) begin
            state <= ST_IDLE;
            sda_o <= 1'b1;
        end

        // SCL rising edge — sample SDA
        if (scl_posedge) begin
            case (state)
                ST_ADDR: begin
                    shift_reg <= {shift_reg[6:0], sda_i};
                    bit_count <= bit_count + 1;
                end
                ST_WR_DATA: begin
                    shift_reg <= {shift_reg[6:0], sda_i};
                    bit_count <= bit_count + 1;
                end
                ST_RD_ACK: begin
                    if (sda_i) begin
                        state <= ST_IDLE;
                    end
                end
                default: ;
            endcase
        end

        // SCL falling edge — drive SDA
        if (scl_negedge) begin
            case (state)
                ST_ADDR: begin
                    if (bit_count == 8) begin
                        if (shift_reg[7:1] == SLAVE_ADDR) begin
                            is_read <= shift_reg[0];
                            state <= ST_ADDR_ACK;
                            sda_o <= 1'b0;  // ACK
                            read_idx <= 0;
                        end else begin
                            state <= ST_IDLE;
                            sda_o <= 1'b1;  // NACK
                        end
                    end
                end

                ST_ADDR_ACK: begin
                    bit_count <= 0;
                    if (is_read) begin
                        state <= ST_RD_DATA;
                        sda_o <= read_data[0][7] ? 1'b1 : 1'b0;
                    end else begin
                        state <= ST_WR_DATA;
                        sda_o <= 1'b1;
                    end
                end

                ST_WR_DATA: begin
                    if (bit_count == 8) begin
                        state <= ST_WR_ACK;
                        sda_o <= 1'b0;  // ACK
                        bit_count <= 0;
                    end
                end

                ST_WR_ACK: begin
                    state <= ST_WR_DATA;
                    sda_o <= 1'b1;
                    bit_count <= 0;
                end

                ST_RD_DATA: begin
                    bit_count <= bit_count + 1;
                    if (bit_count < 7) begin
                        sda_o <= read_data[read_idx][6 - bit_count] ? 1'b1 : 1'b0;
                    end else if (bit_count == 7) begin
                        state <= ST_RD_ACK;
                        sda_o <= 1'b1;
                    end
                end

                ST_RD_ACK: begin
                    if (read_idx < 5)
                        read_idx <= read_idx + 1;
                    state <= ST_RD_DATA;
                    bit_count <= 0;
                    sda_o <= read_data[(read_idx < 5) ? read_idx + 1 : 5][7] ? 1'b1 : 1'b0;
                end

                default: ;
            endcase
        end
    end

endmodule
