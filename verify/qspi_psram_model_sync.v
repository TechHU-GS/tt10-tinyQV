// ============================================================================
// QSPI PSRAM Model — Synchronous version for Verilator
// ============================================================================
// Same functionality as test/qspi_psram_model.v but uses system clk +
// edge detection instead of always @(posedge spi_clk) / @(negedge spi_clk).
// ============================================================================

`timescale 1ns / 1ps
`default_nettype none

module qspi_psram_model (
    input  wire       sys_clk,       // System clock (added for sync model)
    input  wire       spi_clk,
    input  wire       spi_cs_n,
    input  wire [3:0] spi_data_in,
    output reg  [3:0] spi_data_out,
    input  wire [3:0] spi_data_oe
);

    localparam MEM_SIZE = 8192;
    localparam ADDR_BITS = 13;
    reg [7:0] mem [0:MEM_SIZE-1];

    integer init_i;
    initial begin
        for (init_i = 0; init_i < MEM_SIZE; init_i = init_i + 1)
            mem[init_i] = 8'hFF;
        spi_data_out = 4'hF;
    end

    localparam S_CMD   = 0;
    localparam S_ADDR  = 1;
    localparam S_DUMMY = 2;
    localparam S_RDATA = 3;
    localparam S_WDATA = 4;

    reg [2:0]  state;
    reg [3:0]  count;
    reg [7:0]  cmd_reg;
    reg [23:0] addr_shift;
    reg [ADDR_BITS-1:0] byte_addr;
    reg        phase;
    reg [7:0]  wr_byte;

    // Edge detection
    reg spi_clk_prev;
    reg spi_cs_n_prev;
    wire spi_clk_posedge = spi_clk && !spi_clk_prev;
    wire spi_clk_negedge = !spi_clk && spi_clk_prev;
    wire spi_cs_n_posedge = spi_cs_n && !spi_cs_n_prev;

    initial begin
        spi_clk_prev = 0;
        spi_cs_n_prev = 1;
        state = S_CMD;
        count = 0;
    end

    always @(posedge sys_clk) begin
        spi_clk_prev <= spi_clk;
        spi_cs_n_prev <= spi_cs_n;

        // CS deassert -> reset
        if (spi_cs_n_posedge) begin
            state <= S_CMD;
            count <= 0;
            spi_data_out <= 4'hF;
        end

        // Rising edge: capture and process
        if (spi_clk_posedge && !spi_cs_n) begin
            case (state)
                S_CMD: begin
                    cmd_reg <= {cmd_reg[3:0], spi_data_in};
                    if (count == 4'd1) begin
                        state <= S_ADDR;
                        count <= 0;
                    end else begin
                        count <= count + 1;
                    end
                end

                S_ADDR: begin
                    addr_shift <= {addr_shift[19:0], spi_data_in};
                    if (count == 4'd5) begin
                        if (cmd_reg == 8'h0B) begin
                            state <= S_DUMMY;
                        end else begin
                            state <= S_WDATA;
                            phase <= 0;
                        end
                        count <= 0;
                        byte_addr <= {addr_shift[ADDR_BITS-5:0], spi_data_in};
                    end else begin
                        count <= count + 1;
                    end
                end

                S_DUMMY: begin
                    if (count == 4'd3) begin
                        state <= S_RDATA;
                        count <= 0;
                        phase <= 0;
                        byte_addr <= addr_shift[ADDR_BITS-1:0];
                    end else begin
                        count <= count + 1;
                    end
                end

                S_RDATA: begin
                    if (phase) begin
                        byte_addr <= byte_addr + 1;
                        phase <= 0;
                    end else begin
                        phase <= 1;
                    end
                end

                S_WDATA: begin
                    if (!phase) begin
                        wr_byte[7:4] <= spi_data_in;
                        phase <= 1;
                    end else begin
                        wr_byte[3:0] <= spi_data_in;
                        if (byte_addr < MEM_SIZE)
                            mem[byte_addr] <= {wr_byte[7:4], spi_data_in};
                        byte_addr <= byte_addr + 1;
                        phase <= 0;
                    end
                end
            endcase
        end

        // Falling edge: drive read data
        if (spi_clk_negedge) begin
            if (!spi_cs_n && state == S_RDATA) begin
                if (!phase)
                    spi_data_out <= (byte_addr < MEM_SIZE) ? mem[byte_addr][7:4] : 4'hF;
                else
                    spi_data_out <= (byte_addr < MEM_SIZE) ? mem[byte_addr][3:0] : 4'hF;
            end else begin
                spi_data_out <= 4'hF;
            end
        end
    end

endmodule
