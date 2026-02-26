// ============================================================================
// QSPI Flash Model — Synchronous version for Verilator
// ============================================================================
// Same functionality as test/qspi_flash_model.v but uses system clk +
// edge detection instead of always @(posedge spi_clk) / @(negedge spi_clk).
// This avoids Verilator derived-clock scheduling issues.
// ============================================================================

`timescale 1ns / 1ps
`default_nettype none

module qspi_flash_model #(
    parameter HEX_FILE = "fw_p0a.hex"
) (
    input  wire       sys_clk,       // System clock (added for sync model)
    input  wire       spi_clk,
    input  wire       spi_cs_n,
    input  wire [3:0] spi_data_in,
    output reg  [3:0] spi_data_out,
    input  wire [3:0] spi_data_oe
);

    // 256KB memory
    reg [7:0] mem [0:262143];

    integer init_i;
    initial begin
        for (init_i = 0; init_i < 262144; init_i = init_i + 1)
            mem[init_i] = 8'hFF;
        $readmemh(HEX_FILE, mem);
    end

    // Protocol states
    localparam S_ADDR  = 0;
    localparam S_MODE  = 1;
    localparam S_DUMMY = 2;
    localparam S_DATA  = 3;

    reg [2:0]  state;
    reg [3:0]  count;
    reg [23:0] addr_shift;
    reg [17:0] byte_addr;
    reg        phase;

    // Edge detection
    reg spi_clk_prev;
    reg spi_cs_n_prev;
    wire spi_clk_posedge = spi_clk && !spi_clk_prev;
    wire spi_clk_negedge = !spi_clk && spi_clk_prev;
    wire spi_cs_n_posedge = spi_cs_n && !spi_cs_n_prev;

    initial begin
        spi_data_out = 4'hF;
        state = S_ADDR;
        count = 0;
        spi_clk_prev = 0;
        spi_cs_n_prev = 1;
    end

    always @(posedge sys_clk) begin
        spi_clk_prev <= spi_clk;
        spi_cs_n_prev <= spi_cs_n;

        // CS deassert -> reset
        if (spi_cs_n_posedge) begin
            state <= S_ADDR;
            count <= 0;
            spi_data_out <= 4'hF;
        end

        // Rising edge of spi_clk: capture address/mode/dummy, advance read
        if (spi_clk_posedge && !spi_cs_n) begin
            case (state)
                S_ADDR: begin
                    addr_shift <= {addr_shift[19:0], spi_data_in};
                    if (count == 4'd5) begin
                        state <= S_MODE;
                        count <= 0;
                    end else begin
                        count <= count + 1;
                    end
                end

                S_MODE: begin
                    if (count == 4'd1) begin
                        state <= S_DUMMY;
                        count <= 0;
                        byte_addr <= addr_shift[17:0];
                    end else begin
                        count <= count + 1;
                    end
                end

                S_DUMMY: begin
                    if (count == 4'd3) begin
                        state <= S_DATA;
                        count <= 0;
                        phase <= 0;
                    end else begin
                        count <= count + 1;
                    end
                end

                S_DATA: begin
                    if (phase) begin
                        byte_addr <= byte_addr + 1;
                        phase <= 0;
                    end else begin
                        phase <= 1;
                    end
                end
            endcase
        end

        // Falling edge of spi_clk: drive data
        if (spi_clk_negedge) begin
            if (!spi_cs_n && state == S_DATA) begin
                if (!phase)
                    spi_data_out <= mem[byte_addr][7:4];
                else
                    spi_data_out <= mem[byte_addr][3:0];
            end else begin
                spi_data_out <= 4'hF;
            end
        end
    end

endmodule
