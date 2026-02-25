// ============================================================================
// Minimal QSPI PSRAM Model for Integration Testbench
// ============================================================================
// Emulates APS6404L-3SQR in SPI mode (not QPI).
// TinyQV's qspi_ctrl uses:
//   Read:  CMD 0x0B (2 nibbles) → ADDR (6 nibbles) → skip DUMMY1 → DUMMY2 (4 nibbles) → DATA
//   Write: CMD 0x02 (2 nibbles) → ADDR (6 nibbles) → DATA (no dummy)
//
// The controller drives spi_data_oe=0xF during CMD+ADDR+WRITE_DATA phases,
// and spi_data_oe=0x0 during DUMMY+READ_DATA phases.
//
// Memory: 8KB (sufficient for firmware stack + data)
// ============================================================================

`timescale 1ns / 1ps

module qspi_psram_model (
    input  wire       spi_clk,
    input  wire       spi_cs_n,      // active low
    input  wire [3:0] spi_data_in,   // data FROM controller TO psram
    output reg  [3:0] spi_data_out,  // data FROM psram TO controller
    input  wire [3:0] spi_data_oe    // controller OE (1=controller drives)
);

    // 8KB memory (addresses 0x000 - 0x1FFF)
    localparam MEM_SIZE = 8192;
    localparam ADDR_BITS = 13;
    reg [7:0] mem [0:MEM_SIZE-1];

    integer init_i;
    initial begin
        for (init_i = 0; init_i < MEM_SIZE; init_i = init_i + 1)
            mem[init_i] = 8'hFF;
        spi_data_out = 4'hF;
    end

    // Protocol states
    localparam S_CMD   = 0;  // Receiving 2 command nibbles
    localparam S_ADDR  = 1;  // Receiving 6 address nibbles
    localparam S_DUMMY = 2;  // 4 dummy nibbles (read only)
    localparam S_RDATA = 3;  // Driving read data nibbles
    localparam S_WDATA = 4;  // Receiving write data nibbles

    reg [2:0]  state;
    reg [3:0]  count;
    reg [7:0]  cmd_reg;
    reg [23:0] addr_shift;
    reg [ADDR_BITS-1:0] byte_addr;
    reg        phase;         // 0=high nibble, 1=low nibble
    reg [7:0]  wr_byte;       // accumulate write nibbles

    // On CS deassert, reset state
    always @(posedge spi_cs_n) begin
        state <= S_CMD;
        count <= 0;
        spi_data_out <= 4'hF;
    end

    // Main logic: capture on rising edge
    always @(posedge spi_clk) begin
        if (!spi_cs_n) begin
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
                        // Check command: 0x0B = read, 0x02 = write
                        if (cmd_reg == 8'h0B) begin
                            state <= S_DUMMY;
                        end else begin
                            state <= S_WDATA;
                            phase <= 0;
                        end
                        count <= 0;
                        // Include current nibble (non-blocking: addr_shift is stale)
                        byte_addr <= {addr_shift[ADDR_BITS-5:0], spi_data_in};
                    end else begin
                        count <= count + 1;
                    end
                end

                S_DUMMY: begin
                    // 4 dummy nibbles for read command
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
                    // Advance address after low nibble
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
                        if (byte_addr < MEM_SIZE) begin
                            mem[byte_addr] <= {wr_byte[7:4], spi_data_in};
                            // $display("[PSRAM] WRITE @0x%04X = 0x%02X", byte_addr, {wr_byte[7:4], spi_data_in});
                        end
                        byte_addr <= byte_addr + 1;
                        phase <= 0;
                    end
                end
            endcase
        end
    end

    // Drive read data on negedge (half-cycle setup time)
    always @(negedge spi_clk) begin
        if (!spi_cs_n && state == S_RDATA) begin
            if (!phase) begin
                spi_data_out <= (byte_addr < MEM_SIZE) ? mem[byte_addr][7:4] : 4'hF;
            end else begin
                spi_data_out <= (byte_addr < MEM_SIZE) ? mem[byte_addr][3:0] : 4'hF;
            end
        end else begin
            spi_data_out <= 4'hF;
        end
    end

endmodule
