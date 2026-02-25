// ============================================================================
// Minimal QSPI Flash Model for P0-A Integration Testbench
// ============================================================================
// Emulates a W25Q128 in Quad I/O continuous read mode.
// TinyQV's qspi_ctrl puts Flash in this mode at initialization time.
// In continuous read mode, the protocol is:
//   CS low → 6 addr nibbles → 2 mode nibbles (0xA0) → 4 dummy → data...
// No CMD byte in continuous mode (unlike PSRAM which sends 0Bh).
//
// The qspi_ctrl clocks: posedge of spi_clk_out = data output from controller,
// negedge = controller samples input. But the actual TT mux spi_clk goes through
// the pin mux, so the timing model here is:
//   - Flash latches address on RISING edge of spi_clk (same as controller output)
//   - Flash drives data that is valid on the NEXT rising edge (1-cycle latency)
//
// Memory: 256KB, initialized via $readmemh("fw_p0a.hex")
// ============================================================================

`timescale 1ns / 1ps

module qspi_flash_model (
    input  wire       spi_clk,
    input  wire       spi_cs_n,      // active low
    input  wire [3:0] spi_data_in,   // data FROM controller TO flash
    output reg  [3:0] spi_data_out,  // data FROM flash TO controller
    input  wire [3:0] spi_data_oe    // controller OE (1=controller drives)
);

    // 256KB memory
    reg [7:0] mem [0:262143];

    integer init_i;
    initial begin
        for (init_i = 0; init_i < 262144; init_i = init_i + 1)
            mem[init_i] = 8'hFF;
        $readmemh("fw_p0a.hex", mem);
    end

    // Protocol states
    localparam S_ADDR  = 0;  // Receiving 6 address nibbles
    localparam S_MODE  = 1;  // Receiving 2 mode nibbles
    localparam S_DUMMY = 2;  // 4 dummy nibbles (we drive high-Z/FF)
    localparam S_DATA  = 3;  // Driving data nibbles

    reg [2:0]  state;
    reg [3:0]  count;        // nibble counter within state
    reg [23:0] addr_shift;   // address accumulator
    reg [17:0] byte_addr;    // current read address
    reg        phase;        // 0=high nibble, 1=low nibble

    // Initialize output
    initial begin
        spi_data_out = 4'hF;
        state = S_ADDR;
        count = 0;
    end

    // On CS deassert, reset state
    always @(posedge spi_cs_n) begin
        state <= S_ADDR;
        count <= 0;
        spi_data_out <= 4'hF;
    end

    // Main logic: capture on rising edge (controller outputs data on rising edge)
    always @(posedge spi_clk) begin
        if (!spi_cs_n) begin
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
                    // Mode bits — we don't check them, just count
                    if (count == 4'd1) begin
                        state <= S_DUMMY;
                        count <= 0;
                        // Latch the full 24-bit address (already shifted in)
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
                    // Data phase: we output on negedge, here we advance address
                    if (phase) begin
                        byte_addr <= byte_addr + 1;
                        phase <= 0;
                    end else begin
                        phase <= 1;
                    end
                end
            endcase
        end
    end

    // Drive data output
    // The qspi_ctrl with delay_cycles_cfg=1 samples data on the rising edge
    // of the *next* spi_clk after it was driven.
    // Since spi_clk_out toggles every system clk cycle:
    //   - posedge spi_clk_out: controller checks nibble count, advances state
    //   - negedge spi_clk_out: (falling edge, driven by negedge of system clk or pos)
    //
    // For simulation without TT mux delay, data must be stable before rising edge.
    // Driving on negedge gives half-cycle setup.
    always @(negedge spi_clk) begin
        if (!spi_cs_n && state == S_DATA) begin
            if (!phase) begin
                spi_data_out <= mem[byte_addr][7:4];
                // $display("[FLASH] @%0t addr=0x%05X byte=0x%02X hi", $time, byte_addr, mem[byte_addr]);
            end else begin
                spi_data_out <= mem[byte_addr][3:0];
            end
        end else begin
            spi_data_out <= 4'hF;
        end
    end

endmodule
