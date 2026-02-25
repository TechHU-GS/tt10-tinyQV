// ============================================================================
// CRC16-MODBUS Engine — bit-serial, 8 cycles per byte
// ============================================================================
// Polynomial: 0xA001 (reflected 0x8005), Init: 0xFFFF
// Interface: write data_in + pulse data_valid, wait 8 clocks, read crc_out
// Compatible with LoraLite protocol CRC16 (loralite_protocol.hpp L174)
//
// Origin: GS_IC/designs/tt_loralite_ihp/src/crc16_engine.v
// Modified: async reset → sync reset (IHP SG13G2 / TinyQV consistency)
//
// Usage:
//   1. Assert init to reset CRC to 0xFFFF
//   2. For each byte: set data_in, pulse data_valid for 1 cycle
//   3. Wait 8 cycles (busy=1)
//   4. After all bytes, read crc_out
//   5. Verification: feed data+CRC bytes, result should be 0x0000
//
// Interface timing rules:
//   - init takes priority over data_valid (if both asserted, only init executes)
//   - data_valid while busy is silently ignored (caller must poll busy=0 first)
//   - busy goes high on the SAME cycle as data_valid (combinational from bit_cnt)
//   - crc_out is valid when busy=0 (stable between bytes)
// ============================================================================

module crc16_engine (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        init,         // reset CRC to 0xFFFF
    input  wire [7:0]  data_in,      // input byte
    input  wire        data_valid,   // pulse to start processing
    output wire [15:0] crc_out,      // current CRC value
    output wire        busy          // high during 8-bit processing
);

    reg [15:0] crc_reg;
    reg [3:0]  bit_cnt;     // 0=idle, 1-8=processing

    assign crc_out = crc_reg;
    assign busy    = (bit_cnt != 4'd0);

    always @(posedge clk) begin
        if (!rst_n) begin
            crc_reg   <= 16'hFFFF;
            bit_cnt   <= 4'd0;
        end else if (init) begin
            crc_reg   <= 16'hFFFF;
            bit_cnt   <= 4'd0;
        end else if (data_valid && bit_cnt == 4'd0) begin
            // Load new byte, XOR LSB with CRC
            bit_cnt   <= 4'd8;
            crc_reg   <= crc_reg ^ {8'd0, data_in};
        end else if (bit_cnt != 4'd0) begin
            // Process one bit per cycle (LSB first, reflected algorithm)
            if (crc_reg[0]) begin
                crc_reg <= (crc_reg >> 1) ^ 16'hA001;
            end else begin
                crc_reg <= crc_reg >> 1;
            end
            bit_cnt <= bit_cnt - 4'd1;
        end
    end

endmodule
