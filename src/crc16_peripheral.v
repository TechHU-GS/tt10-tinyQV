// ============================================================================
// CRC16 Peripheral — TinyQV MMIO bus bridge for crc16_engine
// ============================================================================
// Slot: PERI_CRC16 (0x2) at 0x8000008
//
// Write: {23'b0, init, data[7:0]}
//   - bit[8] = 1: init (reset CRC to 0xFFFF, data[7:0] ignored)
//   - bit[8] = 0: feed data[7:0] (ignored if engine busy)
//   - init and data are mutually exclusive (init=1 → no data_valid)
//
// Read: {15'b0, busy, crc[15:0]}
//   - busy=1: engine processing, writes ignored
//   - busy=0: crc_out valid
//
// Shared CRC engine: when seal_active=1, this peripheral's writes are
// blocked and reads return busy=1 (arbitration in project.v)
// ============================================================================

module crc16_peripheral (
    input  wire        clk,
    input  wire        rst_n,
    // TinyQV bus interface
    input  wire [31:0] data_in,       // write data
    input  wire        wr_en,         // write enable
    output wire [31:0] data_out,      // read data
    // CRC engine interface (directly connected or muxed in project.v)
    output wire        crc_init,      // init pulse to engine
    output wire [7:0]  crc_data,      // data byte to engine
    output wire        crc_data_valid,// data valid pulse
    input  wire [15:0] crc_value,     // current CRC from engine
    input  wire        crc_busy       // engine busy flag
);

    // Decode write commands
    // init: bit[8]=1 → reset CRC, ignore data[7:0]
    // data: bit[8]=0, engine not busy → feed data[7:0]
    assign crc_init       = wr_en && data_in[8];
    assign crc_data       = data_in[7:0];
    assign crc_data_valid = wr_en && !data_in[8] && !crc_busy;

    // Read: {15'b0, busy, crc[15:0]}
    assign data_out = {15'b0, crc_busy, crc_value};

endmodule
