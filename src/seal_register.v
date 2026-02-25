// Seal Register + Monotonic Counter
// Hardware integrity watermark with non-backfillable ordering proof.
//
// Write flow:
//   1. (optional) Write SEAL_CTRL bit[0]=1 → CRC16 reset to 0xFFFF
//   2. Write SEAL_DATA = value[31:0]
//   3. Write SEAL_CTRL = {sensor_id[7:0], commit=1, crc_reset=0}
//   → Hardware: mono_count++, CRC16(sensor_id + value + mono_count), latch record
//
// Read flow (3x SEAL_DATA reads):
//   Read 0: value[31:0]
//   Read 1: {session_id[7:0], mono_count[23:0]}
//   Read 2: {mono_count[31:24], crc16[15:0], 8'h00}
//   → 2-bit read counter auto-increments, wraps after 3
//   → commit forces read counter to 0

module seal_register (
    input         clk,
    input         rst_n,

    // CRC16 engine interface (shared instance)
    output reg [7:0] crc_byte,
    output reg       crc_feed,
    input            crc_busy,
    input  [15:0]    crc_value,
    output reg       crc_init,

    // Bus interface — SEAL_DATA (slot 0xB)
    input         data_wr,
    input  [31:0] data_in,
    output [31:0] data_out,
    input         data_rd,

    // Bus interface — SEAL_CTRL (slot 0xE)
    input         ctrl_wr,
    input  [9:0]  ctrl_in,      // {sensor_id[7:0], commit, crc_reset}
    output [31:0] ctrl_out,

    // Session counter input (from project.v free-running counter)
    input  [7:0]  session_ctr_in
);

    // ================================================================
    // State machine
    // ================================================================
    localparam S_IDLE       = 2'd0;
    localparam S_FEED_BYTES = 2'd1;
    localparam S_LATCH      = 2'd2;

    reg [1:0] state;

    // ================================================================
    // Working registers
    // ================================================================
    reg [31:0] value_reg;       // latched on data_wr
    reg [7:0]  sensor_id_reg;   // latched on commit
    reg [31:0] cur_mono;        // mono_count snapshot at commit time

    // Monotonic counter (persists across commits within power cycle)
    reg [31:0] mono_count;

    // Session ID (locked on first commit)
    reg [7:0]  session_id;
    reg        session_locked;

    // Sealed record (latched after CRC computation)
    reg [31:0] sealed_value;
    reg [31:0] sealed_mono;
    reg [15:0] sealed_crc;
    reg [7:0]  sealed_sid;

    // Byte feed index (0..8 = 9 bytes)
    reg [3:0]  byte_idx;
    // Track if we already sent current byte (wait for busy to clear)
    reg        byte_sent;

    // ================================================================
    // 3x read serialization
    // ================================================================
    reg [1:0] read_seq;

    always @(posedge clk) begin
        if (!rst_n)
            read_seq <= 0;
        else if (state == S_IDLE && ctrl_wr && ctrl_in[1])
            read_seq <= 0;  // commit forces reset
        else if (data_rd)
            read_seq <= (read_seq == 2'd2) ? 2'd0 : read_seq + 1;
    end

    assign data_out =
        (read_seq == 2'd0) ? sealed_value :
        (read_seq == 2'd1) ? {sealed_sid, sealed_mono[23:0]} :
                             {sealed_mono[31:24], sealed_crc, 8'h00};

    // ================================================================
    // Status output
    // ================================================================
    wire seal_busy  = (state != S_IDLE);
    wire seal_ready = (state == S_IDLE);
    assign ctrl_out = {30'b0, seal_ready, seal_busy};

    // ================================================================
    // Byte mux for CRC feed sequence
    // ================================================================
    reg [7:0] feed_byte;
    always @(*) begin
        case (byte_idx)
            4'd0: feed_byte = sensor_id_reg;
            4'd1: feed_byte = value_reg[7:0];
            4'd2: feed_byte = value_reg[15:8];
            4'd3: feed_byte = value_reg[23:16];
            4'd4: feed_byte = value_reg[31:24];
            4'd5: feed_byte = cur_mono[7:0];
            4'd6: feed_byte = cur_mono[15:8];
            4'd7: feed_byte = cur_mono[23:16];
            4'd8: feed_byte = cur_mono[31:24];
            default: feed_byte = 8'h00;
        endcase
    end

    // ================================================================
    // Main state machine
    // ================================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            state          <= S_IDLE;
            value_reg      <= 32'd0;
            sensor_id_reg  <= 8'd0;
            cur_mono       <= 32'd0;
            mono_count     <= 32'd0;
            session_id     <= 8'd0;
            session_locked <= 1'b0;
            sealed_value   <= 32'd0;
            sealed_mono    <= 32'd0;
            sealed_crc     <= 16'd0;
            sealed_sid     <= 8'd0;
            byte_idx       <= 4'd0;
            byte_sent      <= 1'b0;
            crc_byte       <= 8'd0;
            crc_feed       <= 1'b0;
            crc_init       <= 1'b0;
        end else begin
            // Default: clear single-cycle pulses
            crc_feed <= 1'b0;
            crc_init <= 1'b0;

            case (state)
                S_IDLE: begin
                    // Latch value on SEAL_DATA write
                    if (data_wr)
                        value_reg <= data_in;

                    if (ctrl_wr) begin
                        // Commit request — always init CRC to eliminate
                        // arbitration race with CPU CRC peripheral
                        if (ctrl_in[1]) begin
                            crc_init      <= 1'b1;
                            sensor_id_reg <= ctrl_in[9:2];
                            cur_mono      <= mono_count;
                            byte_idx      <= 4'd0;
                            byte_sent     <= 1'b0;
                            state         <= S_FEED_BYTES;
                        end
                        // Standalone CRC reset (no commit)
                        else if (ctrl_in[0])
                            crc_init <= 1'b1;
                    end
                end

                S_FEED_BYTES: begin
                    if (!byte_sent) begin
                        // Wait for CRC engine to be ready, then feed
                        if (!crc_busy) begin
                            crc_byte <= feed_byte;
                            crc_feed <= 1'b1;
                            byte_sent <= 1'b1;
                        end
                    end else begin
                        // Byte was sent, wait for CRC to finish processing
                        if (!crc_busy) begin
                            if (byte_idx == 4'd8) begin
                                // All 9 bytes sent, go to LATCH
                                state <= S_LATCH;
                            end else begin
                                byte_idx  <= byte_idx + 1;
                                byte_sent <= 1'b0;
                            end
                        end
                    end
                end

                S_LATCH: begin
                    // Wait for last CRC byte to finish
                    if (!crc_busy) begin
                        sealed_value <= value_reg;
                        sealed_mono  <= cur_mono;
                        sealed_crc   <= crc_value;

                        // Lock session_id on first commit
                        if (!session_locked) begin
                            session_id     <= session_ctr_in;
                            session_locked <= 1'b1;
                            sealed_sid     <= session_ctr_in;
                        end else begin
                            sealed_sid <= session_id;
                        end

                        // Increment mono counter
                        mono_count <= mono_count + 1;

                        state <= S_IDLE;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
