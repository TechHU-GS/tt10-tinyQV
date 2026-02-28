// ============================================================================
// Formal Verification: seal_register mono_count monotonicity
//
// Proves via BMC (Bounded Model Checking):
//   P1: mono_count never decreases (only +0 or +1 per cycle)
//   P2: mono_count only increments in S_LATCH state
//   P3: sealed_mono == pre-increment mono_count on commit completion
//   P4: session_id immutable after first lock
//   P5: session_locked never reverts 1→0
//   P6: reset guarantees clean state
//   P7: commit_dropped only set when busy
//
// Strategy: Wrap DUT with extra output ports exposing internal state,
// then assert properties on those ports. No hierarchical access needed.
//
// Run: sby -f seal_mono.sby
// ============================================================================

// ============================================================================
// DUT wrapper: exposes internal seal_register state for formal assertions
// ============================================================================
module seal_dut_wrapper (
    input         clk,
    input         rst_n,

    // CRC engine interface
    input         crc_busy,
    input  [15:0] crc_value,

    // Bus interface
    input         data_wr,
    input  [31:0] data_in,
    input         data_rd,
    input         ctrl_wr,
    input  [9:0]  ctrl_in,
    input  [7:0]  session_ctr_in,

    // Standard outputs
    output [31:0] data_out,
    output [31:0] ctrl_out,

    // CRC engine outputs
    output [7:0]  crc_byte,
    output        crc_feed,
    output        crc_init,

    // === Exposed internal state for assertions ===
    output [31:0] obs_mono_count,
    output [1:0]  obs_state,
    output        obs_session_locked,
    output [7:0]  obs_session_id,
    output [31:0] obs_sealed_mono,
    output        obs_commit_dropped
);

    seal_register uut (
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

    // Expose internal registers via hierarchical access
    // (Yosys supports this in the wrapper module)
    assign obs_mono_count     = uut.mono_count;
    assign obs_state          = uut.state;
    assign obs_session_locked = uut.session_locked;
    assign obs_session_id     = uut.session_id;
    assign obs_sealed_mono    = uut.sealed_mono;
    assign obs_commit_dropped = uut.commit_dropped;

endmodule

// ============================================================================
// Formal top: instantiates wrapper + CRC engine, adds assertions
// ============================================================================
module seal_formal (
    input         clk,
    input         rst_n,
    input         data_wr,
    input  [31:0] data_in,
    input         data_rd,
    input         ctrl_wr,
    input  [9:0]  ctrl_in,
    input  [7:0]  session_ctr_in
);

    // CRC engine (shared instance, same as production)
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

    // DUT wrapper
    wire [31:0] data_out, ctrl_out;
    wire [31:0] obs_mono_count;
    wire [1:0]  obs_state;
    wire        obs_session_locked;
    wire [7:0]  obs_session_id;
    wire [31:0] obs_sealed_mono;
    wire        obs_commit_dropped;

    seal_dut_wrapper dut (
        .clk              (clk),
        .rst_n            (rst_n),
        .crc_busy         (crc_busy),
        .crc_value        (crc_value),
        .data_wr          (data_wr),
        .data_in          (data_in),
        .data_rd          (data_rd),
        .ctrl_wr          (ctrl_wr),
        .ctrl_in          (ctrl_in),
        .session_ctr_in   (session_ctr_in),
        .data_out         (data_out),
        .ctrl_out         (ctrl_out),
        .crc_byte         (crc_byte),
        .crc_feed         (crc_feed),
        .crc_init         (crc_init),
        .obs_mono_count     (obs_mono_count),
        .obs_state          (obs_state),
        .obs_session_locked (obs_session_locked),
        .obs_session_id     (obs_session_id),
        .obs_sealed_mono    (obs_sealed_mono),
        .obs_commit_dropped (obs_commit_dropped)
    );

    // ================================================================
    // Shadow state
    // ================================================================
    reg [31:0] prev_mono;
    reg [7:0]  prev_session_id;
    reg        prev_session_locked;
    reg        prev_commit_dropped;
    reg [1:0]  prev_state;
    reg        prev_rst_n;
    reg        past_valid;

    initial past_valid = 0;

    always @(posedge clk) begin
        prev_mono            <= obs_mono_count;
        prev_session_id      <= obs_session_id;
        prev_session_locked  <= obs_session_locked;
        prev_commit_dropped  <= obs_commit_dropped;
        prev_state           <= obs_state;
        prev_rst_n           <= rst_n;
        past_valid           <= 1;
    end

    // ================================================================
    // P1: mono_count NEVER decreases
    //     Only two legal transitions: stay same, or +1
    // ================================================================
    always @(posedge clk) begin
        if (past_valid && rst_n) begin
            assert (obs_mono_count == prev_mono ||
                    obs_mono_count == prev_mono + 32'd1);
        end
    end

    // ================================================================
    // P2: mono_count only changes in S_LATCH (state==2)
    //     If we were in IDLE(0) or FEED_BYTES(1), mono must not change
    // ================================================================
    always @(posedge clk) begin
        if (past_valid && rst_n) begin
            if (prev_state == 2'd0 || prev_state == 2'd1) begin
                assert (obs_mono_count == prev_mono);
            end
        end
    end

    // ================================================================
    // P3: sealed_mono == pre-increment value on commit completion
    //     When S_LATCH(2) → S_IDLE(0), sealed_mono == mono - 1
    // ================================================================
    always @(posedge clk) begin
        if (past_valid && rst_n) begin
            if (prev_state == 2'd2 && obs_state == 2'd0) begin
                assert (obs_sealed_mono == obs_mono_count - 32'd1);
            end
        end
    end

    // ================================================================
    // P4: session_id immutable once locked
    //     If session_locked was 1 last cycle, session_id must not change
    // ================================================================
    always @(posedge clk) begin
        if (past_valid && rst_n && prev_session_locked) begin
            assert (obs_session_id == prev_session_id);
        end
    end

    // ================================================================
    // P5: session_locked never reverts 1→0 (without reset)
    // ================================================================
    always @(posedge clk) begin
        if (past_valid && rst_n && prev_session_locked) begin
            assert (obs_session_locked == 1'b1);
        end
    end

    // ================================================================
    // P6: Reset guarantees clean state
    //     On first cycle after reset deasserts, mono=0, state=IDLE,
    //     session not locked
    // ================================================================
    always @(posedge clk) begin
        if (past_valid && rst_n && !prev_rst_n) begin
            assert (obs_mono_count == 32'd0);
            assert (obs_state == 2'd0);
            assert (obs_session_locked == 1'b0);
        end
    end

    // ================================================================
    // P7: commit_dropped only transitions 0→1 when seal is busy
    //     (state != IDLE means busy)
    // ================================================================
    always @(posedge clk) begin
        if (past_valid && rst_n) begin
            if (!prev_commit_dropped && obs_commit_dropped) begin
                assert (prev_state != 2'd0);
            end
        end
    end

endmodule
