// ============================================================================
// RTC Counter — 32-bit seconds counter
// ============================================================================
// Slot: PERI_RTC (0xA) at 0x8000028
//
// Behavior:
//   - Counts seconds using tick_1us (1MHz from 25MHz/25)
//   - Internal 20-bit microsecond divider (0~999999)
//   - Read: current seconds value (32-bit unix-ish timestamp)
//   - Write: set seconds value + reset microsecond counter
//   - Write priority: same-cycle tick_1us is overridden by write
//
// Precision: depends on tick_1us accuracy (25MHz xtal / 25 = exact 1MHz)
// ============================================================================

`timescale 1ns / 1ps

module rtc_counter (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        tick_1us,
    // Bus interface
    input  wire        wr_en,
    input  wire [31:0] data_in,
    output reg  [31:0] seconds_out
);

    reg [19:0] us_count;  // 0 to 999999

    always @(posedge clk) begin
        if (!rst_n) begin
            seconds_out <= 32'd0;
            us_count    <= 20'd0;
        end else if (wr_en) begin
            // Write priority: set seconds, reset µs counter
            seconds_out <= data_in;
            us_count    <= 20'd0;
        end else if (tick_1us) begin
            if (us_count == 20'd999_999) begin
                us_count    <= 20'd0;
                seconds_out <= seconds_out + 32'd1;
            end else begin
                us_count <= us_count + 20'd1;
            end
        end
    end

endmodule
