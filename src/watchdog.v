// ============================================================================
// Hardware Watchdog Timer
// ============================================================================
// Slot: PERI_WDT (0xD) at 0x8000034
//
// Behavior:
//   - Power-on: disabled (no countdown, no reset)
//   - Write non-zero: load countdown value (microseconds), enable WDT
//   - Write non-zero (already enabled): reload countdown (kick)
//   - Write zero (enabled): ignored (once enabled, cannot be disabled)
//   - Read: remaining countdown microseconds
//   - When countdown reaches 0: wdt_reset pulse (1 cycle)
//
// tick_1us: 1MHz tick from project.v (25MHz / 25)
// ============================================================================

`timescale 1ns / 1ps

module watchdog (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        tick_1us,
    // Bus interface
    input  wire        kick,          // write enable (slot selected)
    input  wire [31:0] kick_value,    // data_to_write
    output wire [31:0] remaining,     // read: remaining microseconds
    // Reset output
    output reg         wdt_reset      // 1-cycle pulse on expiry
);

    reg [31:0] counter;
    reg        enabled;

    assign remaining = counter;

    always @(posedge clk) begin
        if (!rst_n) begin
            counter   <= 32'd0;
            enabled   <= 1'b0;
            wdt_reset <= 1'b0;
        end else begin
            wdt_reset <= 1'b0;  // default: no reset

            if (kick && kick_value != 32'd0) begin
                // Load/reload countdown + enable
                counter <= kick_value;
                enabled <= 1'b1;
            end
            // kick with value==0 while enabled: ignored (cannot disable)
            else if (enabled && tick_1us && counter != 32'd0) begin
                counter <= counter - 32'd1;
                if (counter == 32'd1) begin
                    wdt_reset <= 1'b1;  // 1-cycle pulse
                end
            end
        end
    end

`ifdef FORMAL
    reg f_past_valid;
    initial f_past_valid = 0;
    always @(posedge clk) f_past_valid <= 1;

    // P5: enabled is irreversible — once 1, always 1
    always @(posedge clk)
        if (f_past_valid && rst_n && $past(rst_n) && $past(enabled))
            assert(enabled);

    // P6: counter never underflows — if 0 and no kick, stays 0
    always @(posedge clk)
        if (f_past_valid && rst_n && $past(rst_n)
            && $past(counter) == 32'd0 && !$past(kick && kick_value != 32'd0))
            assert(counter == 32'd0);

    // P7: wdt_reset only fires when counter transitions 1→0
    always @(posedge clk)
        if (f_past_valid && rst_n && $past(rst_n) && wdt_reset)
            assert($past(counter) == 32'd1);
`endif

endmodule
