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

endmodule
