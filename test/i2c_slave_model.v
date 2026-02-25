// ============================================================================
// Simple I2C Slave Model (SHT31 @ addr 0x44)
// ============================================================================
// Behavioral model for integration testing, using push-pull bus model.
// Separate SDA input and output (no inout tristate).
//
// sda_i = bus value (master AND slave, wired-AND)
// sda_o = slave's drive: 1=release, 0=pull low
//
// The testbench computes: bus = master_sda AND slave_sda_o
// and feeds bus back to both sda_i and the DUT's ui_in[3].
// ============================================================================

`timescale 1ns / 1ps

module i2c_slave_model #(
    parameter [6:0] SLAVE_ADDR = 7'h44
) (
    input  wire scl,       // SCL from master
    input  wire sda_i,     // SDA bus value (resolved)
    output reg  sda_o      // SDA slave output: 1=release, 0=pull low
);

    // Preset read data (SHT31 temperature + humidity measurement)
    reg [7:0] read_data [0:5];
    initial begin
        read_data[0] = 8'h63;  // T MSB
        read_data[1] = 8'h32;  // T LSB
        read_data[2] = 8'hA1;  // T CRC
        read_data[3] = 8'h8C;  // H MSB
        read_data[4] = 8'hA4;  // H LSB
        read_data[5] = 8'hDB;  // H CRC
        sda_o = 1'b1;  // released
    end

    // State
    localparam ST_IDLE     = 0;
    localparam ST_ADDR     = 1;  // receiving address byte
    localparam ST_ADDR_ACK = 2;  // sending ACK for address
    localparam ST_WR_DATA  = 3;  // receiving write data
    localparam ST_WR_ACK   = 4;  // sending ACK for write data
    localparam ST_RD_DATA  = 5;  // sending read data
    localparam ST_RD_ACK   = 6;  // receiving ACK/NACK from master

    reg [3:0] state;
    reg [3:0] bit_count;
    reg [7:0] shift_reg;
    reg       is_read;
    reg [2:0] read_idx;

    // Detect SDA falling while SCL high → START
    always @(negedge sda_i) begin
        if (scl) begin
            // START condition
            $display("[I2C_SLAVE] START detected @ %0t", $time);
            state <= ST_ADDR;
            bit_count <= 0;
            shift_reg <= 0;
            sda_o <= 1'b1;  // release during address reception
        end
    end

    // Detect SDA rising while SCL high → STOP
    always @(posedge sda_i) begin
        if (scl) begin
            // STOP condition
            state <= ST_IDLE;
            sda_o <= 1'b1;  // release
        end
    end

    // SCL rising edge — sample SDA bit (master→slave)
    always @(posedge scl) begin
        case (state)
            ST_ADDR: begin
                shift_reg <= {shift_reg[6:0], sda_i};
                bit_count <= bit_count + 1;
            end
            ST_WR_DATA: begin
                shift_reg <= {shift_reg[6:0], sda_i};
                bit_count <= bit_count + 1;
            end
            ST_RD_ACK: begin
                // Master sends ACK(0) or NACK(1) for our read data
                if (sda_i) begin
                    // NACK — master done reading, go idle
                    state <= ST_IDLE;
                end
            end
        endcase
    end

    // SCL falling edge — drive SDA for next bit or handle state transitions
    always @(negedge scl) begin
        case (state)
            ST_ADDR: begin
                if (bit_count == 8) begin
                    // Full address byte received
                    if (shift_reg[7:1] == SLAVE_ADDR) begin
                        $display("[I2C_SLAVE] ADDR 0x%02X match, R/W=%b @ %0t", shift_reg[7:1], shift_reg[0], $time);
                        is_read <= shift_reg[0];
                        state <= ST_ADDR_ACK;
                        sda_o <= 1'b0;  // ACK (pull low)
                        read_idx <= 0;
                    end else begin
                        // Address mismatch — NACK and go idle
                        $display("[I2C_SLAVE] ADDR 0x%02X mismatch @ %0t", shift_reg[7:1], $time);
                        state <= ST_IDLE;
                        sda_o <= 1'b1;  // NACK (release)
                    end
                end
            end

            ST_ADDR_ACK: begin
                // ACK bit was driven, transition to data phase
                bit_count <= 0;
                if (is_read) begin
                    state <= ST_RD_DATA;
                    // Drive MSB of first read byte
                    sda_o <= read_data[0][7] ? 1'b1 : 1'b0;
                    $display("[I2C_SLAVE] → RD_DATA state, driving bit7=%b of byte 0x%02X @ %0t",
                             read_data[0][7], read_data[0], $time);
                end else begin
                    state <= ST_WR_DATA;
                    sda_o <= 1'b1;  // release for master to drive
                end
            end

            ST_WR_DATA: begin
                if (bit_count == 8) begin
                    // Write byte received — ACK
                    state <= ST_WR_ACK;
                    sda_o <= 1'b0;  // ACK
                    bit_count <= 0;
                end
            end

            ST_WR_ACK: begin
                // ACK done, ready for next byte
                state <= ST_WR_DATA;
                sda_o <= 1'b1;  // release for master
                bit_count <= 0;
            end

            ST_RD_DATA: begin
                bit_count <= bit_count + 1;
                if (bit_count < 7) begin
                    // Drive next bit (MSB first)
                    sda_o <= read_data[read_idx][6 - bit_count] ? 1'b1 : 1'b0;
                    $display("[I2C_SLAVE] RD bit%0d = %b @ %0t", bit_count + 1, read_data[read_idx][6 - bit_count], $time);
                end else if (bit_count == 7) begin
                    // All 8 bits sent, release for master ACK/NACK
                    $display("[I2C_SLAVE] RD byte 0x%02X complete, waiting ACK @ %0t", read_data[read_idx], $time);
                    state <= ST_RD_ACK;
                    sda_o <= 1'b1;  // release
                end
            end

            ST_RD_ACK: begin
                // After ACK from master, send next byte
                if (read_idx < 5)
                    read_idx <= read_idx + 1;
                state <= ST_RD_DATA;
                bit_count <= 0;
                // Drive MSB of next byte
                sda_o <= read_data[(read_idx < 5) ? read_idx + 1 : 5][7] ? 1'b1 : 1'b0;
            end
        endcase
    end

    initial begin
        state = ST_IDLE;
        bit_count = 0;
        shift_reg = 0;
        is_read = 0;
        read_idx = 0;
    end

endmodule
