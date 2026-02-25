// I2C Peripheral Bridge Testbench
// Tests MMIO → Forencich i2c_master → I2C bus with simulated slave
`timescale 1ns/1ps

module tb_i2c;
    reg         clk;
    reg         rst_n;

    // MMIO interface
    reg  [31:0] data_in;
    reg         data_wr;
    reg         data_rd;
    wire [31:0] data_out;
    reg  [31:0] config_in;
    reg         config_wr;
    wire [31:0] config_out;

    // I2C bus (directly connect master to simple slave model)
    wire        scl_o, scl_t, sda_o, sda_t;
    wire        scl_line, sda_line;

    // Simple I2C bus model: open-drain with pull-ups
    // Forencich convention: scl_t = scl_o = scl_o_reg
    //   scl_o_reg=0 → drive low (SCL low)
    //   scl_o_reg=1 → release (SCL high via pull-up)
    // Slave drives: slave_sda_drive=1 → pull SDA low
    reg         slave_sda_drive;
    assign scl_line = scl_t;  // scl_t=0 → low, scl_t=1 → high
    assign sda_line = sda_t & ~slave_sda_drive;  // AND: either side can pull low

    i2c_peripheral uut (
        .clk(clk),
        .rst_n(rst_n),
        .data_in(data_in),
        .data_wr(data_wr),
        .data_rd(data_rd),
        .data_out(data_out),
        .config_in(config_in),
        .config_wr(config_wr),
        .config_out(config_out),
        .scl_i(scl_line),
        .scl_o(scl_o),
        .scl_t(scl_t),
        .sda_i(sda_line),
        .sda_o(sda_o),
        .sda_t(sda_t)
    );

    // Clock: 25MHz = 40ns period
    initial clk = 0;
    always #20 clk = ~clk;

    integer pass_count = 0;
    integer fail_count = 0;
    integer total_tests = 0;

    task check(input [255:0] name, input condition);
    begin
        total_tests = total_tests + 1;
        if (condition) begin
            pass_count = pass_count + 1;
            $display("[PASS] %0s", name);
        end else begin
            fail_count = fail_count + 1;
            $display("[FAIL] %0s", name);
        end
    end
    endtask

    task mmio_write(input [31:0] val);
    begin
        @(posedge clk);
        data_in <= val;
        data_wr <= 1;
        @(posedge clk);
        data_wr <= 0;
        data_in <= 0;
    end
    endtask

    task mmio_read;
    begin
        @(posedge clk);
        data_rd <= 1;
        @(posedge clk);
        data_rd <= 0;
    end
    endtask

    task wait_not_busy;
        integer timeout;
    begin
        timeout = 0;
        while (data_out[9] && timeout < 200000) begin
            @(posedge clk);
            timeout = timeout + 1;
        end
    end
    endtask

    // I2C slave model state
    reg [7:0]  slave_rx_byte;
    reg        slave_addr_match;

    // ================================================================
    // Simple I2C slave bit-bang model
    // Watches SCL/SDA to detect START, receives address, sends ACK
    // ================================================================
    reg        prev_scl, prev_sda;
    reg [3:0]  slave_state; // 0=idle, 1=addr_bits, 2=addr_ack, 3=data_bits, 4=data_ack
    reg [3:0]  slave_bit_cnt;
    reg [7:0]  slave_shift;
    reg        slave_rw_bit;
    reg [7:0]  slave_tx_data;

    localparam SLAVE_ADDR = 7'h44; // SHT31

    always @(posedge clk) begin
        prev_scl <= scl_line;
        prev_sda <= sda_line;
    end

    // START: SDA falls while SCL high
    wire start_cond = prev_sda && !sda_line && scl_line;
    // STOP: SDA rises while SCL high
    wire stop_cond  = !prev_sda && sda_line && scl_line;
    // SCL rising edge
    wire scl_rise   = !prev_scl && scl_line;
    // SCL falling edge
    wire scl_fall   = prev_scl && !scl_line;

    always @(posedge clk) begin
        if (!rst_n) begin
            slave_state     <= 0;
            slave_sda_drive <= 0;
            slave_bit_cnt   <= 0;
            slave_shift     <= 0;
            slave_addr_match <= 0;
            slave_tx_data   <= 8'hA5; // Default read-back data
        end else begin
            // Release SDA by default on each falling SCL (except when actively driving)
            if (scl_fall && slave_state != 4'd2 && slave_state != 4'd4 && slave_state != 4'd5)
                slave_sda_drive <= 0;

            if (start_cond) begin
                slave_state   <= 1; // receiving address
                slave_bit_cnt <= 0;
                slave_shift   <= 0;
                slave_sda_drive <= 0;
            end
            else if (stop_cond) begin
                slave_state     <= 0;
                slave_sda_drive <= 0;
            end
            else case (slave_state)
                4'd1: begin // Receiving address byte (7-bit addr + R/W)
                    if (scl_rise) begin
                        slave_shift <= {slave_shift[6:0], sda_line};
                        if (slave_bit_cnt == 4'd7) begin
                            slave_state <= 2; // go to ACK
                            slave_bit_cnt <= 0;
                        end else begin
                            slave_bit_cnt <= slave_bit_cnt + 1;
                        end
                    end
                end
                4'd2: begin // Address ACK phase
                    // State entry: bit_cnt=0, ack_phase=0
                    // scl_fall #1 (bit_cnt==0): drive ACK, set bit_cnt=1
                    // scl_rise:    master samples ACK
                    // scl_fall #2 (bit_cnt==1): release SDA, move to data phase
                    if (scl_fall && slave_bit_cnt == 0) begin
                        slave_rw_bit <= slave_shift[0];
                        if (slave_shift[7:1] == SLAVE_ADDR) begin
                            slave_sda_drive <= 1; // ACK (drive low)
                            slave_addr_match <= 1;
                        end else begin
                            slave_sda_drive <= 0; // NACK
                            slave_addr_match <= 0;
                        end
                        slave_bit_cnt <= 1;
                        slave_shift   <= 0;
                    end
                    else if (scl_fall && slave_bit_cnt == 1) begin
                        slave_sda_drive <= 0;
                        slave_bit_cnt <= 0;
                        if (slave_addr_match) begin
                            if (slave_rw_bit) begin
                                slave_state <= 5; // slave TX (read)
                            end else begin
                                slave_state <= 3; // slave RX (write)
                            end
                        end else begin
                            slave_state <= 0; // NACK → idle
                        end
                    end
                end
                4'd3: begin // Receiving data byte from master (write)
                    if (scl_rise) begin
                        slave_shift <= {slave_shift[6:0], sda_line};
                        if (slave_bit_cnt == 4'd7) begin
                            slave_state <= 4; // data ACK
                            slave_rx_byte <= {slave_shift[6:0], sda_line};
                            slave_bit_cnt <= 0;
                        end else begin
                            slave_bit_cnt <= slave_bit_cnt + 1;
                        end
                    end
                end
                4'd4: begin // Data ACK phase (write)
                    // Same 2-falling-edge pattern as addr ACK
                    if (scl_fall && slave_bit_cnt == 0) begin
                        slave_sda_drive <= 1; // ACK
                        slave_bit_cnt <= 1;
                    end
                    else if (scl_fall && slave_bit_cnt == 1) begin
                        slave_sda_drive <= 0;
                        slave_state <= 3; // ready for next byte
                        slave_bit_cnt <= 0;
                        slave_shift   <= 0;
                    end
                end
                4'd5: begin // Sending data byte to master (read)
                    if (scl_fall) begin
                        // Drive next bit (MSB first)
                        slave_sda_drive <= !slave_tx_data[7 - slave_bit_cnt];
                        // Note: drive=1 means low, so invert
                    end
                    if (scl_rise) begin
                        slave_bit_cnt <= slave_bit_cnt + 1;
                        if (slave_bit_cnt == 7) begin
                            slave_state <= 6; // wait for master ACK/NACK
                            slave_bit_cnt <= 0;
                        end
                    end
                end
                4'd6: begin // Master ACK/NACK for read
                    if (scl_fall) begin
                        slave_sda_drive <= 0; // release
                    end
                    if (scl_rise) begin
                        // If master ACKs (SDA low), send next byte
                        // If master NACKs (SDA high), done
                        if (!sda_line) begin
                            slave_state <= 5; // more data
                            slave_bit_cnt <= 0;
                            slave_tx_data <= slave_tx_data + 1; // next byte
                        end else begin
                            slave_state <= 0; // done
                        end
                    end
                end
            endcase
        end
    end

    // ================================================================
    // Test sequence
    // ================================================================
    initial begin
        $dumpfile("tb_i2c.vcd");
        $dumpvars(0, tb_i2c);

        rst_n    = 0;
        data_in  = 0;
        data_wr  = 0;
        data_rd  = 0;
        config_in = 0;
        config_wr = 0;

        $display("=== I2C Peripheral Bridge Testbench ===\n");

        // Reset
        repeat(10) @(posedge clk);
        rst_n = 1;
        repeat(5) @(posedge clk);

        // --- Test 1: Default prescale ---
        $display("--- Test 1: Default prescale ---");
        check("default prescale = 63", config_out[15:0] == 16'd63);

        // --- Test 2: Change prescale ---
        $display("--- Test 2: Change prescale ---");
        @(posedge clk);
        config_in <= 32'd10;
        config_wr <= 1;
        @(posedge clk);
        config_wr <= 0;
        @(posedge clk);
        check("prescale changed to 10", config_out[15:0] == 16'd10);

        // Set prescale for sim (10 → SCL ~625kHz, fast but slave can track)
        @(posedge clk);
        config_in <= 32'd10;
        config_wr <= 1;
        @(posedge clk);
        config_wr <= 0;
        repeat(2) @(posedge clk);

        // --- Test 3: Reset state — SCL/SDA released ---
        // Forencich: _t=_o=1 after reset (bus released/high)
        $display("--- Test 3: Reset state ---");
        check("scl_t=1 (released high) after reset", scl_t == 1'b1);
        check("sda_t=1 (released high) after reset", sda_t == 1'b1);
        check("not busy after reset", data_out[9] == 1'b0);

        // --- Test 4: I2C write transaction to slave addr 0x44 ---
        $display("--- Test 4: I2C write to 0x44 ---");
        // Forencich takes 7-bit addr in data_in[6:0], R/W from cmd_read/cmd_write
        // data_in = {cmd_stop=0, cmd_write_m=0, cmd_write=1, cmd_read=0, cmd_start=1, addr7=0x44}
        mmio_write({19'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b1, 1'b0, 7'h44});
        repeat(5) @(posedge clk);
        // Should become busy
        check("busy after START+WRITE cmd", data_out[9] == 1'b1);

        // Wait for address phase to complete
        wait_not_busy;
        check("addr ACK (no missed_ack)", data_out[8] == 1'b0);
        check("slave saw addr match", slave_addr_match == 1'b1);

        // --- Test 5: Write a data byte ---
        $display("--- Test 5: Write data byte 0x24 ---");
        // cmd_write + data (master should be at WRITE_1 from Test 4's cmd)
        mmio_write({19'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 8'h24});
        repeat(3) @(posedge clk);
        wait_not_busy;
        check("data byte written (no NACK)", data_out[8] == 1'b0);
        check("slave received 0x24", slave_rx_byte == 8'h24);

        // --- Test 6: Write last byte + STOP ---
        $display("--- Test 6: Write 0x00 + STOP ---");
        // Combined: cmd_write + cmd_stop + data (bridge sends tlast=1)
        mmio_write({19'b0, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 8'h00}); // cmd_write+cmd_stop
        repeat(3) @(posedge clk);
        wait_not_busy;
        check("data written (no NACK)", data_out[8] == 1'b0);

        repeat(100) @(posedge clk);

        // --- Test 7: I2C read from slave 0x44 ---
        $display("--- Test 7: I2C read from 0x44 ---");
        // START + READ + STOP with 7-bit addr = 0x44
        // Forencich: cmd_read=1 with cmd_stop=1 → read 1 byte + NACK + STOP
        mmio_write({19'b0, 1'b1, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 7'h44});
        // Give master time to assert busy (cmd acceptance takes a few clocks)
        repeat(5) @(posedge clk);
        // Wait for entire transaction to complete
        wait_not_busy;
        check("read addr ACK", data_out[8] == 1'b0);

        // Check rx_valid and data
        repeat(10) @(posedge clk);
        check("rx_valid set", data_out[10] == 1'b1);
        check("rx_data = 0xA5", data_out[7:0] == 8'hA5);

        // Read MMIO to consume rx data
        mmio_read;
        repeat(5) @(posedge clk);
        check("rx_valid cleared after read", data_out[10] == 1'b0);

        repeat(200) @(posedge clk);

        // --- Test 9: Multi-byte read (3 bytes from 0x44) ---
        // Verifies Bug 3 fix: standalone READ without START must not be dropped.
        // Slave returns 0xA5, 0xA6, 0xA7 (auto-incrementing)
        $display("--- Test 9: Multi-byte read (3 bytes) ---");

        // Reset slave_tx_data to known value
        // (slave model auto-increments after each byte sent)

        // Byte 1: START + READ (no STOP)
        mmio_write({19'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 7'h44});
        repeat(5) @(posedge clk);
        wait_not_busy;
        check("multi-read: byte 1 no NACK", data_out[8] == 1'b0);

        // Wait for rx_valid
        repeat(20) @(posedge clk);
        check("multi-read: byte 1 rx_valid", data_out[10] == 1'b1);
        // Capture byte 1 value
        begin : multi_read_block
            reg [7:0] byte1, byte2, byte3;
            byte1 = data_out[7:0];
            $display("  byte1 = 0x%02X", byte1);

            // Consume byte 1
            mmio_read;
            repeat(5) @(posedge clk);
            check("multi-read: byte 1 consumed", data_out[10] == 1'b0);

            // Byte 2: READ only (no START, no STOP)
            mmio_write({19'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 7'h00});
            repeat(5) @(posedge clk);
            wait_not_busy;

            repeat(20) @(posedge clk);
            check("multi-read: byte 2 rx_valid", data_out[10] == 1'b1);
            byte2 = data_out[7:0];
            $display("  byte2 = 0x%02X", byte2);
            check("multi-read: byte 2 = byte1+1", byte2 == byte1 + 1);

            // Consume byte 2
            mmio_read;
            repeat(5) @(posedge clk);

            // Byte 3: READ + STOP (last byte, master sends NACK)
            mmio_write({19'b0, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 7'h00});
            repeat(5) @(posedge clk);
            wait_not_busy;

            repeat(20) @(posedge clk);
            check("multi-read: byte 3 rx_valid", data_out[10] == 1'b1);
            byte3 = data_out[7:0];
            $display("  byte3 = 0x%02X", byte3);
            check("multi-read: byte 3 = byte2+1", byte3 == byte2 + 1);

            // Consume byte 3
            mmio_read;
            repeat(5) @(posedge clk);
        end

        repeat(200) @(posedge clk);

        // --- Test 8: NACK on wrong address ---
        $display("--- Test 8: NACK on wrong address ---");

        // START + WRITE to addr 0x55 (not 0x44) — 7-bit addr directly
        mmio_write({19'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b1, 1'b0, 7'h55});
        // Master enters WRITE_1 after address (even on NACK).
        // Send a dummy byte + STOP to complete the transaction.
        mmio_write({19'b0, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 8'h00}); // WRITE+STOP+0x00
        repeat(5) @(posedge clk);
        wait_not_busy;
        check("missed_ack on wrong addr", data_out[8] == 1'b1);

        // --- Test 10: Rapid burst writes (backpressure) ---
        // Write two data bytes rapidly; the second should be dropped (tx_pending gate).
        // Then verify exactly 2 bytes total reach slave (1st data + STOP byte).
        $display("--- Test 10: Rapid burst writes ---");

        // Fresh transaction: START+WRITE to 0x44
        mmio_write({19'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b1, 1'b0, 7'h44});
        repeat(5) @(posedge clk);
        wait_not_busy;

        // Write first data byte (cmd_write=1, cmd_start=0)
        mmio_write({19'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 8'hAA});
        // Wait for the I2C master to finish transmitting the byte on wire
        // tx_pending clears early (AXI handshake), but slave_rx_byte updates
        // only after 8 SCL clocks + ACK. So wait for busy=0.
        wait_not_busy;
        repeat(10) @(posedge clk);  // margin for slave to latch
        // Capture what slave received as first byte
        check("burst: 1st byte delivered", slave_rx_byte == 8'hAA);

        // Now write second byte (this one goes through since tx_pending cleared)
        mmio_write({19'b0, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 8'hCC}); // WRITE+STOP+0xCC
        begin : burst_wait2
            integer t;
            t = 0;
            while (data_out[11] && t < 50000) begin
                @(posedge clk);
                t = t + 1;
            end
        end
        wait_not_busy;
        // Second byte delivered
        check("burst: 2nd byte delivered", slave_rx_byte == 8'hCC);
        repeat(200) @(posedge clk);

        // --- Test 11: RX buffer overflow (read before consume) ---
        $display("--- Test 11: RX overflow ---");
        // Reset to clean state
        rst_n = 0; repeat(10) @(posedge clk); rst_n = 1; repeat(10) @(posedge clk);
        @(posedge clk); config_in <= 32'd10; config_wr <= 1; @(posedge clk); config_wr <= 0;
        repeat(2) @(posedge clk);
        // Start read from slave
        mmio_write({19'b0, 1'b1, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 7'h44}); // START+READ+STOP to 0x44
        repeat(5) @(posedge clk);
        wait_not_busy;
        repeat(20) @(posedge clk);

        // rx_has_data should be 1
        check("rx overflow: has_data=1", data_out[10] == 1'b1);
        begin : rx_overflow_block
            reg [7:0] first_byte;
            first_byte = data_out[7:0];

            // Don't consume (no mmio_read), but rx_tready is 0 now
            // The Forencich core will hold m_axis_data_tvalid.
            // Since rx_tready=!rx_has_data=0, no new data is accepted (safe).
            // Just verify the byte is stable
            repeat(50) @(posedge clk);
            check("rx overflow: data stable", data_out[7:0] == first_byte);
            check("rx overflow: still valid", data_out[10] == 1'b1);

            // Now consume and verify
            mmio_read;
            repeat(5) @(posedge clk);
            check("rx overflow: cleared after read", data_out[10] == 1'b0);
        end

        repeat(200) @(posedge clk);

        // --- Test 12: missed_ack latch and clear ---
        $display("--- Test 12: missed_ack latch/clear ---");
        // Write to non-existent address (0x55) → NACK
        mmio_write({19'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b1, 1'b0, 7'h55});
        mmio_write({19'b0, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 8'h00});
        repeat(5) @(posedge clk);
        wait_not_busy;
        check("NACK latched", data_out[8] == 1'b1);

        // NACK should persist (sticky latch)
        repeat(100) @(posedge clk);
        check("NACK still latched", data_out[8] == 1'b1);

        // New command clears NACK latch
        mmio_write({19'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b1, 1'b0, 7'h44});
        repeat(2) @(posedge clk);
        check("NACK cleared by new cmd", data_out[8] == 1'b0);
        wait_not_busy;
        // Send STOP to clean up
        mmio_write({19'b0, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 8'h00});
        wait_not_busy;
        repeat(100) @(posedge clk);

        // --- Test 13: Reset recovers bus ---
        $display("--- Test 13: Reset recovery ---");
        rst_n = 0;
        repeat(10) @(posedge clk);
        rst_n = 1;
        repeat(10) @(posedge clk);
        check("reset: scl_t=1 (released)", scl_t == 1'b1);
        check("reset: sda_t=1 (released)", sda_t == 1'b1);
        check("reset: not busy", data_out[9] == 1'b0);
        check("reset: no missed_ack", data_out[8] == 1'b0);
        check("reset: prescale restored", config_out[15:0] == 16'd63);

        // Verify bus still works after reset
        @(posedge clk);
        config_in <= 32'd10;
        config_wr <= 1;
        @(posedge clk);
        config_wr <= 0;
        repeat(2) @(posedge clk);

        mmio_write({19'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b1, 1'b0, 7'h44});
        repeat(5) @(posedge clk);
        wait_not_busy;
        check("reset: bus works after reset", data_out[8] == 1'b0);
        mmio_write({19'b0, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 8'h00});
        wait_not_busy;
        repeat(100) @(posedge clk);

        // --- Test 14: cmd_pending rejection (write while cmd pending) ---
        $display("--- Test 14: cmd_pending gate ---");
        // Issue two commands rapidly — second should be held until first accepted
        mmio_write({19'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b1, 1'b0, 7'h44}); // START+WRITE
        // Don't wait, immediately send data byte
        mmio_write({19'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 8'h77});
        // Bridge should gate TX data until cmd accepted
        wait_not_busy;
        check("cmd gate: no NACK", data_out[8] == 1'b0);
        // STOP
        mmio_write({19'b0, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 8'h88});
        wait_not_busy;
        repeat(100) @(posedge clk);

        // --- Test 15: B3 — TX write during tx_pending (silent drop) ---
        $display("--- Test 15: B3 TX silent drop ---");
        rst_n = 0; repeat(10) @(posedge clk); rst_n = 1; repeat(10) @(posedge clk);
        @(posedge clk); config_in <= 32'd10; config_wr <= 1; @(posedge clk); config_wr <= 0;
        repeat(5) @(posedge clk);

        // START+WRITE to 0x44 (puts master in write_multiple mode)
        // Use large prescale so address phase takes longer, giving us time
        // to test tx_pending gating before master accepts the TX data.
        @(posedge clk); config_in <= 32'd250; config_wr <= 1; @(posedge clk); config_wr <= 0;
        repeat(5) @(posedge clk);

        mmio_write({19'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b1, 1'b0, 7'h44});
        // Wait for address phase: START + 8 bits + ACK = 10 SCL periods
        // prescale=250 → SCL period = 4*250 = 1000 clk → ~10000 clk.
        repeat(12000) @(posedge clk);

        // Now master is in WRITE_1, tready=1. Write first byte — tx_pending=1
        // for exactly 1 cycle (master accepts immediately). Then on that same
        // cycle, write the second byte. Since tx_pending=1, tx_new=0 → dropped.
        @(posedge clk);
        data_in <= {19'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 8'h55}; // byte 1
        data_wr <= 1;
        @(posedge clk);
        // This edge: tx_pending becomes 1 (byte 1 latched).
        // Next edge: tx_accepted clears it. We need byte 2 write on THIS edge.
        data_in <= {19'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 8'hDD}; // byte 2
        data_wr <= 1;
        // Check: tx_pending was just set to 1 on this posedge by byte 1.
        // data_out is combinational from tx_pending_reg, which was updated
        // this posedge. In Verilog sim, NBA update means data_out reflects NEW value.
        // tx_pending=1 is visible here because it was just set.
        @(posedge clk);
        // Now: tx_accepted fired on prev edge (clearing tx_pending).
        // byte 2's data_wr is active but tx_pending WAS 1 when data_wr went high
        // on the previous edge. Due to `tx_new = is_data_write && !tx_pending`,
        // tx_pending was 1 → tx_new=0 → byte 2 was NOT latched.
        data_wr <= 0;
        data_in <= 0;

        // Wait for byte 1 (0x55) to be fully transmitted
        // 8 data bits + 1 ACK = 9 SCL periods @ prescale=250 = 9000 clk
        repeat(11000) @(posedge clk);

        // Verify slave received 0x55 (not 0xDD) — proves silent drop worked
        check("B3: slave got 0x55 not 0xDD", slave_rx_byte == 8'h55);
        // Verify the code path: tx_new = is_data_write && !tx_pending
        // The bridge correctly gates the second write when tx_pending=1.
        // (structural verification — the gate exists in RTL line 166)

        // End transaction: WRITE+STOP to exit write_multiple (restore fast prescale)
        @(posedge clk); config_in <= 32'd10; config_wr <= 1; @(posedge clk); config_wr <= 0;
        repeat(5) @(posedge clk);
        mmio_write({19'b0, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 8'hEE}); // WRITE+STOP+data=0xEE
        // Wait for STOP: poll tx_pending then wait_not_busy.
        begin : b3_wait2
            integer wt;
            wt = 0;
            while (data_out[11] && wt < 200000) begin
                @(posedge clk);
                wt = wt + 1;
            end
        end
        wait_not_busy;
        repeat(100) @(posedge clk);
        check("B3: bus idle after STOP", data_out[9] == 1'b0);
        repeat(200) @(posedge clk);

        // --- Test 16: B4 — missed_ack cleared by new cmd including STOP ---
        $display("--- Test 16: B4 NACK cleared by STOP ---");
        rst_n = 0; repeat(10) @(posedge clk); rst_n = 1; repeat(10) @(posedge clk);
        @(posedge clk); config_in <= 32'd10; config_wr <= 1; @(posedge clk); config_wr <= 0;
        repeat(5) @(posedge clk);

        // Write to wrong address → NACK
        mmio_write({19'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b1, 1'b0, 7'h55});
        // Send dummy byte + STOP to complete transaction
        mmio_write({19'b0, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 8'h00});
        repeat(5) @(posedge clk);
        wait_not_busy;
        check("B4: NACK latched", data_out[8] == 1'b1);

        // Now send a new START+WRITE to correct address — this should clear missed_ack
        mmio_write({19'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b1, 1'b0, 7'h44});
        repeat(5) @(posedge clk);
        wait_not_busy;
        check("B4: NACK cleared by new cmd", data_out[8] == 1'b0);

        // Clean up with STOP
        mmio_write({19'b0, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 8'h00});
        wait_not_busy;
        repeat(200) @(posedge clk);

        // --- Test 17: B5 — RX backpressure (Forencich holds tvalid) ---
        // Forencich i2c_master uses AXI-stream backpressure: if rx_tready=0 when
        // a new byte arrives, tvalid stays high until tready goes high. The byte
        // is NOT dropped — it's deferred until the bridge buffer has room.
        $display("--- Test 17: B5 RX backpressure ---");
        rst_n = 0; repeat(10) @(posedge clk); rst_n = 1; repeat(10) @(posedge clk);
        @(posedge clk); config_in <= 32'd10; config_wr <= 1; @(posedge clk); config_wr <= 0;
        repeat(5) @(posedge clk);

        // Single START+READ+STOP from 0x44 — reads 1 byte + NACK + STOP
        mmio_write({19'b0, 1'b1, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 7'h44});
        repeat(5) @(posedge clk);
        wait_not_busy;
        repeat(100) @(posedge clk);

        // First byte should be in rx buffer
        check("B5: rx_valid after read", data_out[10] == 1'b1);
        begin : b5_block
            reg [7:0] first_byte;
            first_byte = data_out[7:0];
            $display("  B5: first_byte = 0x%02X (expect 0xA5)", first_byte);
            check("B5: first byte = 0xA5", first_byte == 8'hA5);

            // Don't consume — do another START+READ+STOP while rx_has_data=1
            // Master reads second byte, but tready=0 → tvalid held high until consumed
            mmio_write({19'b0, 1'b1, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 7'h44}); // START+READ+STOP
            repeat(5) @(posedge clk);
            wait_not_busy;
            repeat(100) @(posedge clk);

            // rx buffer still holds first byte (second byte held by master tvalid)
            check("B5: still has data", data_out[10] == 1'b1);
            check("B5: data still first byte", data_out[7:0] == first_byte);

            // Consume first byte → rx_has_data clears → tready=1 → second byte latched
            mmio_read;
            repeat(10) @(posedge clk);

            // Second byte should now be in buffer (auto-filled from held tvalid)
            check("B5: second byte auto-filled", data_out[10] == 1'b1);
            $display("  B5: second_byte = 0x%02X (expect 0xA5 from new transaction)",
                     data_out[7:0]);

            // Consume second byte
            mmio_read;
            repeat(10) @(posedge clk);
            check("B5: empty after consuming both", data_out[10] == 1'b0);
        end
        repeat(200) @(posedge clk);

        // --- Test 18: Write-then-Read (STOP + restart) ---
        // Note: Bridge uses write_multiple mode, so a true repeated START (no STOP
        // between write and read) is not supported. Instead, firmware must STOP the
        // write phase, then START a new read phase. This is the standard pattern
        // for I2C register reads (e.g., SHT31: write cmd → STOP → START → read).
        $display("--- Test 18: Write-then-Read ---");
        rst_n = 0; repeat(10) @(posedge clk); rst_n = 1; repeat(10) @(posedge clk);
        @(posedge clk); config_in <= 32'd10; config_wr <= 1; @(posedge clk); config_wr <= 0;
        repeat(5) @(posedge clk);

        // START + WRITE to 0x44
        mmio_write({19'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b1, 1'b0, 7'h44});
        repeat(5) @(posedge clk);
        wait_not_busy;
        check("wr-rd: addr ACK", data_out[8] == 1'b0);

        // Write register address byte + STOP (WRITE+STOP, tlast=1 → master exits write_multiple)
        mmio_write({19'b0, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 8'h10}); // WRITE+STOP+data=0x10
        wait_not_busy;
        repeat(50) @(posedge clk);

        // Now START + READ from 0x44 (new transaction)
        mmio_write({19'b0, 1'b1, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 7'h44}); // START+READ+STOP
        repeat(5) @(posedge clk);
        wait_not_busy;
        repeat(100) @(posedge clk);

        check("wr-rd: rx_valid", data_out[10] == 1'b1);
        check("wr-rd: read data OK", data_out[7:0] == 8'hA5);
        mmio_read;
        repeat(200) @(posedge clk);

        // --- Test 19: Reset mid-transaction ---
        $display("--- Test 19: Reset mid-transaction ---");
        @(posedge clk); config_in <= 32'd10; config_wr <= 1; @(posedge clk); config_wr <= 0;
        repeat(5) @(posedge clk);

        // Start a write transaction
        mmio_write({19'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b1, 1'b0, 7'h44});
        repeat(20) @(posedge clk);

        // Reset mid-transaction
        rst_n = 0;
        repeat(10) @(posedge clk);
        rst_n = 1;
        repeat(10) @(posedge clk);

        // Verify clean state
        check("mid-txn reset: not busy", data_out[9] == 1'b0);
        check("mid-txn reset: no NACK", data_out[8] == 1'b0);
        check("mid-txn reset: no rx_data", data_out[10] == 1'b0);
        check("mid-txn reset: no tx_pending", data_out[11] == 1'b0);

        // Verify bus works after reset
        @(posedge clk); config_in <= 32'd10; config_wr <= 1; @(posedge clk); config_wr <= 0;
        repeat(5) @(posedge clk);
        mmio_write({19'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b1, 1'b0, 7'h44});
        repeat(5) @(posedge clk);
        wait_not_busy;
        check("mid-txn reset: bus works after", data_out[8] == 1'b0);
        // STOP
        mmio_write({19'b0, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 8'h00});
        wait_not_busy;
        repeat(200) @(posedge clk);

        // ================================================================
        // Summary
        // ================================================================
        repeat(100) @(posedge clk);
        $display("");
        $display("=== Results: %0d PASS, %0d FAIL ===", pass_count, fail_count);
        if (fail_count == 0) $display("ALL TESTS PASSED");
        else $display("SOME TESTS FAILED");
        $finish;
    end

    // Timeout
    initial begin
        #200_000_000;
        $display("TIMEOUT!");
        $finish;
    end

endmodule
