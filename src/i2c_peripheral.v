// I2C MMIO Bridge for TinyQV bus → Forencich i2c_master
// Slot 0x6: I2C_DATA, Slot 0x7: I2C_CONFIG
//
// Write I2C_DATA: {cmd_stop, cmd_write_multiple, cmd_write, cmd_read, cmd_start, data[7:0]}
// Read  I2C_DATA: {21'b0, rx_valid, busy, missed_ack, rx_data[7:0]}
// Write I2C_CONFIG: prescale[15:0]
// Read  I2C_CONFIG: {16'b0, prescale[15:0]}
//
// Bridge strategy for Forencich i2c_master AXI Stream protocol:
//   - START+WRITE: send cmd (write_multiple), then stream TX data bytes
//   - Data bytes:  push to TX data channel only (no new cmd)
//   - STOP:        send as cmd (stop only)
//   - START+READ:  send cmd (read + optional stop)
//
// The key insight: after START+WRITE, the master loops in WRITE_1 waiting
// for data (using write_multiple mode). Each byte gets tlast=0 except
// when cmd_stop is requested, which sends tlast=1 to finish the transfer.

module i2c_peripheral (
    input         clk,
    input         rst_n,

    // TinyQV MMIO — Slot 0x6: I2C_DATA
    input  [31:0] data_in,
    input         data_wr,
    input         data_rd,
    output [31:0] data_out,

    // TinyQV MMIO — Slot 0x7: I2C_CONFIG
    input  [31:0] config_in,
    input         config_wr,
    output [31:0] config_out,

    // I2C physical pins
    input         scl_i,
    output        scl_o,
    output        scl_t,
    input         sda_i,
    output        sda_o,
    output        sda_t
);

    wire rst = !rst_n;

    // Prescale register (default 63 → 25MHz/4/63 ≈ 99.2kHz)
    reg [15:0] prescale_reg;
    always @(posedge clk) begin
        if (rst)
            prescale_reg <= 16'd63;
        else if (config_wr)
            prescale_reg <= config_in[15:0];
    end

    // ================================================================
    // Decode MMIO write fields
    // ================================================================
    wire mmio_cmd_start     = data_in[8];
    wire mmio_cmd_read      = data_in[9];
    wire mmio_cmd_write     = data_in[10];
    wire mmio_cmd_write_m   = data_in[11];
    wire mmio_cmd_stop      = data_in[12];
    wire [7:0] mmio_data    = data_in[7:0];
    wire [6:0] mmio_addr    = data_in[6:0];

    wire cmd_any = mmio_cmd_start | mmio_cmd_read | mmio_cmd_write | mmio_cmd_write_m | mmio_cmd_stop;

    // Classify the MMIO write:
    //   has_start = START condition requested
    //   has_write = data byte to write
    //   has_read  = read byte requested
    //   has_stop  = STOP condition requested (standalone or with last byte)
    wire is_start_write = data_wr && mmio_cmd_start && (mmio_cmd_write || mmio_cmd_write_m);
    wire is_read_cmd    = data_wr && mmio_cmd_read;  // any READ (with or without START)
    wire is_data_write  = data_wr && !mmio_cmd_start && (mmio_cmd_write || mmio_cmd_write_m);
    // WARNING: do NOT send stop-only during write_multiple — master is waiting
    // for TX data with tlast=1. Use cmd_write+cmd_stop+data to end transaction.
    wire is_stop_only   = data_wr && mmio_cmd_stop && !mmio_cmd_start &&
                          !mmio_cmd_write && !mmio_cmd_write_m && !mmio_cmd_read;

    // ================================================================
    // Address latch — Forencich checks address on every cmd.
    // Latch on START so subsequent data/stop cmds use correct address.
    // ================================================================
    reg [6:0] addr_latch;
    always @(posedge clk) begin
        if (rst)
            addr_latch <= 7'd0;
        else if (data_wr && mmio_cmd_start)
            addr_latch <= mmio_addr;
    end

    // ================================================================
    // Command channel — AXI Stream latch
    // Sends cmds for: START+WRITE, any READ, STOP-only.
    // Data-only writes go directly to TX data channel.
    // ================================================================
    wire s_axis_cmd_ready;
    wire cmd_accepted = cmd_pending && s_axis_cmd_ready;

    reg        cmd_pending;
    reg [6:0]  cmd_addr_reg;
    reg        cmd_start_reg;
    reg        cmd_read_reg;
    reg        cmd_write_reg;
    reg        cmd_write_m_reg;
    reg        cmd_stop_reg;

    // Need a cmd for: START+WRITE, any READ (with/without START), STOP-only
    wire needs_cmd = is_start_write || is_read_cmd || is_stop_only;

    always @(posedge clk) begin
        if (rst) begin
            cmd_pending     <= 1'b0;
            cmd_addr_reg    <= 7'd0;
            cmd_start_reg   <= 1'b0;
            cmd_read_reg    <= 1'b0;
            cmd_write_reg   <= 1'b0;
            cmd_write_m_reg <= 1'b0;
            cmd_stop_reg    <= 1'b0;
        end else if (needs_cmd && !cmd_pending) begin
            cmd_pending <= 1'b1;
            if (is_start_write) begin
                // START+WRITE: use write_multiple so master loops in WRITE_1
                // mode_stop=1 so WRITE_3 issues STOP when tlast=1
                cmd_addr_reg    <= mmio_addr;
                cmd_start_reg   <= 1'b1;
                cmd_read_reg    <= 1'b0;
                cmd_write_reg   <= 1'b0;
                cmd_write_m_reg <= 1'b1;  // write_multiple for streaming
                cmd_stop_reg    <= 1'b1;  // STOP when tlast=1
            end else if (is_read_cmd) begin
                // Any READ: START+READ for first byte, standalone READ for
                // subsequent bytes, READ+STOP for last byte — all from firmware bits.
                cmd_addr_reg    <= mmio_cmd_start ? mmio_addr : addr_latch;
                cmd_start_reg   <= mmio_cmd_start;
                cmd_read_reg    <= 1'b1;
                cmd_write_reg   <= 1'b0;
                cmd_write_m_reg <= 1'b0;
                cmd_stop_reg    <= mmio_cmd_stop;
            end else begin // is_stop_only
                cmd_addr_reg    <= addr_latch;
                cmd_start_reg   <= 1'b0;
                cmd_read_reg    <= 1'b0;
                cmd_write_reg   <= 1'b0;
                cmd_write_m_reg <= 1'b0;
                cmd_stop_reg    <= 1'b1;
            end
        end else if (cmd_accepted) begin
            cmd_pending <= 1'b0;
        end
    end

    // ================================================================
    // TX Data channel — AXI Stream latch
    // START+WRITE only sends a cmd (address phase); no TX data.
    // Subsequent data-only writes push bytes here (master is in WRITE_1 loop).
    // tlast=1 when this is the last byte before STOP.
    // ================================================================
    wire s_axis_data_tready;

    reg       tx_pending;
    reg [7:0] tx_data_reg;
    reg       tx_last_reg;

    // TX data only for data-only writes (not START+WRITE which is address phase)
    wire tx_new = is_data_write && !tx_pending;

    // tlast=1 when stop is requested with this write
    wire tx_last_in = mmio_cmd_stop;

    // Gate TX valid: don't present data until cmd is accepted
    // (only matters for START+WRITE where cmd and data arrive together)
    wire tx_valid_gated = tx_pending && !cmd_pending;
    wire tx_accepted = tx_valid_gated && s_axis_data_tready;

    always @(posedge clk) begin
        if (rst) begin
            tx_pending  <= 1'b0;
            tx_data_reg <= 8'd0;
            tx_last_reg <= 1'b0;
        end else if (tx_new) begin
            tx_pending  <= 1'b1;
            tx_data_reg <= mmio_data;
            tx_last_reg <= tx_last_in;
        end else if (tx_accepted) begin
            tx_pending <= 1'b0;
        end
    end

    // ================================================================
    // RX Data — 1-deep skid buffer, combinational ready
    // AXI-stream handshake: transfer when tvalid && tready on same cycle.
    // NOTE: reading I2C_DATA consumes the RX byte (rx_has_data cleared).
    //       Firmware must save the value before the next read.
    // ================================================================
    wire [7:0] rx_tdata;
    wire       rx_tvalid;
    wire       rx_tready = !rx_has_data;  // ready when buffer empty
    wire       rx_fire   = rx_tvalid && rx_tready;
    reg [7:0]  rx_latch;
    reg        rx_has_data;

    always @(posedge clk) begin
        if (rst) begin
            rx_latch    <= 8'd0;
            rx_has_data <= 1'b0;
        end else begin
            if (rx_fire) begin
                rx_latch    <= rx_tdata;
                rx_has_data <= 1'b1;
            end
            // Clear on MMIO read of I2C_DATA
            if (data_rd && rx_has_data)
                rx_has_data <= 1'b0;
        end
    end

    // ================================================================
    // Status — missed_ack latch
    // Forencich pulses missed_ack for 1 clock only; we latch it.
    // Cleared when a new MMIO command is written.
    // ================================================================
    wire i2c_busy;
    wire i2c_missed_ack;

    reg missed_ack_latch;
    always @(posedge clk) begin
        if (rst)
            missed_ack_latch <= 1'b0;
        else if (i2c_missed_ack)
            missed_ack_latch <= 1'b1;
        else if (data_wr && cmd_any)
            missed_ack_latch <= 1'b0;  // clear on new MMIO command
    end

    // ================================================================
    // Forencich i2c_master instance
    // ================================================================
    i2c_master i_i2c (
        .clk(clk),
        .rst(rst),
        // Command — held until ready
        .s_axis_cmd_address(cmd_addr_reg),
        .s_axis_cmd_start(cmd_start_reg),
        .s_axis_cmd_read(cmd_read_reg),
        .s_axis_cmd_write(cmd_write_reg),
        .s_axis_cmd_write_multiple(cmd_write_m_reg),
        .s_axis_cmd_stop(cmd_stop_reg),
        .s_axis_cmd_valid(cmd_pending),
        .s_axis_cmd_ready(s_axis_cmd_ready),
        // TX data — gated until cmd accepted
        .s_axis_data_tdata(tx_data_reg),
        .s_axis_data_tvalid(tx_valid_gated),
        .s_axis_data_tready(s_axis_data_tready),
        .s_axis_data_tlast(tx_last_reg),
        // RX data
        .m_axis_data_tdata(rx_tdata),
        .m_axis_data_tvalid(rx_tvalid),
        .m_axis_data_tready(rx_tready),
        .m_axis_data_tlast(),
        // I2C pins
        .scl_i(scl_i),
        .scl_o(scl_o),
        .scl_t(scl_t),
        .sda_i(sda_i),
        .sda_o(sda_o),
        .sda_t(sda_t),
        // Status
        .busy(i2c_busy),
        .bus_control(),
        .bus_active(),
        .missed_ack(i2c_missed_ack),
        // Config
        .prescale(prescale_reg),
        .stop_on_idle(1'b0)
    );

    // Read outputs
    // bit[11]=tx_pending: firmware must poll tx_pending=0 before writing next byte
    //   in write_multiple mode (i2c_busy stays high during entire transaction)
    assign data_out   = {20'b0, tx_pending, rx_has_data, i2c_busy, missed_ack_latch, rx_latch};
    assign config_out = {16'b0, prescale_reg};

endmodule
