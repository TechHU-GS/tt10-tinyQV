// ============================================================================
// TB: cocotb Peripheral Cross-Validation Wrapper
// ============================================================================
// Based on tb_project.v bus interface approach. Uses force/release controlled
// by tb_bus_override flag to let cocotb drive the TinyQV data bus.
// Also wires up I2C slave model for I2C testing.
// ============================================================================

`default_nettype none
`timescale 1ns / 1ps

module tb_periph;

    reg clk = 0;
    always #20 clk = ~clk;  // 25 MHz

    reg rst_n;
    reg [7:0] ui_in_base;       // cocotb drives this
    wire [7:0] uo_out;
    wire [7:0] uio_out;
    wire [7:0] uio_oe;
    reg [7:0] uio_in;

    // I2C SDA bus feedback: override ui_in[3] with wired-AND bus value
    wire i2c_sda_slave;
    wire i2c_sda_bus = uo_out[6] & i2c_sda_slave;  // forward declaration
    wire [7:0] ui_in = {ui_in_base[7:4], i2c_sda_bus, ui_in_base[2:0]};

    // ================================================================
    // DUT
    // ================================================================
    tt_um_techhu_rv32_trial dut (
        .ui_in(ui_in), .uo_out(uo_out),
        .uio_in(uio_in), .uio_out(uio_out), .uio_oe(uio_oe),
        .ena(1'b1), .clk(clk), .rst_n(rst_n)
    );

    // ================================================================
    // Bus override registers (driven by cocotb)
    // ================================================================
    reg        tb_bus_override = 0;
    reg [27:0] tb_addr = 0;
    reg  [1:0] tb_write_n = 2'b11;
    reg  [1:0] tb_read_n = 2'b11;
    reg        tb_read_complete = 0;
    reg [31:0] tb_wdata = 0;

    // Read data observation (directly from project.v internal wire)
    wire [31:0] tb_rdata = dut.data_from_read;

    // Internal state observation
    wire        tb_rst_reg_n     = dut.rst_reg_n;
    wire [31:0] tb_timer_count   = dut.timer_count;
    wire        tb_timer_irq     = dut.timer_irq;
    wire [3:0]  tb_irq           = dut.interrupt_req;
    wire [15:0] tb_pps_count     = dut.pps_count;
    wire [7:0]  tb_session_ctr   = dut.session_ctr;
    wire        tb_seal_using    = dut.seal_using_crc;
    wire [7:0]  tb_gpio_out      = dut.gpio_out;
    wire [7:0]  tb_gpio_out_sel  = dut.gpio_out_sel;

    // ================================================================
    // Force/release bus override (Verilog procedural force works in Icarus)
    // When tb_bus_override=1, cocotb controls the bus.
    // When tb_bus_override=0, CPU controls the bus (normal operation).
    // ================================================================
    // Force/release on posedge clk to avoid combinational scheduling issues.
    always @(posedge clk) begin
        if (tb_bus_override) begin
            force dut.i_tinyqv.data_addr          = tb_addr;
            force dut.i_tinyqv.data_write_n       = tb_write_n;
            force dut.i_tinyqv.data_read_n        = tb_read_n;
            force dut.i_tinyqv.data_read_complete = tb_read_complete;
            force dut.i_tinyqv.data_out           = tb_wdata;
        end else begin
            release dut.i_tinyqv.data_addr;
            release dut.i_tinyqv.data_write_n;
            release dut.i_tinyqv.data_read_n;
            release dut.i_tinyqv.data_read_complete;
            release dut.i_tinyqv.data_out;
        end
    end

    // ================================================================
    // I2C slave model wiring
    // ================================================================
    // I2C pin mapping:
    //   SCL output: uo_out[2] (when gpio_out_sel[2]=0)
    //   SDA output: uo_out[6] (when gpio_out_sel[6]=0)
    //   SDA input:  ui_in[3]
    //
    // Push-pull emulating open-drain:
    //   *_t = 1 → released (bus high via pullup)
    //   *_t = 0 → driven low
    // Wired-AND: bus = master_out & slave_out

    // i2c_sda_bus and i2c_sda_slave are declared above (before DUT instantiation)
    // because ui_in[3] needs the wired-AND SDA feedback.
    //   i2c_sda_bus = uo_out[6] & i2c_sda_slave  (wired-AND open-drain)
    //   ui_in[3]    = i2c_sda_bus                  (feedback to DUT)
    // SCL comes from uo_out[2] (push-pull, single master, no stretching).

    i2c_slave_model #(.SLAVE_ADDR(7'h44)) i_i2c_slave (
        .scl   (uo_out[2]),
        .sda_i (i2c_sda_bus),
        .sda_o (i2c_sda_slave)
    );

    // 1PPS input: ui_in[4] = ui_in_base[4], driven by cocotb

    // ================================================================
    // VCD dump
    // ================================================================
    initial begin
        $dumpfile("tb_periph.vcd");
        $dumpvars(0, tb_periph);
    end

endmodule
