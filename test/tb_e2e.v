// ============================================================================
// TB: cocotb E2E Peripheral Cross-Validation Wrapper
// ============================================================================
// Extends tb.v with I2C slave model wiring for end-to-end testing via CPU
// instruction injection. The CPU executes RISC-V instructions fetched over
// QSPI and accesses peripherals through the real data bus path.
// ============================================================================

`default_nettype none
`timescale 1ns / 100ps

module tb_e2e ();

  // Wire up the inputs and outputs:
  reg clk;
  reg rst_n;
  reg ena;
  reg [7:0] ui_in_base;
  wire [7:0] ui_in;
  reg [7:0] uio_in;
  wire [7:0] uo_out;
  wire [7:0] uio_out;
  wire [7:0] uio_oe;

  wire [3:0] qspi_data_in;
  reg [2:0] latency_cfg;
  assign {uio_in[5:4], uio_in[2:1]} = rst_n ? qspi_data_in : {1'b0, latency_cfg};

  wire [3:0] qspi_data_out = {uio_out[5:4], uio_out[2:1]};
  wire [3:0] qspi_data_oe  = {uio_oe[5:4],  uio_oe[2:1]};
  wire qspi_clk_out = uio_out[3];
  wire qspi_flash_select = uio_out[0];
  wire qspi_ram_a_select = uio_out[6];
  wire qspi_ram_b_select = uio_out[7];

  wire spi_miso = ui_in_base[2];
  wire spi_cs = uo_out[4];
  wire spi_sck = uo_out[5];
  wire spi_mosi = uo_out[3];
  wire spi_dc = uo_out[2];

  wire mhz_clk = ui_in_base[3];
  wire game_latch = ui_in_base[4];
  wire game_clk = ui_in_base[5];
  wire game_data = ui_in_base[6];

  wire uart_tx = uo_out[0];
  wire uart_rts = uo_out[1];
  wire debug_uart_tx = uo_out[6];
  wire uart_rx = ui_in_base[7];

  // ================================================================
  // I2C slave model wiring
  // ================================================================
  // I2C pin mapping (from project.v):
  //   SCL output: uo_out[2] (when gpio_out_sel[2]=0)
  //   SDA output: uo_out[6] (when gpio_out_sel[6]=0)
  //   SDA input:  ui_in[3]
  //
  // Wired-AND open-drain emulation:
  //   sda_bus = master_sda_out & slave_sda_out
  //   ui_in[3] = sda_bus (feedback to DUT)

  wire i2c_sda_slave;
  wire i2c_sda_bus = uo_out[6] & i2c_sda_slave;

  // ui_in: bits 7,6,5,4 from ui_in_base; bit 3 = I2C SDA bus; bits 2,1,0 from ui_in_base
  assign ui_in = {uart_rx, game_data, game_clk, game_latch, i2c_sda_bus, spi_miso, ui_in_base[1:0]};

  i2c_slave_model #(.SLAVE_ADDR(7'h44)) i_i2c_slave (
      .scl   (uo_out[2]),
      .sda_i (i2c_sda_bus),
      .sda_o (i2c_sda_slave)
  );

  // ================================================================
  // Internal state observation (for cocotb assertions)
  // ================================================================
  wire [31:0] tb_rdata        = user_project.data_from_read;
  wire        tb_rst_reg_n    = user_project.rst_reg_n;
  wire [31:0] tb_timer_count  = user_project.timer_count;
  wire        tb_timer_irq    = user_project.timer_irq;
  wire [3:0]  tb_irq          = user_project.interrupt_req;
  wire [15:0] tb_pps_count    = user_project.pps_count;
  wire [7:0]  tb_session_ctr  = user_project.session_ctr;
  wire        tb_seal_using   = user_project.seal_using_crc;
  wire [7:0]  tb_gpio_out     = user_project.gpio_out;
  wire [7:0]  tb_gpio_out_sel = user_project.gpio_out_sel;

`ifdef GL_TEST
  wire VPWR = 1'b1;
  wire VGND = 1'b0;
`endif

  tt_um_techhu_rv32_trial user_project (
`ifdef GL_TEST
      .VPWR(VPWR),
      .VGND(VGND),
`endif
      .ui_in  (ui_in),
      .uo_out (uo_out),
      .uio_in (uio_in),
      .uio_out(uio_out),
      .uio_oe (uio_oe),
      .ena    (ena),
      .clk    (clk),
      .rst_n  (rst_n)
  );

  // VCD dump
  initial begin
      $dumpfile("tb_e2e.vcd");
      $dumpvars(0, tb_e2e);
  end

endmodule
