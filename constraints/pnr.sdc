# ============================================================
# LoRa Edge SoC — PNR SDC (25 MHz)
# ============================================================

# Primary clock: 25 MHz (40ns period)
create_clock -name clk -period 40.0 [get_ports clk]

# Async reset — no timing path
set_false_path -from [get_ports rst_n]

# QSPI Flash/PSRAM I/O (bidirectional, relative to clk)
# QSPI SCK is derived from clk inside qspi_ctrl, not a separate clock
set_input_delay  -clock clk -max 15.0 [get_ports {uio_in[*]}]
set_input_delay  -clock clk -min  1.0 [get_ports {uio_in[*]}]
set_output_delay -clock clk -max 15.0 [get_ports {uio_out[*]}]
set_output_delay -clock clk -min  1.0 [get_ports {uio_out[*]}]

# Dedicated inputs (ui_in): SX1268 signals, I2C SDA in, UART RX, 1PPS
set_input_delay  -clock clk -max 15.0 [get_ports {ui_in[*]}]
set_input_delay  -clock clk -min  1.0 [get_ports {ui_in[*]}]

# Dedicated outputs (uo_out): UART TX, SPI, I2C, LED
set_output_delay -clock clk -max 15.0 [get_ports {uo_out[*]}]
set_output_delay -clock clk -min  1.0 [get_ports {uo_out[*]}]
