# ============================================================
# LoRa Edge SoC — PNR SDC (25 MHz)
# ============================================================
# Strategy: PNR constraints are intentionally LOOSER than signoff
# (max=15ns vs signoff max=12ns) to give the placer more freedom
# for area optimization, while signoff tightens to ensure real
# board-level I/O timing. Hold (min) is SAME in both (2.0ns)
# to ensure sufficient hold margin is built in during PNR.
# ============================================================

# Primary clock: 25 MHz (40ns period)
create_clock -name clk -period 40.0 [get_ports clk]

# Async reset — no timing path
set_false_path -from [get_ports rst_n]

# TT framework enable — unused, no timing path
set_false_path -from [get_ports ena]

# Async inputs — no fixed phase relationship to clk
# DIO1 (2-stage sync in RTL), 1PPS (2-stage sync), UART RX (internal), I2C SDA (internal)
set_false_path -from [get_ports {ui_in[0] ui_in[3] ui_in[4] ui_in[7]}]

# Synchronous inputs (SPI MISO, SX1268 BUSY, spare GPIO)
set_input_delay  -clock clk -max 15.0 [get_ports {ui_in[1] ui_in[2] ui_in[5] ui_in[6]}]
set_input_delay  -clock clk -min  2.0 [get_ports {ui_in[1] ui_in[2] ui_in[5] ui_in[6]}]

# QSPI Flash/PSRAM I/O (bidirectional, relative to clk)
set_input_delay  -clock clk -max 15.0 [get_ports {uio_in[*]}]
set_input_delay  -clock clk -min  2.0 [get_ports {uio_in[*]}]
set_output_delay -clock clk -max 15.0 [get_ports {uio_out[*]}]
set_output_delay -clock clk -min  2.0 [get_ports {uio_out[*]}]

# QSPI direction control (uio_oe) — critical for bus turnaround timing
set_output_delay -clock clk -max 15.0 [get_ports {uio_oe[*]}]
set_output_delay -clock clk -min  2.0 [get_ports {uio_oe[*]}]

# Dedicated outputs (uo_out): UART TX, SPI, I2C, LED
set_output_delay -clock clk -max 15.0 [get_ports {uo_out[*]}]
set_output_delay -clock clk -min  2.0 [get_ports {uo_out[*]}]
