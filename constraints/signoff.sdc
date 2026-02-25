# ============================================================
# LoRa Edge SoC — Signoff SDC (25 MHz)
# ============================================================

create_clock -name clk -period 40.0 [get_ports clk]

set_false_path -from [get_ports rst_n]
set_false_path -from [get_ports ena]

# Async inputs
set_false_path -from [get_ports {ui_in[0] ui_in[3] ui_in[4] ui_in[7]}]

# Synchronous inputs (relaxed vs PNR)
set_input_delay  -clock clk -max 12.0 [get_ports {ui_in[1] ui_in[2] ui_in[5] ui_in[6]}]
set_input_delay  -clock clk -min  2.0 [get_ports {ui_in[1] ui_in[2] ui_in[5] ui_in[6]}]

# QSPI I/O
set_input_delay  -clock clk -max 12.0 [get_ports {uio_in[*]}]
set_input_delay  -clock clk -min  2.0 [get_ports {uio_in[*]}]
set_output_delay -clock clk -max 12.0 [get_ports {uio_out[*]}]
set_output_delay -clock clk -min  2.0 [get_ports {uio_out[*]}]

# QSPI direction control
set_output_delay -clock clk -max 12.0 [get_ports {uio_oe[*]}]
set_output_delay -clock clk -min  2.0 [get_ports {uio_oe[*]}]

# Dedicated outputs
set_output_delay -clock clk -max 12.0 [get_ports {uo_out[*]}]
set_output_delay -clock clk -min  2.0 [get_ports {uo_out[*]}]
