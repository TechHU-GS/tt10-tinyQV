# ============================================================
# LoRa Edge SoC — Signoff SDC (25 MHz)
# ============================================================

create_clock -name clk -period 40.0 [get_ports clk]

set_false_path -from [get_ports rst_n]

# QSPI I/O (tighter for signoff)
set_input_delay  -clock clk -max 12.0 [get_ports {uio_in[*]}]
set_input_delay  -clock clk -min  2.0 [get_ports {uio_in[*]}]
set_output_delay -clock clk -max 12.0 [get_ports {uio_out[*]}]
set_output_delay -clock clk -min  2.0 [get_ports {uio_out[*]}]

# Dedicated inputs
set_input_delay  -clock clk -max 12.0 [get_ports {ui_in[*]}]
set_input_delay  -clock clk -min  2.0 [get_ports {ui_in[*]}]

# Dedicated outputs
set_output_delay -clock clk -max 12.0 [get_ports {uo_out[*]}]
set_output_delay -clock clk -min  2.0 [get_ports {uo_out[*]}]
