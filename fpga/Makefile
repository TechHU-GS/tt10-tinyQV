# =============================================================================
# Makefile — LoRa Edge SoC on Alchitry Cu V2 (iCE40HX8K-CB132)
# =============================================================================

PROJ    = lora_edge_soc
DEVICE  = hx8k
PACKAGE = cb132
FREQ    = 25

# Source directories
SRCDIR  = ../src
CPUDIR  = $(SRCDIR)/tinyQV/cpu
UARTDIR = $(SRCDIR)/tinyQV/peri/uart
SPIDIR  = $(SRCDIR)/tinyQV/peri/spi

# FPGA wrapper
FPGA_SRC = alchitry_cu_v2.v

# LoRa Edge SoC top + custom peripherals
SOC_SRC = \
	$(SRCDIR)/project.v \
	$(SRCDIR)/crc16_engine.v \
	$(SRCDIR)/crc16_peripheral.v \
	$(SRCDIR)/i2c_master.v \
	$(SRCDIR)/i2c_peripheral.v \
	$(SRCDIR)/watchdog.v \
	$(SRCDIR)/rtc_counter.v \
	$(SRCDIR)/seal_register.v \
	$(SRCDIR)/latch_mem.v

# TinyQV CPU core
CPU_SRC = \
	$(CPUDIR)/tinyqv.v \
	$(CPUDIR)/cpu.v \
	$(CPUDIR)/core.v \
	$(CPUDIR)/alu.v \
	$(CPUDIR)/decode.v \
	$(CPUDIR)/register.v \
	$(CPUDIR)/mem_ctrl.v \
	$(CPUDIR)/qspi_ctrl.v \
	$(CPUDIR)/counter.v \
	$(CPUDIR)/latch_reg.v

# TinyQV peripherals (UART + SPI)
PERI_SRC = \
	$(UARTDIR)/uart_tx.v \
	$(UARTDIR)/uart_rx.v \
	$(SPIDIR)/spi.v

# All sources
SRC = $(FPGA_SRC) $(SOC_SRC) $(CPU_SRC) $(PERI_SRC)

PCF = alchitry_cu_v2.pcf

# =============================================================================
# Build targets
# =============================================================================

all: $(PROJ).bin

# Synthesis
$(PROJ).json: $(SRC)
	yosys -p "synth_ice40 -top fpga_top -json $@" $(SRC)

# Place & Route
$(PROJ).asc: $(PROJ).json $(PCF)
	nextpnr-ice40 --$(DEVICE) --package $(PACKAGE) --freq $(FREQ) \
		--json $< --pcf $(PCF) --asc $@

# Bitstream
$(PROJ).bin: $(PROJ).asc
	icepack $< $@

# Program (via iceprog / Alchitry Loader)
prog: $(PROJ).bin
	iceprog $<

# Synthesis-only check (no board needed)
synth: $(PROJ).json
	@echo "Synthesis complete. Check $(PROJ).json"

# Report utilization
report: $(PROJ).json
	yosys -p "synth_ice40 -top fpga_top" $(SRC) 2>&1 | tail -30

clean:
	rm -f $(PROJ).json $(PROJ).asc $(PROJ).bin

.PHONY: all prog synth report clean
