SIM ?= icarus
TOPLEVEL_LANG ?= verilog

SRC_DIR = $(PWD)/../src

PROJECT_SOURCES = project.v latch_mem.v crc16_engine.v crc16_peripheral.v \
                  seal_register.v i2c_peripheral.v i2c_master.v watchdog.v \
                  rtc_counter.v \
                  tinyQV/cpu/*.v tinyQV/peri/uart/*.v tinyQV/peri/spi/*.v

SIM_BUILD = sim_build/periph

VERILOG_SOURCES += $(addprefix $(SRC_DIR)/,$(PROJECT_SOURCES))
VERILOG_SOURCES += $(PWD)/tb_periph.v $(PWD)/i2c_slave_model.v

COMPILE_ARGS += -DSIM -I$(SRC_DIR) -g2012

TOPLEVEL = tb_periph
MODULE = test_peripherals

include $(shell python3 -m cocotb_tools.config --makefiles)/Makefile.sim
