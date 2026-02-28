TOP_DIR     := ../..
SRC_DIR     := $(TOP_DIR)/src
HUB_INC     := /Users/techhu/Code/HUB_Rev1/include

VERILOG_SRCS := seal_tb_top.v $(SRC_DIR)/seal_register.v $(SRC_DIR)/crc16_engine.v

.PHONY: build run clean

build:
	verilator --cc --exe --build \
	    -Wall -Wno-fatal \
	    --top-module seal_tb_top \
	    $(VERILOG_SRCS) \
	    sim_seal.cpp \
	    -CFLAGS "-std=c++17 -I$(HUB_INC) -I. -O2" \
	    -o sim_seal

run: build
	./obj_dir/sim_seal

clean:
	rm -rf obj_dir
