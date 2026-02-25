/*
 * LoRa Edge SoC — TTIHP 26a
 * Based on tt10-tinyQV by Michael Bell (Apache-2.0)
 * Modified for LoRa node SoC with CRC16, I2C, WDT, RTC, Seal peripherals
 *
 * Copyright (c) 2024 Michael Bell (original)
 * Copyright (c) 2026 TechHU-GS (modifications)
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tt_um_MichaelBell_tinyQV (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);

    // ================================================================
    // Peripheral address map (slot = addr[5:2], 16 slots)
    // ================================================================
    localparam PERI_NONE       = 5'h1F;
    localparam PERI_GPIO_OUT   = 5'h0;   // R/W: GPIO output
    localparam PERI_GPIO_IN    = 5'h1;   // R:   GPIO input
    localparam PERI_CRC16      = 5'h2;   // R/W: CRC16 engine
    localparam PERI_GPIO_OUT_SEL = 5'h3; // R/W: Pin function select
    localparam PERI_UART       = 5'h4;   // R/W: UART data
    localparam PERI_UART_STATUS = 5'h5;  // R:   UART status
    localparam PERI_I2C_DATA   = 5'h6;   // R/W: I2C data + command
    localparam PERI_I2C_CONFIG = 5'h7;   // R/W: I2C prescale
    localparam PERI_SPI        = 5'h8;   // R/W: SPI data
    localparam PERI_SPI_STATUS = 5'h9;   // R/W: SPI config + status
    localparam PERI_RTC        = 5'hA;   // R/W: RTC seconds
    localparam PERI_SEAL_DATA  = 5'hB;   // R/W: Seal data (3x read serialization)
    localparam PERI_TIMER      = 5'hC;   // R/W: Countdown timer
    localparam PERI_WDT        = 5'hD;   // R/W: Watchdog kick
    localparam PERI_SEAL_CTRL  = 5'hE;   // R/W: Seal control
    localparam PERI_SYSINFO    = 5'hF;   // R/W: SYS_INFO + soft reset

    // ================================================================
    // Reset: sync on posedge (changed from tt10's negedge for WDT/soft reset)
    // ================================================================
    reg rst_reg_n;

    // WDT / soft reset hold counter (32 clk minimum hold)
    reg [5:0] reset_hold_counter;
    wire wdt_reset;
    wire soft_reset = (write_n != 2'b11) && (connect_peripheral == PERI_SYSINFO)
                      && (data_to_write[7:0] == 8'hA5);
    wire reset_trigger = wdt_reset || soft_reset;

    always @(posedge clk) begin
        if (!rst_n)
            reset_hold_counter <= 0;
        else if (reset_trigger)
            reset_hold_counter <= 6'd32;
        else if (reset_hold_counter != 0)
            reset_hold_counter <= reset_hold_counter - 1;
    end
    wire combined_rst_n = rst_n && (reset_hold_counter == 0);
    always @(posedge clk) rst_reg_n <= combined_rst_n;

    // ================================================================
    // QSPI interface (unchanged from tt10)
    // ================================================================
    wire [3:0] qspi_data_in = {uio_in[5:4], uio_in[2:1]};
    wire [3:0] qspi_data_out;
    wire [3:0] qspi_data_oe;
    wire       qspi_clk_out;
    wire       qspi_flash_select;
    wire       qspi_ram_a_select;
    wire       qspi_ram_b_select;
    assign uio_out = {qspi_ram_b_select, qspi_ram_a_select, qspi_data_out[3:2],
                      qspi_clk_out, qspi_data_out[1:0], qspi_flash_select};
    assign uio_oe = rst_n ? {2'b11, qspi_data_oe[3:2], 1'b1, qspi_data_oe[1:0], 1'b1} : 8'h00;

    // ================================================================
    // TinyQV CPU data bus signals
    // ================================================================
    wire [27:0] addr;
    wire  [1:0] write_n;
    wire  [1:0] read_n;
    wire        read_complete;
    wire [31:0] data_to_write;
    wire        data_ready;
    reg  [31:0] data_from_read;

    // Latch memory (32 bytes on-chip)
    wire       lmem_data_ready;
    wire [1:0] lmem_write_n;
    wire [1:0] lmem_read_n;
    wire [31:0] lmem_data_from_read;

    reg [4:0] connect_peripheral;

    // All transactions to peripherals complete immediately
    assign data_ready = addr[26] ? lmem_data_ready : 1'b1;
    assign lmem_write_n = addr[26] ? write_n : 2'b11;
    assign lmem_read_n = addr[26] ? read_n : 2'b11;

    // ================================================================
    // Peripheral I/O pin assignments (LoRa Edge SoC)
    // ================================================================
    wire       spi_miso   = ui_in[2];   // SPI MISO ← SX1268
    wire       uart_rxd   = ui_in[7];   // UART RX
    wire       i2c_sda_i  = ui_in[3];   // I2C SDA input
    // ui_in[0] = SX1268 DIO1 (IRQ), ui_in[1] = SX1268 BUSY (polling)
    // ui_in[4] = 1PPS input, ui_in[5:6] = GPIO in (spare)

    wire       spi_cs;
    wire       spi_sck;
    wire       spi_mosi;
    wire       uart_txd;
    reg  [7:0] gpio_out_sel;
    reg  [7:0] gpio_out;

    // I2C signals (from i2c_peripheral)
    wire       i2c_scl_t;   // 1=drive low, 0=release
    wire       i2c_sda_t;   // 1=drive low, 0=release

    // ================================================================
    // UART
    // ================================================================
    wire uart_tx_busy;
    wire uart_rx_valid;
    wire [7:0] uart_rx_data;
    wire uart_tx_start = write_n != 2'b11 && connect_peripheral == PERI_UART;

    // ================================================================
    // SPI
    // ================================================================
    wire spi_start = write_n != 2'b11 && connect_peripheral == PERI_SPI;
    wire [7:0] spi_data;
    wire spi_busy;

    // ================================================================
    // 1MHz tick generator (25MHz / 25 = 1MHz)
    // ================================================================
    reg [4:0] us_divider;
    wire tick_1us = (us_divider == 5'd24);
    always @(posedge clk) begin
        if (!rst_reg_n || tick_1us) us_divider <= 0;
        else us_divider <= us_divider + 1;
    end

    // ================================================================
    // Countdown timer (replaces tinyQV_time, 1 slot)
    // ================================================================
    reg [31:0] timer_count;
    reg        timer_irq;
    always @(posedge clk) begin
        if (!rst_reg_n) begin
            timer_count <= 0;
            timer_irq   <= 0;
        end else begin
            if (write_n != 2'b11 && connect_peripheral == PERI_TIMER) begin
                timer_count <= data_to_write[31:0];
                timer_irq   <= 0;
            end
            else if (tick_1us && timer_count != 0) begin
                timer_count <= timer_count - 1;
                if (timer_count == 32'd1) timer_irq <= 1;
            end
        end
    end

    // ================================================================
    // 1PPS synchronizer + counter
    // ================================================================
    reg [1:0] pps_sync;
    reg       pps_prev;
    always @(posedge clk) begin
        pps_sync <= {pps_sync[0], ui_in[4]};
        pps_prev <= pps_sync[1];
    end
    wire pps_rising = pps_sync[1] && !pps_prev;

    reg [15:0] pps_count;
    always @(posedge clk) begin
        if (!rst_reg_n) pps_count <= 0;
        else if (pps_rising) pps_count <= pps_count + 1;
    end

    // ================================================================
    // SYS_INFO constants
    // ================================================================
    localparam CHIP_ID = 8'h01;  // LoRa Edge SoC v1
    localparam VERSION = 8'h10;  // v1.0
    wire [31:0] sysinfo_read = {pps_count, CHIP_ID, VERSION};

    // ================================================================
    // session_ctr: free-running counter for Seal session_id
    // ================================================================
    reg [9:0]  session_ms_div;
    reg [7:0]  session_ctr;
    always @(posedge clk) begin
        if (!rst_reg_n) begin
            session_ms_div <= 0;
            session_ctr    <= 0;
        end else if (tick_1us) begin
            if (session_ms_div == 10'd999) begin
                session_ms_div <= 0;
                session_ctr    <= session_ctr + 1;
            end else
                session_ms_div <= session_ms_div + 1;
        end
    end

    // ================================================================
    // Interrupt mapping (timer only via IRQ17, timer_interrupt tied 0)
    // ================================================================
    // 2-stage synchronizer for async DIO1 input (metastability protection)
    reg [1:0] dio1_sync;
    always @(posedge clk) begin
        if (!rst_reg_n)
            dio1_sync <= 2'b0;
        else
            dio1_sync <= {dio1_sync[0], ui_in[0]};
    end
    wire [3:0] interrupt_req = {
        1'b0,              // [3] IRQ19: reserved (TX ready is level — use polling)
        uart_rx_valid,     // [2] IRQ18: UART RX data
        timer_irq,         // [1] IRQ17: countdown expired
        dio1_sync[1]       // [0] IRQ16: SX1268 DIO1 (level-sensitive, cleared via SPI)
    };

    // ================================================================
    // TinyQV CPU instance (unchanged core)
    // ================================================================

    // Debug signals — directly expose selected ones, no mux
    wire       debug_instr_complete;
    wire       debug_instr_ready;
    wire       debug_instr_valid;
    wire       debug_fetch_restart;
    wire       debug_data_ready;
    wire       debug_interrupt_pending;
    wire       debug_branch;
    wire       debug_early_branch;
    wire       debug_ret;
    wire       debug_reg_wen;
    wire       debug_counter_0;
    wire       debug_data_continue;
    wire       debug_stall_txn;
    wire       debug_stop_txn;
    wire [3:0] debug_rd;

    tinyQV i_tinyqv(
        .clk(clk),
        .rstn(rst_reg_n),

        .data_addr(addr),
        .data_write_n(write_n),
        .data_read_n(read_n),
        .data_read_complete(read_complete),
        .data_out(data_to_write),

        .data_ready(data_ready),
        .data_in(data_from_read),

        .interrupt_req(interrupt_req),
        .timer_interrupt(1'b0),  // Timer only via IRQ17, not mtip

        .spi_data_in(qspi_data_in),
        .spi_data_out(qspi_data_out),
        .spi_data_oe(qspi_data_oe),
        .spi_clk_out(qspi_clk_out),
        .spi_flash_select(qspi_flash_select),
        .spi_ram_a_select(qspi_ram_a_select),
        .spi_ram_b_select(qspi_ram_b_select),

        .debug_instr_complete(debug_instr_complete),
        .debug_instr_ready(debug_instr_ready),
        .debug_instr_valid(debug_instr_valid),
        .debug_fetch_restart(debug_fetch_restart),
        .debug_data_ready(debug_data_ready),
        .debug_interrupt_pending(debug_interrupt_pending),
        .debug_branch(debug_branch),
        .debug_early_branch(debug_early_branch),
        .debug_ret(debug_ret),
        .debug_reg_wen(debug_reg_wen),
        .debug_counter_0(debug_counter_0),
        .debug_data_continue(debug_data_continue),
        .debug_stall_txn(debug_stall_txn),
        .debug_stop_txn(debug_stop_txn),
        .debug_rd(debug_rd)
    );

    // ================================================================
    // Output pin assignments (LoRa Edge SoC)
    // ================================================================
    assign uo_out[0] = gpio_out_sel[0] ? gpio_out[0] : uart_txd;         // UART TX
    assign uo_out[1] = gpio_out_sel[1] ? gpio_out[1] : 1'b1;             // SX1268 RESET (default high=not reset)
    assign uo_out[2] = gpio_out_sel[2] ? gpio_out[2] : i2c_scl_t;  // I2C SCL (Forencich: _t=_o, 0=low 1=release)
    assign uo_out[3] = gpio_out_sel[3] ? gpio_out[3] : spi_mosi;         // SPI MOSI → SX1268
    assign uo_out[4] = gpio_out_sel[4] ? gpio_out[4] : spi_cs;           // SPI CS → SX1268
    assign uo_out[5] = gpio_out_sel[5] ? gpio_out[5] : spi_sck;          // SPI SCK → SX1268
    assign uo_out[6] = gpio_out_sel[6] ? gpio_out[6] : i2c_sda_t;  // I2C SDA (Forencich: _t=_o, 0=low 1=release)
    assign uo_out[7] = gpio_out_sel[7] ? gpio_out[7] : 1'b0;             // LED GPIO

    // ================================================================
    // Peripheral address decode
    // ================================================================
    always @(*) begin
        if ({addr[27:7], addr[1:0]} == 23'h400000)
            connect_peripheral = addr[6:2];
        else
            connect_peripheral = PERI_NONE;
    end

    // ================================================================
    // CRC16 engine (shared) + peripheral bridge
    // ================================================================
    wire        crc16_wr = (write_n != 2'b11) && (connect_peripheral == PERI_CRC16);

    // Shared CRC engine signals
    wire        crc_engine_init;
    wire [7:0]  crc_engine_data;
    wire        crc_engine_dv;
    wire [15:0] crc_engine_out;
    wire        crc_engine_busy;

    // Peripheral bridge signals (may be muxed with seal in v1.0)
    wire        crc_peri_init;
    wire [7:0]  crc_peri_data;
    wire        crc_peri_dv;
    wire [31:0] crc_peri_data_out;

    // CRC arbitration: seal takes priority over CPU peripheral bridge
    wire seal_using_crc = (seal_ctrl_out[0]);  // seal busy = state != IDLE
    assign crc_engine_init = seal_using_crc ? seal_crc_init  : crc_peri_init;
    assign crc_engine_data = seal_using_crc ? seal_crc_byte  : crc_peri_data;
    assign crc_engine_dv   = seal_using_crc ? seal_crc_feed  : crc_peri_dv;

    // When seal is active, CPU CRC16_DATA read shows busy=1
    wire [31:0] crc16_read = seal_using_crc ? {15'b0, 1'b1, crc_engine_out} : crc_peri_data_out;

    crc16_engine i_crc16 (
        .clk        (clk),
        .rst_n      (rst_reg_n),
        .init       (crc_engine_init),
        .data_in    (crc_engine_data),
        .data_valid (crc_engine_dv),
        .crc_out    (crc_engine_out),
        .busy       (crc_engine_busy)
    );

    crc16_peripheral i_crc16_peri (
        .clk            (clk),
        .rst_n          (rst_reg_n),
        .data_in        (data_to_write),
        .wr_en          (crc16_wr),
        .data_out       (crc_peri_data_out),
        .crc_init       (crc_peri_init),
        .crc_data       (crc_peri_data),
        .crc_data_valid (crc_peri_dv),
        .crc_value      (crc_engine_out),
        .crc_busy       (crc_engine_busy)
    );

    // ================================================================
    // Watchdog Timer
    // ================================================================
    wire        wdt_kick = (write_n != 2'b11) && (connect_peripheral == PERI_WDT);
    wire [31:0] wdt_remaining;

    watchdog i_wdt (
        .clk        (clk),
        .rst_n      (rst_reg_n),
        .tick_1us   (tick_1us),
        .kick       (wdt_kick),
        .kick_value (data_to_write),
        .remaining  (wdt_remaining),
        .wdt_reset  (wdt_reset)
    );

    // ================================================================
    // RTC Counter
    // ================================================================
    wire        rtc_wr = (write_n != 2'b11) && (connect_peripheral == PERI_RTC);
    wire [31:0] rtc_seconds;

    rtc_counter i_rtc (
        .clk         (clk),
        .rst_n       (rst_reg_n),
        .tick_1us    (tick_1us),
        .wr_en       (rtc_wr),
        .data_in     (data_to_write),
        .seconds_out (rtc_seconds)
    );

    // ================================================================
    // Seal Register + Monotonic Counter
    // ================================================================
    wire        seal_data_wr = (write_n != 2'b11) && (connect_peripheral == PERI_SEAL_DATA);
    wire        seal_data_rd = (read_n  != 2'b11) && (connect_peripheral == PERI_SEAL_DATA);
    wire        seal_ctrl_wr = (write_n != 2'b11) && (connect_peripheral == PERI_SEAL_CTRL);
    wire [31:0] seal_data_out;
    wire [31:0] seal_ctrl_out;

    // Seal ↔ CRC engine signals
    wire [7:0]  seal_crc_byte;
    wire        seal_crc_feed;
    wire        seal_crc_init;

    seal_register i_seal (
        .clk            (clk),
        .rst_n          (rst_reg_n),
        .crc_byte       (seal_crc_byte),
        .crc_feed       (seal_crc_feed),
        .crc_busy       (crc_engine_busy),
        .crc_value      (crc_engine_out),
        .crc_init       (seal_crc_init),
        .data_wr        (seal_data_wr),
        .data_in        (data_to_write),
        .data_out       (seal_data_out),
        .data_rd        (seal_data_rd),
        .ctrl_wr        (seal_ctrl_wr),
        .ctrl_in        (data_to_write[9:0]),
        .ctrl_out       (seal_ctrl_out),
        .session_ctr_in (session_ctr)
    );

    // ================================================================
    // I2C Master (Forencich) + MMIO Bridge
    // ================================================================
    wire        i2c_data_wr = (write_n != 2'b11) && (connect_peripheral == PERI_I2C_DATA);
    wire        i2c_data_rd = (read_n  != 2'b11) && (connect_peripheral == PERI_I2C_DATA);
    wire        i2c_config_wr = (write_n != 2'b11) && (connect_peripheral == PERI_I2C_CONFIG);
    wire [31:0] i2c_data_out;
    wire [31:0] i2c_config_out;

    i2c_peripheral i_i2c_peri (
        .clk        (clk),
        .rst_n      (rst_reg_n),
        .data_in    (data_to_write),
        .data_wr    (i2c_data_wr),
        .data_rd    (i2c_data_rd),
        .data_out   (i2c_data_out),
        .config_in  (data_to_write),
        .config_wr  (i2c_config_wr),
        .config_out (i2c_config_out),
        .scl_i      (1'b1),         // Single master, no clock stretching detection
        .scl_o      (),
        .scl_t      (i2c_scl_t),
        .sda_i      (i2c_sda_i),
        .sda_o      (),
        .sda_t      (i2c_sda_t)
    );

    // ================================================================
    // Read data mux
    // ================================================================
    always @(*) begin
        if (addr[26]) data_from_read = lmem_data_from_read;
        else begin
            case (connect_peripheral)
                PERI_GPIO_OUT:     data_from_read = {24'h0, uo_out};
                PERI_GPIO_IN:      data_from_read = {24'h0, ui_in};
                PERI_CRC16:        data_from_read = crc16_read;
                PERI_GPIO_OUT_SEL: data_from_read = {24'h0, gpio_out_sel};
                PERI_UART:         data_from_read = {24'h0, uart_rx_data};
                PERI_UART_STATUS:  data_from_read = {30'h0, uart_rx_valid, uart_tx_busy};
                PERI_I2C_DATA:     data_from_read = i2c_data_out;
                PERI_I2C_CONFIG:   data_from_read = i2c_config_out;
                PERI_SPI:          data_from_read = {24'h0, spi_data};
                PERI_SPI_STATUS:   data_from_read = {31'h0, spi_busy};
                PERI_RTC:          data_from_read = rtc_seconds;
                PERI_SEAL_DATA:    data_from_read = seal_data_out;
                PERI_TIMER:        data_from_read = timer_count;
                PERI_WDT:          data_from_read = wdt_remaining;
                PERI_SEAL_CTRL:    data_from_read = seal_ctrl_out;
                PERI_SYSINFO:      data_from_read = sysinfo_read;
                default:           data_from_read = 32'hFFFF_FFFF;
            endcase
        end
    end

    // ================================================================
    // GPIO Out
    // ================================================================
    always @(posedge clk) begin
        if (!rst_reg_n) begin
            gpio_out_sel <= 8'b0000_0000;
            gpio_out <= 0;
        end else if (write_n != 2'b11) begin
            if (connect_peripheral == PERI_GPIO_OUT) gpio_out <= data_to_write[7:0];
            if (connect_peripheral == PERI_GPIO_OUT_SEL) gpio_out_sel <= data_to_write[7:0];
        end
    end

    // ================================================================
    // UART (25MHz, 115200 baud)
    // ================================================================
    uart_tx #(.CLK_HZ(25_000_000), .BIT_RATE(115_200)) i_uart_tx(
        .clk(clk),
        .resetn(rst_reg_n),
        .uart_txd(uart_txd),
        .uart_tx_en(uart_tx_start),
        .uart_tx_data(data_to_write[7:0]),
        .uart_tx_busy(uart_tx_busy)
    );

    uart_rx #(.CLK_HZ(25_000_000), .BIT_RATE(115_200)) i_uart_rx(
        .clk(clk),
        .resetn(rst_reg_n),
        .uart_rxd(uart_rxd),
        .uart_rts(),  // RTS not connected (pin used for SX1268 RESET)
        .uart_rx_read(connect_peripheral == PERI_UART && read_complete),
        .uart_rx_valid(uart_rx_valid),
        .uart_rx_data(uart_rx_data)
    );

    // ================================================================
    // SPI Master (full 4-bit divider, fix from tt10 2-bit)
    // ================================================================
    spi_ctrl i_spi(
        .clk(clk),
        .rstn(rst_reg_n),

        .spi_miso(spi_miso),
        .spi_select(spi_cs),
        .spi_clk_out(spi_sck),
        .spi_mosi(spi_mosi),
        .spi_dc(),  // DC not connected (pin used for I2C SCL)

        .dc_in(data_to_write[9]),
        .end_txn(data_to_write[8]),
        .data_in(data_to_write[7:0]),
        .start(spi_start),
        .data_out(spi_data),
        .busy(spi_busy),

        .set_config(connect_peripheral == PERI_SPI_STATUS && write_n != 2'b11),
        .divider_in(data_to_write[3:0]),
        .read_latency_in(data_to_write[8])
    );

    // ================================================================
    // Latch memory (32 bytes on-chip)
    // ================================================================
    latch_mem i_latch_mem (
        .clk(clk),
        .rstn(rst_reg_n),

        .addr_in(addr[4:0]),
        .data_in(data_to_write),

        .data_write_n(lmem_write_n),
        .data_read_n(lmem_read_n),

        .data_out(lmem_data_from_read),
        .data_ready(lmem_data_ready)
    );

    // ================================================================
    // Unused inputs
    // ================================================================
    wire _unused = &{ena, uio_in[7:6], uio_in[3], uio_in[0],
                     debug_instr_complete, debug_instr_ready, debug_instr_valid,
                     debug_fetch_restart, debug_data_ready, debug_interrupt_pending,
                     debug_branch, debug_early_branch, debug_ret, debug_reg_wen,
                     debug_counter_0, debug_data_continue, debug_stall_txn,
                     debug_stop_txn, debug_rd, read_complete, 1'b0};

endmodule
