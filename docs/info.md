<!---

This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than
512 kb in size, and the combined size of all images must be less than 1 MB.
-->

# How it works

TinyQV is a small Risc-V SoC, implementing the RV32EC instruction set plus the Zcb and Zicond extensions, with a couple of caveats:

* Addresses are 28-bits
* Program addresses are 24-bits
* gp is hardcoded to 0x1000400, tp is hardcoded to 0x8000000.

Instructions are read using QSPI from Flash, and a QSPI PSRAM is used for memory.  The QSPI clock and data lines are shared between the flash and the RAM, so only one can be accessed simultaneously.

Code can only be executed from flash.  Data can be read from flash and RAM, and written to RAM.

The SoC includes a UART, SPI controller, I2C master, CRC16 engine, hardware watchdog, RTC, countdown timer, and a cryptographic seal register. Target platform: TTIHP 26a (IHP SG13G2 130nm), clock 25MHz.

## Address map

| Address range | Device |
| ------------- | ------ |
| 0x0000000 - 0x0FFFFFF | Flash (QSPI, execute-only for code) |
| 0x1000000 - 0x17FFFFF | RAM A (QSPI PSRAM) |
| 0x1800000 - 0x1FFFFFF | RAM B (QSPI PSRAM) |
| 0x7FFFF00 - 0x7FFFFFF | Internal RAM (32 bytes, latch-based, wrapped) |

### MMIO Peripherals (0x8000000 - 0x800003C)

Peripheral address decoding: `slot = addr[6:2]`, 16 slots of 4 bytes each.

| Slot | Address | Peripheral |
| ---- | ------- | ---------- |
| 0x0 | 0x8000000 | GPIO_OUT — Output register (R/W) |
| 0x1 | 0x8000004 | GPIO_IN — Input register (R) |
| 0x2 | 0x8000008 | CRC16 — CRC16 engine data (R/W) |
| 0x3 | 0x800000C | GPIO_OUT_SEL — Pin function select (R/W) |
| 0x4 | 0x8000010 | UART — Data register (R/W) |
| 0x5 | 0x8000014 | UART_STATUS — TX busy / RX valid (R) |
| 0x6 | 0x8000018 | I2C_DATA — I2C data + command (R/W) |
| 0x7 | 0x800001C | I2C_CONFIG — I2C prescale (R/W) |
| 0x8 | 0x8000020 | SPI — SPI data (R/W) |
| 0x9 | 0x8000024 | SPI_STATUS — SPI config + busy (R/W) |
| 0xA | 0x8000028 | RTC — Seconds counter (R/W) |
| 0xB | 0x800002C | SEAL_DATA — Seal data (R/W) |
| 0xC | 0x8000030 | TIMER — Countdown timer (R/W) |
| 0xD | 0x8000034 | WDT — Watchdog (R/W) |
| 0xE | 0x8000038 | SEAL_CTRL — Seal control (R/W) |
| 0xF | 0x800003C | SYSINFO — System info + soft reset (R/W) |

### GPIO

| Register | Address | Description |
| -------- | ------- | ----------- |
| OUT      | 0x8000000 (W) | Control out0-7, if the corresponding bit in SEL is high |
| OUT      | 0x8000000 (R) | Reads the current state of out0-7 |
| IN       | 0x8000004 (R) | Reads the current state of in0-7 |
| SEL      | 0x800000C (R/W) | Bits 0-7 enable general purpose output on the corresponding bit on out0-7.  Bit 8 enables PWM output on out7, bit 9 enables PWM output on io7. |

### UART

| Register | Address | Description |
| -------- | ------- | ----------- |
| DATA     | 0x8000010 (W) | Transmits the byte |
| DATA     | 0x8000010 (R) | Reads any received byte |
| STATUS   | 0x8000014 (R) | Bit 0 indicates whether the UART TX is busy, bytes should not be written to the data register while this bit is set.  Bit 1 indicates whether a received byte is available to be read. |

### CRC16

Slot 0x2 (0x8000008). CRC-16/MODBUS engine (polynomial 0xA001, init 0xFFFF). Shared with Seal register — when Seal is active, CPU reads return busy=1.

| Register | Address | Description |
| -------- | ------- | ----------- |
| DATA     | 0x8000008 (W) | bit[8]=1: init (reset CRC to 0xFFFF). bit[8]=0: feed data[7:0] (ignored if busy) |
| DATA     | 0x8000008 (R) | `{15'b0, busy, crc[15:0]}`. busy=1: engine processing or Seal active |

### SPI

Slot 0x8 (0x8000020) + Slot 0x9 (0x8000024). SPI master for SX1268 radio. MSB-first, configurable clock divider.

| Register | Address | Description |
| -------- | ------- | ----------- |
| DATA     | 0x8000020 (W) | Transmits byte in bits[7:0]. bit[8]=end_txn (release CS after this byte). bit[9]=DC (Data/Command, directly drives out3 when not GPIO-overridden) |
| DATA     | 0x8000020 (R) | Reads the last received byte (MISO shift register) |
| CONFIG   | 0x8000024 (W) | bits[3:0]=divider: SCK = clk / (2*(divider+1)). bit[8]=read_latency: adds half a cycle to MISO sample when set |
| STATUS   | 0x8000024 (R) | bit[0]=busy. Do not write DATA or read DATA while busy |

### I2C

Slot 0x6 (0x8000018) + Slot 0x7 (0x800001C). I2C master based on Forencich i2c_master (AXI Stream). Single master, no clock stretching. Default prescale 63 → 25MHz/4/63 ≈ 99.2kHz.

| Register | Address | Description |
| -------- | ------- | ----------- |
| DATA     | 0x8000018 (W) | `{cmd_stop, cmd_write_multiple, cmd_write, cmd_read, cmd_start, addr/data[7:0]}`. See command encoding below |
| DATA     | 0x8000018 (R) | `{21'b0, rx_valid, busy, miss_ack, rx_data[7:0]}`. rx_valid cleared on read (uses `read_complete`) |
| CONFIG   | 0x800001C (W) | prescale[15:0]. SCL freq = clk / (4 * prescale). Default: 63 |
| CONFIG   | 0x800001C (R) | `{16'b0, prescale[15:0]}` |

**I2C command encoding** (write to DATA):

| Operation | cmd bits [12:8] | data[7:0] |
| --------- | --------------- | --------- |
| START + WRITE | `01101` (start + write_multiple) | 7-bit addr << 1 \| 0 |
| TX data byte | `00000` (data only, no cmd) | byte to send |
| STOP | `10000` (stop, with tlast=1) | ignored |
| START + READ | `10011` (start + read + stop) | 7-bit addr << 1 \| 1 |

### RTC

Slot 0xA (0x8000028). 32-bit seconds counter driven by 1MHz tick (25MHz / 25). Internal 20-bit microsecond divider.

| Register | Address | Description |
| -------- | ------- | ----------- |
| SECONDS  | 0x8000028 (W) | Set seconds value, resets internal microsecond divider to 0 |
| SECONDS  | 0x8000028 (R) | Current 32-bit seconds count |

### Timer

Slot 0xC (0x8000030). Countdown timer in microseconds. Fires IRQ17 on expiry.

| Register | Address | Description |
| -------- | ------- | ----------- |
| COUNT    | 0x8000030 (W) | Load countdown value (microseconds). Clears IRQ. Writing 0 stops the timer |
| COUNT    | 0x8000030 (R) | Remaining countdown value (0 = expired or stopped) |

### WDT

Slot 0xD (0x8000034). Hardware watchdog timer. Once enabled, **cannot be disabled** (write 0 is ignored). Triggers system reset on expiry.

| Register | Address | Description |
| -------- | ------- | ----------- |
| KICK     | 0x8000034 (W) | Write non-zero: load countdown (microseconds) + enable WDT. Write 0: ignored if already enabled |
| REMAINING | 0x8000034 (R) | Remaining countdown microseconds. 0 = disabled or expired |

### SEAL

Slot 0xB (SEAL_DATA) + Slot 0xE (SEAL_CTRL). Cryptographic monotonic counter with CRC16 integrity.

See [seal docs](seal.md)

### SysInfo

Slot 0xF (0x800003C). System identification and soft reset.

| Register | Address | Description |
| -------- | ------- | ----------- |
| SYSINFO  | 0x800003C (R) | `{pps_count[15:0], chip_id[7:0], version[7:0]}`. chip_id=0x01, version=0x10 (v1.0) |
| SOFT_RST | 0x800003C (W) | Write 0xA5 to trigger soft reset (32-cycle hold). Other values ignored |

`pps_count` increments on each rising edge of `ui_in[4]` (1PPS input), 2-stage CDC synchronized.

### PWM

Slot (legacy, no dedicated slot — uses GPIO_OUT_SEL bit[8:9] to route PWM to out7/io7).

| Register | Address | Description |
| -------- | ------- | ----------- |
| LEVEL    | 0x8000028 (W) | Set the PWM output level (0-255) |

*Note: PWM shares the RTC address slot in this fork. PWM is not connected in LoRa Edge SoC; the slot is used by RTC.*

### DEBUG

See [debug docs](debug.md)

## Pin mapping

### Dedicated outputs (`uo_out[7:0]`)

Each pin has a default hardware function. Setting the corresponding bit in `GPIO_OUT_SEL` overrides it with the `GPIO_OUT` register value.

| Pin | Default function | GPIO override |
| --- | ---------------- | ------------- |
| uo_out[0] | UART TX | gpio_out[0] |
| uo_out[1] | SX1268 RESET (default HIGH = not reset) | gpio_out[1] |
| uo_out[2] | I2C SCL (0=pull low, 1=release) | gpio_out[2] |
| uo_out[3] | SPI MOSI → SX1268 | gpio_out[3] |
| uo_out[4] | SPI CS → SX1268 | gpio_out[4] |
| uo_out[5] | SPI SCK → SX1268 | gpio_out[5] |
| uo_out[6] | I2C SDA (0=pull low, 1=release) | gpio_out[6] |
| uo_out[7] | LED GPIO (default LOW) | gpio_out[7] |

### Dedicated inputs (`ui_in[7:0]`)

| Pin | Function |
| --- | -------- |
| ui_in[0] | SX1268 DIO1 (IRQ) |
| ui_in[1] | SX1268 BUSY (polling) |
| ui_in[2] | SPI MISO ← SX1268 |
| ui_in[3] | I2C SDA input |
| ui_in[4] | 1PPS input (GPS) |
| ui_in[5] | GPIO in (spare) |
| ui_in[6] | GPIO in (spare) |
| ui_in[7] | UART RX |

### Bidirectional IOs (`uio[7:0]`)

All bidirectional IOs are used by the QSPI interface for Flash and PSRAM. Active only when `rst_n` is high.

| Pin | Function |
| --- | -------- |
| uio[0] | QSPI Flash CS |
| uio[1:2] | QSPI SD0-SD1 |
| uio[3] | QSPI CLK |
| uio[4:5] | QSPI SD2-SD3 |
| uio[6] | QSPI RAM A CS |
| uio[7] | QSPI RAM B CS |

# How to test

Load an image into flash and then select the design.

Reset the design as follows:

* Set rst_n high and then low to ensure the design sees a falling edge of rst_n.  The bidirectional IOs are all set to inputs while rst_n is low.
* Program the flash and leave flash in continuous read mode, and the PSRAMs in QPI mode
* Drive all the QSPI CS high and set SD1:SD0 to the read latency of the QSPI flash and PSRAM in cycles.
* Clock at least 8 times and stop with clock high
* Release all the QSPI lines
* Set rst_n high
* Set clock low
* Start clocking normally

Target clock is 25MHz. A read latency of 2 or 3 is likely required for the QSPI interface.

Build programs using the RISC-V toolchain:

```
riscv64-elf-gcc -march=rv32ec_zicsr -mabi=ilp32e -nostdlib -Os -T fw.ld -o fw.elf fw.c
riscv64-elf-objcopy -O verilog --verilog-data-width=4 fw.elf fw.hex
```

See also [tinyQV-sdk](https://github.com/MichaelBell/tinyQV-sdk) and [tinyQV-projects](https://github.com/MichaelBell/tinyQV-projects) for reference firmware examples.

# External hardware

The design is intended to be used with this [QSPI PMOD](https://github.com/mole99/qspi-pmod) on the bidirectional PMOD.  This has a 16MB flash and 2 8MB RAMs.

The UART is on the correct pins to be used with the hardware UART on the RP2040 on the demo board.

The SPI controller drives a Semtech SX1268 LoRa transceiver. The I2C master connects to environmental sensors (e.g., SHT31 temperature/humidity at address 0x44). A 1PPS GPS signal can be connected to `ui_in[4]` for time synchronization.
