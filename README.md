![](../../workflows/gds/badge.svg) ![](../../workflows/docs/badge.svg) ![](../../workflows/test/badge.svg) ![](../../workflows/fpga/badge.svg)

# LoRa Edge SoC

A RISC-V IoT SoC targeting TTIHP 26a (IHP SG13G2 130nm), built for LoRa edge node applications.

## Architecture

Based on [MichaelBell/tinyQV](https://github.com/MichaelBell/tinyQV) (RV32EC @ 25MHz, 4-bit serial datapath), extended with 8 custom peripherals:

| Peripheral | Description |
|------------|-------------|
| CRC16 | Hardware CRC16-CCITT engine with byte-level feed |
| I2C Master | Full I2C master (Forencich AXI-Stream core + bridge) |
| Watchdog | Hardware WDT with configurable timeout + reboot |
| RTC | 32-bit real-time counter with 1PPS sync |
| Timer | Countdown timer with IRQ |
| Seal Register | Cryptographic integrity watermark with monotonic counter |
| SysInfo | Chip ID, build info, session counter |
| Latch Memory | 256-byte scratchpad (latch-based SRAM alternative) |

## Key Specs

- **Process**: IHP SG13G2 130nm (TTIHP 26a shuttle)
- **Tile size**: 4x2 (854 x 314 um)
- **Clock**: 25MHz
- **ISA**: RV32EC + Zcb + Zicond
- **Memory**: External QSPI Flash (code) + QSPI PSRAM (data)
- **I/O**: UART, SPI, I2C, GPIO, SX1268 LoRa interface

## Documentation

- [Detailed datasheet (register map, pinout, how to test)](docs/info.md)

## Credits

CPU core: [tinyQV](https://github.com/MichaelBell/tinyQV) by Michael Bell, originally designed for TT10.
