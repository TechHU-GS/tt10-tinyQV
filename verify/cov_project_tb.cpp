// cov_project_tb.cpp — Verilator coverage testbench for full LoRa Edge SoC
// Boots the POST firmware via QSPI flash model and monitors UART output.
// No waveform tracing — coverage data only.

#include "Vcov_project_wrap.h"
#include "verilated.h"
#include "verilated_cov.h"

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <memory>

static Vcov_project_wrap *dut;
static VerilatedContext *contextp;

// UART receiver state (115200 baud @ 25 MHz = 217 clocks/bit)
static const int UART_BIT_CLKS = 217;
static int uart_bit_cnt = -1;
static int uart_clk_cnt = 0;
static uint8_t uart_shift = 0;
static uint8_t uart_prev_txd = 1;
static uint8_t uart_buf[256];
static int uart_idx = 0;

static void uart_sample(uint8_t txd) {
    uint8_t start_edge = uart_prev_txd && !txd;
    uart_prev_txd = txd;

    if (uart_bit_cnt == -1) {
        if (start_edge) {
            uart_bit_cnt = 0;
            uart_clk_cnt = UART_BIT_CLKS + (UART_BIT_CLKS / 2);
        }
        return;
    }

    if (uart_clk_cnt > 0) {
        uart_clk_cnt--;
        return;
    }

    uart_clk_cnt = UART_BIT_CLKS;
    if (uart_bit_cnt < 8) {
        uart_shift = (uart_shift >> 1) | (txd << 7);
        uart_bit_cnt++;
    } else {
        // Stop bit — byte complete
        if (uart_idx < 256) {
            uart_buf[uart_idx] = uart_shift;
            if (uart_shift >= 0x20 && uart_shift < 0x7F)
                printf("[UART] byte %d: 0x%02X '%c'\n", uart_idx, uart_shift, uart_shift);
            else
                printf("[UART] byte %d: 0x%02X\n", uart_idx, uart_shift);
            uart_idx++;
        }
        uart_bit_cnt = -1;
    }
}

// One full clock cycle: fall then rise
// Advance simulation time to allow --timing edge detection on derived clocks
static void tick() {
    // Falling edge
    dut->clk = 0;
    contextp->timeInc(1);
    dut->eval();

    // Rising edge
    dut->clk = 1;
    contextp->timeInc(1);
    dut->eval();

    // Sample UART on uo_out[0] after rising edge
    uart_sample(dut->uo_out & 0x01);
}

int main(int argc, char **argv) {
    contextp = new VerilatedContext;
    contextp->commandArgs(argc, argv);

    dut = new Vcov_project_wrap{contextp};

    printf("=== LoRa Edge SoC — Verilator Branch Coverage ===\n");
    printf("Booting POST firmware...\n\n");

    // Reset
    dut->rst_n = 0;
    dut->clk = 0;
    for (int i = 0; i < 20; i++) tick();

    // Release reset
    dut->rst_n = 1;
    printf("Reset released. Running POST firmware...\n");

    // Run until we see 26 UART bytes (full POST output) or timeout
    // POST takes ~75M cycles at 25MHz. Verilator is fast enough.
    const uint64_t MAX_CYCLES = 80000000ULL;  // 80M cycles safety margin
    const int EXPECTED_CHARS = 26;

    // Early diagnostic: check DUT is generating SPI clock activity
    int spi_clk_transitions = 0;
    uint8_t prev_uio = 0;

    for (uint64_t cyc = 0; cyc < MAX_CYCLES; cyc++) {
        tick();

        // Track SPI clock (uio_out[3]) transitions in first 1000 cycles
        if (cyc < 1000) {
            uint8_t cur_uio = dut->uio_out;
            if (((cur_uio ^ prev_uio) & 0x08) != 0) spi_clk_transitions++;
            prev_uio = cur_uio;
        }
        if (cyc == 1000) {
            printf("  [diag] SPI clk transitions in first 1000 cycles: %d\n", spi_clk_transitions);
            printf("  [diag] uio_out=0x%02X uio_oe=0x%02X uo_out=0x%02X\n",
                   dut->uio_out, dut->uio_oe, dut->uo_out);
        }

        // Check for completion every 1M cycles to avoid overhead
        if ((cyc & 0xFFFFF) == 0 && uart_idx >= EXPECTED_CHARS) {
            printf("\nPOST complete after ~%lluM cycles.\n", (unsigned long long)(cyc / 1000000));
            break;
        }

        // Print progress every 10M cycles
        if ((cyc % 10000000) == 0 && cyc > 0) {
            printf("  ... %lluM cycles, %d UART bytes, uo_out=0x%02X\n",
                   (unsigned long long)(cyc / 1000000), uart_idx, dut->uo_out);
        }
    }

    printf("\n--- Received %d UART bytes ---\n", uart_idx);

    // Verify POST results
    int pass = 0, fail = 0;

    // Check banner "POST\n"
    if (uart_idx >= 5 &&
        uart_buf[0] == 'P' && uart_buf[1] == 'O' &&
        uart_buf[2] == 'S' && uart_buf[3] == 'T' &&
        uart_buf[4] == '\n') {
        printf("[PASS] Banner: POST\\n\n");
        pass++;
    } else {
        printf("[FAIL] Banner\n");
        fail++;
    }

    // Check 2-char pairs
    auto check2 = [&](int idx, char tag, char val, const char *name) {
        if (uart_idx > idx + 1 &&
            uart_buf[idx] == (uint8_t)tag && uart_buf[idx+1] == (uint8_t)val) {
            printf("[PASS] %s: %c%c\n", name, tag, val);
            pass++;
        } else {
            printf("[FAIL] %s\n", name);
            fail++;
        }
    };

    check2(5,  'Y', '1', "SYSINFO");
    check2(7,  'C', '1', "CRC16");
    check2(9,  'T', '1', "Timer");
    check2(11, 'W', '1', "WDT");
    check2(13, 'I', '1', "I2C");
    check2(15, 'L', '1', "Seal_1");
    check2(17, 'L', '2', "Seal_2");
    check2(19, 'M', '1', "PSRAM");
    check2(21, 'R', '1', "RTC");

    // Check "DN\n"
    if (uart_idx >= 26 &&
        uart_buf[23] == 'D' && uart_buf[24] == 'N' &&
        uart_buf[25] == '\n') {
        printf("[PASS] Completion: DN\\n\n");
        pass++;
    } else {
        printf("[FAIL] Completion\n");
        fail++;
    }

    printf("\n=== Results: %d PASS, %d FAIL ===\n", pass, fail);

    // Finalize and write coverage
    dut->final();
    VerilatedCov::write("coverage.dat");
    printf("Coverage written to: coverage.dat\n");

    delete dut;
    delete contextp;
    return (fail == 0) ? 0 : 1;
}
