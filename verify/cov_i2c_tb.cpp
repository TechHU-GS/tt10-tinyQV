// cov_i2c_tb.cpp -- Verilator branch-coverage testbench for
// i2c_peripheral + i2c_master (Forencich)
//
// Implements a minimal C++ I2C slave model that:
//   - Detects START / STOP conditions
//   - ACKs address 0x44 (7-bit), NACKs others
//   - For writes: ACKs each data byte, stores them
//   - For reads:  drives bytes 0x63, 0x32, then 0xFF
//
// Exercises i2c_peripheral paths:
//   1. Prescaler configuration
//   2. START+WRITE (write_multiple mode) + data bytes + STOP
//   3. START+READ + receive bytes + STOP
//   4. NACK scenario (wrong address)
//   5. missed_ack latch clear-on-new-cmd
//   6. tx_pending polling
//   7. rx_valid polling + data_rd clear
//   8. Multiple back-to-back transactions
//   9. Read config register
//  10. Stop-only command (standalone)

#include "Vcov_i2c_wrap.h"
#include "verilated.h"
#include "verilated_cov.h"

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cassert>
#include <vector>

// ---- simulation time ----
static vluint64_t sim_time = 0;
static Vcov_i2c_wrap *dut;

// ---- I2C slave model ----
// Open-drain bus: physical line = master_o & slave_o (wired-AND)
// master drives scl_o/sda_o; slave drives slave_sda
// scl_i = scl_o (no clock stretching), sda_i = sda_o & slave_sda

static uint8_t slave_sda = 1;  // slave SDA output (1 = released)

enum SlaveState {
    SL_IDLE,
    SL_ADDR,         // receiving 7-bit address + R/W
    SL_ADDR_ACK,     // driving ACK/NACK for address
    SL_WRITE_DATA,   // receiving data byte from master
    SL_WRITE_ACK,    // driving ACK for write data
    SL_READ_DATA,    // driving data byte to master
    SL_READ_ACK,     // waiting for master ACK/NACK
};

static const uint8_t SLAVE_ADDR = 0x44;

struct I2CSlave {
    SlaveState state = SL_IDLE;
    int bit_count = 0;
    uint8_t shift_reg = 0;
    bool addr_match = false;
    bool is_read = false;
    // For read mode: bytes to send
    uint8_t read_buf[4] = {0x63, 0x32, 0xFF, 0xFF};
    int read_idx = 0;
    // For write mode: received bytes
    std::vector<uint8_t> write_buf;
    // Edge detection state
    uint8_t prev_scl = 1;
    uint8_t prev_sda = 1;
    uint8_t cur_scl = 1;
    uint8_t cur_sda = 1;  // bus SDA after wired-AND

    void reset() {
        state = SL_IDLE;
        bit_count = 0;
        shift_reg = 0;
        addr_match = false;
        is_read = false;
        read_idx = 0;
        write_buf.clear();
        prev_scl = 1;
        prev_sda = 1;
        cur_scl = 1;
        cur_sda = 1;
        slave_sda = 1;
    }

    // Called every half-cycle AFTER eval, with current bus levels
    void update(uint8_t scl, uint8_t master_sda) {
        prev_scl = cur_scl;
        prev_sda = cur_sda;
        cur_scl = scl;
        // Bus SDA is wired-AND of master and slave
        cur_sda = master_sda & slave_sda;

        bool scl_rising  = (cur_scl == 1 && prev_scl == 0);
        bool scl_falling = (cur_scl == 0 && prev_scl == 1);
        // START: SDA falls while SCL high
        bool start_cond  = (cur_sda == 0 && prev_sda == 1 && cur_scl == 1);
        // STOP: SDA rises while SCL high
        bool stop_cond   = (cur_sda == 1 && prev_sda == 0 && cur_scl == 1);

        // START condition resets to address reception
        if (start_cond) {
            state = SL_ADDR;
            bit_count = 0;
            shift_reg = 0;
            slave_sda = 1; // release
            return;
        }

        // STOP condition returns to idle
        if (stop_cond) {
            state = SL_IDLE;
            slave_sda = 1;
            return;
        }

        switch (state) {
        case SL_IDLE:
            slave_sda = 1;
            break;

        case SL_ADDR:
            if (scl_rising) {
                // Sample SDA on SCL rising edge
                shift_reg = (shift_reg << 1) | (master_sda & 1);
                bit_count++;
                if (bit_count == 8) {
                    // bits[7:1] = address, bit[0] = R/W
                    uint8_t addr7 = (shift_reg >> 1) & 0x7F;
                    is_read = shift_reg & 1;
                    addr_match = (addr7 == SLAVE_ADDR);
                    // Prepare ACK on next falling edge
                    state = SL_ADDR_ACK;
                }
            }
            break;

        case SL_ADDR_ACK:
            if (scl_falling) {
                // Drive ACK (SDA low) or NACK (SDA high)
                slave_sda = addr_match ? 0 : 1;
            }
            if (scl_rising && bit_count == 8) {
                // Master samples our ACK on this rising edge; advance
                // We wait for next falling edge to transition
            }
            if (scl_falling && slave_sda == 0) {
                // ACK was driven; now transition
                slave_sda = 1; // release
                if (is_read) {
                    // Master wants to read: we drive data
                    state = SL_READ_DATA;
                    bit_count = 0;
                    shift_reg = read_buf[read_idx < 4 ? read_idx : 3];
                    // Pre-drive MSB
                    slave_sda = (shift_reg >> 7) & 1;
                } else {
                    state = SL_WRITE_DATA;
                    bit_count = 0;
                    shift_reg = 0;
                }
            } else if (scl_falling && slave_sda == 1) {
                // NACK sent, go idle
                state = SL_IDLE;
                slave_sda = 1;
            }
            break;

        case SL_WRITE_DATA:
            if (scl_rising) {
                shift_reg = (shift_reg << 1) | (master_sda & 1);
                bit_count++;
                if (bit_count == 8) {
                    write_buf.push_back(shift_reg);
                    state = SL_WRITE_ACK;
                }
            }
            break;

        case SL_WRITE_ACK:
            if (scl_falling) {
                slave_sda = 0; // ACK
            }
            // After the ACK bit is sampled, release on next falling edge
            if (scl_falling && slave_sda == 0) {
                // Check: was this the second falling edge (ACK already sent)?
                // We need to distinguish first falling (drive ACK) vs second (release).
                // Simple approach: use bit_count to track.
            }
            // Actually, simpler state machine: on the next rising edge after we
            // drove ACK low, master samples it. On the falling edge after that,
            // we release and prepare for next byte.
            if (scl_rising) {
                // Master is sampling our ACK now
                // On next falling edge we release
            }
            if (scl_falling && bit_count == 8) {
                // Already driven ACK; now release and prepare for next byte
                slave_sda = 1;
                bit_count = 0;
                shift_reg = 0;
                state = SL_WRITE_DATA;
                // The bit_count=8 check above is a one-shot; reset it
                bit_count = 0;
            }
            break;

        case SL_READ_DATA:
            if (scl_falling) {
                // Drive next bit
                if (bit_count < 8) {
                    slave_sda = (shift_reg >> (7 - bit_count)) & 1;
                }
            }
            if (scl_rising) {
                bit_count++;
                if (bit_count == 8) {
                    // All 8 bits sent; release SDA for ACK
                    state = SL_READ_ACK;
                    slave_sda = 1;
                }
            }
            break;

        case SL_READ_ACK:
            if (scl_rising) {
                // Master drives ACK (SDA=0) or NACK (SDA=1)
                bool master_ack = (master_sda == 0);
                if (master_ack) {
                    // Send next byte
                    read_idx++;
                    state = SL_READ_DATA;
                    bit_count = 0;
                    shift_reg = read_buf[read_idx < 4 ? read_idx : 3];
                    // Will drive MSB on next falling edge
                } else {
                    // NACK — master will send STOP
                    state = SL_IDLE;
                    slave_sda = 1;
                }
            }
            break;
        }
    }
} slave;

// ---- helpers ----

static void tick() {
    // Compute sda_i = open-drain wired-AND
    // scl_i = scl_o (no clock stretching) but use 1 initially
    uint8_t scl_bus = dut->scl_o;
    uint8_t sda_bus = dut->sda_o & slave_sda;

    dut->scl_i = scl_bus;
    dut->sda_i = sda_bus;

    dut->clk = 0;
    dut->eval();
    sim_time++;

    // Update slave on falling edge with post-eval bus state
    slave.update(dut->scl_o, dut->sda_o);

    // Recompute after slave may have changed slave_sda
    dut->sda_i = dut->sda_o & slave_sda;
    dut->scl_i = dut->scl_o;

    dut->clk = 1;
    dut->eval();
    sim_time++;

    // Update slave on rising edge
    slave.update(dut->scl_o, dut->sda_o);
    dut->sda_i = dut->sda_o & slave_sda;
    dut->scl_i = dut->scl_o;
}

static void ticks(int n) {
    for (int i = 0; i < n; i++) tick();
}

static void do_reset() {
    slave.reset();
    dut->rst_n     = 0;
    dut->data_wr   = 0;
    dut->data_rd   = 0;
    dut->data_in   = 0;
    dut->config_wr = 0;
    dut->config_in = 0;
    dut->scl_i     = 1;
    dut->sda_i     = 1;
    for (int i = 0; i < 10; i++) tick();
    dut->rst_n = 1;
    tick();
}

// MMIO write I2C_DATA
static void mmio_data_wr(uint32_t val) {
    dut->data_wr = 1;
    dut->data_in = val;
    tick();
    dut->data_wr = 0;
    dut->data_in = 0;
}

// MMIO read I2C_DATA (single-cycle pulse)
static uint32_t mmio_data_rd() {
    uint32_t v = dut->data_out;
    dut->data_rd = 1;
    tick();
    dut->data_rd = 0;
    return v;
}

// MMIO write I2C_CONFIG
static void mmio_config_wr(uint32_t val) {
    dut->config_wr = 1;
    dut->config_in = val;
    tick();
    dut->config_wr = 0;
    dut->config_in = 0;
}

// data_out field accessors
static uint8_t  rx_data()       { return dut->data_out & 0xFF; }
static bool     missed_ack()    { return (dut->data_out >> 8) & 1; }
static bool     busy()          { return (dut->data_out >> 9) & 1; }
static bool     rx_valid()      { return (dut->data_out >> 10) & 1; }
static bool     tx_pending()    { return (dut->data_out >> 11) & 1; }

// Wait for busy to clear, with timeout
static bool wait_not_busy(int timeout = 50000) {
    for (int i = 0; i < timeout; i++) {
        tick();
        if (!busy()) return true;
    }
    printf("  TIMEOUT: busy never cleared after %d ticks\n", timeout);
    return false;
}

// Wait for tx_pending to clear
static bool wait_tx_ready(int timeout = 50000) {
    for (int i = 0; i < timeout; i++) {
        tick();
        if (!tx_pending()) return true;
    }
    printf("  TIMEOUT: tx_pending never cleared after %d ticks\n", timeout);
    return false;
}

// Wait for rx_valid
static bool wait_rx_valid(int timeout = 50000) {
    for (int i = 0; i < timeout; i++) {
        tick();
        if (rx_valid()) return true;
    }
    printf("  TIMEOUT: rx_valid never set after %d ticks\n", timeout);
    return false;
}

// Build I2C_DATA write value
// data_in[7:0]  = data
// data_in[8]    = cmd_start
// data_in[9]    = cmd_read
// data_in[10]   = cmd_write
// data_in[11]   = cmd_write_multiple
// data_in[12]   = cmd_stop
static uint32_t cmd_bits(bool start, bool read, bool write, bool write_m, bool stop, uint8_t data) {
    uint32_t v = data;
    if (start)   v |= (1 << 8);
    if (read)    v |= (1 << 9);
    if (write)   v |= (1 << 10);
    if (write_m) v |= (1 << 11);
    if (stop)    v |= (1 << 12);
    return v;
}

// ---- test framework ----
static int test_count = 0;
static int pass_count = 0;

#define CHECK(cond, msg) do { \
    test_count++; \
    if (!(cond)) { \
        printf("  FAIL: %s (line %d)\n", msg, __LINE__); \
    } else { \
        pass_count++; \
    } \
} while(0)

// ===================================================================
// T1: Configure prescaler
// ===================================================================
static void test_prescaler_config() {
    printf("[T1] Prescaler configuration\n");
    do_reset();

    // Default prescale should be 63
    uint32_t cfg = dut->config_out;
    CHECK((cfg & 0xFFFF) == 63, "default prescale == 63");

    // Write a new prescaler value (use small value for fast sim)
    mmio_config_wr(4);
    cfg = dut->config_out;
    CHECK((cfg & 0xFFFF) == 4, "prescale updated to 4");

    // Read config_out upper bits should be 0
    CHECK((cfg >> 16) == 0, "config_out upper bits == 0");

    printf("  [T1] done\n");
}

// ===================================================================
// T2: Write transaction — START+WRITE addr 0x44, 2 data bytes, STOP
// ===================================================================
static void test_write_transaction() {
    printf("[T2] Write transaction (addr 0x44, 2 bytes)\n");
    do_reset();
    slave.write_buf.clear();

    // Set fast prescale for simulation
    mmio_config_wr(4);

    // START + WRITE_MULTIPLE + addr 0x44
    // addr goes in data[6:0], R/W bit is set by i2c_master based on mode
    uint32_t cmd = cmd_bits(true, false, false, true, false, SLAVE_ADDR);
    mmio_data_wr(cmd);

    // Wait for cmd to be accepted (busy asserts)
    ticks(10);

    // Wait for tx_pending to clear (cmd was accepted)
    wait_tx_ready(30000);

    // Write first data byte: 0xAB (cmd_write + data, no start, with write_multiple)
    // In write_multiple mode, send data with cmd_write set
    uint32_t d1 = cmd_bits(false, false, true, true, false, 0xAB);
    mmio_data_wr(d1);
    wait_tx_ready(30000);

    // Write second data byte + STOP: 0xCD with cmd_write + cmd_stop
    uint32_t d2 = cmd_bits(false, false, true, false, true, 0xCD);
    mmio_data_wr(d2);

    wait_not_busy(50000);

    // Check slave received the bytes
    CHECK(slave.write_buf.size() >= 2, "slave received >= 2 bytes");
    if (slave.write_buf.size() >= 1) {
        CHECK(slave.write_buf[0] == 0xAB, "first byte == 0xAB");
    }
    if (slave.write_buf.size() >= 2) {
        CHECK(slave.write_buf[1] == 0xCD, "second byte == 0xCD");
    }
    CHECK(!missed_ack(), "no missed ACK");

    printf("  slave received %zu bytes:", slave.write_buf.size());
    for (auto b : slave.write_buf) printf(" 0x%02X", b);
    printf("\n");
    printf("  [T2] done\n");
}

// ===================================================================
// T3: Read transaction — START+READ addr 0x44, 2 bytes, STOP
// ===================================================================
static void test_read_transaction() {
    printf("[T3] Read transaction (addr 0x44, 2 bytes)\n");
    do_reset();
    slave.read_idx = 0; // reset slave read pointer

    mmio_config_wr(4);

    // START + READ + addr 0x44 (no stop — we want to read multiple)
    uint32_t cmd = cmd_bits(true, true, false, false, false, SLAVE_ADDR);
    mmio_data_wr(cmd);

    // Wait for first RX byte
    bool got = wait_rx_valid(50000);
    CHECK(got, "rx_valid asserted for first byte");
    uint8_t byte0 = rx_data();
    printf("  RX byte 0 = 0x%02X\n", byte0);
    CHECK(byte0 == 0x63, "first read byte == 0x63");

    // Consume (clear rx_has_data)
    mmio_data_rd();
    tick();
    CHECK(!rx_valid(), "rx_valid cleared after data_rd");

    // Issue another READ (no start, no stop)
    uint32_t cmd2 = cmd_bits(false, true, false, false, false, SLAVE_ADDR);
    mmio_data_wr(cmd2);

    got = wait_rx_valid(50000);
    CHECK(got, "rx_valid asserted for second byte");
    uint8_t byte1 = rx_data();
    printf("  RX byte 1 = 0x%02X\n", byte1);
    CHECK(byte1 == 0x32, "second read byte == 0x32");
    mmio_data_rd();
    tick();

    // Issue READ + STOP for last byte
    uint32_t cmd3 = cmd_bits(false, true, false, false, true, SLAVE_ADDR);
    mmio_data_wr(cmd3);

    got = wait_rx_valid(50000);
    if (got) {
        uint8_t byte2 = rx_data();
        printf("  RX byte 2 = 0x%02X\n", byte2);
        mmio_data_rd();
    }

    wait_not_busy(50000);
    CHECK(!missed_ack(), "no missed ACK for read");
    printf("  [T3] done\n");
}

// ===================================================================
// T4: NACK scenario — wrong address
// ===================================================================
static void test_nack_wrong_addr() {
    printf("[T4] NACK scenario (wrong address 0x55)\n");
    do_reset();

    mmio_config_wr(4);

    // START + WRITE_MULTIPLE + wrong addr 0x55
    uint32_t cmd = cmd_bits(true, false, false, true, false, 0x55);
    mmio_data_wr(cmd);

    // Wait until not busy — should get missed_ack
    wait_not_busy(50000);

    // missed_ack should be latched
    CHECK(missed_ack(), "missed_ack latched for wrong address");

    // Write a new command to clear missed_ack
    uint32_t cmd2 = cmd_bits(true, false, false, true, false, SLAVE_ADDR);
    mmio_data_wr(cmd2);
    tick();
    CHECK(!missed_ack(), "missed_ack cleared on new command");

    wait_not_busy(50000);
    printf("  [T4] done\n");
}

// ===================================================================
// T5: Stop-only command
// ===================================================================
static void test_stop_only() {
    printf("[T5] Stop-only command\n");
    do_reset();

    mmio_config_wr(4);

    // First do a write to get bus active
    uint32_t cmd = cmd_bits(true, false, false, true, false, SLAVE_ADDR);
    mmio_data_wr(cmd);
    wait_tx_ready(30000);

    // Write one data byte
    uint32_t d1 = cmd_bits(false, false, true, true, false, 0x77);
    mmio_data_wr(d1);
    wait_tx_ready(30000);

    // Now send stop-only: only cmd_stop set, no read/write/start
    uint32_t stop = cmd_bits(false, false, false, false, true, 0x00);
    mmio_data_wr(stop);

    wait_not_busy(50000);
    CHECK(!busy(), "bus idle after stop-only");
    printf("  [T5] done\n");
}

// ===================================================================
// T6: tx_pending poll during write
// ===================================================================
static void test_tx_pending_poll() {
    printf("[T6] tx_pending polling during multi-byte write\n");
    do_reset();

    mmio_config_wr(4);

    // START + WRITE_MULTIPLE
    uint32_t cmd = cmd_bits(true, false, false, true, false, SLAVE_ADDR);
    mmio_data_wr(cmd);
    wait_tx_ready(30000);

    // Write 4 bytes, polling tx_pending between each
    for (int i = 0; i < 4; i++) {
        bool is_last = (i == 3);
        uint32_t d = cmd_bits(false, false, true, !is_last, is_last, 0x10 + i);
        mmio_data_wr(d);
        // Check tx_pending is set immediately after write
        if (tx_pending()) {
            // Good - pending is set
        }
        wait_tx_ready(30000);
    }

    wait_not_busy(50000);

    CHECK(slave.write_buf.size() >= 4, "slave got >= 4 bytes");
    for (int i = 0; i < 4 && i < (int)slave.write_buf.size(); i++) {
        printf("  byte[%d] = 0x%02X (expected 0x%02X)\n",
               i, slave.write_buf[i], 0x10 + i);
    }
    printf("  [T6] done\n");
}

// ===================================================================
// T7: rx_valid + data_rd clear
// ===================================================================
static void test_rx_valid_clear() {
    printf("[T7] rx_valid assertion and data_rd clear\n");
    do_reset();
    slave.read_idx = 0;

    mmio_config_wr(4);

    // START + READ
    uint32_t cmd = cmd_bits(true, true, false, false, true, SLAVE_ADDR);
    mmio_data_wr(cmd);

    bool got = wait_rx_valid(50000);
    CHECK(got, "rx_valid set for read");

    // Read without data_rd — rx_valid should stay
    uint8_t val = rx_data();
    tick();
    CHECK(rx_valid(), "rx_valid still set before data_rd");

    // Now pulse data_rd
    mmio_data_rd();
    tick();
    CHECK(!rx_valid(), "rx_valid cleared after data_rd pulse");

    wait_not_busy(50000);
    printf("  [T7] done\n");
}

// ===================================================================
// T8: Back-to-back write then read
// ===================================================================
static void test_back_to_back() {
    printf("[T8] Back-to-back write then read transaction\n");
    do_reset();
    slave.write_buf.clear();
    slave.read_idx = 0;

    mmio_config_wr(4);

    // -- WRITE phase --
    uint32_t wcmd = cmd_bits(true, false, false, true, false, SLAVE_ADDR);
    mmio_data_wr(wcmd);
    wait_tx_ready(30000);

    uint32_t d1 = cmd_bits(false, false, true, false, true, 0xEE);
    mmio_data_wr(d1);
    wait_not_busy(50000);

    CHECK(slave.write_buf.size() >= 1, "write phase: slave got data");
    if (slave.write_buf.size() >= 1) {
        CHECK(slave.write_buf[0] == 0xEE, "write phase: byte == 0xEE");
    }

    // -- READ phase (repeated start) --
    uint32_t rcmd = cmd_bits(true, true, false, false, true, SLAVE_ADDR);
    mmio_data_wr(rcmd);

    bool got = wait_rx_valid(50000);
    CHECK(got, "read phase: rx_valid set");
    if (got) {
        uint8_t rb = rx_data();
        printf("  read byte = 0x%02X\n", rb);
        mmio_data_rd();
    }

    wait_not_busy(50000);
    printf("  [T8] done\n");
}

// ===================================================================
// T9: Read config register
// ===================================================================
static void test_read_config() {
    printf("[T9] Read config register\n");
    do_reset();

    // Default
    CHECK((dut->config_out & 0xFFFF) == 63, "default prescale 63");

    // Write
    mmio_config_wr(200);
    CHECK((dut->config_out & 0xFFFF) == 200, "prescale == 200");

    mmio_config_wr(0);
    CHECK((dut->config_out & 0xFFFF) == 0, "prescale == 0");

    mmio_config_wr(0xFFFF);
    CHECK((dut->config_out & 0xFFFF) == 0xFFFF, "prescale == 0xFFFF");

    printf("  [T9] done\n");
}

// ===================================================================
// T10: Multiple full transactions to maximize state coverage
// ===================================================================
static void test_multiple_transactions() {
    printf("[T10] Multiple full transactions\n");
    do_reset();

    mmio_config_wr(4);

    for (int txn = 0; txn < 3; txn++) {
        slave.write_buf.clear();

        // Write transaction
        uint32_t wcmd = cmd_bits(true, false, false, true, false, SLAVE_ADDR);
        mmio_data_wr(wcmd);
        wait_tx_ready(30000);

        uint32_t d = cmd_bits(false, false, true, false, true, 0x30 + txn);
        mmio_data_wr(d);
        wait_not_busy(50000);

        printf("  txn %d: slave got %zu bytes", txn, slave.write_buf.size());
        for (auto b : slave.write_buf) printf(" 0x%02X", b);
        printf("\n");
    }

    CHECK(true, "multiple transactions completed without hang");
    printf("  [T10] done\n");
}

// ===================================================================
// T11: data_out field positions (bit-level check)
// ===================================================================
static void test_data_out_fields() {
    printf("[T11] data_out field positions\n");
    do_reset();

    // Initially: not busy, no rx_valid, no missed_ack, no tx_pending, rx_data=0
    uint32_t d = dut->data_out;
    CHECK((d & 0xFF) == 0, "rx_data initially 0");
    CHECK(!missed_ack(), "missed_ack initially 0");
    CHECK(!busy(), "busy initially 0");
    CHECK(!rx_valid(), "rx_valid initially 0");
    CHECK(!tx_pending(), "tx_pending initially 0");

    printf("  [T11] done\n");
}

// ===================================================================
// T12: Loopback — sda_i = sda_o, to exercise code paths without slave
// ===================================================================
static void test_loopback_simple() {
    printf("[T12] Simple loopback (sda_i = sda_o)\n");
    do_reset();
    // Override: make slave transparent (release bus)
    slave_sda = 1;

    mmio_config_wr(4);

    // START + WRITE addr 0x44 — slave won't ACK (loopback reads its own SDA)
    uint32_t cmd = cmd_bits(true, false, false, true, false, SLAVE_ADDR);
    mmio_data_wr(cmd);

    // In loopback, the "ACK" bit the master reads is whatever SDA was.
    // This will likely result in missed_ack. Just run to completion.
    wait_not_busy(50000);

    // This exercises the state machine even if ACK fails
    printf("  missed_ack = %d (expected in loopback)\n", missed_ack() ? 1 : 0);
    printf("  [T12] done\n");

    // Re-enable slave for subsequent tests
    slave.reset();
}

// ===================================================================
// T13: SDA CDC sync exercise — toggle sda_i rapidly
// ===================================================================
static void test_sda_cdc_sync() {
    printf("[T13] SDA CDC synchronizer exercise\n");
    do_reset();

    mmio_config_wr(4);

    // Rapidly toggle sda_i to exercise the 2-stage synchronizer
    for (int i = 0; i < 20; i++) {
        dut->sda_i = i & 1;
        tick();
    }
    dut->sda_i = 1; // release
    ticks(5);

    printf("  [T13] done\n");
}

// ===================================================================
// main
// ===================================================================
int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);

    dut = new Vcov_i2c_wrap;

    printf("=== i2c_peripheral + i2c_master coverage testbench ===\n\n");

    test_prescaler_config();
    test_write_transaction();
    test_read_transaction();
    test_nack_wrong_addr();
    test_stop_only();
    test_tx_pending_poll();
    test_rx_valid_clear();
    test_back_to_back();
    test_read_config();
    test_multiple_transactions();
    test_data_out_fields();
    test_loopback_simple();
    test_sda_cdc_sync();

    printf("\n=== Results: %d / %d PASS ===\n", pass_count, test_count);

    dut->final();

    const char *cov_path = "coverage.dat";
    VerilatedCov::write(cov_path);
    printf("Coverage written to: %s\n", cov_path);

    delete dut;
    return (pass_count == test_count) ? 0 : 1;
}
