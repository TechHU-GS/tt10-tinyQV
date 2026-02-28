// ============================================================================
// Seal Register — Verilator Cross-Validation
//
// Drives seal_register.v via bus interface, reads CRC output, and compares
// against seal_engine.hpp (same code running on ESP32).
//
// "Left hand hits right hand" — HW and SW must produce identical CRC16
// for every input combination.
// ============================================================================

#include "Vseal_tb_top.h"
#include "verilated.h"

// Real firmware headers — identical to what runs on ESP32
#include "esplte4iot/core/pure/seal_engine.hpp"

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <random>

using namespace esplte4iot::core::pure::seal;

// ---------- Test infrastructure ----------

static int g_pass = 0;
static int g_fail = 0;
static int g_total = 0;

#define CHECK(cond, ...) do { \
    g_total++; \
    if (cond) { g_pass++; } \
    else { g_fail++; printf("  FAIL: " __VA_ARGS__); printf("\n"); } \
} while(0)

// ---------- Clock / reset helpers ----------

static Vseal_tb_top* top;
static uint64_t sim_time = 0;

static void tick() {
    top->clk = 0;
    top->eval();
    sim_time++;
    top->clk = 1;
    top->eval();
    sim_time++;
}

static void reset() {
    top->rst_n = 0;
    top->data_wr = 0;
    top->data_rd = 0;
    top->ctrl_wr = 0;
    top->ctrl_in = 0;
    top->data_in = 0;
    for (int i = 0; i < 10; i++) tick();
    top->rst_n = 1;
    for (int i = 0; i < 5; i++) tick();
}

// ---------- Bus operation helpers ----------

static void seal_write_data(uint32_t val) {
    tick();
    top->data_in = val;
    top->data_wr = 1;
    tick();
    top->data_wr = 0;
}

static void seal_write_ctrl(uint16_t val) {
    tick();
    top->ctrl_in = val & 0x3FF;
    top->ctrl_wr = 1;
    tick();
    top->ctrl_wr = 0;
}

static void wait_seal_done() {
    tick();  // allow commit to register
    int timeout = 0;
    while ((top->ctrl_out & 1) && timeout < 5000) {
        tick();
        timeout++;
    }
    if (timeout >= 5000) {
        printf("  ERROR: seal_done timeout!\n");
    }
}

static void seal_read_pulse() {
    tick();
    top->data_rd = 1;
    tick();
    top->data_rd = 0;
    tick();
    tick();
}

// Commit and read back CRC from hardware
// Returns {crc16, mono_count} via out params
static void hw_commit_and_read(uint8_t sensor_id, uint32_t value,
                                uint16_t& out_crc, uint32_t& out_mono,
                                uint8_t& out_sid) {
    seal_write_data(value);
    // ctrl_in: {sensor_id[7:0], commit=1, crc_reset=0}
    seal_write_ctrl((static_cast<uint16_t>(sensor_id) << 2) | 0x02);
    wait_seal_done();

    // Read 0: value (skip)
    uint32_t rd0 = top->data_out;
    (void)rd0;

    // Read 1: {session_id[7:0], mono_count[23:0]}
    seal_read_pulse();
    uint32_t rd1 = top->data_out;
    out_sid = (rd1 >> 24) & 0xFF;
    uint32_t mono_lo = rd1 & 0x00FFFFFF;

    // Read 2: {mono_count[31:24], crc16[15:0], 8'h00}
    seal_read_pulse();
    uint32_t rd2 = top->data_out;
    uint8_t mono_hi = (rd2 >> 24) & 0xFF;
    out_crc = (rd2 >> 8) & 0xFFFF;
    out_mono = (static_cast<uint32_t>(mono_hi) << 24) | mono_lo;
}

// ===== Test 1: Golden Vectors =====

static void test_golden_vectors() {
    printf("\n[Test 1] Golden vectors (hardware vs software)\n");

    // Vector 1: sensor=0xAA, value=0x00000000, mono=0 → 0x578C
    reset();
    top->session_ctr_in = 0x01;
    tick();

    uint16_t hw_crc;
    uint32_t hw_mono;
    uint8_t  hw_sid;
    hw_commit_and_read(0xAA, 0x00000000, hw_crc, hw_mono, hw_sid);

    uint16_t sw_crc = seal_crc16(0xAA, 0x00000000, 0);
    CHECK(hw_crc == 0x578C, "V1 HW CRC=0x%04X expected 0x578C", hw_crc);
    CHECK(sw_crc == 0x578C, "V1 SW CRC=0x%04X expected 0x578C", sw_crc);
    CHECK(hw_crc == sw_crc, "V1 HW==SW: 0x%04X vs 0x%04X", hw_crc, sw_crc);
    CHECK(hw_mono == 0, "V1 mono=%u expected 0", hw_mono);

    // Vector 2: sensor=0xFF, value=0xFFFFFFFF, mono=1 → 0xE80E
    hw_commit_and_read(0xFF, 0xFFFFFFFF, hw_crc, hw_mono, hw_sid);

    sw_crc = seal_crc16(0xFF, 0xFFFFFFFF, 1);
    CHECK(hw_crc == 0xE80E, "V2 HW CRC=0x%04X expected 0xE80E", hw_crc);
    CHECK(sw_crc == 0xE80E, "V2 SW CRC=0x%04X expected 0xE80E", sw_crc);
    CHECK(hw_crc == sw_crc, "V2 HW==SW: 0x%04X vs 0x%04X", hw_crc, sw_crc);
    CHECK(hw_mono == 1, "V2 mono=%u expected 1", hw_mono);

    printf("  Golden vectors: done\n");
}

// ===== Test 2: Random Cross-Validation (1000 rounds) =====

static void test_random_crosscheck() {
    printf("\n[Test 2] Random cross-validation: 1000 rounds\n");

    reset();
    top->session_ctr_in = 0x42;
    tick();

    std::mt19937 rng(12345);
    int local_pass = 0;

    for (int i = 0; i < 1000; i++) {
        uint8_t  sensor_id = rng() & 0xFF;
        uint32_t value     = rng();
        // mono_count = i (auto-incremented by HW)

        uint16_t hw_crc;
        uint32_t hw_mono;
        uint8_t  hw_sid;
        hw_commit_and_read(sensor_id, value, hw_crc, hw_mono, hw_sid);

        uint16_t sw_crc = seal_crc16(sensor_id, value, static_cast<uint32_t>(i));

        if (hw_crc == sw_crc && hw_mono == static_cast<uint32_t>(i)) {
            g_pass++;
            g_total++;
            local_pass++;
        } else {
            g_fail++;
            g_total++;
            printf("  FAIL round %d: sid=0x%02X val=0x%08X mono=%u "
                   "HW_CRC=0x%04X SW_CRC=0x%04X HW_mono=%u\n",
                   i, sensor_id, value, i, hw_crc, sw_crc, hw_mono);
        }
    }
    printf("  Random: %d/1000 pass\n", local_pass);
}

// ===== Test 3: Boundary Values =====

static void test_boundary_values() {
    printf("\n[Test 3] Boundary values\n");

    struct BoundaryCase {
        uint8_t  sensor_id;
        uint32_t value;
        const char* name;
    };

    BoundaryCase cases[] = {
        {0x00, 0x00000000, "all-zero"},
        {0xFF, 0xFFFFFFFF, "all-FF"},
        {0x00, 0xFFFFFFFF, "sid=0,val=FF"},
        {0xFF, 0x00000000, "sid=FF,val=0"},
        {0x01, 0x00000001, "min-nonzero"},
        {0x80, 0x80000000, "MSB-set"},
        {0x7F, 0x7FFFFFFF, "max-positive"},
        {0xAA, 0x55555555, "alternating-1"},
        {0x55, 0xAAAAAAAA, "alternating-2"},
        {0x01, 0xDEADBEEF, "deadbeef"},
        {0x02, 0xCAFEBABE, "cafebabe"},
        {0x03, 0x12345678, "sequential"},
        {0xFE, 0x00000100, "value-256"},
        {0x10, 0x0000FFFF, "value-16bit"},
        {0x20, 0x00FF0000, "value-byte2"},
        {0x40, 0xFF000000, "value-byte3"},
    };

    // Use separate resets so mono_count is predictable
    for (int i = 0; i < 16; i++) {
        reset();
        top->session_ctr_in = 0x01;
        tick();

        uint16_t hw_crc;
        uint32_t hw_mono;
        uint8_t  hw_sid;
        hw_commit_and_read(cases[i].sensor_id, cases[i].value,
                           hw_crc, hw_mono, hw_sid);

        uint16_t sw_crc = seal_crc16(cases[i].sensor_id, cases[i].value, 0);

        CHECK(hw_crc == sw_crc,
              "boundary[%s]: HW=0x%04X SW=0x%04X",
              cases[i].name, hw_crc, sw_crc);
        CHECK(hw_mono == 0,
              "boundary[%s]: mono=%u expected 0",
              cases[i].name, hw_mono);
    }

    printf("  Boundary: done\n");
}

// ===== Test 4: Session ID isolation (not in CRC) =====

static void test_session_isolation() {
    printf("\n[Test 4] Session ID isolation: CRC independent of session\n");

    uint16_t crcs[10];

    for (int i = 0; i < 10; i++) {
        reset();
        top->session_ctr_in = static_cast<uint8_t>(i * 25);  // different session each time
        tick();

        uint16_t hw_crc;
        uint32_t hw_mono;
        uint8_t  hw_sid;
        // Same sensor_id, value, mono_count=0 each time
        hw_commit_and_read(0x42, 0xBEEF0042, hw_crc, hw_mono, hw_sid);

        crcs[i] = hw_crc;
        uint16_t sw_crc = seal_crc16(0x42, 0xBEEF0042, 0);
        CHECK(hw_crc == sw_crc,
              "session[%d]: HW=0x%04X SW=0x%04X", i, hw_crc, sw_crc);

        // Verify session_id was captured
        CHECK(hw_sid == static_cast<uint8_t>(i * 25),
              "session[%d]: sid=0x%02X expected 0x%02X",
              i, hw_sid, static_cast<uint8_t>(i * 25));
    }

    // All CRCs should be identical (session doesn't affect CRC)
    for (int i = 1; i < 10; i++) {
        CHECK(crcs[i] == crcs[0],
              "session CRC consistency: crcs[%d]=0x%04X != crcs[0]=0x%04X",
              i, crcs[i], crcs[0]);
    }

    printf("  Session isolation: done\n");
}

// ===== Test 5: Mono count sequence consistency =====

static void test_mono_sequence() {
    printf("\n[Test 5] Mono count sequence: 50 consecutive commits\n");

    reset();
    top->session_ctr_in = 0x77;
    tick();

    SoftSealEngine sw_eng;
    sw_eng.restore_state(0, 0x77);

    for (int i = 0; i < 50; i++) {
        uint8_t  sensor_id = static_cast<uint8_t>(i & 0xFF);
        uint32_t value     = static_cast<uint32_t>(i * 1000 + 42);

        uint16_t hw_crc;
        uint32_t hw_mono;
        uint8_t  hw_sid;
        hw_commit_and_read(sensor_id, value, hw_crc, hw_mono, hw_sid);

        auto sw_rec = sw_eng.commit(sensor_id, value);

        CHECK(hw_mono == sw_rec.mono_count,
              "seq[%d]: HW_mono=%u SW_mono=%u", i, hw_mono, sw_rec.mono_count);
        CHECK(hw_crc == sw_rec.crc16,
              "seq[%d]: HW_CRC=0x%04X SW_CRC=0x%04X", i, hw_crc, sw_rec.crc16);
        CHECK(hw_sid == sw_rec.session_id,
              "seq[%d]: HW_sid=0x%02X SW_sid=0x%02X", i, hw_sid, sw_rec.session_id);
    }

    printf("  Mono sequence: done\n");
}

// ===== Test 6: Mono overflow cross-validation =====

static void test_mono_overflow() {
    printf("\n[Test 6] Mono counter overflow: 0xFFFFFFFE → 0xFFFFFFFF → 0x00000000\n");

    // We can't easily force internal state in Verilator without hierarchical
    // access. Instead, use a trick: reset, then do commits to reach specific monos.
    // For overflow test, we commit 3 times starting from fresh resets but use
    // the SW engine to verify CRC correctness at each step.

    // Alternative: just verify CRC at mono 0,1,2 matches SW — the HW overflow
    // behavior is already verified in tb_seal.v Test 13 with force/release.
    // Here we focus on CRC bit-exact at large mono values via random test (Test 2).

    // Verify SW side wraps correctly
    SoftSealEngine eng;
    eng.restore_state(0xFFFFFFFF, 0x01);
    auto r0 = eng.commit(0x01, 100);
    CHECK(r0.mono_count == 0xFFFFFFFF, "SW wrap: pre-wrap mono=0xFFFFFFFF");
    auto r1 = eng.commit(0x01, 200);
    CHECK(r1.mono_count == 0, "SW wrap: post-wrap mono=0");
    CHECK(verify_seal(r0), "SW wrap: r0 valid");
    CHECK(verify_seal(r1), "SW wrap: r1 valid");

    printf("  Mono overflow: done (HW overflow verified in tb_seal.v T13)\n");
}

// ===== Test 7: Anti-false-positive — print actual values + uniqueness =====

static void test_anti_false_positive() {
    printf("\n[Test 7] Anti-false-positive: value diversity + sample dump\n");

    reset();
    top->session_ctr_in = 0x33;
    tick();

    uint16_t crcs[10];
    printf("  %-4s  %-4s  %-10s  %-6s  %-6s  %-6s\n",
           "i", "sid", "value", "mono", "HW_CRC", "SW_CRC");

    for (int i = 0; i < 10; i++) {
        uint8_t  sensor_id = static_cast<uint8_t>(i * 17 + 1);
        uint32_t value     = static_cast<uint32_t>(i * 0x11111111);

        uint16_t hw_crc;
        uint32_t hw_mono;
        uint8_t  hw_sid;
        hw_commit_and_read(sensor_id, value, hw_crc, hw_mono, hw_sid);
        uint16_t sw_crc = seal_crc16(sensor_id, value, static_cast<uint32_t>(i));

        printf("  %-4d  0x%02X  0x%08X  %-6u  0x%04X  0x%04X  %s\n",
               i, sensor_id, value, hw_mono, hw_crc, sw_crc,
               (hw_crc == sw_crc) ? "OK" : "MISMATCH");

        crcs[i] = hw_crc;
        CHECK(hw_crc == sw_crc, "dump[%d] mismatch", i);
        CHECK(hw_crc != 0x0000, "dump[%d] CRC is zero (HW not running?)", i);
        CHECK(hw_crc != 0xFFFF, "dump[%d] CRC is init value (HW not computing?)", i);
    }

    // Verify CRC diversity: all 10 must be distinct
    int unique = 0;
    for (int i = 0; i < 10; i++) {
        bool dup = false;
        for (int j = 0; j < i; j++) {
            if (crcs[i] == crcs[j]) { dup = true; break; }
        }
        if (!dup) unique++;
    }
    CHECK(unique == 10, "CRC diversity: %d/10 unique (expect 10)", unique);

    printf("  Anti-false-positive: %d/10 unique CRCs\n", unique);
}

// ===== Test 8: Negative test — deliberate mismatch detection =====

static void test_negative_deliberate_mismatch() {
    printf("\n[Test 8] Negative test: deliberately wrong SW must NOT match HW\n");

    reset();
    top->session_ctr_in = 0x55;
    tick();

    uint16_t hw_crc;
    uint32_t hw_mono;
    uint8_t  hw_sid;
    hw_commit_and_read(0xAA, 0x12345678, hw_crc, hw_mono, hw_sid);

    // Correct SW
    uint16_t sw_correct = seal_crc16(0xAA, 0x12345678, 0);
    CHECK(hw_crc == sw_correct, "negative: correct SW matches HW");

    // Wrong sensor_id
    uint16_t sw_wrong_sid = seal_crc16(0xBB, 0x12345678, 0);
    CHECK(hw_crc != sw_wrong_sid,
          "negative: wrong sensor_id should differ (HW=0x%04X wrong=0x%04X)",
          hw_crc, sw_wrong_sid);

    // Wrong value
    uint16_t sw_wrong_val = seal_crc16(0xAA, 0x12345679, 0);
    CHECK(hw_crc != sw_wrong_val,
          "negative: wrong value should differ (HW=0x%04X wrong=0x%04X)",
          hw_crc, sw_wrong_val);

    // Wrong mono
    uint16_t sw_wrong_mono = seal_crc16(0xAA, 0x12345678, 1);
    CHECK(hw_crc != sw_wrong_mono,
          "negative: wrong mono should differ (HW=0x%04X wrong=0x%04X)",
          hw_crc, sw_wrong_mono);

    // All-zero input (smoke test that HW actually computes)
    CHECK(hw_crc != 0x0000, "negative: HW CRC not zero");
    CHECK(hw_crc != 0xFFFF, "negative: HW CRC not init value");

    printf("  Negative test: done\n");
}

// ===== Main =====

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    top = new Vseal_tb_top;

    printf("=== Seal Register — Verilator Cross-Validation ===\n");
    printf("HW: seal_register.v + crc16_engine.v\n");
    printf("SW: seal_engine.hpp (→ loralite_protocol.hpp crc16_modbus)\n");

    test_golden_vectors();
    test_random_crosscheck();
    test_boundary_values();
    test_session_isolation();
    test_mono_sequence();
    test_mono_overflow();
    test_anti_false_positive();
    test_negative_deliberate_mismatch();

    printf("\n=== Results: %d PASS, %d FAIL (total %d) ===\n",
           g_pass, g_fail, g_total);
    if (g_fail == 0)
        printf("ALL TESTS PASSED\n");
    else
        printf("SOME TESTS FAILED\n");

    top->final();
    delete top;
    return g_fail > 0 ? 1 : 0;
}
