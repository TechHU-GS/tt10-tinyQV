#!/bin/bash
# ============================================================================
# Mutation regression guard — bit-serial read_complete bug (#4 and #6)
# ============================================================================
# These two mutations reproduce the CRITICAL historical bug where read-side-
# effect signals used (read_n != 2'b11) instead of read_complete, causing
# TinyQV's 4-bit serial CPU to clear flags mid-read (8-cycle window).
#
# Each mutation: apply via sed → compile → run integration test → MUST FAIL
# (timeout or assertion failure both count as "detected").
# After all mutations, run lint smoke to confirm clean restore.
#
# #4 uses tb_i2c_stress (exercises I2C RX read path)
# #6 uses tb_integration_b (exercises Seal 3x read serialization)
#
# Other mutations (unit-test level, lower regression risk):
#   #1: seal_register.v  mono_count+1 → mono_count   (tb_seal detects)
#   #2: seal_register.v  read_seq increment removed   (tb_seal detects)
#   #3: watchdog.v       enabled<=1 → enabled<=0      (tb_watchdog detects)
#   #5: crc16_engine.v   0xA001 → 0xA000              (tb_crc16 detects)
# ============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
SRC="$ROOT/src/project.v"
TEST="$ROOT/test"

RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m'

PASS=0
FAIL=0

# Shared RTL source files (everything except the TB)
RTL_SRCS=(
    "$ROOT/src/project.v"
    "$ROOT/src/latch_mem.v"
    "$ROOT/src/crc16_engine.v"
    "$ROOT/src/crc16_peripheral.v"
    "$ROOT/src/seal_register.v"
    "$ROOT/src/i2c_master.v"
    "$ROOT/src/i2c_peripheral.v"
    "$ROOT/src/watchdog.v"
    "$ROOT/src/rtc_counter.v"
    "$ROOT/src/tinyQV/cpu/tinyqv.v"
    "$ROOT/src/tinyQV/cpu/alu.v"
    "$ROOT/src/tinyQV/cpu/core.v"
    "$ROOT/src/tinyQV/cpu/counter.v"
    "$ROOT/src/tinyQV/cpu/cpu.v"
    "$ROOT/src/tinyQV/cpu/decode.v"
    "$ROOT/src/tinyQV/cpu/mem_ctrl.v"
    "$ROOT/src/tinyQV/cpu/qspi_ctrl.v"
    "$ROOT/src/tinyQV/cpu/register.v"
    "$ROOT/src/tinyQV/cpu/latch_reg.v"
    "$ROOT/src/tinyQV/peri/uart/uart_tx.v"
    "$ROOT/src/tinyQV/peri/uart/uart_rx.v"
    "$ROOT/src/tinyQV/peri/spi/spi.v"
)

# Save original for restore
cp "$SRC" "$SRC.mutate_backup"

cleanup() {
    if [ -f "$SRC.mutate_backup" ]; then
        cp "$SRC.mutate_backup" "$SRC"
        rm -f "$SRC.mutate_backup"
    fi
    rm -f "$TEST/mutate_test.vvp" "$TEST/mutate_result.txt"
    rm -f "$SRC.bak"
}
trap cleanup EXIT

# run_mutation ID DESCRIPTION SED_CMD TB_FILES...
# TB_FILES are the testbench + model files (relative to $TEST)
run_mutation() {
    local id="$1"; shift
    local desc="$1"; shift
    local sed_cmd="$1"; shift
    # Remaining args are TB-specific source files
    local tb_files=("$@")

    echo ""
    echo "========================================"
    echo "Mutation #$id: $desc"
    echo "========================================"

    # Apply mutation
    cp "$SRC.mutate_backup" "$SRC"
    eval "$sed_cmd"

    # Verify mutation was applied
    if diff -q "$SRC" "$SRC.mutate_backup" >/dev/null 2>&1; then
        echo -e "${RED}#$id ERROR${NC} — sed did not change file"
        FAIL=$((FAIL + 1))
        return
    fi

    # Build full source list: TB files + RTL
    local all_srcs=()
    for f in "${tb_files[@]}"; do
        all_srcs+=("$TEST/$f")
    done
    all_srcs+=("${RTL_SRCS[@]}")

    # Compile
    cd "$TEST"
    if ! iverilog -g2012 -DSIM -o mutate_test.vvp \
        -I../src -I../src/tinyQV/cpu \
        -I../src/tinyQV/peri/pwm -I../src/tinyQV/peri/spi \
        -I../src/tinyQV/peri/ttgame -I../src/tinyQV/peri/uart \
        "${all_srcs[@]}" 2>/dev/null; then
        echo -e "${GREEN}#$id DETECTED${NC} (compile error)"
        PASS=$((PASS + 1))
        cd "$ROOT"
        return
    fi

    # Run with timeout — mutation should cause hang or failure
    local timeout_sec=120
    if timeout "$timeout_sec" vvp mutate_test.vvp > mutate_result.txt 2>&1; then
        # Exited cleanly — check if tests actually passed
        if grep -q "ALL TESTS PASSED" mutate_result.txt; then
            echo -e "${RED}#$id UNDETECTED${NC} — tests passed with mutation!"
            echo "--- Last 20 lines of output ---"
            tail -20 mutate_result.txt
            FAIL=$((FAIL + 1))
        else
            echo -e "${GREEN}#$id DETECTED${NC} (assertion failure)"
            echo "--- Last 10 lines ---"
            tail -10 mutate_result.txt
            PASS=$((PASS + 1))
        fi
    else
        # Timeout or non-zero exit — mutation detected
        echo -e "${GREEN}#$id DETECTED${NC} (timeout/crash — firmware hung)"
        echo "--- Last 10 lines before timeout ---"
        tail -10 mutate_result.txt 2>/dev/null || true
        PASS=$((PASS + 1))
    fi
    cd "$ROOT"
}

echo "================================================================"
echo "Mutation Regression Guard — bit-serial read_complete bug"
echo "================================================================"

# Mutation #4: i2c_data_rd: read_complete → (read_n != 2'b11)
# Detected by: tb_i2c_stress (firmware hangs in i2c_wait_rx)
run_mutation 4 \
    "i2c_data_rd: read_complete → (read_n != 2'b11)" \
    "sed -i.bak 's/wire        i2c_data_rd = (connect_peripheral == PERI_I2C_DATA) && read_complete;/wire        i2c_data_rd = (connect_peripheral == PERI_I2C_DATA) \&\& (read_n != 2'\''b11);/' \"$SRC\"" \
    tb_i2c_stress.v qspi_flash_model.v qspi_psram_model.v i2c_slave_model.v

# Mutation #6: seal_data_rd: read_complete → (read_n != 2'b11)
# Detected by: tb_integration_b (exercises Seal 3x read serialization)
run_mutation 6 \
    "seal_data_rd: read_complete → (read_n != 2'b11)" \
    "sed -i.bak 's/wire        seal_data_rd = (connect_peripheral == PERI_SEAL_DATA) && read_complete;/wire        seal_data_rd = (connect_peripheral == PERI_SEAL_DATA) \&\& (read_n != 2'\''b11);/' \"$SRC\"" \
    tb_integration_b.v qspi_flash_model.v qspi_psram_model.v i2c_slave_model.v

# Restore original
cp "$SRC.mutate_backup" "$SRC"

# Lint smoke — verify restore is clean
echo ""
echo "========================================"
echo "Lint smoke (verify clean restore)"
echo "========================================"
cd "$ROOT"
LINT_OUT=$(verilator --lint-only --timing -DSIM -Wall \
    src/lint.vlt \
    src/project.v src/latch_mem.v src/crc16_engine.v src/crc16_peripheral.v \
    src/seal_register.v src/watchdog.v src/rtc_counter.v \
    src/i2c_master.v src/i2c_peripheral.v \
    src/tinyQV/cpu/*.v src/tinyQV/peri/*/*.v 2>&1) || true

if echo "$LINT_OUT" | grep -q '%Warning'; then
    echo -e "${RED}LINT SMOKE FAILED${NC} — restore may be incomplete"
    echo "$LINT_OUT" | grep '%Warning'
    FAIL=$((FAIL + 1))
else
    echo -e "${GREEN}Lint clean${NC} — restore verified"
fi

# Summary
echo ""
echo "========================================"
echo "MUTATION SUMMARY: $PASS detected, $FAIL undetected"
echo "========================================"
if [ "$FAIL" -gt 0 ]; then
    echo -e "${RED}FAIL${NC} — some mutations went undetected"
    exit 1
else
    echo -e "${GREEN}ALL MUTATIONS DETECTED${NC}"
    exit 0
fi
