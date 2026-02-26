#!/usr/bin/env python3
"""Generate 100 Seal CRC16-MODBUS golden vectors for tb_seal.v.

Each vector: (sensor_id, value, mono_count) -> CRC16-MODBUS
CRC is computed over 9 bytes in this order:
  sensor_id (1 byte)
  value[7:0], value[15:8], value[23:16], value[31:24]  (little-endian)
  mono_count[7:0], mono_count[15:8], mono_count[23:16], mono_count[31:24]  (little-endian)

Polynomial: 0xA001 (reflected 0x8005), init: 0xFFFF.

Output: test/seal_golden.mem  ($readmemh format)
  Each line: sensor_id(8) value(32) mono_count(32) expected_crc(16)
  Format: SS_VVVVVVVV_MMMMMMMM_CCCC  (hex, underscores for readability)
"""

import random
import struct
import os

NUM_VECTORS = 100
SEED = 42  # deterministic for reproducibility


def crc16_modbus(data: bytes) -> int:
    """Compute CRC16-MODBUS: init=0xFFFF, poly=0xA001 (reflected 0x8005)."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def make_seal_bytes(sensor_id: int, value: int, mono_count: int) -> bytes:
    """Build the 9-byte sequence the seal_register feeds to CRC16."""
    buf = bytearray(9)
    buf[0] = sensor_id & 0xFF
    # value: little-endian 32-bit
    buf[1] = (value >> 0) & 0xFF
    buf[2] = (value >> 8) & 0xFF
    buf[3] = (value >> 16) & 0xFF
    buf[4] = (value >> 24) & 0xFF
    # mono_count: little-endian 32-bit
    buf[5] = (mono_count >> 0) & 0xFF
    buf[6] = (mono_count >> 8) & 0xFF
    buf[7] = (mono_count >> 16) & 0xFF
    buf[8] = (mono_count >> 24) & 0xFF
    return bytes(buf)


def main():
    random.seed(SEED)

    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_dir = os.path.dirname(script_dir)
    out_path = os.path.join(project_dir, "test", "seal_golden.mem")

    vectors = []
    for i in range(NUM_VECTORS):
        sensor_id = random.randint(0, 255)
        value = random.randint(0, 0xFFFFFFFF)
        mono_count = i  # mono_count starts at 0 and auto-increments per commit
        data = make_seal_bytes(sensor_id, value, mono_count)
        crc = crc16_modbus(data)
        vectors.append((sensor_id, value, mono_count, crc))

    with open(out_path, "w") as f:
        f.write("// Seal CRC16-MODBUS golden vectors (auto-generated)\n")
        f.write("// Format per line: sensor_id[7:0] value[31:0] expected_crc[15:0]\n")
        f.write("// mono_count = line index (0..99), auto-incremented by DUT\n")
        f.write(f"// {NUM_VECTORS} vectors, seed={SEED}\n")
        for sid, val, mono, crc in vectors:
            # $readmemh format: each line is one hex word
            # We pack: {sensor_id[7:0], value[31:0], crc[15:0]} = 56 bits = 14 hex digits
            packed = (sid << 48) | (val << 16) | crc
            f.write(f"{packed:014X}\n")

    print(f"Generated {NUM_VECTORS} golden vectors -> {out_path}")

    # Also print a few for verification
    print("\nSample vectors:")
    print(f"  {'#':>3}  {'SID':>4}  {'VALUE':>10}  {'MONO':>10}  {'CRC':>6}  {'BYTES'}")
    for i in range(min(5, NUM_VECTORS)):
        sid, val, mono, crc = vectors[i]
        data = make_seal_bytes(sid, val, mono)
        hexbytes = " ".join(f"{b:02X}" for b in data)
        print(f"  {i:3d}  0x{sid:02X}  0x{val:08X}  0x{mono:08X}  0x{crc:04X}  [{hexbytes}]")
    print("  ...")
    for i in range(max(5, NUM_VECTORS - 3), NUM_VECTORS):
        sid, val, mono, crc = vectors[i]
        data = make_seal_bytes(sid, val, mono)
        hexbytes = " ".join(f"{b:02X}" for b in data)
        print(f"  {i:3d}  0x{sid:02X}  0x{val:08X}  0x{mono:08X}  0x{crc:04X}  [{hexbytes}]")

    # Cross-check: verify first two vectors match Test 8 in tb_seal.v
    # Test 8 V1: sensor=0xAA, value=0x00000000, mono=0 -> CRC=0x578C
    check_data = make_seal_bytes(0xAA, 0x00000000, 0)
    check_crc = crc16_modbus(check_data)
    assert check_crc == 0x578C, f"Cross-check V1 failed: got 0x{check_crc:04X}, expected 0x578C"
    print(f"\nCross-check V1 (sid=0xAA, val=0, mono=0): CRC=0x{check_crc:04X} == 0x578C OK")

    # Test 8 V2: sensor=0xFF, value=0xFFFFFFFF, mono=1 -> CRC=0xE80E
    check_data = make_seal_bytes(0xFF, 0xFFFFFFFF, 1)
    check_crc = crc16_modbus(check_data)
    assert check_crc == 0xE80E, f"Cross-check V2 failed: got 0x{check_crc:04X}, expected 0xE80E"
    print(f"Cross-check V2 (sid=0xFF, val=0xFFFF.., mono=1): CRC=0x{check_crc:04X} == 0xE80E OK")


if __name__ == "__main__":
    main()
