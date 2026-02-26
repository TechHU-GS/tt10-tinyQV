# LoRa Edge SoC RTL 验证方法论

> 本文档记录 LoRa Edge SoC (TTIHP 26a) 的验证流程和结果，同时作为未来 RTL 项目的验证 pipeline 参考模板。

## 项目信息

| 项目 | 数据 |
|------|------|
| 工艺 | IHP SG13G2 130nm |
| CPU | TinyQV RV32EC @ 25MHz (4-bit 串行数据通路) |
| 自有外设 | CRC16, I2C, WDT, Seal, RTC, Timer, SysInfo, latch_mem |
| 综合规模 | 11,620 cells |
| 验证完成 | 2026-02-26 |

## 一、验证策略

### 1.1 四维交叉验证

| 方法 | 工具 | 适用场景 | 产出 |
|------|------|---------|------|
| **仿真** | Icarus Verilog | 功能正确性、时序交互 | PASS/FAIL 断言 |
| **形式验证** | SymbiYosys + Z3 | 安全性质 (不可逆/单调/互斥) | 数学证明 |
| **突变测试** | 手动注入 | 测试套件灵敏度 | 检出/遗漏 |
| **覆盖率** | Verilator --coverage | 代码路径盲区 | 分支覆盖百分比 |

**原则**: 单一方法无法保证质量。仿真能发现功能 bug 但无法穷举状态空间；形式验证能证明性质但只能处理小模块；突变测试验证测试套件本身的有效性；覆盖率量化盲区。四者交叉才可信。

### 1.2 测试分层

```
┌─────────────────────────────────────┐
│ 固件驱动测试 (fw_*.c + tb_*.v)      │  真实 CPU 执行，端到端验证
├─────────────────────────────────────┤
│ 集成测试 (tb_integration*.v)        │  CPU + Flash/PSRAM + 行为模型
├─────────────────────────────────────┤
│ 总线级测试 (tb_project.v)           │  force/release CPU 总线，不需固件
├─────────────────────────────────────┤
│ 单元测试 (tb_crc16.v 等)            │  直接端口驱动，隔离单模块
└─────────────────────────────────────┘
```

### 1.3 何时用哪层

| 场景 | 推荐层级 | 原因 |
|------|---------|------|
| 新外设开发 | 单元测试 | 快速迭代，不依赖其他模块 |
| 地址译码/仲裁 | 总线级 (tb_project) | 直接控制总线信号，精确定位 |
| 多外设交互/中断 | 固件驱动 | 需要真实 ISR 流程 |
| 安全关键性质 | 形式验证 | 仿真无法穷举 |
| 历史 bug 回归 | 突变测试 | 证明修复不可退化 |

## 二、仿真测试

### 2.1 测试清单

#### 单元测试 (5 个)
| TB | 被测模块 | PASS | 重点 |
|----|---------|------|------|
| tb_crc16.v | crc16_engine + peripheral | 33 | Modbus 多项式、busy 等待 |
| tb_i2c.v | i2c_peripheral + master | 57 | AXI Stream 桥接、NACK |
| tb_seal.v | seal_register | 169 | mono_count + 100 golden CRC vector |
| tb_watchdog.v | watchdog | 21 | 使能不可逆、kick 续命 |
| tb_rtc.v | rtc_counter | 20 | 白盒预置法 (force us_count) |

#### 总线级测试 (1 个)
| TB | GROUP | check() | 重点 |
|----|-------|---------|------|
| tb_project.v | 80 (G1-G80) | 268 | 全 16 MMIO slot、CRC 仲裁、复位链、SPI 路径 |

#### 集成测试 (3 个)
| TB | 说明 | PASS |
|----|------|------|
| tb_integration.v | P0-A: Flash XIP boot | 5 |
| tb_integration_b.v | P0-B: + PSRAM + I2C + Seal | 10 |
| tb_read_clear_regression.v | bit-serial read_complete 回归 | 18 |

#### 固件驱动测试 (10 个)
| TB | 固件 | UART 签名 | 验证要点 |
|----|------|-----------|---------|
| tb_irq_timer | fw_irq_timer | I1I2DN | Timer IRQ17 触发/清除 |
| tb_wdt_reboot | fw_wdt_reboot | B1B2DN | WDT 超时 → 系统复位 → 恢复 |
| tb_soft_reset | fw_soft_reset | S1S2DN | 0xA5 magic → reset_hold → 重启 |
| tb_i2c_stress | fw_i2c_stress | D1D2DN | I2C 连续 write_multiple |
| tb_crc_arb | fw_crc_arb | E1E2E3DN | Seal commit 期间 CPU CRC busy |
| tb_timer_edge | fw_timer_edge | F1F2F3DN | timer_count=1 边界 |
| tb_i2c_nack | fw_i2c_nack | G1G2DN | NACK 检测 + 恢复 |
| tb_concurrent | fw_concurrent | H1H2H3DN | 多外设并发 |
| tb_post | fw_post | POST\n...DN\n | 全 9 外设上电自检 |
| tb_irq_priority | fw_irq_priority | P1P2P3P4DN | IRQ16 > IRQ17 优先级仲裁 |

**汇总: 619 PASS, 0 FAIL (14 CI TBs)**

### 2.2 CI 流水线 (test.yaml)

```
步骤 1:  timescale 检查 (8 自有模块)
步骤 2:  Verilator lint -Wall (0 warning)
步骤 3:  Yosys synth check (7 模块, 0 problems)
步骤 4-8:  单元测试 (crc16/wdt/i2c/seal/rtc)
步骤 9:  tb_project bus-level (268 checks)
步骤 10-11: 集成测试 (P0-A/P0-B)
步骤 12: 回归测试 (read_clear_regression)
步骤 13-18: 固件测试 A-H
```

每个仿真步骤以 `grep -q "ALL TESTS PASSED"` 作为门控。

### 2.3 固件编译链

```bash
riscv64-elf-gcc -march=rv32ec_zicsr -mabi=ilp32e -nostdlib -Os \
  -T test/fw_p0b.ld -o fw.elf fw.c
riscv64-elf-objcopy -O verilog --verilog-data-width=4 fw.elf fw.hex
```

linker script 要点:
- `.text._vectors` 在 0x0 (向量表必须在 Flash 起始)
- `.data` 在 PSRAM (0x01000000)
- 栈指针在 PSRAM 顶部

### 2.4 行为模型

| 模型 | 文件 | 仿真对象 |
|------|------|---------|
| QSPI Flash | test/qspi_flash_model.v | SPI NOR Flash XIP (HEX_FILE 参数) |
| QSPI PSRAM | test/qspi_psram_model.v | PSRAM 8KB |
| I2C Slave | test/i2c_slave_model.v | SHT31 温湿度传感器 @ 0x44 |

Verilator 覆盖率版本在 verify/ 目录 (*_sync.v)，使用系统时钟 + 边沿检测替代派生时钟。

## 三、形式验证

### 3.1 工具链

- **SymbiYosys**: 形式验证前端 (读 .sby 配置文件)
- **Z3**: SMT solver (作为 smtbmc 引擎)
- **模式**: BMC (有界模型检查) + prove (k-induction 无界证明)

安装: `brew install --cask yosyshq/tap/oss-cad-suite`

### 3.2 属性设计原则

**适合形式验证的性质**:
- 不可逆 (一旦 enabled 永远 enabled)
- 单调递增 (mono_count 只能 +1)
- 互斥 (两个 master 不能同时控制一个资源)

**不适合形式验证的**:
- 功能正确性 (CRC 计算结果是否正确 → 用仿真)
- 长时序链 (UART 发送完整字符串 → 状态空间太大)

### 3.3 属性清单

#### seal_register (verify/seal_formal.sby, depth=40)
```verilog
`ifdef FORMAL
  // P1: mono_count 每周期只能 +0 或 +1
  assert(mono_count == $past(mono_count) || mono_count == $past(mono_count) + 1);
  // P2: commit 完成 (S_LATCH→S_IDLE) 恰好 +1
  // P3: 非 commit 周期 mono 不变
  // P4: session_locked 后 session_id 不可变
  // P5: session_locked 单调
  // P6: IDLE 状态下 MMIO 写不改 mono (无软件后门)
`endif
```

#### watchdog (verify/wdt_formal.sby, depth=30)
- enabled 不可逆
- counter 零不下溢
- wdt_reset 精确触发条件

#### CRC 仲裁 (verify/crc_arb_formal.sby, depth=5)
- seal_using_crc 决定 init/data/dv 路由
- 全信号同步切换
- 空闲时零输出

### 3.4 .sby 配置模板

```ini
[tasks]
bmc
prove

[options]
bmc: mode bmc
bmc: depth 40
prove: mode prove
prove: depth 40

[engines]
smtbmc z3

[script]
read_verilog -formal -DFORMAL ../src/module.v
prep -top module_name

[files]
../src/module.v
```

## 四、突变测试

### 4.1 方法

```
1. 选择突变点 (安全关键行或历史 bug 行)
2. 手动修改一行 RTL
3. 编译 + 运行目标 TB
4. 验证 TB 报 FAIL (如果 PASS 说明测试套件有盲区)
5. 恢复原始代码
```

### 4.2 突变点选择原则

- **历史致命 bug**: 精确复现曾经的错误代码，确保回归测试能永久锁定
- **安全关键路径**: seal mono_count 递增、WDT enabled 锁定
- **算法核心**: CRC 多项式常数

### 4.3 结果

| # | 突变 | 检出 TB | 类型 |
|---|------|---------|------|
| 1 | seal mono_count+1 → mono_count | tb_seal | 安全 |
| 2 | seal read_seq 固定为 0 | tb_seal | 功能 |
| 3 | WDT enabled<=1 → enabled<=0 | tb_watchdog | 安全 |
| 4 | i2c_data_rd read_complete→read_n | tb_read_clear_regression | 回归 |
| 5 | CRC 多项式 0xA001→0xA000 | tb_crc16 | 算法 |
| 6 | seal_data_rd read_complete→read_n | tb_read_clear_regression | 回归 |

**6/6 全部检出。** #4/#6 由项目级 TB 检出，单元级 tb_i2c 无法检出 — 这说明集成测试不可替代。

## 五、覆盖率分析

### 5.1 工具

```bash
# 编译 (单模块)
verilator --cc --coverage -DSIM module.v --Mdir obj --exe tb.cpp
make -C obj -f Vmodule.mk

# 编译 (全 SoC，需 --no-timing + 同步模型)
verilator --cc --coverage --no-timing -DSIM -Wno-fatal \
  wrapper.v ../src/*.v ... --exe tb.cpp

# 运行
./obj/Vmodule

# 分析
verilator_coverage --annotate ann_dir coverage.dat
# 看 ann_dir/ 中的 .v 文件: %=hit, ~=partial, %000000=zero-hit
```

### 5.2 结果

| 模块 | Hit | Partial | Zero-Hit | 覆盖率 |
|------|-----|---------|----------|--------|
| crc16_engine | 18 | 0 | 0 | **100%** |
| crc16_peripheral | comb | 2 | 0 | **100%** |
| watchdog | 17 | 2 | 0 | **100%** |
| rtc_counter | 10 | 1 | 0 | **100%** |
| latch_mem | 9 | 7(toggle) | 0 | **100%** |
| i2c_peripheral | 37 | 8 | 1(artifact) | **99.7%** |
| seal_register | 92/121 | — | 2(default) | **76%** |
| project.v | 65 | 35 | 30 | **81.8%** |

### 5.3 Zero-hit 分析方法

每条 zero-hit 必须归类为以下之一:
1. **不可达** (unreachable default 分支) → 接受
2. **测试环境限制** (无 SPI slave 模型) → 用其他 TB 补 (如 Icarus bus-level)
3. **真实功能缺口** → 必须补测试

project.v 30 条 zero-hit 全部属于前两类。

## 六、可复用测试模式

### 6.1 bus_write/bus_read task

```verilog
task bus_write(input [4:0] slot, input [31:0] data);
    force dut.i_tinyqv.data_addr    = {1'b1, 20'b0, slot, 2'b00};
    force dut.i_tinyqv.data_write_n = 2'b10;
    force dut.i_tinyqv.data_read_n  = 2'b11;
    force dut.i_tinyqv.data_out     = data;
    @(posedge clk);
    repeat(4) @(posedge clk);
    force dut.i_tinyqv.data_write_n = 2'b11;
    @(posedge clk);
    release dut.i_tinyqv.data_addr;
    release dut.i_tinyqv.data_write_n;
    release dut.i_tinyqv.data_read_n;
    release dut.i_tinyqv.data_out;
endtask
```

这个模式绕过 CPU 直接驱动总线，适用于测试地址译码和外设寄存器。

### 6.2 check() 断言 task

```verilog
integer pass_count = 0, fail_count = 0;
task check(input [255:0] name, input condition);
    if (condition) begin
        $display("[PASS] %0s", name);
        pass_count = pass_count + 1;
    end else begin
        $display("[FAIL] %0s", name);
        fail_count = fail_count + 1;
    end
endtask
```

### 6.3 UART 字节监控器

```verilog
// 115200 baud @ 25MHz = 217 clocks/bit
localparam BAUD_CLKS = 217;
// 检测 start bit → 等 1.5 bit → 逐 bit shift → stop bit
```

### 6.4 固件 UART 签名协议

每个测试点输出 2 字节: `tag_char` + `result_digit`。例如 `I1` = I2C test 1 PASS。
TB 用 `check_2char(tag, val, name)` 逐对验证。
以 `DN\n` 标记固件执行完毕。

## 七、关键教训

### 7.1 TinyQV bit-serial 多周期读

TinyQV 是 4-bit 串行 CPU，32-bit MMIO 读需要 8 个时钟周期，`read_n != 2'b11` 在整个 8 周期内有效。

**RULE A**: 清除标志/弹出 FIFO/推进指针 → 必须用 `read_complete` (8 周期结束后的单脉冲)
**RULE B**: `data_out` 必须在整个多周期读期间保持稳定

违反这两条规则会导致 CPU 读到不一致的数据。单元测试无法发现此问题（因为单元测试用单周期 data_rd 脉冲），只有集成测试或 tb_read_clear_regression 才能检出。

### 7.2 IRQ 优先级假阳性

测试两个 IRQ 的优先级时，如果一个 IRQ 先到达、先被处理，第二个 IRQ 后到达、后被处理，测试会 PASS 但并没有验证优先级编码器。

**正确方法**: 关闭全局中断 → 同时触发两个 IRQ → 等待两个 mip bit 都置位 → 开启中断 → 优先级编码器在同一周期仲裁。

### 7.3 Verilator 派生时钟问题

行为模型中 `always @(posedge spi_clk)` 使用派生时钟。Verilator `--timing` 无法正确调度这类信号。

**解决**: 创建同步版本，用系统时钟 + 边沿检测:
```verilog
reg spi_clk_prev;
always @(posedge sys_clk) begin
    spi_clk_prev <= spi_clk;
    if (spi_clk && !spi_clk_prev) begin // 上升沿
        // 原 @(posedge spi_clk) 中的逻辑
    end
end
```

编译时使用 `--no-timing`。

## 八、验证结果总表

| 维度 | 数据 |
|------|------|
| 仿真 PASS 点 | **619** (14 CI TBs, 0 FAIL) |
| 形式证明 | **19 条** (3 模块, k-induction 全 PASS) |
| 突变检出率 | **6/6 = 100%** |
| 覆盖率量化 | **8/8 自有模块** (5 个 100%) |
| 已发现真实 bug | **4 个** (2 致命 + 2 隐患) |
| CI 步骤 | **18** (静态分析 + 仿真) |
| 测试/RTL 行数比 | **4.3 : 1** |
| 测试文件 | 21 TB + 12 固件 + 3 行为模型 |
