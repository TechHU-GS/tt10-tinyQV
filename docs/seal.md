# Seal Register — 硬件时序证明模块

## 它解决什么问题

物联网采集数据的时间戳是整条数据链里最不可信的一环。传感器 RTC 会漂移、掉电后时间丢失、固件 bug 可能写错时间、运维人员可以手动改设备时钟补录数据。数据到了云端，一条记录说"14:32 采集的温度是 27.3°C"，但没有任何手段验证 14:32 这个时间是不是真的。

Seal 不解决"精确时间"问题。它解决的是**数据时序的硬件级保证**。

## 核心机制

`mono_count` 是一个 32-bit 硬件递增计数器。每次 Seal commit，硬件自动 +1，固件不可写、不可清零（复位除外）。它给每条数据盖一个序号戳——你不知道精确的 UTC 时间，但你能确定"第 1078 号数据一定在第 1077 号之后采集"。这个时序关系由硅片保证，不依赖 RTC、不依赖软件、不依赖网络。

配合网关 ACK 中捎带的 UTC 时间同步，可以建立**时间锚点**——"第 1000 号数据对应 14:00:00 UTC"。有了锚点和单调递增序号，就可以反推每条数据的大致采集时间。即使 RTC 漂移了几分钟，mono_count 的顺序仍然绝对可信。

## 价值定位

Seal 是**工程级数据质量保证**，不是法律级可信时间戳。

**它能做的：**
- mono_count 断号 → 检测数据丢失
- mono_count 乱序 → 检测设备或链路 bug
- session_id 变化 → 检测设备重启
- CRC 不匹配 → 检测传输 bit 错误
- 把篡改门槛从"改一行代码"提高到"需要硬件级访问"

**它不能做的：**
- 不提供精确 UTC 时间（只有相对顺序 + 锚点估算）
- 不防物理攻击（有硬件访问能力的攻击者可以绕过）
- 不防数据链路下游伪造（数据离开 SoC UART 后进入软件世界）
- 没有密码学签名，不符合 RFC 3161 / TSA 标准
- 不能直接用于法律证据

**安全边界精确到 SoC 的 UART TX 引脚。** 数据在芯片内部是硬件保护的；出了芯片就进入软件可控世界。信任锚在硅片上，不在协议里。

## 寄存器接口

| 地址 | 寄存器 | 读/写 | 说明 |
|------|--------|-------|------|
| 0x800002C | SEAL_DATA | W | 写入 value[31:0]，暂存到 value_reg |
| 0x800002C | SEAL_DATA | R | 3 次连续读取出完整 Seal 记录（自动递进） |
| 0x8000038 | SEAL_CTRL | W | {sensor_id[7:0], commit, crc_reset} |
| 0x8000038 | SEAL_CTRL | R | {29'b0, commit_dropped, seal_ready, seal_busy} |

## 写入流程

```c
// 1. 写入传感器数据
*(volatile uint32_t *)0x800002C = sensor_value;

// 2. 触发 commit（sensor_id=0x01, commit=1, crc_reset=0）
*(volatile uint32_t *)0x8000038 = (0x01 << 2) | 0x02;

// 3. 等待完成
while (*(volatile uint32_t *)0x8000038 & 0x01);  // poll seal_busy
```

硬件自动完成：
1. 快照 mono_count → cur_mono，然后 mono_count++
2. 锁定 session_id（首次 commit 时从自由计数器取值）
3. 初始化 CRC16 为 0xFFFF
4. 逐字节喂入 CRC16 引擎（9 字节，见下文字节序）
5. 锁存 {value, mono_count, session_id, crc16} 到 sealed 寄存器

## 读取流程

连续读 3 次 SEAL_DATA，read_seq 自动递进：

| 读次序 | 返回值 | 内容 |
|--------|--------|------|
| Read 0 | sealed_value[31:0] | 传感器数据 |
| Read 1 | {sealed_sid[7:0], sealed_mono[23:0]} | session_id + mono 低 24 位 |
| Read 2 | {sealed_mono[31:24], sealed_crc[15:0], 8'h00} | mono 高 8 位 + CRC16 |

read_seq 在第 3 次读后 wrap 回 0。commit 也会强制 reset 到 0。

## CRC16 字节喂入顺序

CRC16-MODBUS（多项式 0x8005 reflected，初值 0xFFFF），9 字节，小端序：

```
byte 0: sensor_id[7:0]
byte 1: value[7:0]
byte 2: value[15:8]
byte 3: value[23:16]
byte 4: value[31:24]
byte 5: mono_count[7:0]
byte 6: mono_count[15:8]
byte 7: mono_count[23:16]
byte 8: mono_count[31:24]
```

软件参考实现必须使用**完全一致的字节顺序**，否则 CRC 不匹配。

## 时间锚点约定

使用保留的 sensor_id 值标识特殊记录：

| sensor_id | 含义 | value 字段 |
|-----------|------|-----------|
| 0x00 | 保留 | — |
| 0x01-0xFD | 普通传感器 | 传感器采样值 |
| 0xFE | UTC 时间同步 | unix_timestamp (秒) |
| 0xFF | 保留 | — |

当网关 ACK 携带 UTC 时间时，固件应将其作为 sensor_id=0xFE 的 Seal 记录提交。
这样时间锚点本身也受 mono_count 保护，不能被事后插入。

## 已知限制与固件约束

| 限制 | 说明 |
|------|------|
| mono_count 溢出 | 2^32 次 commit 后 wrap 到 0（约 136 年 @ 1次/秒） |
| session_id 不跨电源周期唯一 | 8-bit 自由计数器，256ms 周期，不同上电可能重复 |
| commit_dropped | busy 期间到达的 commit 被丢弃（sticky flag），固件须检查 |
| read_seq 无越界保护 | 读超过 3 次会 wrap 回 Read 0，看到旧 value |
| CRC 仲裁 | Seal 占用 CRC16 引擎时 CPU 直接访问 CRC 外设返回 busy=1 |
| 复位清零 | 硬复位/WDT 复位/软复位均会清零 mono_count 和 session_locked |

## 软件参考实现

```c
uint16_t seal_crc16(uint8_t sensor_id, uint32_t value, uint32_t mono) {
    uint16_t crc = 0xFFFF;
    uint8_t buf[9] = {
        sensor_id,
        (value >>  0) & 0xFF,
        (value >>  8) & 0xFF,
        (value >> 16) & 0xFF,
        (value >> 24) & 0xFF,
        (mono  >>  0) & 0xFF,
        (mono  >>  8) & 0xFF,
        (mono  >> 16) & 0xFF,
        (mono  >> 24) & 0xFF,
    };
    for (int i = 0; i < 9; i++) {
        crc ^= buf[i];
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xA001;
            else         crc >>= 1;
        }
    }
    return crc;
}
```

## 升级路径

当前 Seal 不含密码学。如果未来需要走合规路线（碳交易审计、环保执法取证），需补两层：

1. **可信时间源**：GPS/北斗授时模块，提供亚秒级 UTC 精度
2. **密码学签名**：ESP32 eFuse 存 HMAC-SHA256 密钥，对完整 Seal 记录签名

升级后的记录格式（向后兼容）：

```
v1 (当前): | ver=0x01 | sid | value | mono | session | crc16 |
v2 (未来): | ver=0x02 | sid | value | mono | session | crc16 | hmac[32] |
```

v2 的 HMAC 覆盖整个 v1 record（含 crc16），老系统只验 v1 部分仍然有效。
硬件 Seal 不需要改动（RTL 冻结），HMAC 由 ESP32 在数据离开 SoC 后追加。
