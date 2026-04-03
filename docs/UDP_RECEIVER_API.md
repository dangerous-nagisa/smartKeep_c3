# smartKeep UDP 接收端对接文档

> **版本:** v2.0 (5节点复用版)
> **更新日期:** 2026-04-03
> **适用固件:** smartKepp_C3 main 分支

---

## 目录

1. [网络拓扑](#1-网络拓扑)
2. [数据包协议 — node_packet_t](#2-数据包协议)
3. [CRC-8 校验算法](#3-crc-8-校验算法)
4. [远程配置协议](#4-远程配置协议)
5. [佩戴模板定义](#5-佩戴模板定义)
6. [Python 接收器使用指南](#6-python-接收器使用指南)
7. [自行对接指南](#7-自行对接指南)
8. [CSV 日志格式](#8-csv-日志格式)
9. [常见问题](#9-常见问题)

---

## 1. 网络拓扑

```
┌──────────┐  ┌──────────┐  ┌──────────┐
│  节点 1  │  │  节点 2  │  │  节点 N  │   (最多 5 个)
│ ESP32-C3 │  │ ESP32-C3 │  │ ESP32-C3 │
└────┬─────┘  └────┬─────┘  └────┬─────┘
     │             │             │
     └─────────────┼─────────────┘
                   │  WiFi 2.4GHz (STA 模式)
                   ▼
           ┌───────────────┐
           │   WiFi 热点   │
           │  (手机/路由)  │
           └───────┬───────┘
                   │
                   ▼
           ┌───────────────┐
           │    PC 接收端   │
           │ 0.0.0.0:8888  │  ← 数据端口 (UDP, 被动监听)
           │ 0.0.0.0:8889  │  ← 配置端口 (UDP, 按需使用)
           └───────────────┘
```

| 参数 | 值 |
|:---|:---|
| 传输协议 | UDP |
| 数据端口 | **8888** |
| 配置端口 | **8889** |
| 广播地址 | 255.255.255.255 |
| 每节点数据率 | 50 包/s (50 Hz) |
| 5 节点总带宽 | ~11 KB/s |

---

## 2. 数据包协议

### 2.1 node_packet_t 结构 (44 字节)

固件端 C 定义:

```c
typedef struct {
    uint8_t  node_id;      // 节点 ID (1~5)
    uint8_t  battery;      // 电量 (0~100, 0xFF=未实现)
    uint16_t seq;          // 包序号 (0~65535 循环)
    uint32_t timestamp;    // 毫秒时间戳 (自启动)

    float    euler[3];     // [roll, pitch, yaw] (度)
    float    accel[3];     // [ax, ay, az] (m/s², 滤波后)
    float    gyro_norm;    // |ω| 角速度模 (°/s)
    float    jerk;         // |da/dt| 加加速度模 (m/s³)

    uint8_t  step_flag;    // 0=无 1=触地(HS) 2=离地(TO)
    uint8_t  crc8;         // CRC-8 校验
    // 2 字节对齐填充 (隐含)
} node_packet_t;           // sizeof = 44
```

### 2.2 字段偏移量表

| 偏移 | 长度 | 类型 | 字段 | 说明 |
|:---:|:---:|:---|:---|:---|
| 0 | 1 | uint8 | `node_id` | 硬件节点编号 1~5 |
| 1 | 1 | uint8 | `battery` | 电量百分比, 0xFF=未实现 |
| 2 | 2 | uint16_le | `seq` | 递增序号, 溢出回绕 |
| 4 | 4 | uint32_le | `timestamp` | esp_timer 毫秒, 溢出 ~49 天 |
| 8 | 4 | float_le | `roll` | 横滚角 (°) |
| 12 | 4 | float_le | `pitch` | 俯仰角 (°) |
| 16 | 4 | float_le | `yaw` | 航向角 (°), 无磁力计会漂移 |
| 20 | 4 | float_le | `ax` | X 加速度 (m/s²) |
| 24 | 4 | float_le | `ay` | Y 加速度 (m/s²) |
| 28 | 4 | float_le | `az` | Z 加速度 (m/s²) |
| 32 | 4 | float_le | `gyro_norm` | 角速度模 (°/s) |
| 36 | 4 | float_le | `jerk` | 加加速度模 (m/s³) |
| 40 | 1 | uint8 | `step_flag` | 步态事件编码 |
| 41 | 1 | uint8 | `crc8` | CRC-8 校验值 |
| 42 | 2 | — | (padding) | 对齐填充, 忽略 |

**字节序:** 全部 **小端 (Little-Endian)**，与 ESP32-C3 (RISC-V) 原生一致。

### 2.3 step_flag 编码

| 值 | 含义 | 临床事件 |
|:---:|:---|:---|
| 0 | 无事件 | — |
| 1 | Heel Strike (触地) | 足跟着地, jerk 尖峰 |
| 2 | Toe Off (离地) | 脚趾离地, 角速度峰值 |

### 2.4 Python struct 解包

```python
import struct

PKT_FMT  = '<BBHIffffffffBBxx'   # xx = 2 字节 padding
PKT_SIZE = 44

data = sock.recvfrom(64)[0]
assert len(data) == PKT_SIZE

(node_id, battery, seq, timestamp,
 roll, pitch, yaw,
 ax, ay, az,
 gyro_norm, jerk,
 step_flag, crc8) = struct.unpack(PKT_FMT, data)
```

### 2.5 其他语言解包示例

**C# / Unity:**
```csharp
using System;
using System.Runtime.InteropServices;

[StructLayout(LayoutKind.Sequential, Pack = 1)]
struct NodePacket {
    public byte nodeId;
    public byte battery;
    public ushort seq;
    public uint timestamp;
    public float roll, pitch, yaw;
    public float ax, ay, az;
    public float gyroNorm;
    public float jerk;
    public byte stepFlag;
    public byte crc8;
    // 2 bytes padding auto-handled
}

// 解包
byte[] data = udpClient.Receive(ref remoteEP);
NodePacket pkt = MemoryMarshal.Read<NodePacket>(data);
```

**JavaScript / Node.js:**
```javascript
function parsePacket(buf) {
    if (buf.length !== 44) return null;
    return {
        node_id:   buf.readUInt8(0),
        battery:   buf.readUInt8(1),
        seq:       buf.readUInt16LE(2),
        timestamp: buf.readUInt32LE(4),
        roll:      buf.readFloatLE(8),
        pitch:     buf.readFloatLE(12),
        yaw:       buf.readFloatLE(16),
        ax:        buf.readFloatLE(20),
        ay:        buf.readFloatLE(24),
        az:        buf.readFloatLE(28),
        gyro_norm: buf.readFloatLE(32),
        jerk:      buf.readFloatLE(36),
        step_flag: buf.readUInt8(40),
        crc8:      buf.readUInt8(41),
    };
}
```

---

## 3. CRC-8 校验算法

| 参数 | 值 |
|:---|:---|
| 多项式 | 0x07 (x⁸ + x² + x + 1) |
| 初始值 | 0x00 |
| 输入反转 | 否 |
| 输出反转 | 否 |
| 校验范围 | 字节 0~40 (crc8 字段之前的 41 字节) |

### Python 实现

```python
def crc8_calc(data: bytes) -> int:
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) if (crc & 0x80) else (crc << 1)
            crc &= 0xFF
    return crc

# 校验
received_crc = data[41]
calculated_crc = crc8_calc(data[:41])
is_valid = (received_crc == calculated_crc)
```

### C 实现

```c
uint8_t crc8_calc(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? ((crc << 1) ^ 0x07) : (crc << 1);
        }
    }
    return crc;
}

// 校验
uint8_t calc = crc8_calc((uint8_t*)&pkt, 41);
bool valid = (calc == pkt.crc8);
```

---

## 4. 远程配置协议

### 4.1 概述

通过 UDP 端口 **8889** 可远程修改/查询节点 ID，无需物理接触设备。

### 4.2 请求包 (PC → 节点, 6 字节)

```
 0       3   4     5
┌────────┬───┬─────┐
│ "SKCF" │cmd│value│
│ 4 字节 │1B │ 1B  │
└────────┴───┴─────┘
```

### 4.3 应答包 (节点 → PC, 6 字节)

```
 0       3   4     5
┌────────┬───┬─────┐
│ "SKCK" │cmd│value│
│ 4 字节 │1B │ 1B  │
└────────┴───┴─────┘
```

### 4.4 命令列表

| cmd | 名称 | 请求 value | 应答 value |
|:---:|:---|:---|:---|
| 0x01 | SET_ID | 新 ID (1~5) | 设置成功的 ID, 或 0xFF=错误 |
| 0x02 | GET_ID | 忽略 (填 0) | 当前 ID |

### 4.5 交互示例

**设置节点 ID 为 3:**
```
PC → 节点:  53 4B 43 46 01 03    ("SKCF" + SET_ID + 3)
节点 → PC:  53 4B 43 4B 01 03    ("SKCK" + SET_ID + 3)  ← 成功
```

**查询节点 ID:**
```
PC → 节点:  53 4B 43 46 02 00    ("SKCF" + GET_ID + 0)
节点 → PC:  53 4B 43 4B 02 03    ("SKCK" + GET_ID + 3)  ← 当前 ID=3
```

**设置无效 ID:**
```
PC → 节点:  53 4B 43 46 01 08    ("SKCF" + SET_ID + 8)
节点 → PC:  53 4B 43 4B 01 FF    ("SKCK" + SET_ID + 0xFF)  ← 错误
```

### 4.6 Python 配置代码

```python
import socket

CFG_PORT = 8889

def set_node_id(ip: str, new_id: int) -> bool:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(2.0)
    try:
        sock.sendto(b'SKCF' + bytes([0x01, new_id]), (ip, CFG_PORT))
        data, _ = sock.recvfrom(16)
        return len(data) >= 6 and data[:4] == b'SKCK' and data[5] == new_id
    except socket.timeout:
        return False
    finally:
        sock.close()

def get_node_id(ip: str) -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(2.0)
    try:
        sock.sendto(b'SKCF' + bytes([0x02, 0x00]), (ip, CFG_PORT))
        data, _ = sock.recvfrom(16)
        return data[5] if len(data) >= 6 and data[:4] == b'SKCK' else -1
    except:
        return -1
    finally:
        sock.close()
```

### 4.7 注意事项

- 配置指令需要**单播**到目标节点 IP（不是广播）
- 需先确定目标节点 IP 地址（可通过路由器 DHCP 列表、或解析数据包来源 IP）
- 修改 ID 后**立即生效**，同时持久化到 NVS（断电不丢失）
- 超时 2 秒无应答视为失败

---

## 5. 佩戴模板定义

节点硬件是**通用的**（只有 ID 1~5），身体位置映射由接收端根据测试类型选择。

### 5.1 模板 A: 步态分析 (`gait`)

适用测试: TUG (计时起走), 10MWT (10米步行)

| node_id | 位置 | 英文标识 |
|:---:|:---|:---|
| 1 | 腰部 | WAIST |
| 2 | 左脚 | L_FOOT |
| 3 | 右脚 | R_FOOT |
| 4 | 左大腿 | L_THIGH |
| 5 | 右大腿 | R_THIGH |

### 5.2 模板 B: 坐站转移 (`sit2stand`)

适用测试: 5xSTS (5次坐站), Berg 平衡量表

| node_id | 位置 | 英文标识 |
|:---:|:---|:---|
| 1 | 腰部 | WAIST |
| 2 | 左大腿 | L_THIGH |
| 3 | 右大腿 | R_THIGH |
| 4 | 左小腿 | L_SHANK |
| 5 | 右小腿 | R_SHANK |

### 5.3 模板 C: 上肢评估 (`upper`)

适用测试: Fugl-Meyer 上肢评估

| node_id | 位置 | 英文标识 |
|:---:|:---|:---|
| 1 | 胸部 | CHEST |
| 2 | 左上臂 | L_ARM |
| 3 | 右上臂 | R_ARM |
| 4 | 左前臂 | L_FOREARM |
| 5 | 右前臂 | R_FOREARM |

### 5.4 模板 D: 通用 (`generic`)

不绑定身体位置，仅按 ID 显示。

### 5.5 自定义模板

在 `udp_receiver.py` 的 `TEMPLATES` 字典中添加新条目即可:

```python
TEMPLATES['my_custom'] = {
    1: ('头部', 'HEAD'),
    2: ('左手', 'L_HAND'),
    3: ('右手', 'R_HAND'),
    4: ('左脚', 'L_FOOT'),
    5: ('右脚', 'R_FOOT'),
}
```

---

## 6. Python 接收器使用指南

### 6.1 环境要求

- Python 3.7+
- 无第三方依赖（仅用标准库）

### 6.2 命令行参数

| 参数 | 说明 | 默认值 |
|:---|:---|:---|
| `--port <N>` | 数据监听端口 | 8888 |
| `--raw` | 原始流模式 (逐包打印) | 关闭 |
| `--log` | 保存 CSV 日志文件 | 关闭 |
| `--template <name>` / `-t` | 佩戴模板 | gait |
| `--set-id <IP> <ID>` | 远程设置节点 ID | — |
| `--get-id <IP>` | 查询节点 ID | — |

### 6.3 使用示例

```bash
# 仪表盘模式 (默认步态模板)
python tools/udp_receiver.py

# 切换模板
python tools/udp_receiver.py -t sit2stand
python tools/udp_receiver.py -t upper
python tools/udp_receiver.py -t generic

# 原始流调试
python tools/udp_receiver.py --raw

# 保存 CSV 日志
python tools/udp_receiver.py --log

# 组合使用
python tools/udp_receiver.py --raw --log -t gait

# 远程配置节点
python tools/udp_receiver.py --set-id 192.168.1.100 3
python tools/udp_receiver.py --get-id 192.168.1.100
```

### 6.4 仪表盘模式显示

```
╔══════════════════════════════════════════════════════════════════════════════╗
║  smartKeep 5节点 UDP 接收器                                     运行:  125s  ║
╠══════════════════════════════════════════════════════════════════════════════╣
║ ID │ 位置     │ 状态   │   seq │   Roll │  Pitch │    Yaw │  丢包 │ pkt/s║
╠══════════════════════════════════════════════════════════════════════════════╣
║  1 │ WAIST    │ ● 在线 │ 12845 │  +12.3 │   -5.1 │  180.2 │  0.0% │  49.8║
║  2 │ L_FOOT   │ ● 在线 │ 12840 │   +8.7 │   -3.2 │  185.6 │  0.1% │  49.6║
║  3 │ R_FOOT   │ ○ 离线 │     - │      - │      - │      - │     - │     -║
║  4 │ L_THIGH  │ ○ 离线 │     - │      - │      - │      - │     - │     -║
║  5 │ R_THIGH  │ ○ 离线 │     - │      - │      - │      - │     - │     -║
╚══════════════════════════════════════════════════════════════════════════════╝

  在线: 2/5  总包数: 25340  丢失: 8
  按 Ctrl+C 退出
```

- **● 在线**: 最近 2 秒内有数据
- **○ 离线**: 超过 2 秒无数据
- **pkt/s**: 滑动窗口 (2 秒) 计算的实时包率
- **丢包**: 基于 seq 跳跃统计

### 6.5 原始流模式显示

```
===============================================================================
  smartKeep UDP 接收器 (原始流模式)
===============================================================================
  ID    seq     Roll    Pitch      Yaw      ax     ay     az   |ω|   Jerk  gait CRC
-----------------------------------------------------------------------------------------------
   1  12845   +12.3    -5.1    180.2    +0.3   -0.5   +9.7    2.1    3.2    -  OK
>> 1  12846   +11.8    -4.9    180.1    +1.2   -2.1  +10.5   12.3   52.1   HS  OK
<< 1  12890   +13.1    -5.3    180.4    +0.1   -0.2   +9.3  115.2    8.7   TO  OK
```

- `>>` 表示 Heel Strike 事件
- `<<` 表示 Toe Off 事件

---

## 7. 自行对接指南

### 7.1 最小接收端 (Python, 10 行)

```python
import socket, struct

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', 8888))

while True:
    data, addr = sock.recvfrom(64)
    if len(data) != 44:
        continue
    vals = struct.unpack('<BBHIffffffffBBxx', data)
    node_id, battery, seq, ts = vals[:4]
    roll, pitch, yaw = vals[4:7]
    ax, ay, az = vals[7:10]
    gyro_norm, jerk = vals[10:12]
    step_flag, crc8 = vals[12:14]
    print(f"[{node_id}] seq={seq} R={roll:+.1f} P={pitch:+.1f} Y={yaw:.1f}")
```

### 7.2 自行对接清单

1. **绑定 UDP 0.0.0.0:8888** — 被动监听广播
2. **过滤包长 44 字节** — 丢弃非标准包
3. **小端解包** — 按偏移表解析各字段
4. **CRC-8 校验** — data[0:41] 计算, 与 data[41] 比较
5. **按 node_id 分流** — 多节点数据通过 node_id 区分
6. **seq 丢包检测** — 序号跳跃 = 丢包
7. **在线检测** — 2 秒无数据 = 离线

### 7.3 数据流率参考

| 节点数 | 包率 | 数据率 | 备注 |
|:---:|:---:|:---:|:---|
| 1 | 50 pkt/s | 2.2 KB/s | 单节点调试 |
| 3 | 150 pkt/s | 6.6 KB/s | TUG/10MWT 配置 |
| 5 | 250 pkt/s | 11 KB/s | 满配 |

---

## 8. CSV 日志格式

使用 `--log` 参数时自动保存，文件名格式: `smartkeep_YYYYMMDD_HHMMSS.csv`

### 8.1 列定义

```csv
node_id,battery,seq,timestamp,roll,pitch,yaw,ax,ay,az,gyro_norm,jerk,step_flag,crc8,crc_ok,recv_time
1,255,0,1234,12.3,-5.1,180.2,0.3,-0.5,9.7,2.1,3.2,0,171,True,1712345678.123
```

| 列名 | 类型 | 说明 |
|:---|:---|:---|
| `node_id` | int | 节点 ID |
| `battery` | int | 电量 (255=未实现) |
| `seq` | int | 包序号 |
| `timestamp` | int | 固件毫秒时间戳 |
| `roll` | float | 横滚角 (°) |
| `pitch` | float | 俯仰角 (°) |
| `yaw` | float | 航向角 (°) |
| `ax, ay, az` | float | 加速度 (m/s²) |
| `gyro_norm` | float | 角速度模 (°/s) |
| `jerk` | float | 加加速度模 (m/s³) |
| `step_flag` | int | 步态事件 (0/1/2) |
| `crc8` | int | CRC 校验值 |
| `crc_ok` | bool | CRC 校验结果 |
| `recv_time` | float | PC 端接收时间 (Unix 时间戳) |

### 8.2 用 pandas 加载

```python
import pandas as pd

df = pd.read_csv('smartkeep_20260403_143000.csv')

# 按节点过滤
node1 = df[df.node_id == 1]

# 绘制欧拉角
node1[['roll', 'pitch', 'yaw']].plot()
```

---

## 9. 常见问题

### Q: 收不到数据?

1. 确认 PC 和节点连接**同一 WiFi 网络**
2. 检查防火墙是否放行 UDP 8888 端口
3. 确认节点串口日志显示 `WiFi 已连接` 和 `UDP socket 就绪`
4. Windows 用户: 检查 Windows Defender 防火墙规则

### Q: 丢包率高?

1. 减少 WiFi 环境干扰
2. 缩短节点与热点的距离
3. 检查热点是否启用了 AP 隔离（需关闭）
4. 正常情况 5 节点丢包率应 < 1%

### Q: 多个节点显示相同 ID?

新板子首次上电默认 ID=1。使用远程配置分别设置:
```bash
python tools/udp_receiver.py --set-id <节点IP> <ID>
```

### Q: 如何获取节点 IP 地址?

1. 查看路由器/热点的 DHCP 客户端列表
2. 或在节点串口日志中查找 `WiFi 已连接 IP=x.x.x.x`

### Q: Yaw 角度漂移?

正常现象。BMI270 无磁力计，Yaw 通道只有陀螺仪积分，长时间会漂移。
Roll 和 Pitch 由加速度计修正，不受影响。

### Q: timestamp 字段溢出?

uint32 毫秒时间戳约 49 天溢出回零。正常使用不会遇到。
如需长时间记录，使用 CSV 中的 `recv_time` (PC 端时间戳) 作为时间参考。
