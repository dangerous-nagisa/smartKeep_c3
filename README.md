# smartKeep_C3 — IMU 姿态解算传感器节点

基于 **ESP32-C3 + BMI270** 的可穿戴运动传感器节点，用于临床步态评估与康复训练监测。

## 系统架构

```
BMI270 (200Hz)
   │ I2C 400kHz
   ▼
ESP32-C3
   │ 零偏校准 → IIR 低通 → Madgwick AHRS → 欧拉角
   │ 步态状态机 (Heel Strike / Toe Off)
   │
   ▼ node_packet_t (44B, 50Hz)
UDP 广播 ──WiFi──► ESP32-P4 网关 ──MQTT──► 云端/PC
```

## 功能特性

- **6 轴姿态解算**: Madgwick AHRS 四元数滤波，输出 Roll/Pitch/Yaw
- **步态检测**: 双状态机 (SWING ↔ STANCE)，Jerk 触地检测 + 角速度离地检测
- **50Hz 数据输出**: 44 字节 `node_packet_t`，CRC-8 校验
- **WiFi UDP 广播**: 255.255.255.255:8888，P4 网关或 PC 均可接收
- **多节点支持**: NVS 持久化节点 ID (1~5)，UDP 远程配置 (端口 8889)
- **零偏校准**: 上电静置 2 秒自动校准陀螺仪

## 硬件连接

| 引脚 | 功能 | 外设 |
|:---:|:---|:---|
| GPIO4 | I2C SDA | BMI270 |
| GPIO5 | I2C SCL | BMI270 |
| GPIO2 | ADC | 电池采样 (TODO) |
| GPIO6 | PWM | 蜂鸣器 (TODO) |

> BMI270 I2C 地址: `0x68` (SDO→GND) 或 `0x69` (SDO→VCC)

## 快速开始

### 环境要求

- [ESP-IDF v5.5.x](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32c3/get-started/)
- 目标芯片: ESP32-C3

### 编译烧录

```bash
idf.py set-target esp32c3
idf.py build
idf.py -p COM3 flash monitor
```

### 首次使用

1. 烧录后上电，节点默认 ID=1
2. 串口日志显示校准过程（请保持静止 2 秒）
3. 校准完成后自动连接 WiFi 并开始发送数据

### 远程配置节点 ID

```bash
# 查询节点当前 ID
python tools/udp_receiver.py --get-id <节点IP>

# 设置节点 ID 为 3
python tools/udp_receiver.py --set-id <节点IP> 3
```

## 数据包格式

`node_packet_t` — 44 字节，小端序:

| 偏移 | 类型 | 字段 | 说明 |
|:---:|:---|:---|:---|
| 0 | uint8 | node_id | 节点 ID (1~5) |
| 1 | uint8 | battery | 电量 (0xFF=未实现) |
| 2 | uint16 | seq | 包序号 |
| 4 | uint32 | timestamp | 毫秒时间戳 |
| 8 | float×3 | euler | Roll, Pitch, Yaw (度) |
| 20 | float×3 | accel | ax, ay, az (m/s²) |
| 32 | float | gyro_norm | 角速度模 (°/s) |
| 36 | float | jerk | 加加速度模 (m/s³) |
| 40 | uint8 | step_flag | 0=无 1=触地 2=离地 |
| 41 | uint8 | crc8 | CRC-8 (poly=0x07) |

## PC 端工具

### UDP 直接接收 (调试用)

```bash
# 仪表盘
python tools/udp_receiver.py

# 原始流
python tools/udp_receiver.py --raw

# 指定佩戴模板
python tools/udp_receiver.py -t gait      # 步态分析
python tools/udp_receiver.py -t sit2stand  # 坐站转移
python tools/udp_receiver.py -t upper      # 上肢评估
```

### MQTT 订阅 (经 P4 网关)

```bash
pip install paho-mqtt
python tools/mqtt_subscriber.py            # 仪表盘
python tools/mqtt_subscriber.py --raw      # 原始帧
```

## 项目结构

```
smartKepp_C3/
├── main/
│   ├── main.c              ← 完整固件 (~850 行)
│   ├── node_config.h/c     ← 节点 ID 配置 (NVS + UDP 远程)
│   ├── CMakeLists.txt
│   └── idf_component.yml
├── tools/
│   ├── udp_receiver.py     ← UDP 直接接收
│   └── mqtt_subscriber.py  ← MQTT 订阅
├── docs/
│   └── UDP_RECEIVER_API.md ← 对接文档
└── WORK_REPORT.md          ← 设计工作报告
```

## 信号处理流水线

```
BMI270 (200Hz, ±4g, ±2000°/s)
  │
  ├─ LSB → 物理量 (m/s², °/s)
  ├─ 陀螺仪零偏校准 (200 样本, ~2s)
  ├─ 一阶 IIR 低通 (α_acc=0.3, α_gyr=0.5)
  ├─ Madgwick AHRS 6 轴 (β_fast=2.0 → β_normal=0.04)
  ├─ 四元数 → 欧拉角 (ZYX)
  ├─ Jerk 计算 (|da/dt|)
  └─ 步态状态机
       SWING ──触地(jerk>50 || |a|>14.7)──► STANCE
       STANCE ──离地(|ω|>100 && |a|<11)──► SWING
```

## 多节点部署

本项目设计为 **5 节点复用系统**，同一固件烧录所有板子:

| 节点 ID | 通过 UDP 远程配置 |
|:---:|:---|
| 1~5 | `python tools/udp_receiver.py --set-id <IP> <ID>` |

身体位置映射由接收端模板决定，节点本身是通用硬件。

## 相关项目

- [smartKeep_P4](https://github.com/dangerous-nagisa/smartKeep_P4) — ESP32-P4 网关 (UDP → MQTT)

## 许可证

MIT
