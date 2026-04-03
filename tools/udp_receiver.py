#!/usr/bin/env python3
"""
smartKeep 5节点 UDP 接收器

功能:
  - 仪表盘模式 (默认): 实时显示所有节点状态
  - 原始流模式 (--raw): 逐包打印
  - 远程配置: --set-id <ip> <id> 修改节点 ID
  - 多种佩戴模板: --template <name>
  - CSV 日志: --log

用法:
    python udp_receiver.py                    # 仪表盘
    python udp_receiver.py --raw              # 调试流
    python udp_receiver.py --template gait    # 步态模板
    python udp_receiver.py --set-id 192.168.1.100 3  # 设置节点ID
    python udp_receiver.py --log              # 保存CSV
"""

import socket
import struct
import argparse
import time
import sys
import os
import select
from datetime import datetime
from collections import defaultdict

# =============================================================================
# 佩戴模板 — 节点 ID → 身体位置映射
# =============================================================================

TEMPLATES = {
    # 下肢步态分析
    'gait': {
        1: ('腰部',   'WAIST'),
        2: ('左脚',   'L_FOOT'),
        3: ('右脚',   'R_FOOT'),
        4: ('左大腿', 'L_THIGH'),
        5: ('右大腿', 'R_THIGH'),
    },
    # 坐站转移测试
    'sit2stand': {
        1: ('腰部',   'WAIST'),
        2: ('左大腿', 'L_THIGH'),
        3: ('右大腿', 'R_THIGH'),
        4: ('左小腿', 'L_SHANK'),
        5: ('右小腿', 'R_SHANK'),
    },
    # 上肢评估
    'upper': {
        1: ('胸部',   'CHEST'),
        2: ('左上臂', 'L_ARM'),
        3: ('右上臂', 'R_ARM'),
        4: ('左前臂', 'L_FOREARM'),
        5: ('右前臂', 'R_FOREARM'),
    },
    # 通用 (仅显示 ID)
    'generic': {
        1: ('节点1', 'NODE_1'),
        2: ('节点2', 'NODE_2'),
        3: ('节点3', 'NODE_3'),
        4: ('节点4', 'NODE_4'),
        5: ('节点5', 'NODE_5'),
    },
}

# =============================================================================
# node_packet_t 解析 (44 字节)
# =============================================================================

PKT_FMT  = '<BBHIffffffffBBxx'
PKT_SIZE = struct.calcsize(PKT_FMT)  # 44

FIELD_NAMES = [
    'node_id', 'battery', 'seq', 'timestamp',
    'roll', 'pitch', 'yaw',
    'ax', 'ay', 'az',
    'gyro_norm', 'jerk',
    'step_flag', 'crc8'
]

# CRC-8 (poly=0x07, init=0x00)
def crc8_calc(data: bytes) -> int:
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) if (crc & 0x80) else (crc << 1)
            crc &= 0xFF
    return crc

def parse_packet(data: bytes) -> dict:
    if len(data) != PKT_SIZE:
        return None
    fields = struct.unpack(PKT_FMT, data)
    pkt = dict(zip(FIELD_NAMES, fields))
    pkt['crc_ok'] = (pkt['crc8'] == crc8_calc(data[:41]))
    return pkt

# =============================================================================
# 节点状态跟踪
# =============================================================================

class NodeStats:
    def __init__(self, node_id):
        self.node_id = node_id
        self.last_pkt = None
        self.last_time = 0
        self.pkt_count = 0
        self.loss_count = 0
        self.last_seq = -1
        self.rate_window = []  # (time, count) 用于计算 pkt/s

    def update(self, pkt):
        now = time.time()
        self.last_pkt = pkt
        self.last_time = now
        self.pkt_count += 1

        # 丢包检测
        if self.last_seq >= 0:
            expected = (self.last_seq + 1) & 0xFFFF
            if pkt['seq'] != expected:
                gap = (pkt['seq'] - self.last_seq) & 0xFFFF
                self.loss_count += max(0, gap - 1)
        self.last_seq = pkt['seq']

        # 速率统计
        self.rate_window.append((now, 1))
        # 保留最近 2 秒
        self.rate_window = [(t, c) for t, c in self.rate_window if now - t < 2.0]

    @property
    def online(self):
        return time.time() - self.last_time < 2.0

    @property
    def loss_rate(self):
        total = self.pkt_count + self.loss_count
        return (self.loss_count / total * 100) if total > 0 else 0

    @property
    def pkt_rate(self):
        if len(self.rate_window) < 2:
            return 0
        t0, t1 = self.rate_window[0][0], self.rate_window[-1][0]
        dt = t1 - t0
        return len(self.rate_window) / dt if dt > 0.1 else 0

# =============================================================================
# 仪表盘显示
# =============================================================================

def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')

STEP_SYMBOLS = {0: '  ', 1: 'HS', 2: 'TO'}

def draw_dashboard(nodes: dict, template: dict, elapsed: float):
    clear_screen()

    # 标题
    print("╔" + "═" * 78 + "╗")
    print(f"║  smartKeep 5节点 UDP 接收器                                 运行: {elapsed:6.0f}s  ║")
    print("╠" + "═" * 78 + "╣")

    # 表头
    print(f"║ {'ID':>2} │ {'位置':<8} │ {'状态':<6} │ {'seq':>5} │ {'Roll':>6} │ {'Pitch':>6} │ {'Yaw':>6} │ {'丢包':>5} │{'pkt/s':>5}║")
    print("╠" + "═" * 78 + "╣")

    # 节点行
    for nid in range(1, 6):
        loc_cn, loc_en = template.get(nid, (f'节点{nid}', f'NODE_{nid}'))
        stats = nodes.get(nid)

        if stats and stats.online and stats.last_pkt:
            p = stats.last_pkt
            status = "● 在线"
            seq_s = f"{p['seq']:>5}"
            roll_s = f"{p['roll']:>+6.1f}"
            pitch_s = f"{p['pitch']:>+6.1f}"
            yaw_s = f"{p['yaw']:>6.1f}"
            loss_s = f"{stats.loss_rate:>4.1f}%"
            rate_s = f"{stats.pkt_rate:>5.1f}"
        else:
            status = "○ 离线"
            seq_s = roll_s = pitch_s = yaw_s = "    -"
            loss_s = "    -"
            rate_s = "    -"

        print(f"║ {nid:>2} │ {loc_en:<8} │ {status:<6} │ {seq_s} │ {roll_s} │ {pitch_s} │ {yaw_s} │ {loss_s} │{rate_s}║")

    print("╚" + "═" * 78 + "╝")

    # 总计
    total_pkt = sum(s.pkt_count for s in nodes.values())
    total_loss = sum(s.loss_count for s in nodes.values())
    online_count = sum(1 for s in nodes.values() if s.online)
    print(f"\n  在线: {online_count}/5  总包数: {total_pkt}  丢失: {total_loss}")
    print("  按 Ctrl+C 退出")

# =============================================================================
# 原始流打印
# =============================================================================

def print_raw_header():
    print("\n" + "=" * 95)
    print("  smartKeep UDP 接收器 (原始流模式)")
    print("=" * 95)
    print(f"  {'ID':>2}  {'seq':>5}  {'Roll':>7}  {'Pitch':>7}  {'Yaw':>7}  "
          f"{'ax':>6}  {'ay':>6}  {'az':>6}  |ω|   Jerk  gait CRC")
    print("-" * 95)

STEP_LABELS = {0: '  -', 1: ' HS', 2: ' TO'}

def print_raw_packet(pkt: dict):
    crc_mark = "OK" if pkt['crc_ok'] else "ERR"
    step = STEP_LABELS.get(pkt['step_flag'], '???')

    prefix = "  "
    if pkt['step_flag'] == 1:
        prefix = ">>"
    elif pkt['step_flag'] == 2:
        prefix = "<<"

    print(f"{prefix}{pkt['node_id']:>2}  {pkt['seq']:>5}  "
          f"{pkt['roll']:>+7.1f}  {pkt['pitch']:>+7.1f}  {pkt['yaw']:>7.1f}  "
          f"{pkt['ax']:>+6.1f}  {pkt['ay']:>+6.1f}  {pkt['az']:>+6.1f}  "
          f"{pkt['gyro_norm']:>5.1f}  {pkt['jerk']:>5.1f}  "
          f"{step}  {crc_mark}")

# =============================================================================
# 远程配置
# =============================================================================

CFG_PORT = 8889
CFG_MAGIC = b'SKCF'
CFG_ACK_MAGIC = b'SKCK'
CFG_CMD_SET_ID = 0x01
CFG_CMD_GET_ID = 0x02

def remote_set_id(ip: str, new_id: int) -> bool:
    """发送 SET_ID 指令到指定节点"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(2.0)

    pkt = CFG_MAGIC + bytes([CFG_CMD_SET_ID, new_id])
    try:
        sock.sendto(pkt, (ip, CFG_PORT))
        data, _ = sock.recvfrom(16)

        if len(data) >= 6 and data[:4] == CFG_ACK_MAGIC:
            cmd, val = data[4], data[5]
            if cmd == CFG_CMD_SET_ID and val == new_id:
                print(f"✓ 成功: {ip} 节点 ID 已设置为 {new_id}")
                return True
            elif val == 0xFF:
                print(f"✗ 失败: 无效 ID {new_id}")
                return False
    except socket.timeout:
        print(f"✗ 超时: {ip} 无响应 (确保节点已连接同一网络)")
    except Exception as e:
        print(f"✗ 错误: {e}")
    finally:
        sock.close()
    return False

def remote_get_id(ip: str) -> int:
    """查询节点当前 ID"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(2.0)

    pkt = CFG_MAGIC + bytes([CFG_CMD_GET_ID, 0])
    try:
        sock.sendto(pkt, (ip, CFG_PORT))
        data, _ = sock.recvfrom(16)

        if len(data) >= 6 and data[:4] == CFG_ACK_MAGIC:
            return data[5]
    except:
        pass
    finally:
        sock.close()
    return -1

# =============================================================================
# 主程序
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description='smartKeep 5节点 UDP 接收器')
    parser.add_argument('--port', type=int, default=8888, help='数据监听端口 (默认 8888)')
    parser.add_argument('--raw', action='store_true', help='原始流模式 (逐包打印)')
    parser.add_argument('--log', action='store_true', help='保存 CSV 日志')
    parser.add_argument('--template', '-t', choices=list(TEMPLATES.keys()),
                        default='gait', help='佩戴模板 (默认 gait)')
    parser.add_argument('--set-id', nargs=2, metavar=('IP', 'ID'),
                        help='远程设置节点 ID (如: --set-id 192.168.1.100 3)')
    parser.add_argument('--get-id', metavar='IP',
                        help='查询节点当前 ID')
    args = parser.parse_args()

    # 远程配置模式
    if args.set_id:
        ip, nid = args.set_id[0], int(args.set_id[1])
        remote_set_id(ip, nid)
        return

    if args.get_id:
        nid = remote_get_id(args.get_id)
        if nid > 0:
            print(f"节点 {args.get_id} 当前 ID: {nid}")
        else:
            print(f"无法获取 {args.get_id} 的 ID")
        return

    # 选择模板
    template = TEMPLATES[args.template]

    # 创建 socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('0.0.0.0', args.port))
    sock.setblocking(False)

    # CSV 日志
    csv_file = None
    if args.log:
        filename = f"smartkeep_{datetime.now():%Y%m%d_%H%M%S}.csv"
        csv_file = open(filename, 'w')
        csv_file.write(','.join(FIELD_NAMES + ['crc_ok', 'recv_time']) + '\n')
        print(f"日志保存到: {filename}")

    # 节点状态
    nodes = {}
    t_start = time.time()
    last_dashboard = 0

    if args.raw:
        print_raw_header()
        print(f"  监听 0.0.0.0:{args.port} ...")
        print(f"  模板: {args.template}")
        print()

    try:
        while True:
            # 非阻塞接收
            readable, _, _ = select.select([sock], [], [], 0.1)

            for s in readable:
                data, addr = s.recvfrom(128)

                if len(data) != PKT_SIZE:
                    continue

                pkt = parse_packet(data)
                if pkt is None:
                    continue

                nid = pkt['node_id']
                if nid not in nodes:
                    nodes[nid] = NodeStats(nid)
                nodes[nid].update(pkt)

                # 原始流打印
                if args.raw:
                    print_raw_packet(pkt)

                # CSV
                if csv_file:
                    vals = [str(pkt[k]) for k in FIELD_NAMES]
                    vals.append(str(pkt['crc_ok']))
                    vals.append(f"{time.time():.3f}")
                    csv_file.write(','.join(vals) + '\n')

            # 仪表盘刷新 (200ms)
            if not args.raw:
                now = time.time()
                if now - last_dashboard > 0.2:
                    draw_dashboard(nodes, template, now - t_start)
                    last_dashboard = now
                    if csv_file:
                        csv_file.flush()

    except KeyboardInterrupt:
        elapsed = time.time() - t_start
        total_pkt = sum(s.pkt_count for s in nodes.values())
        print(f"\n\n停止接收.")
        print(f"总计: {total_pkt} 包 | {elapsed:.1f}s | {total_pkt/elapsed:.1f} pkt/s")
    finally:
        sock.close()
        if csv_file:
            csv_file.close()

if __name__ == '__main__':
    main()
