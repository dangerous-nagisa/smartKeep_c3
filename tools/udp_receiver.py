#!/usr/bin/env python3
"""
smartKeep UDP 数据接收器

监听 UDP 8888 端口，解析并打印 node_packet_t (44 字节)

用法:
    python udp_receiver.py              # 默认监听 0.0.0.0:8888
    python udp_receiver.py --port 9999  # 自定义端口
    python udp_receiver.py --log        # 同时保存 CSV 日志
"""

import socket
import struct
import argparse
import time
import sys
import os
from datetime import datetime

# =============================================================================
# node_packet_t 解析定义 (44 字节, 小端序)
#
# 内存布局:
#   offset  0: uint8_t  node_id     (B)
#   offset  1: uint8_t  battery     (B)
#   offset  2: uint16_t seq         (H)
#   offset  4: uint32_t timestamp   (I)
#   offset  8: float    euler[3]    (fff)   roll/pitch/yaw
#   offset 20: float    accel[3]    (fff)   ax/ay/az
#   offset 32: float    gyro_norm   (f)
#   offset 36: float    jerk        (f)
#   offset 40: uint8_t  step_flag   (B)
#   offset 41: uint8_t  crc8        (B)
#   offset 42: 2 bytes padding      (xx)
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

# CRC-8 校验 (poly=0x07, init=0x00) —— 与固件一致
def crc8_calc(data: bytes) -> int:
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) if (crc & 0x80) else (crc << 1)
            crc &= 0xFF
    return crc


def parse_packet(data: bytes) -> dict:
    """解析 44 字节为字段字典，校验 CRC"""
    if len(data) != PKT_SIZE:
        return None

    fields = struct.unpack(PKT_FMT, data)
    pkt = dict(zip(FIELD_NAMES, fields))

    # CRC 校验: crc8 在 offset 41, 校验范围 offset 0~40
    expected_crc = crc8_calc(data[:41])
    pkt['crc_ok'] = (pkt['crc8'] == expected_crc)

    return pkt


def print_header():
    print()
    print("=" * 90)
    print("  smartKeep UDP 接收器")
    print("=" * 90)
    print(f"  {'seq':>6s}  {'Roll':>7s}  {'Pitch':>7s}  {'Yaw':>7s}  "
          f"{'ax':>6s}  {'ay':>6s}  {'az':>6s}  "
          f"|ω|   {'Jerk':>5s}  gait CRC")
    print("-" * 90)


STEP_LABELS = {0: '  -', 1: ' HS', 2: ' TO'}  # HS=Heel Strike, TO=Toe Off


def print_packet(pkt: dict):
    crc_mark = "OK" if pkt['crc_ok'] else "ERR"

    step = STEP_LABELS.get(pkt['step_flag'], '???')
    # 步态事件高亮
    prefix = "  "
    if pkt['step_flag'] == 1:
        prefix = ">>"
    elif pkt['step_flag'] == 2:
        prefix = "<<"

    print(f"{prefix}{pkt['seq']:>5d}  "
          f"{pkt['roll']:>+7.1f}  {pkt['pitch']:>+7.1f}  {pkt['yaw']:>7.1f}  "
          f"{pkt['ax']:>+6.1f}  {pkt['ay']:>+6.1f}  {pkt['az']:>+6.1f}  "
          f"{pkt['gyro_norm']:>5.1f}  {pkt['jerk']:>5.1f}  "
          f"{step}  {crc_mark}")


def main():
    parser = argparse.ArgumentParser(description='smartKeep UDP 数据接收器')
    parser.add_argument('--port', type=int, default=8888, help='监听端口 (默认 8888)')
    parser.add_argument('--log', action='store_true', help='保存 CSV 日志文件')
    args = parser.parse_args()

    # 创建 UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('0.0.0.0', args.port))

    print_header()
    print(f"  监听 0.0.0.0:{args.port} ...")
    print()

    # CSV 日志
    csv_file = None
    if args.log:
        filename = f"smartkeep_{datetime.now():%Y%m%d_%H%M%S}.csv"
        csv_file = open(filename, 'w')
        csv_file.write(','.join(FIELD_NAMES + ['crc_ok', 'recv_time']) + '\n')
        print(f"  日志保存到: {filename}")
        print()

    pkt_count = 0
    err_count = 0
    t_start = time.time()

    try:
        while True:
            data, addr = sock.recvfrom(128)

            if len(data) != PKT_SIZE:
                err_count += 1
                continue

            pkt = parse_packet(data)
            if pkt is None:
                err_count += 1
                continue

            pkt_count += 1

            # 每 50 包重新打印表头 (方便翻看)
            if pkt_count % 50 == 1 and pkt_count > 1:
                elapsed = time.time() - t_start
                rate = pkt_count / elapsed if elapsed > 0 else 0
                print(f"\n  --- {pkt_count} 包, {rate:.1f} pkt/s, "
                      f"丢包: {err_count}, 来源: {addr[0]} ---")
                print(f"  {'seq':>6s}  {'Roll':>7s}  {'Pitch':>7s}  {'Yaw':>7s}  "
                      f"{'ax':>6s}  {'ay':>6s}  {'az':>6s}  "
                      f"|ω|   {'Jerk':>5s}  stp  CRC")
                print("-" * 90)

            print_packet(pkt)

            # 写 CSV
            if csv_file:
                vals = [str(pkt[k]) for k in FIELD_NAMES]
                vals.append(str(pkt['crc_ok']))
                vals.append(f"{time.time():.3f}")
                csv_file.write(','.join(vals) + '\n')
                if pkt_count % 50 == 0:
                    csv_file.flush()

    except KeyboardInterrupt:
        elapsed = time.time() - t_start
        rate = pkt_count / elapsed if elapsed > 0 else 0
        print(f"\n\n  停止接收.")
        print(f"  总计: {pkt_count} 包 | {elapsed:.1f}s | {rate:.1f} pkt/s | 错误: {err_count}")
    finally:
        sock.close()
        if csv_file:
            csv_file.close()


if __name__ == '__main__':
    main()
