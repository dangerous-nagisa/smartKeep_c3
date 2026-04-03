#!/usr/bin/env python3
"""
smartKeep MQTT 订阅器 — 接收 P4 网关转发的二进制帧

帧类型:
  0x01 data   (194B, 50Hz)  — 5 节点聚合数据
  0x02 event  (42B, 触发式)  — 步态事件
  0x03 status (22B, 1Hz)    — 网关心跳

Topic 结构:
  smartkeep/{device_id}/{session_id}/data
  smartkeep/{device_id}/{session_id}/event
  smartkeep/{device_id}/status

用法:
  pip install paho-mqtt
  python mqtt_subscriber.py                    # 仪表盘
  python mqtt_subscriber.py --raw              # 原始帧打印
  python mqtt_subscriber.py --log              # CSV 日志
  python mqtt_subscriber.py --broker <uri>     # 自定义 broker
"""

import struct
import argparse
import time
import os
import sys
from datetime import datetime

try:
    import paho.mqtt.client as mqtt
except ImportError:
    print("需要安装 paho-mqtt: pip install paho-mqtt")
    sys.exit(1)

# =============================================================================
# 协议常量
# =============================================================================

FRAME_TYPE_DATA   = 0x01
FRAME_TYPE_EVENT  = 0x02
FRAME_TYPE_STATUS = 0x03

TASK_NAMES = {
    0x00: 'IDLE',
    0x01: 'STS',
    0x02: 'TUG',
    0x03: '10MWT',
    0x04: 'BALANCE',
    0x05: 'FMA',
}

EVENT_NAMES = {
    0x01: 'STEP_ON',
    0x02: 'STEP_OFF',
    0x11: 'STS_START',
    0x12: 'STS_PEAK',
    0x13: 'STS_END',
    0x21: 'TUG_TURN_START',
    0x22: 'TUG_TURN_END',
    0x31: 'BALANCE_FALL',
    0x41: 'JERK_PEAK',
}

STEP_LABELS = {0: '-', 1: 'HS', 2: 'TO'}

# =============================================================================
# 帧解析
# =============================================================================

# node_data_t (36B)
NODE_DATA_FMT = '<ffffffBBH'  # 6 float + 2 uint8 + 1 uint16 = 36
NODE_DATA_SIZE = struct.calcsize(NODE_DATA_FMT)

# mqtt_data_frame_t header (12B) + 5 * node_data_t (180B) + crc16 (2B) = 194
DATA_HEADER_FMT = '<BBHIHBB'   # frame_type, node_mask, frame_seq, frame_ts, session_id, task_id, reserved
DATA_HEADER_SIZE = struct.calcsize(DATA_HEADER_FMT)

# mqtt_event_frame_t (41B packed)
EVENT_FMT = '<BBHIHBBffffffB'
EVENT_SIZE = 41

# mqtt_status_frame_t (23B packed)
STATUS_FMT = '<BBHI5B5bBBBBB'
STATUS_SIZE = 23


def parse_data_frame(data: bytes) -> dict:
    """解析 194 字节数据帧"""
    if len(data) != 194:
        return None

    hdr = struct.unpack_from(DATA_HEADER_FMT, data, 0)
    frame = {
        'frame_type': hdr[0],
        'node_mask': hdr[1],
        'frame_seq': hdr[2],
        'frame_ts': hdr[3],
        'session_id': hdr[4],
        'task_id': hdr[5],
        'nodes': [],
    }

    offset = DATA_HEADER_SIZE
    for i in range(5):
        nd = struct.unpack_from(NODE_DATA_FMT, data, offset)
        node = {
            'euler': (nd[0], nd[1], nd[2]),
            'accel': (nd[3], nd[4], nd[5]),
            'gyro_norm': nd[6],
            'jerk': nd[7],
            'step_flag': nd[8],
            'rssi': nd[9],
            'seq': nd[10],
            'valid': bool(frame['node_mask'] & (1 << i)),
        }
        frame['nodes'].append(node)
        offset += NODE_DATA_SIZE

    frame['crc16'] = struct.unpack_from('<H', data, offset)[0]
    return frame


def parse_event_frame(data: bytes) -> dict:
    """解析 42 字节事件帧"""
    if len(data) != 41:
        return None

    vals = struct.unpack(EVENT_FMT, data)
    return {
        'frame_type': vals[0],
        'node_id': vals[1],
        'frame_seq': vals[2],
        'event_ts': vals[3],
        'session_id': vals[4],
        'task_id': vals[5],
        'event_type': vals[6],
        'event_value': vals[7],
        'euler_snap': (vals[8], vals[9], vals[10]),
        'accel_snap': (vals[11], vals[12], vals[13]),
        'crc8': vals[14],
    }


def parse_status_frame(data: bytes) -> dict:
    """解析 22 字节状态帧"""
    if len(data) != 23:
        return None

    vals = struct.unpack(STATUS_FMT, data)
    return {
        'frame_type': vals[0],
        'node_mask_online': vals[1],
        'session_id': vals[2],
        'uptime_s': vals[3],
        'battery': vals[4:9],
        'rssi': vals[9:14],
        'p4_battery': vals[14],
        'wifi_rssi': vals[15],
        'mqtt_reconnect_cnt': vals[16],
        'task_id': vals[17],
        'crc8': vals[18],
    }

# =============================================================================
# 显示
# =============================================================================

def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')


class Dashboard:
    def __init__(self):
        self.last_data = None
        self.last_status = None
        self.events = []      # 最近 10 条事件
        self.data_count = 0
        self.event_count = 0
        self.t_start = time.time()
        self.last_draw = 0

    def update_data(self, frame):
        self.last_data = frame
        self.data_count += 1

    def update_event(self, frame):
        self.events.append(frame)
        if len(self.events) > 10:
            self.events.pop(0)
        self.event_count += 1

    def update_status(self, frame):
        self.last_status = frame

    def draw(self):
        now = time.time()
        if now - self.last_draw < 0.2:
            return
        self.last_draw = now
        elapsed = now - self.t_start

        clear_screen()
        d = self.last_data
        s = self.last_status

        print("╔" + "═" * 78 + "╗")
        task_name = TASK_NAMES.get(d['task_id'], '?') if d else '---'
        print(f"║  smartKeep MQTT 订阅器                  任务: {task_name:<8}    运行: {elapsed:6.0f}s  ║")
        print("╠" + "═" * 78 + "╣")
        print(f"║ {'节点':>4} │ {'状态':<6} │ {'seq':>5} │ {'Roll':>7} │ {'Pitch':>7} │ {'Yaw':>7} │ {'|ω|':>5} │ {'Jerk':>5} │ {'步态':>4} ║")
        print("╠" + "═" * 78 + "╣")

        if d:
            for i in range(5):
                n = d['nodes'][i]
                if n['valid']:
                    step = STEP_LABELS.get(n['step_flag'], '?')
                    print(f"║  {i+1:>2}  │ ● 在线 │ {n['seq']:>5} │ {n['euler'][0]:>+7.1f} │ "
                          f"{n['euler'][1]:>+7.1f} │ {n['euler'][2]:>7.1f} │ {n['gyro_norm']:>5.1f} │ "
                          f"{n['jerk']:>5.1f} │  {step:>2}  ║")
                else:
                    print(f"║  {i+1:>2}  │ ○ 离线 │     - │       - │       - │       - │     - │     - │   -  ║")
        else:
            for i in range(5):
                print(f"║  {i+1:>2}  │ ○ 等待 │     - │       - │       - │       - │     - │     - │   -  ║")

        print("╚" + "═" * 78 + "╝")

        # 状态信息
        if s:
            online = bin(s['node_mask_online']).count('1')
            print(f"\n  网关运行: {s['uptime_s']}s | 在线: {online}/5 | "
                  f"MQTT重连: {s['mqtt_reconnect_cnt']}")

        # 最近事件
        if self.events:
            print(f"\n  最近事件 ({self.event_count} 总计):")
            for e in self.events[-5:]:
                evt_name = EVENT_NAMES.get(e['event_type'], f"0x{e['event_type']:02x}")
                print(f"    [{e['event_ts']:>8}ms] 节点{e['node_id']} {evt_name} "
                      f"val={e['event_value']:.1f}")

        print(f"\n  数据帧: {self.data_count} | 事件帧: {self.event_count}")
        print("  按 Ctrl+C 退出")


def print_raw_data(frame):
    mask_str = f"{frame['node_mask']:05b}"
    print(f"[DATA] seq={frame['frame_seq']:>5} ts={frame['frame_ts']:>8} "
          f"mask={mask_str} task={TASK_NAMES.get(frame['task_id'], '?')}")
    for i, n in enumerate(frame['nodes']):
        if n['valid']:
            step = STEP_LABELS.get(n['step_flag'], '?')
            print(f"  N{i+1}: R={n['euler'][0]:+.1f} P={n['euler'][1]:+.1f} "
                  f"Y={n['euler'][2]:.1f} |ω|={n['gyro_norm']:.1f} "
                  f"J={n['jerk']:.1f} [{step}]")


def print_raw_event(frame):
    evt = EVENT_NAMES.get(frame['event_type'], f"0x{frame['event_type']:02x}")
    print(f"[EVENT] node={frame['node_id']} {evt} val={frame['event_value']:.2f} "
          f"ts={frame['event_ts']}ms")


def print_raw_status(frame):
    online = bin(frame['node_mask_online']).count('1')
    print(f"[STATUS] online={online}/5 uptime={frame['uptime_s']}s "
          f"mqtt_reconn={frame['mqtt_reconnect_cnt']} "
          f"task={TASK_NAMES.get(frame['task_id'], '?')}")

# =============================================================================
# MQTT 回调
# =============================================================================

dashboard = None
raw_mode = False
csv_file = None


def on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        print("MQTT 已连接")
        client.subscribe("smartkeep/#")
        print("已订阅 smartkeep/#")
    else:
        print(f"MQTT 连接失败, rc={rc}")


def on_message(client, userdata, msg):
    global dashboard, csv_file

    data = msg.payload
    if len(data) < 1:
        return

    frame_type = data[0]

    if frame_type == FRAME_TYPE_DATA and len(data) == 194:
        frame = parse_data_frame(data)
        if frame is None:
            return
        if raw_mode:
            print_raw_data(frame)
        elif dashboard:
            dashboard.update_data(frame)
            dashboard.draw()

        # CSV 日志
        if csv_file:
            for i, n in enumerate(frame['nodes']):
                if n['valid']:
                    csv_file.write(f"{frame['frame_seq']},{frame['frame_ts']},"
                                   f"{i+1},{n['seq']},"
                                   f"{n['euler'][0]:.3f},{n['euler'][1]:.3f},{n['euler'][2]:.3f},"
                                   f"{n['accel'][0]:.3f},{n['accel'][1]:.3f},{n['accel'][2]:.3f},"
                                   f"{n['gyro_norm']:.3f},{n['jerk']:.3f},"
                                   f"{n['step_flag']},{time.time():.3f}\n")

    elif frame_type == FRAME_TYPE_EVENT and len(data) == 41:
        frame = parse_event_frame(data)
        if frame is None:
            return
        if raw_mode:
            print_raw_event(frame)
        elif dashboard:
            dashboard.update_event(frame)
            dashboard.draw()

    elif frame_type == FRAME_TYPE_STATUS and len(data) == 23:
        frame = parse_status_frame(data)
        if frame is None:
            return
        if raw_mode:
            print_raw_status(frame)
        elif dashboard:
            dashboard.update_status(frame)

# =============================================================================
# 主程序
# =============================================================================

def main():
    global dashboard, raw_mode, csv_file

    parser = argparse.ArgumentParser(description='smartKeep MQTT 订阅器')
    parser.add_argument('--broker', default='o1bc1a32.ala.cn-hangzhou.emqxsl.cn',
                        help='MQTT broker 地址')
    parser.add_argument('--port', type=int, default=8883, help='MQTT 端口 (TLS)')
    parser.add_argument('--username', default='djrs', help='MQTT 用户名')
    parser.add_argument('--password', default='111111', help='MQTT 密码')
    parser.add_argument('--raw', action='store_true', help='原始帧打印模式')
    parser.add_argument('--log', action='store_true', help='保存 CSV 日志')
    args = parser.parse_args()

    raw_mode = args.raw

    if not raw_mode:
        dashboard = Dashboard()

    if args.log:
        filename = f"smartkeep_mqtt_{datetime.now():%Y%m%d_%H%M%S}.csv"
        csv_file = open(filename, 'w')
        csv_file.write("frame_seq,frame_ts,node_id,node_seq,"
                        "roll,pitch,yaw,ax,ay,az,"
                        "gyro_norm,jerk,step_flag,recv_time\n")
        print(f"日志保存到: {filename}")

    # MQTT 客户端
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.username_pw_set(args.username, args.password)
    client.tls_set()  # 使用系统 CA 证书验证
    client.on_connect = on_connect
    client.on_message = on_message

    print(f"连接到 {args.broker}:{args.port} (TLS) ...")
    try:
        client.connect(args.broker, args.port, 60)
    except Exception as e:
        print(f"连接失败: {e}")
        return

    try:
        client.loop_forever()
    except KeyboardInterrupt:
        print("\n\n停止订阅.")
        if dashboard:
            print(f"总计: 数据帧 {dashboard.data_count} | 事件帧 {dashboard.event_count}")
    finally:
        client.disconnect()
        if csv_file:
            csv_file.close()


if __name__ == '__main__':
    main()
