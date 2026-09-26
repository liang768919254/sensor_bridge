#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_bridge_logic.py —— 不碰真串口的桥节点逻辑自测（Step 2 / Step 6 双版本配套）

【为什么能这么测】
  serial_bridge_node 唯一的外部依赖是 `serial.Serial`。
  把它换成假的，节点的「字节流 → ROS 消息」这段逻辑就能在几秒内被完整验证：
  不需要 socat、不需要真板子、不需要另一个终端。

  这正是把「纯逻辑」从「IO + 框架」里剥出来之后拿到的收益。
  等你上真板子时，这段逻辑已经被证明是对的 → 排查范围直接砍一半。

【⚠️ 两个实现版本，必须选一个来测】（2026-09-22 修订）
  同一个 ROS 入口名 `serial_bridge` 底下，历史上存在两份实现，行为不一样：

  ┌─────────────┬──────────────────────────┬────────────────────────────┐
  │             │ v1（最小闭环）            │ hardened（加固版/默认）      │
  ├─────────────┼──────────────────────────┼────────────────────────────┤
  │ 文件         │ serial_bridge_node_v1.py │ serial_bridge_node.py      │
  │ 发布时机     │ 收到一帧就 publish        │ 定时器按 publish_rate 发布  │
  │ v2 四字段    │ float() 失败 → 丢弃       │ ✅ 解析，取 fields[1]        │
  │ 600 字节超长 │ float() → inf，照发 ✗     │ parse_line 拦掉 → 坏帧 ✓    │
  │ 断线重连     │ 无                        │ 有                         │
  └─────────────┴──────────────────────────┴────────────────────────────┘

  本脚本默认测 `hardened`（因为 ~/ros_study 里现在就是它），
  想复现 Step 2 的原始现象（含 `inf` 那条已知局限）请用 `--impl v1`。

运行：
  cd ~/ros_study
  source /opt/ros/humble/setup.bash && source install/setup.bash
  /usr/bin/python3 "…/08_阶段二_STM32串口接入/无硬件自测/test_bridge_logic.py"
  /usr/bin/python3 "…/test_bridge_logic.py" --impl v1        # 复现 Step 2 现象
"""

import argparse
import os
import sys
import time
from unittest import mock

import rclpy
import serial

# 直接测源码，不用先 colcon build
SRC = os.path.expanduser('~/ros_study/src/sensor_bridge')
if SRC not in sys.path:
    sys.path.insert(0, SRC)


# ── 喂给节点的「样本行」───────────────────────────────────────
#   每项: (原始字节, 期望解析出的值 / None=丢弃, 说明, 是否已知局限)
#
#   ⚠️ 倒数第二项是刻意保留的历史样本：600 字节的超长行（'9'×600）会被
#      float() 成功解析成 inf —— Python 的 float() 对超出双精度范围的数字串
#      不报错，直接返回 inf。v1 最小版没有长度闸门，inf 被当成合法数据发出去。
#      加固版用 serial_protocol.parse_line() 的 max_len=256 在 decode 之前挡掉了。
V1_CASES = [
    (b'STM32 serial bridge ready | proto=v2 | baud=115200\r\n', None, '上电横幅 → 丢弃', False),
    (b'\x1f\x9c\x03\x8a\r\n',                                    None, '乱码字节 → 丢弃', False),
    (b'\r\n',                                                    None, '空行 → 丢弃', False),
    (b'\n',                                                      None, '只有换行 → 丢弃', False),
    (b'1234\n',                                                1234.0, 'v1 整数值', False),
    (b'12.5\r\n',                                               12.5, '带 \\r\\n 的小数', False),
    (b'  -0.5  \n',                                             -0.5, '前后空格 + 负数', False),
    (b'1,2,3,4\n',                                               None, 'v2 四字段 → v1 版只认单值，丢弃', False),
    (b'4095\n',                                               4095.0, 'ADC 上限', False),
    (b'0\n',                                                      0.0, 'ADC 零点（0.0 是 falsy，别用 if value 判断）', False),
    (b'9' * 600 + b'\n',                                    float('inf'),
     '已知局限（仅 v1）：无长度闸门 → float() 得到 inf 并照发', True),
    (b'2560\n',                                                2560.0, '尾帧', False),
]

# 加固版用的流：v2 四字段是「能吃」的，超长行是「能挡」的，所以期望值不同
HARDENED_CASES = [
    (b'STM32 serial bridge ready | proto=v2 | baud=115200\r\n', None, '上电横幅 → 坏帧', False),
    (b'\x1f\x9c\x03\x8a\r\n',                                    None, '乱码字节 → 坏帧', False),
    (b'\r\n',                                                    None, '空行 → 坏帧', False),
    (b'\n',                                                      None, '只有换行 → 坏帧', False),
    (b'1234\n',                                                1234.0, 'v1 整数值', False),
    (b'12.5\r\n',                                               12.5, '带 \\r\\n 的小数', False),
    (b'  -0.5  \n',                                             -0.5, '前后空格 + 负数', False),
    (b'214,2768,3434,2468\n',                                 2768.0, 'v2 四字段 → ✅ 解析，取 fields[1]', False),
    (b'4095\n',                                               4095.0, 'ADC 上限', False),
    (b'0\n',                                                      0.0, 'ADC 零点', False),
    (b'9' * 600 + b'\n',                                          None, '超长行 → ✅ 被 max_len=256 挡住，不再产生 inf', False),
    (b'2560\n',                                                2560.0, '尾帧', False),
]


class FakeSerial:
    """假串口：按顺序吐预置行，吐完就一直『超时』返回 b''"""

    def __init__(self, port, baud, timeout=None, stream=None):
        self.port = port
        self.baud = baud
        self.timeout = timeout          # 顺便断言调用方真的传了 timeout
        self.stream = list(stream or [])
        self.i = 0
        self.closed = False

    def readline(self):
        if self.i < len(self.stream):
            raw = self.stream[self.i]
            self.i += 1
            return raw
        time.sleep(0.01)                # 模拟 timeout 等待，别让循环空转
        return b''

    def close(self):
        self.closed = True


class FakePub:
    def __init__(self):
        self.got = []

    def publish(self, msg):
        # 兜底：只收 Float32，别把别的消息类型混进来
        if hasattr(msg, 'data'):
            self.got.append(msg.data)


def _patch_and_build(module, stream, pub, holder):
    """替换 serial.Serial 与 /sensor/value 发布者，返回建好的节点"""
    def fake_serial_ctor(port, baud, **kw):
        holder['ser'] = FakeSerial(port, baud, **kw, stream=stream)
        return holder['ser']

    real_create_publisher = rclpy.node.Node.create_publisher

    # 只替换 /sensor/value 这一个发布者；节点内部还有 parameter_event 等
    # 其它发布者，必须放行给原始实现，否则会被误当成数据收集进来。
    def fake_create_publisher(self, msg_type, topic, *a, **k):
        if topic == '/sensor/value':
            return pub
        return real_create_publisher(self, msg_type, topic, *a, **k)

    node = None
    with mock.patch.object(serial, 'Serial', fake_serial_ctor), \
         mock.patch.object(rclpy.node.Node, 'create_publisher',
                           fake_create_publisher):
        node = module.SerialBridgeNode('serial_bridge')
    return node


def run_v1():
    """Step 2 最小闭环版：收到一帧发一帧，不需要 spin"""
    from sensor_bridge import serial_bridge_node_v1 as impl

    cases = V1_CASES
    pub, holder = FakePub(), {}
    node = _patch_and_build(impl, [c[0] for c in cases], pub, holder)
    time.sleep(0.6)                      # 等读线程把这些行吃完

    ok = True
    want = [c[1] for c in cases if c[1] is not None]
    _print_table('v1 最小闭环版', cases)

    print('期望发布序列:', want)
    print('实际发布序列:', pub.got)
    if pub.got == want:
        print('✅ 解析结果逐项吻合（含上面那条已知局限）')
    else:
        ok = False
        print('❌ 解析结果不吻合！')
        extra = set(pub.got) - set(want)
        miss = set(want) - set(pub.got)
        if extra:
            print('   多出来的:', extra)
        if miss:
            print('   漏掉的  :', miss)

    known = [c[2] for c in cases if c[3]]
    if known:
        print()
        print('── 本次自测挖出的已知局限（v1 预期内，加固版已修）──')
        for why in known:
            print(f'   · {why}')

    ok &= _check_common(node, holder)
    return ok


def run_hardened():
    """Step 6 加固版：发布走定时器，必须用 spin_once 驱动"""
    from sensor_bridge import serial_bridge_node as impl

    cases = HARDENED_CASES
    pub, holder = FakePub(), {}
    node = _patch_and_build(impl, [c[0] for c in cases], pub, holder)

    # 加固版是「定时器 + 最新值」，回调只有被 spin 派发才会跑。
    # 这一点本身就是本测要验证的设计：发布节奏与到达节奏解耦。
    t0 = time.monotonic()
    while time.monotonic() - t0 < 2.0:
        rclpy.spin_once(node, timeout_sec=0.02)

    _print_table('hardened 加固版', cases)

    ok = True
    checks = [
        ('收帧数 == 7（6 条单值 + 1 条 v2 四字段）',
         node._rx_count == 7, f'实测 {node._rx_count}'),
        ('坏帧数 == 5（横幅/乱码/空行/纯换行/超长行）',
         node._bad_count == 5, f'实测 {node._bad_count}'),
        ('超长行被挡（不再产生 inf）',
         all(v != float('inf') for v in pub.got),
         f'发布值样例 {sorted(set(pub.got))}'),
        ('发布由定时器驱动（有帧发出）', len(pub.got) >= 5,
         f'{len(pub.got)} 帧 / 2s'),
        ('发布的是最新值 2560.0',
         all(v == 2560.0 for v in pub.got),
         f'实际 {sorted(set(pub.got))}'),
    ]
    print('加固版专项断言：')
    for name, passed, detail in checks:
        print(f'  {"✅" if passed else "❌"} {name}  （{detail}）')
        ok &= passed

    print()
    print('注：v2 四字段已能被解析 → 加固版不再需要 sender 加 --v1；')
    print('    并且它按 publish_rate 定频发布，发布频率与到达频率无关。')

    ok &= _check_common(node, holder)
    return ok


def _print_table(title, cases):
    print(f"{'输入（最多 46 字符）':<50} {'期望':>8}   说明")
    print('-' * 96)
    for raw, exp, why, is_known in cases:
        shown = (raw[:44].decode('ascii', 'backslashreplace')
                 .replace('\r', '\\r').replace('\n', '\\n'))
        label = '坏帧' if exp is None else repr(exp)
        mark = '[KNOWN]' if is_known else ''
        print(f'{shown:<50} {label:>8} {mark:<8}{why}')
    print(f"（当前模式：{title}）")
    print()


def _check_common(node, holder):
    ok = True
    # ── 通用断言 1：timeout 真的传进去了 ────────────────────
    t = holder['ser'].timeout
    print(f"\n串口 timeout 参数       : {t}"
          f"  {'✅ 非 None，readline 可中断' if t else '❌ 为 None，readline 会永久阻塞'}")
    ok &= bool(t)

    # ── 通用断言 2：destroy_node 能停掉读线程 ───────────────
    node.destroy_node()
    time.sleep(0.4)
    alive = node._thread.is_alive()
    print(f"destroy_node 后读线程退出: {'❌ 仍存活' if alive else '✅ 已退出'}"
          f"   (串口 closed={holder['ser'].closed})")
    ok &= (not alive)
    return ok


def main():
    ap = argparse.ArgumentParser(description='串口桥节点逻辑自测（mock 掉串口）')
    ap.add_argument('--impl', choices=['v1', 'hardened', 'both'],
                    default='hardened',
                    help='被测实现：v1=Step2 最小闭环 / hardened=Step6 加固版 / both=都跑')
    args = ap.parse_args()

    rclpy.init()
    try:
        if args.impl in ('v1', 'both'):
            print('════════ 模式 A：Step 2 最小闭环版 ════════')
            ok_a = run_v1()
            print()
        else:
            ok_a = True

        if args.impl in ('hardened', 'both'):
            print('════════ 模式 B：Step 6 加固版 ════════')
            ok_b = run_hardened()
        else:
            ok_b = True
    finally:
        if rclpy.ok():
            rclpy.shutdown()

    print()
    print('总判定:', '全部通过 ✅' if (ok_a and ok_b) else '有失败项 ❌')
    return 0 if (ok_a and ok_b) else 1


if __name__ == '__main__':
    sys.exit(main())
