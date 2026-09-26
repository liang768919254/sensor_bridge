#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
virtual_serial_sender.py —— 假装自己是 STM32，往虚拟串口灌数据

配套 socat 用：
  终端 A:  socat -d -d pty,raw,echo=0,link=/tmp/ttyV0 pty,raw,echo=0,link=/tmp/ttyV1
  终端 B:  python3 virtual_serial_sender.py --port /tmp/ttyV0 --rate 20
  终端 C:  ros2 run sensor_bridge serial_bridge --ros-args -p port:=/tmp/ttyV1

它不只是发好数据，还【故意使坏】——因为这三类脏东西真板子上天天出现：
  ① 启动横幅  "STM32 serial bridge ready | proto=v2 | baud=115200"
  ② 乱码行    含非 ASCII 字节，波特率抖动/电气噪声的典型产物
  ③ 超长行    600 字节，模拟半包 / 缓冲错位

先用这个脚本把 PC 侧的五类异常处理验完，再上真板子。
这样一旦出问题，你就能确定「不是板子的问题」——排查范围直接砍一半。
"""

import argparse
import math
import os
import sys
import time

import serial

# 复用协议模块里的组装函数（正式版里它住在 ROS2 包里）
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '参考源码'))
from serial_protocol import format_line  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description='虚拟串口数据灌入器（假 STM32）')
    ap.add_argument('--port', required=True,
                    help='要写入的虚拟串口（socat 的 ttyV0 那一头）')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--rate', type=float, default=20.0, help='发送频率 Hz')
    ap.add_argument('--v1', action='store_true',
                    help='用 v1 协议（只发一个数），默认发 v2 四字段')
    ap.add_argument('--no-banner', action='store_true', help='不发启动横幅')
    ap.add_argument('--no-garbage', action='store_true', help='不插乱码/超长行')
    ap.add_argument('--garbage-every', type=float, default=5.0,
                    help='每多少秒插一次坏数据（默认 5s）')
    ap.add_argument('--frames', type=int, default=0,
                    help='只发这么多帧就退出（0 = 一直发，默认 0）')
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1, write_timeout=1)
    except serial.SerialException as e:
        print(f'[ERR] 打不开 {args.port}：{e}')
        print('      检查 socat 是否还在跑，以及 /tmp/ttyV0 这个软链是否存在')
        print('      （socat 一停，ttyV0/ttyV1 这两个链接就消失了）')
        return 1

    print(f'[OK] 已打开 {args.port} @ {args.baud}，频率 {args.rate}Hz，'
          f'协议 {"v1" if args.v1 else "v2"}')

    # ── ① 上电横幅：真板子上电都会打这个 ─────────────────────
    if not args.no_banner:
        ser.write(b'STM32 serial bridge ready | proto=v2 | baud=115200\r\n')
        print('[TX] 横幅（PC 端应该静默忽略）')

    period = 1.0 / max(args.rate, 0.1)
    t = 0.0
    n = 0                                     # 总帧序号（无限递增）
    next_garbage = time.monotonic() + args.garbage_every
    t0 = time.monotonic()

    print('[OK] 开始发送，Ctrl+C 停止')
    try:
        while True:
            seq = n % 65536                    # 协议里的帧号是 16 位，会回绕
            # 模拟 ADC：中心 2048，幅度 ±1800，0.2Hz 慢变 → 曲线好看
            adc = 2048.0 + 1800.0 * math.sin(2.0 * math.pi * 0.2 * t)
            adc = max(0.0, min(4095.0, adc))
            mv = int(adc * 3300.0 / 4095.0)          # 毫伏，整数化
            temp = 2500 + int(150.0 * math.sin(2.0 * math.pi * 0.05 * t))

            if args.v1:
                line = format_line([mv])
            else:
                line = format_line([seq, mv, int(adc), temp])

            try:
                ser.write(line)
            except serial.SerialTimeoutException:
                print('[WARN] 写超时：对端没人读、缓冲区满了。'
                      '检查桥节点是否在跑。')

            if seq % max(int(args.rate), 1) == 0:
                print(f'[TX] {line!r}')

            # ── ②③ 定时插坏数据 ───────────────────────────────
            if not args.no_garbage and time.monotonic() >= next_garbage:
                ser.write(b'\x1f\x9c\x03\x8a\r\n')
                ser.write(b'9' * 600 + b'\r\n')
                ser.write(b',,,,\r\n')
                ser.write(b'\r\n')
                print('[TX] 坏数据 x4（乱码 / 超长行 / 空字段 / 空行）'
                      ' → PC 端应静默计入坏帧，不刷屏、不崩')
                next_garbage = time.monotonic() + args.garbage_every

            n += 1
            t += period

            if args.frames and n >= args.frames:
                print(f'[STOP] 已发够 {args.frames} 帧，退出')
                break

            # 按"理想时间轴"对齐，避免累积漂移（阶段一那条原则的同一条）
            sleep = t0 + n * period - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            elif sleep < -period:
                # 落后超过一整帧（比如机器被别的进程占满），重新对齐基准
                t0 = time.monotonic() - n * period

    except KeyboardInterrupt:
        print(f'\n[STOP] 共发送 {n} 帧')
    finally:
        ser.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
