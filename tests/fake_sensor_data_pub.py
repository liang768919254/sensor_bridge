#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
fake_sensor_data_pub.py —— 假传感器：直接发 SensorData 消息（不经过串口！）

【它是干什么的】
  阶段二用 socat 造假串口验证「串口 → 桥」这半条链路；
  这个脚本验证「桥 → 话题 → 订阅端」那半条链路，**连串口都不需要**。
  两半合起来，就是完整链路 —— 而你现在可以在没有板子、没有 socat 的情况下
  把阶段三的自定义消息部分全部验完。

【用法】
  # 终端 A：假传感器（10Hz，正弦，含温度通道）
  ros2 run 不行 —— 它是独立脚本，直接跑：
  /usr/bin/python3 fake_sensor_data_pub.py --rate 10 --amplitude 1.0 --plus 1650

  # 终端 B：结构化监控
  ros2 run sensor_bridge monitor_msg --ros-args -p topic:=/sensor/value

  # 终端 C：看结构化内容 / 频率
  ros2 topic echo /sensor/value --once
  ros2 topic hz /sensor/value

【故意使坏参数】
  --drop-every 7     每 7 帧丢 1 帧（把 seq 跳号）→ 验证订阅端的丢帧检测
  --frame-id X       换坐标名 → 验证 frame_id 变化只在跳变时打日志
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from sensor_interfaces.msg import SensorData


class FakeSensorDataPub(Node):

    def __init__(self, args):
        super().__init__('fake_sensor_data_pub')

        # 与桥保持一致的"结构参数只读一次 / 数据参数每帧现读"习惯：
        # 这里的幅值频率都从命令行来，运行时不改，所以直接存下来
        self.rate = args.rate
        self.amplitude = args.amplitude
        self.base = args.plus
        self.freq = args.freq
        self.drop_every = args.drop_every
        self.frame_id = args.frame_id

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.pub = self.create_publisher(SensorData, args.topic, qos)

        self.step = 1.0 / self.rate
        self.t = 0.0
        self.seq = 0
        self.sent = 0
        self.skipped = 0

        self.timer = self.create_timer(self.step, self.on_timer)
        self.get_logger().info(
            f'假传感器启动 | {args.topic} | {self.rate:g}Hz | '
            f'幅值={self.amplitude:g} 基值={self.base:g} 频率={self.freq:g}Hz | '
            f'frame_id={self.frame_id}'
            + (f' | 每 {self.drop_every} 帧故意丢 1 帧' if self.drop_every else ''))

    # ──────────────────────────────────────────────────────────
    def on_timer(self):
        # ① 时间轴自己累加，不用 time.time()
        #    —— 用挂钟时间会吃定时器抖动，波形忽密忽疏（阶段一那条原则）
        phase = 2.0 * math.pi * self.freq * self.t
        value_mv = self.base + self.amplitude * 1000.0 * math.sin(phase)
        temp_c = 25.0 + 0.6 * math.sin(2.0 * math.pi * 0.05 * self.t)

        self.t += self.step
        self.seq = (self.seq + 1) % (1 << 32)

        # ② 故意丢帧：跳过 publish，但帧号照样递增
        #    这样订阅端能看到 seq 跳号 —— 这是"话题有数据"和"一帧没丢"的区别
        if self.drop_every and self.sent % self.drop_every == self.drop_every - 1:
            self.skipped += 1
            self.sent += 1
            return

        msg = SensorData()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.seq = self.seq
        msg.value = float(value_mv)                  # 毫伏
        msg.temperature = float(temp_c)              # 摄氏度
        msg.raw_adc = int(max(0.0, min(4095.0, value_mv * 4095.0 / 3300.0)))
        msg.mcu_tick_ms = int(self.t * 1000.0)

        self.pub.publish(msg)
        self.sent += 1


def main(argv=None):
    ap = argparse.ArgumentParser(description='假 SensorData 发布者（不需要串口）')
    ap.add_argument('--topic', default='/sensor/value')
    ap.add_argument('--rate', type=float, default=10.0, help='发布频率 Hz')
    ap.add_argument('--amplitude', type=float, default=1.0, help='正弦幅值（伏）')
    ap.add_argument('--plus', type=float, default=1650.0, help='直流基值（毫伏）')
    ap.add_argument('--freq', type=float, default=0.5, help='正弦频率 Hz')
    ap.add_argument('--frame-id', default='sensor_link')
    ap.add_argument('--drop-every', type=int, default=0,
                    help='每 N 帧故意丢 1 帧（0=不丢），用来验证订阅端丢帧检测')
    ap.add_argument('--seconds', type=float, default=0.0,
                    help='跑多久自动退出（0=一直跑）')
    args, _ = ap.parse_known_args(argv)

    rclpy.init()
    node = FakeSensorDataPub(args)
    deadline = time.monotonic() + args.seconds if args.seconds > 0 else None
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if deadline and time.monotonic() > deadline:
                break
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.get_logger().info(
            f'退出 | 发出={node.sent - node.skipped} 故意丢={node.skipped}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
