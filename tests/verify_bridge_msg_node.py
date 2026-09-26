#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""verify_stage3_bridge.py —— 验证阶段三：SerialBridgeMsgNode 子类在增强版基类下能实例化 + 打包

不依赖真串口：用桩 serial 模块顶替 pyserial，只验证「继承链 + 钩子 + 打包」这条最核心的链路。
跑法（需先 source ROS）：
  source /opt/ros/humble/setup.sh && source ~/ros_study/install/setup.sh
  /usr/bin/python3 verify_stage3_bridge.py
"""
import sys
import types

# ── 1. 桩 serial：只实现 serial_bridge_node.py 用到的最小接口 ──
_stub = types.ModuleType('serial')


class SerialException(Exception):
    pass


class Serial:
    def __init__(self, *args, **kwargs):
        self.closed = False

    def readline(self):
        return b''          # 永远返回空 = 串口一直无数据（超时语义）

    def close(self):
        self.closed = True


_stub.Serial = Serial
_stub.SerialException = SerialException
sys.modules['serial'] = _stub

# ── 2. 真正 import 工作空间里的包 ──
import rclpy  # noqa: E402
from sensor_bridge.serial_bridge_msg_node import SerialBridgeMsgNode  # noqa: E402
from sensor_interfaces.msg import SensorData  # noqa: E402
from builtin_interfaces.msg import Time  # noqa: E402

rclpy.init()
node = SerialBridgeMsgNode('test_msg_bridge')

# ── 3. 断言 1：EXTRA_PARAMS 被基类声明（阶段三新增的两个参数）──
assert node.get_parameter('frame_id').value == 'sensor_link', \
    'frame_id 参数未声明'
assert node.get_parameter('temp_scale').value == 0.01, \
    'temp_scale 参数未声明'
print('[1] EXTRA_PARAMS 声明生效：frame_id=sensor_link, temp_scale=0.01')

# ── 4. 断言 2：_create_publisher 用的是 SensorData（不是 Float32）──
assert node.pub.msg_type is SensorData, \
    f'发布类型错误：{node.pub.msg_type}'
print('[2] 发布者类型 =', node.pub.msg_type.__name__)

# ── 5. 断言 3：_pack_msg 把 v3 五字段正确映射到消息 ──
stamp = Time()
stamp.sec = 1
stamp.nanosec = 2
node._latest_fields = (1024, 1650, 2048, 2500, 187340)   # v3 一行
node._latest_seq = 1024
msg = node._pack_msg(stamp)

assert isinstance(msg, SensorData), '返回的不是 SensorData'
assert msg.seq == 1024, f'seq={msg.seq}'
assert msg.value == 1650.0, f'value={msg.value}'
assert msg.raw_adc == 2048, f'raw_adc={msg.raw_adc}'
assert abs(msg.temperature - 25.0) < 1e-6, f'temperature={msg.temperature}'
assert msg.mcu_tick_ms == 187340, f'mcu_tick_ms={msg.mcu_tick_ms}'
assert msg.header.frame_id == 'sensor_link', f'frame_id={msg.header.frame_id}'
assert (msg.header.stamp.sec, msg.header.stamp.nanosec) == (1, 2), 'stamp 未写入'
print('[3] _pack_msg 打包正确：seq=1024 value=1650.0 raw_adc=2048 '
      f'temperature={msg.temperature:.2f} mcu_tick_ms=187340')

# ── 6. 断言 4：_msg_type_name 返回自定义类型名 ──
assert node._msg_type_name() == 'sensor_interfaces/msg/SensorData'
print('[4] _msg_type_name =', node._msg_type_name())

node.destroy_node()
rclpy.shutdown()
print('\n全部通过：SerialBridgeMsgNode 继承链 + 钩子 + 打包 端到端正确')
