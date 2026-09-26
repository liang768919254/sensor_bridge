#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_sensor_data_codec.py —— 阶段三纯逻辑单测（不需要板子、不需要串口、不需要 ROS 图）

覆盖三块：
  A. sensor_data_codec：协议字段 → 消息字段的映射、协议版本判定、非法帧处理
  B. sensor_stats.Welford：与朴素公式在 float32 下的精度对比（把上一轮的争议）实测钉死）
  C. sensor_stats.SeqTracker：丢帧 / 回绕 / 重启

【怎么跑】
  方式 1（推荐，用真的消息类）：
      source /opt/ros/humble/setup.sh
      source ~/ros_study/install/setup.bash
      /usr/bin/python3 test_sensor_data_codec.py
  方式 2（消息包还没编译时也能跑，自动退回桩类）：
      /usr/bin/python3 test_sensor_data_codec.py
  方式 3（pytest）：
      /usr/bin/python3 -m pytest test_sensor_data_codec.py -v

  脚本会打印它用的是「真消息类」还是「桩类」——看到"桩类"说明你的
  sensor_interfaces 还没 build 或还没 source，A 组只验了逻辑、没验真实字段名。
"""

import math
import os
import random
import struct
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '参考源码'))

from sensor_data_codec import LAYOUT_V1, LAYOUT_V2, LAYOUT_V3  # noqa: E402
from sensor_data_codec import detect_layout, format_line_v3, pack_sensor_data  # noqa: E402
from sensor_stats import ChannelSet, SeqTracker, Welford  # noqa: E402


# ══════════════════════════════════════════════════════════════
#  拿到 SensorData 类：优先真的，拿不到就用桩（保证任何环境都能跑）
# ══════════════════════════════════════════════════════════════
class _StubTime:
    def __init__(self):
        self.sec = 0
        self.nanosec = 0


class _StubHeader:
    def __init__(self):
        self.stamp = _StubTime()
        self.frame_id = ''


class _StubSensorData:
    def __init__(self):
        self.header = _StubHeader()
        self.seq = 0
        self.value = 0.0
        self.temperature = 0.0
        self.raw_adc = 0
        self.mcu_tick_ms = 0


try:
    from sensor_interfaces.msg import SensorData  # type: ignore
    MSG_CLASS = SensorData
    MSG_SOURCE = '真消息类（sensor_interfaces.msg.SensorData）'
    # ★ 真消息类会做类型校验：header.stamp 必须是 builtin_interfaces/Time，
    #   传字符串会直接抛 "The 'stamp' field must be a sub message of type 'Time'"。
    #   桩类不会校验 —— 所以**单测只有跑到真类上才算数**。
    from builtin_interfaces.msg import Time  # type: ignore

    def make_stamp(sec=1234, nanosec=5678):
        t = Time()
        t.sec = sec
        t.nanosec = nanosec
        return t

    def stamp_matches(got, expected):
        return got.sec == expected.sec and got.nanosec == expected.nanosec
except ImportError:
    MSG_CLASS = _StubSensorData
    MSG_SOURCE = '桩类（sensor_interfaces 还没 build / 没 source）'

    class _PlainStamp:
        pass

    def make_stamp(sec=1234, nanosec=5678):
        s = _PlainStamp()
        s.sec = sec
        s.nanosec = nanosec
        return s

    def stamp_matches(got, expected):
        return got is expected or (getattr(got, 'sec', None) == expected.sec
                                   and getattr(got, 'nanosec', None) == expected.nanosec)


STAMP = make_stamp()


# ══════════════════════════════════════════════════════════════
#  A 组：编解码
# ══════════════════════════════════════════════════════════════

def test_detect_layout_v1():
    assert detect_layout([1.5]) == (1, LAYOUT_V1)


def test_detect_layout_v2_v3():
    assert detect_layout([1, 2, 3, 4]) == (2, LAYOUT_V2)
    assert detect_layout([1, 2, 3, 4, 5]) == (3, LAYOUT_V3)


def test_extra_fields_are_ignored():
    """向前兼容：以后协议加第 6 个字段，老节点不该崩"""
    assert detect_layout([1, 2, 3, 4, 5, 6, 7]) == (3, LAYOUT_V3)


def test_ambiguous_field_count_rejected():
    """2 / 3 个字段 = 协议错误 → 整帧丢弃。
       宁可丢帧，也不要字段错位的"看起来正常"的数据。"""
    assert detect_layout([1, 2]) == (None, None)
    assert detect_layout([1, 2, 3]) == (None, None)
    assert detect_layout([]) == (None, None)


def test_pack_v3_field_mapping():
    """v3 五个字段必须一一落到对的消息字段上"""
    fields = [1024, 1650, 2048, 2500, 187340]
    msg = pack_sensor_data(MSG_CLASS, fields, stamp=STAMP,
                           frame_id='imu_link',
                           value_scale=1.0, temp_scale=0.01)
    assert msg is not None
    assert msg.seq == 1024
    assert msg.value == 1650.0
    assert msg.raw_adc == 2048
    assert abs(msg.temperature - 25.00) < 1e-9      # 2500 × 0.01
    assert msg.mcu_tick_ms == 187340
    assert msg.header.frame_id == 'imu_link'
    assert stamp_matches(msg.header.stamp, STAMP)


def test_pack_v1_fills_missing_with_zero():
    """v1 只有 value：其余字段填 0，不报错、不崩"""
    msg = pack_sensor_data(MSG_CLASS, [7.5], stamp=STAMP, seq_fallback=42)
    assert msg.value == 7.5
    assert msg.seq == 42               # 没有帧号 → 用 PC 侧自增兜底
    assert msg.raw_adc == 0
    assert msg.temperature == 0.0
    assert msg.mcu_tick_ms == 0


def test_pack_scaling():
    msg = pack_sensor_data(MSG_CLASS, [0, 1500, 0, 2500], stamp=STAMP,
                           value_scale=0.001, temp_scale=0.01)
    assert abs(msg.value - 1.5) < 1e-9
    assert abs(msg.temperature - 25.0) < 1e-9


def test_pack_rejects_unknown_layout():
    assert pack_sensor_data(MSG_CLASS, [1, 2], stamp=STAMP) is None
    assert pack_sensor_data(MSG_CLASS, [1, 2, 3], stamp=STAMP) is None


def test_seq_wraps_into_u32():
    """seq 是 uint32：超范围要绕回去，不能抛异常"""
    msg = pack_sensor_data(MSG_CLASS, [1 << 40, 1.0, 0, 0], stamp=STAMP)
    assert 0 <= msg.seq < (1 << 32)


def test_raw_adc_truncates_to_int():
    msg = pack_sensor_data(MSG_CLASS, [0, 1, 2047.9, 25], stamp=STAMP)
    assert msg.raw_adc == 2047
    assert isinstance(msg.raw_adc, int)


def test_format_line_v3_roundtrip():
    line = format_line_v3(1024, 1650, 2048, 2500, 187340)
    assert line == b'1024,1650,2048,2500,187340\n'
    # 纯 ASCII 且以 \n 结尾 —— MCU 侧照这个格式发就行
    assert line.decode('ascii').endswith('\n')


# ══════════════════════════════════════════════════════════════
#  B 组：Welford 的精度价值（实测，不是背书）
# ══════════════════════════════════════════════════════════════

def _f32(x):
    """把 double 舍入成 float32 再放回来 —— 模拟 C 语言的 float"""
    return struct.unpack('f', struct.pack('f', x))[0]


def _naive_var(vals, prec):
    n = prec(len(vals))
    s1 = prec(0.0)
    s2 = prec(0.0)
    for v in vals:
        s1 = prec(s1 + prec(v))
        s2 = prec(s2 + prec(prec(v) * prec(v)))
    m = prec(s1 / n)
    return prec(prec(s2 / n) - prec(m * m))


def _welford_var(vals, prec):
    n = 0
    mean = prec(0.0)
    m2 = prec(0.0)
    for v in vals:
        n += 1
        d = prec(prec(v) - mean)
        mean = prec(mean + prec(d / n))
        m2 = prec(m2 + prec(d * prec(prec(v) - mean)))
    return prec(m2 / n)


def test_float64_naive_is_fine():
    """★ 更正：float64 下朴素公式是安全的，偏置 1e7 也不出事。
       上一轮把它说成"必撞"是错的，这里用断言把它钉住。"""
    random.seed(1)
    bad = 0
    for _ in range(200):
        vals = [2048.0 + random.gauss(0, 5.0) for _ in range(50)]
        if _naive_var(vals, lambda x: x) < 0:
            bad += 1
    assert bad == 0, f'float64 下朴素公式不该算出负数，实际 {bad}/200'


def test_float32_naive_breaks():
    """float32 下才会炸：偏置 1e6 + 波动 1 → 大量负方差
       （负方差会被 max(0.0, ...) 兜成 0 → 标准差恒为 0 且不报错）"""
    random.seed(2)
    bad = 0
    for _ in range(200):
        vals = [1e6 + random.gauss(0, 1.0) for _ in range(50)]
        if _naive_var(vals, _f32) < 0:
            bad += 1
    assert bad > 0, 'float32 下应当出现负方差，说明本用例没构造出临界条件'


def test_welford_survives_float32():
    """Welford 在同条件下不出负数，且精度与 double 同级"""
    random.seed(3)
    errs = []
    for _ in range(200):
        vals = [1e6 + random.gauss(0, 1.0) for _ in range(50)]
        vw = _welford_var(vals, _f32)
        assert vw > 0
        errs.append(abs(vw - 1.0))
    assert sum(errs) / len(errs) < 0.5, f'float32 Welford 偏差过大: {sum(errs)/len(errs)}'


def test_welford_matches_theory_on_sine():
    """无噪声正弦的方差必须 ≈ A²/2（阶段一那个"理论锚点"，换成 Welford 也一样）"""
    A = 1.0
    w = Welford()
    for i in range(50):
        w.update(A * math.sin(2 * math.pi * 0.5 * (0.37 + 0.1 * i)))
    assert abs(w.variance - 0.5) < 0.01
    assert abs(w.std - 0.7071) < 0.01


def test_welford_is_o1_and_tracks_minmax():
    w = Welford()
    for v in (3.0, 1.0, 4.0, 1.0, 5.0):
        w.update(v)
    assert w.n == 5
    assert abs(w.mean - 2.8) < 1e-12
    assert w.vmin == 1.0 and w.vmax == 5.0
    w.reset()
    assert w.n == 0 and w.mean == 0.0 and w.std == 0.0


def test_channel_set_ignores_unknown_name():
    cs = ChannelSet(['value', 'temperature'])
    cs.update('value', 1.0)
    cs.update('不存在的通道', 1.0)          # 不该抛异常
    assert cs.report()['value'].n == 1
    cs.reset()
    assert cs.report()['value'].n == 0


# ══════════════════════════════════════════════════════════════
#  C 组：帧号追踪
# ══════════════════════════════════════════════════════════════

def test_seq_continuous():
    t = SeqTracker()
    for s in range(100, 120):
        t.update(s)
    assert t.snapshot() == (0, 0, 0)


def test_seq_drop_detected():
    t = SeqTracker()
    for s in (1, 2, 3, 6, 7):
        t.update(s)
    assert t.lost == 2


def test_seq_wraparound_is_not_a_drop():
    """255 → 0 是正常回绕，不能算丢帧（用减法硬比会误判）"""
    t = SeqTracker()
    for s in (253, 254, 255, 0, 1):
        t.update(s, wrap=256)
    assert t.snapshot() == (0, 0, 0)


def test_seq_backward_is_out_of_order_not_drop():
    """板子重启：帧号从 5001 掉回 3 → 记乱序，不该算丢 4998 帧"""
    t = SeqTracker()
    for s in (5000, 5001, 3):
        t.update(s)
    assert t.lost == 0 and t.out_of_order == 1


# ══════════════════════════════════════════════════════════════
#  无 pytest 也能跑
# ══════════════════════════════════════════════════════════════

def _run_all():
    tests = [(n, f) for n, f in sorted(globals().items())
             if n.startswith('test_') and callable(f)]
    failed = []
    for name, fn in tests:
        try:
            fn()
            print(f'  PASS  {name}')
        except AssertionError as e:
            failed.append(name)
            print(f'  FAIL  {name}  {e}')
        except Exception as e:
            failed.append(name)
            print(f'  ERROR {name}  {type(e).__name__}: {e}')
    print(f'\n{len(tests) - len(failed)}/{len(tests)} passed')
    if failed:
        print('失败：' + ', '.join(failed))
        return 1
    return 0


if __name__ == '__main__':
    print('阶段三纯逻辑单测（不需要板子 / 串口 / ROS 图）')
    print(f'消息类来源：{MSG_SOURCE}\n')
    sys.exit(_run_all())
