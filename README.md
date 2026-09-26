# sensor_bridge — ROS2 串口传感器桥

> STM32 采集 → USB 串口 → ROS2 话题 → 可视化/监控 的完整链路。
> ROS2 21 讲结业项目，分三个阶段逐步加厚：**假数据 → 接硬件 → 自定义消息**。

## 一句话

一条把 STM32 传感器数据搬进 ROS2 的桥，从「一个 `float32`」进化到「带时间戳、坐标、多通道测量、设备时钟的自定义消息」。

## 三阶段架构

这三个阶段不是三个项目，是**同一条链路被加厚了三次**：

| 阶段 | 主题 | 练什么 | 关键产物 |
|---|---|---|---|
| 一 | 纯软件仿真 | **设计**（先用假数据把接口定死） | 假传感器 / 监控 / 窗口统计 / launch 编排 |
| 二 | STM32 串口接入 | **工程**（把生命周期扛住） | 串口桥（读线程·断线自愈·陈旧保护）/ 固件 / udev 别名 |
| 三 | 自定义消息 | **分层**（把数据格式切出来） | `SensorData.msg` / 模板方法换出口 / 丢帧与数据年龄检测 |

```
阶段一  假数据 → 把接口定死      （练「设计」）
阶段二  真数据 → 把生命周期扛住  （练「工程」）
阶段三  换出口 → 把数据格式切出来（练「分层」）
```

## 特性

- **纯 Python 的 ROS2 节点**（ROS2 Humble），两个包：`sensor_bridge`（实现）+ `sensor_interfaces`（接口）
- **串口桥的工程化加固**：阻塞读挪进独立线程、断线自动重连、乱码静默计数、陈旧数据保护、设备名走 udev 别名
- **自定义消息 `SensorData`**：用 `std_msgs/Header` 承载时间戳与 `frame_id`，多通道字段同一帧对齐
- **模板方法模式换出口**：同一条桥，覆盖两个钩子即可从 `Float32` 换成任意自定义消息，骨架零改动
- **无硬件自测**：纯逻辑单测 + 假发布者 + 桩串口，不接板子也能验完整链路
- **全整数定点协议**：MCU 侧完全避开浮点 `printf`，PC 侧用 `scale` 参数乘回工程量

## 目录结构

```
sensor_bridge/
├── ros2_ws/src/
│   ├── sensor_bridge/          # 实现包（ament_python）
│   │   ├── sensor_bridge/      #   Python 模块（各节点）
│   │   ├── launch/             #   3 个 launch 文件
│   │   ├── setup.py / package.xml
│   │   └── test/               #   协议单测 + ament lint
│   └── sensor_interfaces/      # 接口包（ament_cmake）
│       └── msg/SensorData.msg
├── firmware/adc_serial/        # STM32F103C8T6 固件（CubeMX 工程，HAL）
├── udev/99-stm32-bridge.rules  # CH340 → /dev/stm32_bridge 固定别名
├── docs/                       # 三阶段教程 + 总复盘（中文）
└── tests/                      # 无硬件自测脚本
```

## 节点一览

| 节点 | 阶段 | 说明 |
|---|---|---|
| `my_sensor` / `my_sensor_wave` | 一 | 假传感器，10Hz 发正弦（`std_msgs/Float32`），支持多波形与参数 |
| `monitor_node` / `stats_node` | 一 | 订阅端：累计统计 / 窗口统计 |
| `serial_bridge` | 二 | 串口桥，发布 `std_msgs/Float32` |
| `serial_bridge_msg` | 三 | 同一条桥，出口换成 `SensorData`（60 行子类） |
| `monitor_msg` | 三 | 结构化订阅端：多通道 Welford 统计 + 丢帧 + 数据年龄 + `frame_id` 变化告警 |

## 硬件依赖

- 开发板：STM32F103C8T6（蓝板，Cortex-M3）
- 采集：ADC1（12 位，轮询单次转换）
- 输出：USART1（115200 8N1），经 CH340 USB-TTL 接入 PC
- 工具链（固件）：`arm-none-eabi-gcc` + `make`，烧录用 `st-flash`

## 快速开始

### 0. 依赖

- ROS2 Humble，系统 Python 3.10
- `pyserial`：`sudo apt install python3-serial`
- 固件工具链：`sudo apt install gcc-arm-none-eabi stlink-tools`

### 1. 构建 ROS2 包

```bash
cd ros2_ws
colcon build            # 注意：用系统 /usr/bin/python3（3.10），别让其他 python3 抢 PATH
source install/setup.bash
```

> ⚠️ 若编译中途报 `No module named 'em'`，是 PATH 里的 Python 3.x 抢了系统 3.10 的位置，清一下 PATH 再 build。
> ⚠️ 不要用 `--symlink-install`（新 setuptools + Humble 下会让 `ros2 run` 报 `PackageNotFoundError`）。

### 2. 无硬件跑通（阶段一 / 三的 PC 侧）

```bash
# 假传感器 + 监控
ros2 launch sensor_bridge sensor.launch.py

# 自定义消息版：假发布者（不经过串口）
source install/setup.bash
python3 ../tests/fake_sensor_data_pub.py --rate 10
ros2 run sensor_bridge monitor_msg
ros2 topic echo /sensor/value --once
```

### 3. 接真硬件（阶段二 / 三）

```bash
# ① 固定串口别名（一次）
sudo cp udev/99-stm32-bridge.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
# 把你加入 dialout 组（改完注销重登）
sudo usermod -aG dialout $USER

# ② 编译并烧录固件
cd firmware/adc_serial && make && ./flash

# ③ 起桥（Float32）或自定义消息版
ros2 launch sensor_bridge serial_bridge.launch.py
ros2 launch sensor_bridge serial_bridge_msg.launch.py port:=/dev/stm32_bridge
```

## 串口协议（行协议，ASCII）

| 版本 | 格式 | 说明 |
|---|---|---|
| v1 | `<value>\n` | 单值 |
| v2 | `<seq>,<value>,<raw_adc>,<temp>\n` | 多字段 |
| **v3** | `<seq>,<value_mv>,<raw_adc>,<temp_c_x100>,<mcu_tick_ms>\n` | 全整数定点 + 设备时钟 |

- 行尾统一 `\n`（允许 `\r\n`）；空行、乱码、超长行一律整帧丢弃
- `temp_c_x100` 是「℃ × 100」的定点整数（2500 = 25.00℃），PC 侧用 `temp_scale:=0.01` 乘回
- 固件侧初始化后必须 `setvbuf(stdout, NULL, _IONBF, 0)`，否则 newlib-nano 全缓冲会导致约 17 秒才吐一次

## 自定义消息

```plain
std_msgs/Header header   # stamp=到达时刻；frame_id=安装坐标名
uint32  seq              # MCU 帧号，订阅端检测丢帧
float32 value            # 主工程量（默认 mV）
float32 temperature      # ℃
uint16  raw_adc          # ADC 原始值 0~4095
uint32  mcu_tick_ms      # MCU 上电以来的 ms
```

## 测试

```bash
# 纯逻辑单测（协议解析 / Welford 精度 / 丢帧回绕，不依赖 ROS 图）
python3 tests/test_serial_protocol.py
python3 tests/test_sensor_data_codec.py     # 需先 source ROS2 环境（用真消息类 21/21）

# 桩串口验证桥子类（不接板子）
python3 tests/verify_bridge_msg_node.py
```

## 文档

三阶段教程与总复盘见 `docs/`（中文），推荐按 `阶段一 → 阶段二 → 阶段三 → 总复盘` 顺序阅读。

## License

[MIT](./LICENSE) © 2026 liang768919254
