# sensor_bridge

STM32 采集的 ADC 数据，走 USB 串口传到 ROS2 话题，再交给可视化或监控节点。这是我学 ROS2 时做的结业项目，分三个阶段做出来的：先用假数据把链路跑通，再接真板子，最后把消息类型换成自定义的 `SensorData`。

## 三个阶段做了什么

- **阶段一（纯软件）**：假传感器节点按 10Hz 发正弦，配监控和窗口统计节点，用 launch 一次起两个节点。这个阶段没碰硬件，只是把话题名、消息类型、参数这些接口先定下来。
- **阶段二（接硬件）**：STM32F103 通过 CH340 串口把 ADC 数据发到 PC，`serial_bridge` 节点负责读串口、解析、发话题。重点在工程化：读串口的阻塞调用放进独立线程、断线自动重连、乱码静默计数、设备名用 udev 固定。
- **阶段三（自定义消息）**：把桥发的东西从 `std_msgs/Float32` 换成自定义的 `SensorData`（带时间戳、坐标、多通道字段、设备时钟）。桥的骨架没动，只是用子类覆盖了两个钩子，顺便在订阅端加了丢帧检测和数据年龄。

## 目录

```
sensor_bridge/
├── ros2_ws/src/
│   ├── sensor_bridge/          # ROS2 包（Python），各节点和 launch 都在这里
│   │   ├── sensor_bridge/      #   节点实现
│   │   ├── launch/             #   launch 文件
│   │   └── test/               #   协议单测
│   └── sensor_interfaces/      # 接口包，SensorData.msg 在这里
├── firmware/adc_serial/        # STM32F103 固件（CubeMX 工程，HAL 库）
├── udev/99-stm32-bridge.rules  # CH340 固定成 /dev/stm32_bridge
├── docs/                       # 三个阶段的教程和总复盘（中文）
└── tests/                      # 不接板子也能跑的测试脚本
```

## 节点

- `my_sensor` / `my_sensor_wave`：假传感器，发正弦，参数可调（阶段一）
- `monitor_node` / `stats_node`：订阅端，做累计统计和窗口统计（阶段一）
- `serial_bridge`：串口桥，发 `std_msgs/Float32`（阶段二）
- `serial_bridge_msg`：同一条桥，发 `SensorData`（阶段三）
- `monitor_msg`：订阅 `SensorData`，做多通道统计、丢帧检测、数据年龄（阶段三）

## 硬件

- STM32F103C8T6（蓝板），ADC1 采集，USART1 输出，115200 8N1
- CH340 USB-TTL 接到 PC
- 固件工具链：`arm-none-eabi-gcc` + `make`，烧录用 `st-flash`

## 跑起来

依赖：ROS2 Humble、`python3-serial`（`sudo apt install python3-serial`）；固件侧要 `gcc-arm-none-eabi` 和 `stlink-tools`。

### 构建 ROS2 包

```bash
cd ros2_ws
colcon build
source install/setup.bash
```

两个坑，注意：

- 编译中途报 `No module named 'em'`：是 PATH 里别的 Python 抢了系统 3.10 的位置，清一下 PATH 再 build。
- 别用 `--symlink-install`：新 setuptools + Humble 下会让 `ros2 run` 报 `PackageNotFoundError`。

### 不接板子先跑通

```bash
ros2 launch sensor_bridge sensor.launch.py

# 或者自定义消息版（假发布者直接发 SensorData，不走串口）
python3 tests/fake_sensor_data_pub.py --rate 10
ros2 run sensor_bridge monitor_msg
ros2 topic echo /sensor/value --once
```

### 接真板子

```bash
# 串口固定别名（做一次就行）
sudo cp udev/99-stm32-bridge.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG dialout $USER   # 改完要注销重登

# 编译烧录固件
cd firmware/adc_serial && make && ./flash

# 起桥
ros2 launch sensor_bridge serial_bridge.launch.py
# 或者自定义消息版
ros2 launch sensor_bridge serial_bridge_msg.launch.py port:=/dev/stm32_bridge
```

## 串口协议

板子和 PC 之间是一行一行的 ASCII，逗号分隔：

| 版本 | 格式 |
|---|---|
| v1 | `<value>\n` |
| v2 | `<seq>,<value>,<raw_adc>,<temp>\n` |
| v3 | `<seq>,<value_mv>,<raw_adc>,<temp_c_x100>,<mcu_tick_ms>\n` |

几个约定：

- 行尾是 `\n`（`\r\n` 也行）；空行、乱码、超长行整行丢弃
- `temp_c_x100` 是「℃×100」的定点整数，2500 就是 25.00℃，PC 侧用 `temp_scale:=0.01` 乘回来
- 固件初始化后要 `setvbuf(stdout, NULL, _IONBF, 0)`，否则 newlib-nano 全缓冲，数据会攒十几秒才吐一次

自定义消息 `SensorData` 长这样：

```plain
std_msgs/Header header   # 时间戳 + frame_id
uint32  seq              # MCU 帧号，订阅端靠它检测丢帧
float32 value            # 主工程量（默认 mV）
float32 temperature      # ℃
uint16  raw_adc          # ADC 原始值 0~4095
uint32  mcu_tick_ms      # MCU 上电以来的 ms
```

## 测试

```bash
python3 tests/test_serial_protocol.py
python3 tests/test_sensor_data_codec.py     # 先 source ROS2 环境，用真消息类 21/21
python3 tests/verify_bridge_msg_node.py     # 桩串口验证桥子类
```

## 文档

`docs/` 里有三个阶段的教程和一篇总复盘，按阶段一 → 二 → 三 → 总复盘的顺序看。

## License

[MIT](./LICENSE)
