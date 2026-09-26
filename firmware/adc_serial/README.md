# adc_serial — STM32F103C8T6 ｜ ADC → 串口 → ROS2 桥

---

## ⚠️ 本目录是唯一权威源（Source of Truth）

自 **2026-09-21** 起，本工程以 **Linux 侧 `~/stm32_ws/adc_serial`** 为唯一编辑源。

- Windows 侧 `E:\stm32_ws\adc_serial`
  （Linux 挂载点 `/media/liang/新加卷/stm32_ws/adc_serial`）
  **停在 2026-09-21 14:30，已冻结，请勿再编辑。**
- 原因：两个系统都装了 STM32CubeMX，两边都能「Generate Code」——同时编辑必然分叉，
  而且很难察觉哪份是新的（`main.c` 的 md5 已经不一致过）。
- **改代码、改 CubeMX 配置，只在 Linux 这一份上做。**

---

## 一键编译 + 烧录

```bash
./flash.sh            # 编译；机器码与上次烧录的不同才烧（自动比 md5）
./flash.sh --force    # 无条件重烧
./flash.sh --build    # 只编译，不烧
```

> `flash.sh` 是手写脚本，不是 Makefile 的 target —— Makefile 由 CubeMX 生成，
> 每次点 `GENERATE CODE` 都会被整份重写，自定义 target 会丢。

## 硬件引脚

| 功能 | 引脚 | 说明 |
|---|---|---|
| ADC 输入 | **PA0** | ADC1_IN0，接电位器中间抽头 |
| 串口 TX / RX | **PA9 / PA10** | USART1，115200 8N1 |
| 状态 LED | **PC13** | 低电平点亮，0.5 s 翻转一次 |
| 调试口 | SWD 4 线 | `Serial Wire` 模式，未接 NRST |

⚠️ **ST-Link/V2 不提供虚拟串口**（只有 V2-1 才有 VCP）。
`/dev/ttyUSB*` 为空是正常的——要看串口输出必须另接 USB-TTL：
**PA9→RXD、PA10→TXD、GND→GND**，切勿往 PA9/PA10 接 5V/3.3V。

## 协议 v1（固件 → PC）

- 每 **100 ms** 发一行，行尾 `\r\n`，内容为**毫伏整数**（如 `3300`）
- 上电先发一行横幅：`STM32 serial bridge ready | proto=v1 | baud=115200`
  （PC 侧 `float()` 解析失败会静默丢弃，不影响数据）
- **帧里不要加逗号字段** —— PC 侧是 `float(text)`，加任何前缀/后缀整行都会被丢掉

## 快速验证

```bash
./flash.sh                                   # 烧录
# ① 看 LED：PC13 每 0.5 s 闪 = 程序活着（零成本，先看这个）
# ② 看串口：
python3 -m serial.tools.miniterm /dev/ttyUSB0 115200
# 看到毫伏数字滚动 → 板子 PASS → 再启 ROS2
ros2 run sensor_bridge serial_bridge --ros-args -p port:=/dev/ttyUSB0
```

## 相关

- 代码审查记录与状态更新：**`REVIEW.md`**
- PC 侧 ROS2 节点：`~/ros_study/src/sensor_bridge/sensor_bridge/serial_bridge_node.py`
- 编译/烧录排错清单：技能 `stm32-cubemx-gcc-build`

---

*构建环境：arm-none-eabi-gcc 10.3.1 ｜ STM32CubeMX 6.18.1 ｜ st-flash 1.7.0*
