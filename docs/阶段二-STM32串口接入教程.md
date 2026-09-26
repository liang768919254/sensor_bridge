# 阶段二教程：把 STM32 接进 ROS2（serial_bridge 串口桥）

> 制定日期：2026-09-19　**最新复核：2026-09-22**（复核内容见文末「维护记录」）
> 上游文档：`ROS2 21讲结业清单与下一步路线.md` 第三节「阶段 2：接硬件」
> 前置状态：阶段一（`阶段一：软件模拟仿真.md` + `阶段一测试题与答案.md`）已完成
> 适用方向：边缘 Infra / 具身智能本体工程向（软硬中间层）

---

## ⚠️ 动手前必读：两个实现版本，别选错

`serial_bridge` 这个入口底下历史上有两份实现，**行为不一样**，本教程两面都讲到：

| | **v1 最小闭环版** | **hardened 加固版（当前默认）** |
|---|---|---|
| 文件 | `serial_bridge_node_v1.py` | `serial_bridge_node.py` |
| 对应步骤 | **Step 2** | **Step 6**（Step 7 加 `parse_line`） |
| 发布时机 | 收到一帧就 `publish` | 定时器按 `publish_rate` 发布 |
| v2 四字段 | `float()` 失败 → **丢弃** | ✅ 解析，取 `fields[1]` |
| 600 字节超长行 | → `inf`（已知局限） | ✅ 被 `raw_line_max=256` 拦掉 |
| 断线重连 / 陈旧保护 | 无 | 有 |

**`~/ros_study` 里现在用的是加固版。** 想复现 Step 2 的现象请用 v1。

> 配套自测脚本已同步支持两个版本：
> `/usr/bin/python3 无硬件自测/test_bridge_logic.py --impl v1|hardened|both`（默认 `hardened`）
> 实测（2026-09-22）：`v1` 全绿、`hardened` 5 项断言全绿。

---

## 〇、先读这段：这份教程的定位与边界

**一句话目标**

> 让一块真实的 STM32 通过 USB 串口，把数据变成一条 ROS2 话题，并在 `rqt_plot` 里画出真实曲线。

**为什么这一步是你整条学习路线里性价比最高的一步**

阶段一练的是「没有硬件时把接口定死」；阶段二练的是「真数据进来之后，接口要扛得住脏东西」。
前者的能力是**设计**，后者的能力是**工程**。本体工程师 80% 的日常就是后者：串口、CAN、以太网，
协议一改、线一拔、板子一重启，你就得让整条链路活着。

**术语澄清（避免和另一份文档打架）**

| 说法 | 出处 | 含义 |
|---|---|---|
| 阶段一 / 阶段二 / 阶段三 | 本文档 | **ROS2 21 讲结业项目 `sensor_bridge` 的三个阶段**（纯软件 → 接硬件 → 自定义消息） |
| 阶段一 / 阶段二 / 阶段三 | `具身智能B站课程体系与18个月进阶路径.md` | 18 个月大路线（入门 / 进阶 / 实战），粒度完全不同 |

本文从头到尾只讲**前者**。你问的「阶段二」= 结业清单里的「阶段 2：接硬件」。

**范围红线（这段比后面所有内容都重要）**

阶段二的时间预算是 **2 个晚上 + 1 个周末**，不是「STM32 全面复习」，更不是「把 MCU 通信协议栈学一遍」。
下面这张表里的东西，看到就跳过，写进「以后再说」清单：

| 看到就想加的东西 | 为什么现在不加 |
|---|---|
| CRC / 校验和 / 帧头帧尾 / 转义 | 电平可靠时纯属自娱自乐；等你要上 CAN → 以太网时再统一学 |
| 二进制协议 / 结构体直接 memcpy | 阶段三的自定义消息会覆盖这个需求，现在加等于做两遍 |
| DMA + 环形缓冲 + 空闲中断 | 这是 MCU 侧的性能优化，跟 ROS2 桥没关系；先把 `printf` 跑通 |
| micro-ROS / 把 ROS2 塞进 STM32 | 完全另一条技术路线，学完对「本体工程」帮助有限，且极易陷进去 |
| ros2_control / 双向控制（PC 发指令给板子） | 这是阶段三之后的事（还需要 service / action） |
| 硬件流控 RTS/CTS | 你现在波特率低、数据量小，用不上 |

**唯一可以「多做」的一件事**：把协议设计成「一行多字段」（本文 Step 7）。
这不是扩范围，这是给阶段三铺路，成本 20 行代码。

---

## 一、阶段二的明确目标与范围

### 1.1 目标：四个层次，缺一层都不算完成

| 层次 | 目标 | 怎么算达标 |
|---|---|---|
| **L1 功能** | STM32 → USB 串口 → `/sensor/value` 话题 → 可视化 | `ros2 topic echo` 看得到数、`rqt_plot` 画得出线 |
| **L2 工程** | 断线能自愈、乱码不崩、设备名不漂移、频率可验证 | 拔线插线后节点自动恢复，无需重启 |
| **L3 能力** | 能独立设计「一条串口链路」的线程模型与发布策略 | 能口述讲清「为什么读串口必须另开线程」 |
| **L4 衔接** | 协议字段定好，阶段三只做「打包成自定义 msg」 | 协议文档 v2 写完了，阶段三不用重新设计协议 |

**L2 是这一阶段真正的重点。** L1 你一个晚上就能摸出来，L2 才是「工程师」和「会跑 demo 的人」的分界线。

### 1.2 范围：做什么 / 不做什么

**做（In Scope）**

1. 串口链路：`pyserial` 读串口 → 按行解析 ASCII → 发布 `std_msgs/msg/Float32`
2. 线程模型：读线程 + 发布定时器，二者解耦
3. 健壮性：设备不存在 / 断线 / 乱码 / 超长行 / 陈旧数据，五种情况都不崩
4. 设备名固化：udev 规则把 `/dev/ttyUSB0` 变成 `/dev/stm32_bridge`
5. 参数化：`port` / `baud` / `publish_rate` / `stale_timeout` 全部走 ROS2 参数
6. MCU 侧：`printf` 重定向到 UART，按行输出
7. 可观测：`ros2 topic hz` 验证频率、`ros2 bag record` 留证据

**不做（Out of Scope，写进"以后再说"）**

二进制协议、CRC、DMA、micro-ROS、双向控制、ros2_control、硬件流控、多串口管理。

### 1.3 交付骨架（先看终点）

```
STM32 固件                 PC 侧（Ubuntu + ROS2 Humble）
┌──────────────┐           ┌──────────────────────────────────────┐
│ 传感器/电位器 │           │  /dev/ttyUSB0  →  /dev/stm32_bridge   │
│      ↓       │  USB      │        ↓                              │
│  printf()    │ ────────▶ │  读线程：readline → parse_line        │
│  "1.234\n"   │  115200   │        ↓ 最新值 + 时间戳               │
└──────────────┘  8N1      │  发布定时器：10Hz 检查是否陈旧 → publish│
                           │        ↓                              │
                           │   /sensor/value  (std_msgs/Float32)   │
                           │        ↓              ↓               │
                           │   monitor_node     rqt_plot           │
                           └──────────────────────────────────────┘
```

---

## 二、从阶段一到阶段二的前置条件

前置条件分三类：**代码返工 3 项、环境 3 项、认知 3 问**。
前两类不满足会直接卡住你，第三类不满足会让你「跑通了但不知道为什么」。

### 2.1 代码返工（3 项，约 30 分钟）

我看了你 `~/ros_study/src/sensor_bridge/` 的现状，有 3 处需要先收尾。**这三处不是补作业，它们恰好是阶段二会直接复用的能力。**

#### 返工 ① `stats_node.py` 的窗口清零没生效（必改）

`~/ros_study/src/sensor_bridge/sensor_bridge/stats_node.py` 的 TODO-5：

```python
# ❌ 现状：用 == 做了「比较」，语句本身没有任何副作用
self.count == 0
self.total == 0.0
self.sq_total == 0.0
```

`==` 是判断，不是赋值。这三行的效果是「算了一下，然后扔掉」。
后果：统计量永不清零 → 输出的是**开机到现在的累计统计**，而不是需求要的「最近 5 秒的窗口统计」。

```python
# ✅ 正确
self.count = 0
self.total = 0.0
self.sq_total = 0.0
```

顺手把那句 `if self.count > 0:` 删掉——上面已经 `if self.count == 0: ... return` 了，再判一次是死代码。

**为什么这条和阶段二有关**：阶段二的 `serial_bridge_node` 里，你要统计「接收帧率 / 坏帧数 / 连接状态」，
用的就是同一套「O(1) 累加 + 定时窗口汇报」的结构。这里不清零，那里照样不清零。

验证：

```bash
cd ~/ros_study && colcon build --symlink-install --packages-select sensor_bridge
source install/setup.bash
# 终端 A
ros2 run sensor_bridge my_sensor_wave
# 终端 B
ros2 run sensor_bridge stats_node
```

期望：每 5 秒打印一行的「帧数」在 45~55 之间摆动（10Hz × 5s = 50 帧），**不是**一直递增。

#### 返工 ② 补上阶段一 A5 的概念题（必答）

阶段一 A5 是空白的，而它正好是阶段二天天要用的判断：

- (a) 改 `declare_parameter('noise', 0.0)` 的默认值，需要重新 `colcon build` 吗？
- (b) 改 `setup.py` 里 `entry_points` 的名字呢？

答案：
- (a) **不需要**。`--symlink-install` 下 Python 源文件是软链，改完重启节点即生效。
- (b) **需要**。`entry_points` 属于**安装元数据**，会写进 `egg-info`，必须重新 build。

映射到阶段二：
- `serial_bridge_node.py` 里改「解析逻辑、超时值」→ 不用 build
- 改 `setup.py` 加 `'serial_bridge = sensor_bridge.serial_bridge_node:main'` → **必须 build**
  （而且旧的可执行名会残留在 `install/` 里，实在乱了就删掉 `build/` + `install/` 再 build）

**记住这条判据**：改的是「代码」还是「安装清单」？代码不用 build，清单要 build。

#### 返工 ③ 想清楚「什么时候该拆线程 / 拆节点」（必答）

阶段一 A8 你只写了「有影响」。阶段二会正面撞上这个问题，而且更狠：

> `serial.readline()` 是**阻塞**调用——读不到数据时它会一直等。
> 如果它跑在 `rclpy.spin()` 所在的线程里，整个节点就**停摆**了：
> 定时器不响、参数服务不应答、`ros2 node info` 卡死、Ctrl+C 关不掉、甚至 `/rosout` 不更新。

所以阶段二的核心设计决策就是：**把阻塞的 IO 关到另一个线程里去**。
A8 的两条路（拆节点 / 换执行器+回调组）在阶段二都不如「自己开一个读线程」直接——
因为串口 IO 不是 ROS 回调，`MultiThreadedExecutor` 管不着它。

### 2.2 环境准备（3 项，约 15 分钟）

#### ① 装 `pyserial` —— 注意装到「正确的 python」

**我已经在你机器上核实过：系统 python3.10 目前没有 `serial` 模块。**
这个包必须装给 ROS2 用的那个解释器。

```bash
# 确认 ROS2 用的是哪个 python
head -1 /opt/ros/humble/bin/ros2      # → #!/usr/bin/python3  （即 3.10.12）

# ✅ 推荐：走 apt，一劳永逸，不受 PEP 668 限制
sudo apt install -y python3-serial

# 验证（必须用 /usr/bin/python3，不要用 which python3）
/usr/bin/python3 -c "import serial; print('pyserial', serial.__version__)"
# 期望输出类似：pyserial 3.5
```

⚠️ **不要** `pip install pyserial`（Ubuntu 22.04 会报 `externally-managed-environment`），
更**不要**用虚拟环境或 3.13 的 pip 装——那会造成本章风险表里的头号坑：
**pip 说装好了，节点却说 `ModuleNotFoundError: No module named 'serial'`**。

#### ② 拿到串口权限（`dialout` 组）

```bash
# 看自己是否已在组里（输出有 dialout 就行）
groups | grep dialout

# 不在的话加入，然后【注销并重新登录】（重开终端不够，组身份只在登录时刷新）
sudo usermod -aG dialout $USER

# 插上板子后看权限，期望 crw-rw---- root dialout
ls -l /dev/ttyUSB0
```

> 本机沙箱里我拿到的是 root 身份，看不到你登录会话的组信息，所以这一条请你自己跑一次确认。
> 如果 `ls -l` 显示的属组不是 `dialout`，说明驱动给了别的组名 —— 用实际组名替换上面的命令。

#### ③ 工具与依赖

```bash
sudo apt install -y socat            # 建立虚拟串口对（★ 阶段二最关键的工具）
python3 -c "import serial.tools.list_ports" && echo "serial tools ok"
```

`socat` 的意义：**让你在板子还没接上、固件还没写的时候，就把 PC 侧整条链路跑通**。
阶段二的 70% 工作量在 PC 侧，这 70% 不该等硬件。

### 2.3 硬件清单

| 项 | 说明 | 备注 |
|---|---|---|
| STM32 开发板 | 已有一块即可，F1/F4 都行 | 不确定型号不影响本教程 |
| USB 转串口 | CH340 / CP2102 模块，或板载 USB-TTL | 板子自带 ST-Link 的也可以（有个 VCP 口） |
| 杜邦线 ×3 | TX、RX、GND | **共地是必须的** |
| 一根可测的模拟量源 | 电位器 / 光敏 / 板载温度传感器 / 甚至 ADC 悬空 | 要让曲线「看得出在变」，否则无法验收 |
| USB 线 | 一根能出数据的（有些线只供电） | 真常见，别怀疑人生，先换线 |

### 2.4 认知前置（3 个问题，想不清就先别写代码）

**Q1：STM32 发 `printf("%.3f\n", v)` 会不会有问题？**

会。两点：
- Cortex-M 上用 `printf` 打浮点，Keil/ARMCC 需勾 **Use MicroLIB**；GCC 要加 `-u_printf_float` 链接选项。否则浮点格式化**静默失效**（打出来是空或者乱）。
- 浮点格式化在 MCU 上很慢（几百微秒到毫秒级），10Hz 无所谓，1kHz 就是灾难。

所以阶段二推荐**两段式**：
- 第一版：直接 `printf("%.3f\r\n", v)`，先跑通（简单优先）
- 加固版：整数化传输 `printf("%ld\n", (long)(v * 1000.0f))`，PC 端用 `scale` 参数乘回 0.001

**Q2：为什么不能写死 `/dev/ttyUSB0`？**

因为那是内核**按枚举顺序**分配的临时名字。插两块串口设备、换个 USB 口、先插板子后插别的东西——
名字就变了。真机上这会导致「昨天好好的，今天找不到设备」。
正规做法：udev 规则按 **USB VID:PID（+ 序列号）** 匹配，固定成一个别名（本文 Step 5）。

**Q3：为什么发布要用「定时器 + 最新值缓存」，而不是「收到一帧就发一帧」？**

两个理由，第二个是重点：
1. **频率解耦**：STM32 想发多快都行，ROS2 侧下游看到的是等间隔数据（和阶段一「时间轴自己累加、槽点独立定时器」是同一条设计原则）。
2. **陈旧数据保护**：如果板子挂了、线松了，你手上那个「最新值」会变成几分钟前的鬼魂。定时器每帧检查一次「这值是不是太旧了」，旧了就不发。
   > **下游绝对不能拿到一个「你以为实时、其实是五分钟前」的值。** 这在闭环控制里是会撞坏东西的，在数据记录里是会让整份 bag 报废的。

---

## 三、分步执行流程

八个步骤。每步都标了**时间盒**和**产出物**。卡住超过时间盒的 1.5 倍就记下来往下走——
**别让一个坑吃掉整个阶段二**（见第六节止损规则）。

| Step | 内容 | 时间盒 | 是否依赖硬件 |
|---|---|---|---|
| 0 | 环境与权限确认 | 20 min | 否 |
| 1 | 虚拟串口跑通链路（socat） | 45 min | **否** |
| 2 | 写 `serial_bridge_node` 最小闭环 | 60 min | 否 |
| 3 | STM32 固件：printf 重定向 + 按行发送 | 45 min | 是 |
| 4 | 接真硬件，验证频率与曲线 | 45 min | 是 |
| 5 | udev 固定设备名 | 30 min | 是（要插板子） |
| 6 | 工程化加固（断线自愈 / 异常不崩 / 参数化） | 90 min | 是 |
| 7 | 协议 v2 多字段 + bag 录制（衔接阶段三） | 60 min | 是 |

---

### Step 0 · 环境与权限确认

**做什么**：把 2.2 的三条全部跑一遍，结果记在一张纸上。

**关键命令**

```bash
# 1) ROS2 环境
source /opt/ros/humble/setup.bash
source ~/ros_study/install/setup.bash
ros2 --version

# 2) pyserial
/usr/bin/python3 -c "import serial; print(serial.__version__)"

# 3) 权限
groups | grep dialout

# 4) 工具
which socat && socat -V | head -1
```

**产出物**：一张「环境自检 4 行表」，其中 pyserial 那行必须是具体版本号，不能是「应该装了」。

```bash
# 2) pyserial
liang@liang-Vivobook-ASUSLaptop-K5504VA-K5504VA:~$ /usr/bin/python3 -c "import serial; print(serial.__version__)"
3.5
# 3) 权限
groups | grep dialout
liang adm dialout cdrom sudo dip plugdev lpadmin lxd sambashare
# 4) 工具
liang@liang-Vivobook-ASUSLaptop-K5504VA-K5504VA:~$ which socat && socat -V | head -1
/usr/bin/socat
socat by Gerhard Rieger and contributors - see www.dest-unreach.org

```



---

### Step 1 · 虚拟串口跑通链路（不碰硬件）

**这一步是阶段二的加速器。** 先造一对虚拟串口：一头当「STM32」，一头当「桥」。

**1.1 建虚拟串口对**

```bash
# 前台运行，观察日志；Ctrl+C 结束
socat -d -d pty,raw,echo=0,link=/tmp/ttyV0 pty,raw,echo=0,link=/tmp/ttyV1
```

- `/tmp/ttyV0` → **假装是 STM32**，用来「发」
- `/tmp/ttyV1` → **桥这一侧**，用来「收」

```bash
liang@liang-Vivobook-ASUSLaptop-K5504VA-K5504VA:~$ socat -d -d pty,raw,echo=0,link=/tmp/ttyV0 pty,raw,echo=0,link=/tmp/ttyV1
2026/09/20 19:55:01 socat[6143] N PTY is /dev/pts/1
2026/09/20 19:55:01 socat[6143] N PTY is /dev/pts/2
2026/09/20 19:55:01 socat[6143] N starting data transfer loop with FDs [5,5] and [7,7]
```



**1.2 灌数据**

用本目录 `无硬件自测/virtual_serial_sender（虚拟串口发送器）.py`（已写好）：

```bash
/usr/bin/python3 /home/liang/桌面/具身智能学习路线/08_阶段二_STM32串口接入/无硬件自测/virtual_serial_sender.py --port /tmp/ttyV0 --rate 20
```

它会做三件事，**第 2、3 件是故意使坏**，提前把阶段二的所有异常暴露出来：

1. 按 20Hz 发正弦：`1.234\r\n`
2. 启动时先发一行垃圾**上电横幅**：`STM32 Ready! boot v1.0\r\n`（真板子上电都会打印这个）
3. 每 5 秒插一行**乱码**：`0x1F 0x9C ...\r\n`、以及一行**超长行**（600 字节）

**1.3 用 `miniterm` 先确认数据真的在流**

```bash
/usr/bin/python3 -m serial.tools.miniterm /tmp/ttyV1 115200
```

看到正弦数字滚动 → 虚拟链路成立。Ctrl+] 退出。

**产出物**：`socat` 能起、`/tmp/ttyV0` 与 `/tmp/ttyV1` 存在、`miniterm` 里能看到数据流。

> ⚠️ 坑：`socat` 一旦 Ctrl+C，`/tmp/ttyV0`、`/tmp/ttyV1` 这两个软链会消失。
> 每次重开 socat 后要重新确认软链存在（`ls -l /tmp/ttyV*`）。

---

### Step 2 · 写 `serial_bridge_node` 最小闭环

**第一版只要 40 行**：能在独立线程读串口、能发布 `Float32`、能被 Ctrl+C 干净关掉。
先不要断线重连、先不要统计——**能跑通比写得全重要**。

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""serial_bridge_node —— 第一版：最小闭环"""

import threading

import rclpy
import serial
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32


class SerialBridgeNode(Node):

    def __init__(self, name):
        super().__init__(name)
        self.declare_parameter('port', '/tmp/ttyV1')
        self.declare_parameter('baud', 115200)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        self.pub = self.create_publisher(Float32, '/sensor/value', 10)

        # timeout 必须给！不给的话 readline 会永久阻塞
        self._ser = serial.Serial(port, baud, timeout=0.2)

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()
        self.get_logger().info(f'已打开 {port} @ {baud}')

    # ── 读线程：阻塞的活儿全在这里 ────────────────────────────
    def _reader_loop(self):
        while not self._stop.is_set():
            line = self._ser.readline()          # 读到 \n 或超时返回
            if not line:
                continue                          # 超时，啥也没读到
            text = line.decode('ascii', errors='ignore').strip()
            if not text:
                continue
            try:
                value = float(text)
            except ValueError:
                continue                          # 上电横幅 / 乱码，忽略
            msg = Float32()
            msg.data = value
            self.pub.publish(msg)                 # rclpy 的 publish 线程安全

    def destroy_node(self):
        self._stop.set()
        try:
            self._ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode('serial_bridge')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**注册入口（`setup.py`）**

```python
'serial_bridge = sensor_bridge.serial_bridge_node:main',
```

```bash
cd ~/ros_study
colcon build --symlink-install --packages-select sensor_bridge
source install/setup.bash
```

**验证**

```bash
# 终端 1：socat（Step 1）
# 终端 2：灌数据   ⚠️ 必须带 --v1，理由见下方「踩坑记录」
/usr/bin/python3 "/home/liang/桌面/具身智能学习路线/08_阶段二_STM32串口接入/无硬件自测/virtual_serial_sender.py" \
    --port /tmp/ttyV0 --rate 20 --v1
# 终端 3：桥
ros2 run sensor_bridge serial_bridge --ros-args -p port:=/tmp/ttyV1
# 终端 4：看数据
ros2 topic echo /sensor/value
ros2 topic hz /sensor/value
```

> ⚠️ **踩坑记录：漏掉 `--v1` 会看到什么（2026-09-20 实测）**
>
> `virtual_serial_sender.py` **默认发 v2 四字段**：`214,2768,3434,2468`。
> 而这一版桥节点用 `float(text)`，**只认单值**。于是正常帧全军覆没，
> 你会在终端 4 看到：
>
> | 现象 | 数字 | 它在说什么 |
> |---|---|---|
> | `ros2 topic echo` 满屏 `data: .inf` | — | **不是没收到数据**，是正常帧全被丢了 |
> | `ros2 topic hz` == **0.198 Hz** | 0.198 ≈ 1/5.05 | 正好等于坏数据块每 5 秒来一次 |
> | 为什么偏偏是 `inf` 活下来 | — | 坏数据块里那条 600 字节超长行内容**全是数字**，`float('9'*600)` 不报错，返回 `inf` |
>
> **从频率反推数据来源**，是这一步最值钱的诊断训练：
> `0.198 Hz` 不是随机数，它精确指向 `virtual_serial_sender.py` 里
> `--garbage-every 5.0` 那几行 `ser.write(...)`。
> 一旦把频率和某段代码对上，排查范围就从整个工程缩到三行。
>
> 参考做法：在沙箱里 mock 掉 `serial.Serial`，把 20 行 v2 数据 + 1 个坏数据块喂进去，
> 实测结果是 **24 行输入 → 只发布 1 帧 `[inf]`**，与 0.198 Hz 完全吻合。

**v2 四字段的解析是 Step 4 的活。这一步先用 `--v1` 把闭环跑通。**

**这一步的自查（很重要，问自己，不是问电脑）**

1. 如果我把 `timeout=0.2` 删掉，会发生什么？为什么？
2. 读线程里 publish 会不会和 `spin` 打架？（答案：不会，`publish` 线程安全。但**同时读写同一个成员变量**才需要小心，见 Step 6）
3. Ctrl+C 后进程有没有 Traceback？有的话说明 `destroy_node` 没兜住。

**产出物**：`serial_bridge_node.py`（最小版）+ `ros2 topic echo` 能看到 `/tmp/ttyV1` 来的正弦数据。

> ✅ **实测通过（2026-09-20）**
>
> | 检查项 | 实测值 | 判据 |
> |---|---|---|
> | `ros2 topic hz` | **20.17 ~ 20.24 Hz** | ≈ 20.2（20 正常帧 + 每 5 秒 1 帧 inf） |
> | `ros2 topic echo` 单帧 | `data: 337.0 → 873.0` | 落在 0 ~ 3300 mV |
> | 数值可信度 | 取连续 10 帧反推相位 → **最大偏差 0 mV** | 10/10 落在同一条 0.2Hz 正弦上 |
> | 波形形状 | 逐帧增量 42→46→…→76 递增，二阶差 ≈ 4~5 mV | 理论 5.1 mV，正弦上升加速段 |
> | Ctrl+C | 无 Traceback，干净回到提示符 | 读线程已退出 |
>
> **`hz` 输出里的三个指纹，以后要会读：**
>
> | 指纹 | 从 → 到 | 含义 |
> |---|---|---|
> | `average rate` | 20.00 → **20.2** | 多出来那 0.2 Hz 就是每 5 秒那 1 帧 `inf` |
> | `min` | 0.049s → **0.005s** | 那 5ms 是 inf 帧与相邻正常帧之间的夹缝 |
> | `std dev` | 0.00044 → **0.0049** | 约 11 倍，全部由这 1 个异常间隔贡献 |
>
> 注意 `min` 从 0.049s（= 1/20.4Hz）掉到 0.005s 这件事本身：
> 它精确说明「坏数据块是紧跟在某一帧正常数据之后被写入的」，
> 也就是 `virtual_serial_sender.py` 里先 `ser.write(line)` 再 `ser.write(b'9'*600)` 的顺序。
> **会读这三个指纹，以后所有「频率不对」的问题都能一眼定位。**

---

### Step 3 · STM32 固件：printf 重定向 + 按行发送

**目标**：让板子每 100ms 打一行 `<数值>\n`。

> 📌 **你手上的工程已经做好了**（2026-09-22 复核）：
> `~/stm32_ws/adc_serial_nocube/` —— 免 CubeMX 版，`make` 一次过，Flash 占用 15352 字节。
> 本节的代码是从那份工程里抽出来的讲解，**不是让你从零敲一遍**。
> 直接对着 `Core/Src/main.c` 读这一节，效率最高。

> **前置：Linux 下没有 Keil 怎么办**
>
> Keil MDK 是 **Windows-only**，但「STM32 开发」不等于「Keil」。Linux 侧用 ST 官方的
> GCC 工具链，**代码一个字都不用改** —— 下面 3.1 的两段重定向写法里，`_write` 那一段
> 就是给 GCC 的（`fputc` 那段是 Keil 的）。
>
> 完整的环境搭建（工具链安装、CubeMX 配置、编译、`st-flash`/`openocd` 烧录、
> 接线、故障速查）见同目录：
> **《Step3 操作手册：Linux + F103C8T6 固件开发（免 Keil）.md》**
>
> 三句话版本：
>
> ```bash
> # ① 装工具链（包名已在 jammy 逐个核实）
> sudo apt install -y gcc-arm-none-eabi binutils-arm-none-eabi libnewlib-arm-none-eabi \
>     gdb-multiarch openocd stlink-tools stm32flash
> # ② CubeMX 新建工程时，Toolchain 选 Makefile（不是 MDK-ARM）
> # ③ 烧录
> st-flash --reset write build/xxx.bin 0x8000000
> ```
>
> ⚠️ **两个必看的坑**（手册 3.3 / 10 章有详解）：
> 1. CubeMX 里 **`SYS → Debug` 必须选 `Serial Wire`**，
>    漏了这条，**一次烧录后 ST-Link 就永久连不上**，只能靠拨 `BOOT0` 救。
> 2. 工程路径**不能含中文**（别放桌面），CubeMX 和 make 都受不了。

**3.1 `printf` 重定向（Keil MDK / ARMCC）**

```c
/* 需要勾选 Options → Target → Use MicroLIB，否则 fputc 不会被调用 */
#include <stdio.h>

int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
```

GCC（STM32CubeIDE / Makefile）写法不同，用 `_write`：

```c
#include <sys/stat.h>
int _write(int fd, char *ptr, int len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}
```

> ⚠️ **2026-09-22 实测补充**：你那套免 CubeMX 工程（`~/stm32_ws/adc_serial_nocube/`）
> **已经有一份 `_write` 了**，落在 `Core/Src/syscalls.c`。**不要再往 `main.c` 里写第二份** ——
> `-lnosys` 提供的是弱符号，两份强符号会在链接期直接失败：
>
> ```
> ld: w2.o: in function `_write': multiple definition of `_write'; w1.o:w1.c: first defined here
> ```
>
> 还有一行同样不能漏：**`setvbuf(stdout, NULL, _IONBF, 0)`**。
> Makefile 用了 `-specs=nano.specs`，nano 的 stdout 是**全缓冲**（约 1024 字节），
> 按每帧 6 字节算要攒 170 帧 —— **大约 17 秒才吐一次**，而且一次吐一大坨。
> 少了这行你会以为程序没跑，然后去查波特率、接线、ADC，越查越远。
> 详见配套《Step3 操作手册》§5.1 / §5.2。

**3.2 采集 + 发送主循环**

```c
/* 假设已配置 ADC1 + 串口 USART1 @115200 8N1 */
static uint32_t adc_val;
static uint16_t seq = 0;

while (1)
{
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    adc_val = HAL_ADC_GetValue(&hadc1);         /* 0 ~ 4095 */

    /* 换算成工程量 —— 用整数化输出，避开 MCU 上的 float printf */
    int32_t milli = (int32_t)((float)adc_val * 3.3f / 4095.0f * 1000.0f);

    /* 结尾必须是 \n，PC 侧的 readline 靠它切帧 */
    printf("%ld\n", (long)milli);

    HAL_Delay(100);                             /* 10Hz */
}
```

**3.3 四个必须记住的点**

| 点 | 说明 |
|---|---|
| **行尾必须带 `\n`** | `readline()` 靠 `\n` 判断「一帧结束」。用 `\r\n` 也行，PC 端 `strip()` 会处理掉 `\r` |
| **`setvbuf(stdout, NULL, _IONBF, 0)` 不能漏** | nano.specs 下 stdout 是**全缓冲**（≈1024B）→ 每帧 6 字节要攒 170 帧、**约 17 秒才吐一次**。漏了它你会去查波特率和接线，越查越远（2026-09-22 补） |
| **不要在中断里 `printf`** | `HAL_UART_Transmit` 阻塞发送，在中断里调用会拖垮系统。要么主循环发，要么用「中断/DMA 发送 + 环形缓冲」 |
| **波特率两端必须一致** | 板子上 `MX_USART1_UART_Init` 里写的是 115200，PC 端参数也得是 115200。不一致 → 全是乱码 |

**3.4 先用电脑单独验一下板子（别急着开 ROS2）**

在接 ROS2 之前，先确认板子本身在正常发数据：

```bash
ls -l /dev/ttyUSB* /dev/ttyACM*        # 找设备名
/usr/bin/python3 -m serial.tools.miniterm /dev/ttyUSB0 115200
```

看到数字在滚 → 板子 PASS。看不见 → **先解决这一步，别往下走**。
（这一步能把「板子的问题」和「ROS2 的问题」隔离开，是最省时间的做法。）

**产出物**：能编译烧录的固件 + 一段「板子自证」的 miniterm 截图或粘贴记录。

---

### Step 4 · 接真硬件，验证频率与曲线

```bash
ros2 run sensor_bridge serial_bridge --ros-args -p port:=/dev/ttyUSB0 -p baud:=115200 -p scale:=0.001
```

> `scale(规模)` 参数到 Step 6 才正式加进代码。Step 4 如果板子发的是整数化毫伏，
> 先在代码里写一句 `value = float(text) * scale`，`scale` 从参数读，默认 1.0。

**三个必须跑的验证**

```bash
# ① 频率对不对（10Hz 发、10Hz 收，误差 <5%）
ros2 topic hz /sensor/value

# ② 数值动不动（拧电位器 / 用手捏温度传感器 / 挡住光敏）
ros2 topic echo /sensor/value

# ③ 曲线画不画得出来（注意：要填到字段级！）
ros2 run rqt_plot rqt_plot
#   Topic 框里填：/sensor/value/data      ← 只填 /sensor/value 是画不出来的（阶段一 A7 考过）
```

**产出物**：`ros2 topic hz` 的输出（记录实测值）+ 一条随物理量变化的曲线。

---

### Step 5 · udev 固定设备名

**5.1 查出设备的身份**

```bash
lsusb
# 例： Bus 001 Device 005: ID 1a86:7523 QinHeng Electronics CH340 serial converter
#                                   ^^^^ ^^^^  ← 这就是 idVendor / idProduct

# 更精确：直接问内核这个 tty 设备的祖先信息
udevadm info -a -n /dev/ttyUSB0 | grep -E 'idVendor|idProduct|serial' | head
```

**5.2 写规则**（模板见 `参考源码/99-stm32-bridge.rules`）

```bash
sudo tee /etc/udev/rules.d/99-stm32-bridge.rules > /dev/null <<'EOF'
# STM32 串口桥固定别名
# CH340: 1a86:7523   CP2102: 10c4:ea60   FT232: 0403:6001   STM32 VCP: 0483:5740
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", \
  SYMLINK+="stm32_bridge", GROUP="dialout", MODE="0660"
EOF
```

> ⚠️ **把 VID:PID 换成 `lsusb` 里你实际的**。上面这行只是 CH340 的示例。
> `MODE="0660"` 比 `0666` 稳；如果只你自己用，`0666` 也行，但别在多人机器上这么干。

**5.3 生效并验证**

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -l /dev/stm32_bridge          # 期望： -> ttyUSB0，属组 dialout
```

**5.4 拔插压力测试（必做）**

拔掉 → 插上 → 再 `ls -l /dev/stm32_bridge`，**重复 3 次**。
名字每次都一样 = PASS。换一个 USB 口再试一次。

**产出物**：`/etc/udev/rules.d/99-stm32-bridge.rules` + 3 次拔插都稳定的验证记录。

---

### Step 6 · 工程化加固（阶段二的重头戏）

现在把 Step 2 那个「能跑」的节点，改成「拔线也不慌」的节点。
**最终版完整代码见 `参考源码/serial_bridge_node.py`**，这里讲清每一处「为什么要这么写」。

#### 6.1 线程模型：读线程 + 发布定时器，用最新的值解耦

```
读线程（阻塞 IO）                      发布定时器（ROS 时间轴）
─────────────────                     ──────────────────────
readline()  →  parse_line()           取 self._latest
        ↓                                    ↓
  写 self._latest  ←─── 共享变量 ──────→  检查是否陈旧 → publish
  累加 rx/bad 计数                        按固定频率，等间隔
```

关键点：**读线程不 publish，发布定时器不碰串口**。各自只干一件事。

#### 6.2 五种异常，五种处理

| 异常 | 处理策略 |
|---|---|
| **设备不存在 / 打开失败** | 不崩。进入「未连接」状态，每 1 秒重试一次，日志**限流**（`throttle_duration_sec=5.0`） |
| **断线（拔 USB）** | `readline` 抛 `SerialException` 或一直返回空 → 关闭句柄、转入未连接状态、继续重试 |
| **解析失败（横幅/乱码）** | **静默**计数 `bad_frames += 1`，不逐条打日志。汇报时一起报 |
| **超长行 / 半包** | `readline` 加长度上限判断（如 > 256 字节直接丢弃），防止异常输入把内存吃爆 |
| **陈旧数据** | `now - last_rx_time > stale_timeout` 就**不发布**，并只告警一次 |

#### 6.3 一个容易被忽略的细节：`self._latest` 的读写

读线程写、定时器读，**同一个变量跨线程**。Python 的 GIL 让「单个 float 的赋值/读取」本身是原子的，
所以这里不需要锁。但如果你后面要传的是一整组字段（元组、dict），就可能读到「半新半旧」的组合——
那时候要么加 `threading.Lock`，要么保证「一次赋值一个不可变元组」。
**这是阶段三传自定义消息时会踩的坑，现在先记下这条判据。**

#### 6.4 参数表（最终形态）

| 参数 | 默认值 | 类型 | 作用 |
|---|---|---|---|
| `port` | `/dev/stm32_bridge` | string | 串口设备（固定别名，不写 ttyUSB0） |
| `baud` | `115200` | int | 波特率 |
| `publish_rate` | `10.0` | double | 发布频率（Hz），**结构参数**，启动读一次 |
| `stale_timeout` | `1.0` | double | 超过这么久没新数据就不发布（秒） |
| `scale` | `1.0` | double | 原始值缩放（整数化传输时传 0.001） |
| `report_period` | `5.0` | double | 状态汇报周期（秒） |
| `raw_line_max` | `256` | int | 单行最大长度，超过丢弃 |

注意 `port` / `baud` 是**结构参数**（打开句柄时就固定了），改它们需要重启节点；
`scale` 是**数据参数**，可以每帧现读、`ros2 param set` 立刻生效——
这条判据你在阶段一 C2 已经练过一遍。

#### 6.5 状态汇报（复用阶段一的能力）

每 `report_period` 秒打印一次，格式对齐阶段一 `stats_node` 的习惯：

```
[serial_bridge] 状态=已连接 | 收帧=98 | 坏帧=2 | 实测 9.83 Hz | 距上次数据 0.03s
```

**连接状态发生跳变时（未连接→已连接 / 已连接→未连接），单独打一条 info/warn。**
这比「每 5 秒重复报同一件事」有用得多——日志的价值在于「变化」，不在于「重复」。

**顺手把方差公式换成 Welford（理由和你想的不一样）**

你可能会听说 `σ² = E[v²] − E[v]²` 在「大偏置 + 小波动」时会**灾难性抵消**。
我实测过了（N=50 × 200 次）：**在 Python 的 float64 下它不会坏**——
偏置 2048±5、甚至 1e7±1，200 次里 0 次算出负数。所以**不是"现在就得救"**。

真正会炸的是 **`float32`**：偏置 25000±15 时 11/200 算出负数，2048±0.05 时 70/200，
1e6±1 时 88/200 —— 再被 `max(0.0, var)` 兜成 0，**标准差恒为 0 且不报任何错**。

**为什么这仍然和你有关系**：你的统计逻辑迟早要从 Python 挪到 C
（MCU 侧预处理、以后上边缘端），而嵌入式里 `float` 是默认选择。
**同一份代码在 float64 下对、搬到 float32 下错**，是最难查的一类 bug。

所以换 Welford 的正当理由是：**精度无关，float32 / float64 都准**。

```python
def _update(self, v):          # 每收一帧调一次，O(1) 内存
    self.n += 1
    d = v - self.mean
    self.mean += d / self.n
    self.m2 += d * (v - self.mean)
    # 要标准差时：math.sqrt(self.m2 / self.n)
```

代价只是每帧多几次乘除。完整实测数据见 `阶段一测试题与答案.md` 末尾「附2」。

**另一个相关判断**：如果你后面给假传感器加上 `noise`，会看到方差开始上下跳（±20% 量级）——
**那不是 bug**。n=50 时方差估计本身的统计涨落就是 `√(2/(n−1)) ≈ 20%`。
你之前看到方差 0.497 纹丝不动，恰恰因为那是**确定性正弦 + 零噪声**。

**给自己留一个「理论锚点」**：统计类输出如果没有理论值对照，
你只能看出「它在动」，看不出「它对不对」。
阶段一的窗口统计之所以一眼能验，就是因为它满足 `var + mean² ≡ A²/2` ——
接真板子前，先想清楚你这次的锚点是什么（ADC 满量程？传感器手册的噪声密度？）。

#### 6.6 `package.xml` 补依赖

```xml
<exec_depend>python3-serial</exec_depend>
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

忘了这条，别人 clone 你的仓库后跑不起来，而你本地一切正常——典型的「在我机器上是好的」。

#### 6.7 launch 一次启动

```python
# launch/serial_bridge.launch.py（完整见参考源码）
bridge = Node(package='sensor_bridge', executable='serial_bridge',
              name='serial_bridge', output='screen',
              parameters=[{'port': port, 'baud': baud,
                           'publish_rate': rate, 'scale': scale}])
monitor = Node(package='sensor_bridge', executable='monitor_node',
               name='monitor', output='screen',
               parameters=[{'report_period': 2.0}])
```

```bash
ros2 launch sensor_bridge serial_bridge.launch.py
ros2 launch sensor_bridge serial_bridge.launch.py port:=/dev/ttyUSB0 baud:=115200
```

**`output='screen'` 到底做了什么（源码依据：`launch/actions/execute_process.py` + `launch/logging.get_output_loggers()`）**

| 取值 | stdout | stderr |
|---|---|---|
| **`'log'`（默认）** | 只进 launch 主日志文件 | **既进日志文件，也上屏** |
| `'screen'` | 上屏 | 上屏 |
| `'both'` | 上屏 + 进日志文件 | 上屏 + 进日志文件 |
| `'own_log'` | 各写独立日志文件 | 各写独立日志文件 |

⚠️ **一个反直觉的点**：`rclpy` 的 `get_logger().info()` 走的是 **stderr**，
所以**就算不写 `output='screen'`，你也能在终端看到日志**。
`output='screen'` 真正多捞上屏的，是 Python 裸 `print()` 打出的 **stdout**。
把这条记住，以后就不会出现「我明明设了 output='screen'，为什么还是没输出」这种困惑——
先分清你那个输出走的是 stdout 还是 stderr。

**没上屏的日志去哪了**：`~/.ros/log/<启动时间戳>/launch.log`（主日志）。

**顺带一句 `emulate_tty`**：它默认是 `False`，此时子进程 stdout 是**管道**（块缓冲），
节点里的 `print()` 会攒够一个缓冲区才吐出来，看起来像"卡住不动"。
参考源码里给它设了 `True`（拿到伪终端 → 行缓冲 + 保留颜色）。
用 `get_logger()` 的话本来就不受影响（Python 3.9+ 的 stderr 恒为行缓冲）。

**产出物**：加固版 `serial_bridge_node.py`、`serial_bridge.launch.py`、`package.xml` 依赖、五类异常的实测记录。

---

### Step 7 · 协议 v2 多字段 + bag 录制（衔接阶段三）

#### 7.1 为什么要多字段

阶段一 D1 已经埋了这个伏笔：`value` 和 `noise` 分在两条话题上会**帧不对齐**。
真板子上你会遇到同样的问题：ADC 值、温度、序号、时戳，本来就是**同一时刻的同一帧数据**。

v2 协议就这么定：

```
v1:  <value>\n                          例: 1234\n
v2:  <seq>,<value>,<raw_adc>,<temp>\n   例: 1024,1234,1533,27.5\n
```

- `seq`：帧序号（丢帧检测用，MCU 每次 +1，PC 端发现跳号就说明真丢帧了）
- `value`：主工程量
- `raw_adc`、`temp`：其他字段

**PC 侧解析**：`serial_protocol.py` 里的 `parse_line()` 按 `,` 切分，返回 `list[float]`，
`serial_bridge` 取 `fields[0]`。等阶段三定义了 `SensorData.msg`，
只要把「取 fields[0]」换成「填 msg 的四个字段」——**协议不用重新设计，桥的骨架不用改**。

这就是「先定协议再写代码」的收益。

#### 7.2 顺手补一个纯函数单测

`parse_line()` 不依赖 ROS、不依赖串口，所以能脱离 ROS 单独测：

```bash
cd ~/ros_study/src/sensor_bridge
/usr/bin/python3 -m pytest test/test_serial_protocol.py -v      # 16 passed
# 或者不装 pytest 也能跑（文件自带 __main__ 兜底）：
/usr/bin/python3 test/test_serial_protocol.py                   # 16/16 passed
```

**这是阶段二最值钱的工程习惯之一**：把纯逻辑从「IO + 框架」里剥出来，
它就能被测试、被复用、被搬去别的项目。你以后写 CAN 解析、写协议栈，都是同一招。

> ⚠️ **2026-09-22 实测踩坑：把教程附件里的 `无硬件自测/test_serial_protocol.py`
> 直接复制到 `test/` 下，跑 pytest 会报 `ModuleNotFoundError: No module named 'serial_protocol'`。**
>
> 根因：附件版只有一条 `sys.path.insert(0, '../参考源码')`，指的是**教程目录**里的附件文件夹；
> 复制进工程后，`<pkg>/参考源码` 根本不存在。
>
> 而报错长得很吓人 —— ROS2 的 pytest 挂了 `launch_testing` 插件，它会**先 `import` 每个测试模块**
> 去找 launch test 入口，于是 import 失败被包成十几层 `pluggy` 堆栈抛出来，看起来像 pytest 崩了。
> **遇到这种堆栈，直接翻到最底下几行**，`E   ModuleNotFoundError: ...` 那一行才是真因。
>
> 现在这份 `test/test_serial_protocol.py` 已经改成**三路回退**的 import：
> ① `from sensor_bridge.serial_protocol import ...`（source install 后）
> ② `from serial_protocol import ...`（源码树内包目录）
> ③ 教程附件目录。
> 从任何目录、带不带 ROS 环境都能跑 —— 已实测四种跑法全过。

#### 7.3 录一段 bag 当证据

```bash
ros2 bag record -o ~/bridge_test /sensor/value
# 跑 30 秒，Ctrl+C

ros2 bag info ~/bridge_test
# 看：Duration、Messages 数、Message frequency

# 回放（此时关掉桥，甚至拔掉板子，下游照样收到数据）
ros2 bag play ~/bridge_test
ros2 topic hz /sensor/value
```

**为什么这条算验收项**：它把「实时链路」变成了「可复盘的数据资产」。
你以后调算法、做回归测试、给别人复现，靠的都是 bag。

**产出物**：协议文档 v2（一页纸）、`serial_protocol.py` + 单测、一段 bag + `ros2 bag info` 输出。

---

## 四、关键产出物清单

### 4.1 文件级（都要能指出完整路径）

| # | 产出物 | 路径 | 完成 Step |
|---|---|---|---|
| 1 | 串口桥节点（加固版） | `~/ros_study/src/sensor_bridge/sensor_bridge/serial_bridge_node.py` | 2 / 6 |
| 2 | 协议解析纯函数模块 | `~/ros_study/src/sensor_bridge/sensor_bridge/serial_protocol.py` | 7 |
| 3 | 解析单测 | `~/ros_study/src/sensor_bridge/test/test_serial_protocol.py` | 7 |
| 4 | 桥的 launch | `~/ros_study/src/sensor_bridge/launch/serial_bridge.launch.py` | 6 |
| 5 | `setup.py` 入口 + `package.xml` 依赖 | 同目录 | 2 / 6 |
| 6 | udev 固定设备名规则 | `/etc/udev/rules.d/99-stm32-bridge.rules` | 5 |
| 7 | STM32 侧固件（printf 重定向 + 按行发送） | 你的 Keil/STM32CubeIDE 工程 | 3 |
| 8 | 协议文档 v1 / v2 | 一页纸，写清字段、行尾、波特率 | 7 |
| 9 | 一段 bag | `~/bridge_test/` | 7 |
| 10 | 实验记录（环境自检 + hz 实测 + 异常记录） | 一页纸 | 全程 |

### 4.2 能力级（这些才是真正带得走的）

- 能独立判断「哪里的 IO 必须挪出回调线程」
- 能独立设计「阻塞 IO + ROS 定时器」的线程模型
- 能解释「为什么不能把 /dev/ttyUSB0 写进代码」
- 能把「协议字段」和「消息类型」分开设计，先定协议再写代码
- 能把纯逻辑从框架里剥出来做单测

---

## 五、阶段二验收标准

**过 = A、B、C 三组全勾 + D 组能口述。** 任何一条依赖于「写死了设备名」或「必须重启才能恢复」的，一律返工。

### A 组 · 功能闭环（必过）

- [x] 用 `socat` 虚拟串口就能跑通整条链路，**不依赖硬件**
      验证：`ros2 topic echo /sensor/value` 能看到正弦数据
- [x] 接真板子，`ros2 topic hz /sensor/value` 稳定在设定值 ±5%
      验证：`-p publish_rate:=10.0` 时 hz 输出 9.5~10.5
- [x] 数值随物理量真实变化（拧电位器 / 捏温度传感器，曲线跟着动）
- [x] `rqt_plot` 能画曲线，Topic 填的是字段级（`/sensor/value/data`）
- [x] 改波特率（两端同步改成 9600）后仍能通 → 证明参数真的生效，不是摆设

### B 组 · 健壮性（必过）

- [x] **拔线不崩**：拔掉 USB，节点继续运行，日志最多一条 warn，**不刷屏**
- [x] **插线自愈**：插回 USB 后 ≤3 秒自动恢复发布，**不需要重启节点**
- [x] **坏数据不崩**：灌入横幅行 / 乱码 / 600 字节超长行，节点存活，坏帧计数增加
- [x] **启动时设备不存在也不崩**：`-p port:=/dev/不存在的设备`，节点活着，每 5 秒一条 warn

  > ⚠️ **2026-09-22 实跑：这条很容易「测了个假」。** 写成下面这样会**连 `rclpy.init` 都没进去**就退出：
  >
  > ```bash
  > ros2 run sensor_bridge serial_bridge --ros-args -p /dev/sss baud:=115200   # ❌ 缺 port:=
  > # [ERROR] [rcl]: Failed to parse global arguments
  > # RCLError: failed to initialize rcl: Couldn't parse parameter override rule: '-p /dev/sss'.
  > #   Error: Expected lexeme type (22) not found, search ended at index 8
  > # [ros2run]: Process exited with failure 1
  > ```
  >
  > `Expected lexeme type (22)` = 词法分析器在找 `:=`；`search ended at index 8` ——
  > `/dev/sss` 正好 8 个字符，读到末尾还没看见 `:=` 就绝望了。
  > **节点压根没起来，所以这条根本没验证到「设备不存在」的健壮性** —— 它死在参数解析阶段。
  >
  > ✅ **正确写法（每个参数都有自己的 `-p`，且必须是 `名字:=值`）**：
  >
  > ```bash
  > ros2 run sensor_bridge serial_bridge --ros-args -p port:=/dev/sss -p baud:=115200
  > ```
  >
  > 实测输出（`timeout 13` 后被杀，退出码 124 = **一直在跑，没崩**）：
  >
  > ```
  > [INFO]  [...]：serial_bridge 启动 | 设备=/dev/sss@115200 | 话题=/sensor/value | 发布=10Hz
  > [WARN]  [...]：打开 /dev/sss 失败（每 1s 重试）：[Errno 2] No such file or directory
  > [INFO]  [...]：状态=未连接 | 本窗口收帧=0 坏帧=0 | 实测 0.00 Hz | 距上次数据 -1.00s
  > [WARN]  [...]：打开 /dev/sss 失败（每 1s 重试）：...     ← 间隔 5.006s
  > [INFO]  [...]：状态=未连接 | ...                        ← 间隔 5.008s
  > ```
  >
  > **注意这个细节**：日志说「每 1s 重试」，但 warn 却是**每 5 秒一条** ——
  > 重试是每秒一次，日志用 `throttle_duration_sec=5.0` 限流。**这正是设计意图**：
  > 快速重试保证自愈，日志限流保证不刷屏。
  >
  > **`-p` 语法三条规矩**：① 必须写成 `名字:=值`；② 每个参数一个 `-p`，多个连用；
  > ③ 类型看字面量（`'xxx'`=字符串、`115200`=整数、`0.001`=浮点），字符串要用引号包住或确保无特殊字符。
- [x] **陈旧数据不发**：停掉发送端，`ros2 topic hz` 归零（而不是继续发旧值）
- [x] **Ctrl+C 干净退出**：无 Traceback，无 `KeyboardInterrupt` 堆栈
- [x] **长时间稳定**：连续跑 30 分钟，`ps -o rss= -p <pid>` 前后内存增长 < 5 MB

### C 组 · 工程规范（必过）

- [x] `ls -l /dev/stm32_bridge` 存在；**拔插 3 次 + 换 USB 口**，名字不变
- [x] 代码里搜不到 `ttyUSB0`（除了注释）——设备名全部走参数
- [x] `port` / `baud` / `publish_rate` / `stale_timeout` / `scale` 都是 ROS2 参数
      验证：`ros2 param list /serial_bridge`
- [x] `ros2 param set /serial_bridge scale 0.0005` 立刻改变输出，**无需重启**
- [x] `package.xml` 里有 `<exec_depend>python3-serial</exec_depend>`
- [x] `ros2 launch sensor_bridge serial_bridge.launch.py` 一次起桥 + monitor
- [x] `parse_line()` 在独立模块里，脱离 ROS 能跑测试并通过
- [x] `colcon test --packages-select sensor_bridge` **全绿**（19 tests, 0 failures）
- [x] `ros2 bag record` 录了 30 秒，`ros2 bag play` 后 monitor 仍能收到 —— 有数据资产
- [x] 协议 v2 有文档，字段含义、行尾、波特率都写明了

> #### 关于「独立模块」这条怎么算过（2026-09-22 实测）
>
> 判据是三条 `grep` + 一条命令：
>
> ```bash
> grep -nE "^(import|from) " sensor_bridge/serial_protocol.py
> # → 只有 from typing import ...（标准库）；没有 rclpy、没有 serial
>
> # 把 ROS 环境变量全剥掉，故意为难它
> env -u PYTHONPATH -u AMENT_PREFIX_PATH -u ROS_DISTRO \
>     /usr/bin/python3 test/test_serial_protocol.py      # → 16/16 passed
> ```
>
> **这一条考的不是「会不会写测试」，是「能不能把纯逻辑从框架里剥出来」。**
> 串口测试最难的地方是造异常 —— 真板子上没法按需制造半包、乱码、600 字节超长行；
> 但在纯函数里它们就是几个 `bytes` 字面量。以后写 CAN 解析、协议栈，都是同一招。
>
> #### ★ 关于 ament 的 flake8 / pep257 报错：先分类，再决定「改」还是「有意豁免」
>
> 这两项失败**与功能无关**，是风格检查。处理原则：
>
> | 类型 | 例子 | 怎么处理 |
> |---|---|---|
> | **真问题** | `W291` 行尾空格、`E302` 空行数不对、`E231` 逗号后缺空格、`E128` 续行缩进 | **直接改**（改完零逻辑变化） |
> | **规则与中文冲突** | `D400`/`D415`：要求 docstring 首行以 ASCII `.` 结尾，而中文用「。」 | **有意豁免**，写清理由 |
> | **规则与代码冲突** | `D403`：要求首词大写，但 `readline`/`parse_line` 是标识符，必须小写 | **有意豁免** |
> | **风格流派冲突** | `D406`/`D407`/`D413`：pydocstyle 按 numpy 风格解析 `Returns`，本项目用 Google 风格 `Returns:` | **有意豁免** |
>
> 豁免写在 `test/test_pep257.py` 里（不是删测试、也不是无视报错）：
>
> ```python
> _IGNORE = 'D400,D415,D403,D406,D407,D413'      # 上方注释里逐条写了理由
>
> def test_pep257():
>     rc = main(argv=['.', 'test', '--add-ignore', _IGNORE])
>     assert rc == 0
> ```
>
> **「有意豁免」和「无视报错」是两件事**：前者要留下理由和证据，后者是欠债。
> 工程里你会反复做这个判断 —— 关键是**知道自己为什么放过它**。
>
> #### ⚠️ 顺带发现的一个真坑：`install/` 里可能是**拷贝**而不是软链
>
> 2026-09-22 实测：`~/ros_study/install/sensor_bridge/.../serial_protocol.py` 是**普通文件**（不是软链），
> 说明当时的 `colcon build` **没有加 `--symlink-install`**。
>
> **后果**：改了 `src/` 里的代码，`ros2 run` 跑的还是 `install/` 里的**旧拷贝** —— 静默生效不了。
>
> | 构建方式 | 改 src 代码后 |
> |---|---|
> | `colcon build --symlink-install` | Python 是软链，**改完直接生效**，不用重编 |
> | `colcon build`（默认） | 是拷贝，**必须重新 build** 才生效 |
>
> 判据回顾（阶段一 A5 的同一条）：**改的是代码还是安装清单？**
> 现在要加一句：**还得看你的 build 是软链还是拷贝。**
> 建议统一成 `--symlink-install`（切换前必须先清 `build/` + `install/`，见阶段三教程 6.3）。

### D 组 · 理解题（口述，答不出不算过）

1. 为什么读串口必须在独立线程？如果放在 `spin` 的线程里，具体会发生哪三件坏事？
2. 为什么用「定时器 + 最新值」发布，而不是「收到就发」？说出两个理由。
3. `/dev/ttyUSB0` 为什么会变？udev 规则靠什么匹配到同一块板子？
4. `port` 和 `scale` 一个是结构参数、一个是数据参数，判断依据是什么？
5. v1 协议为什么不够？v2 的多字段解决了什么问题？（要能说到「同一时刻的同一帧」）
6. 现在 `value` 和 `temp` 还是挤在同一条 `Float32` 里只发了第一个字段——
   阶段三要做什么才能把四个字段一起发出去？

### 不能过的情况（出现任意一条 = 返工）

| 情况 | 为什么算失败 |
|---|---|
| 靠写死 `/dev/ttyUSB0` 跑通的 | 换台机器、换一天就废 |
| 拔线就崩，或必须重启节点才能恢复 | 这是「demo」不是「工程」 |
| 解析失败逐条打日志刷屏 | 一个坏帧能把终端和 CPU 一起吃掉 |
| 没有任何 bag / hz 实测记录 | 没有证据 = 没做过 |
| 板子挂了还在发旧值 | 下游会拿着鬼魂数据做决策，这在真机上是事故 |

---

## 六、关键风险点与注意事项

### 6.1 高频坑速查表（按「卡人概率」排序）

| # | 症状 | 根因 | 处置 |
|---|---|---|---|
| 1 | `ModuleNotFoundError: No module named 'serial'`，但 `pip show pyserial` 说装了 | ROS2 用 `/usr/bin/python3`（3.10），pip 装到了别的解释器 | `sudo apt install python3-serial`；用 `/usr/bin/python3 -c "import serial"` 验证 |
| 2 | `SerialException: [Errno 13] Permission denied` | 不在 `dialout` 组 | `sudo usermod -aG dialout $USER` → **注销重登**（重开终端不够） |
| 3 | 设备出现在 `lsusb` 里，但 `/dev/ttyUSB0` 读不到任何数据、`dmesg` 有 `ch341-uart ... disconnected from ttyUSB0` | Ubuntu 自带的 **brltty** 抢占 CH340 | `sudo systemctl stop brltty-udev.service brltty.service && sudo systemctl mask brltty-udev.service` |
| 4 | 昨天 `ttyUSB0` 今天变 `ttyACM0` | 内核按枚举顺序分配，非设备身份 | udev 规则固定别名（Step 5） |
| 5 | 节点像死了一样：`ros2 node info` 卡住、Ctrl+C 关不掉、`/rosout` 不更新 | 在 `spin` 线程里做了阻塞 `readline` | 独立读线程 + `timeout` |
| 6 | 数值偶尔跳变、偶尔解析失败 | 波特率不匹配 / 发送过快导致溢出 / `readline` 切成半包 | 两端波特率一致；降低发送频率；加行长度上限 |
| 7 | 前几帧一直报解析失败 | MCU 上电横幅（`STM32 Ready\r\n`）被当成数据 | 解析失败静默计数，或协议加帧头 |
| 8 | CPU 占用高、话题延迟变大 | 高频路径逐帧 `info` 日志（阶段一 A3 的原题） | `throttle_duration_sec` 或降到 `debug` |
| 9 | 全是乱码 / 全是 `0xFF` | 两端波特率不一致 | 板子 `.ioc` 与 PC 参数对齐 |
| 10 | 完全收不到数据、芯片发烫 | 直连 RS232 电平（±12V）、未共地、TX/RX 未交叉 | TTL 对 TTL；**GND 必须接**；TX→RX 交叉 |
| 11 | `pyserial` 装好了但 `ros2 run` 起来就崩 | 忘了 `package.xml` 加 `exec_depend`（换机器才暴露） | 补 `<exec_depend>python3-serial</exec_depend>` |
| 12 | 改了 `setup.py` 入口却 `No executable found` | `entry_points` 是安装元数据，软链救不了 | 重新 `colcon build`；乱了就删 `build/`+`install/` 再 build |

### 6.2 三个「看起来对但就是不对」的陷阱

**陷阱 ①：`serial.Serial(port)` 不设 `timeout`**

看起来「更简单」，实际是让 `readline()` 变成**永久阻塞**。整个节点从此像死机一样。
判据：**任何阻塞式 IO 打开时，必须显式给超时。** 没有例外的场景是极少数。

**陷阱 ②：拿「PC 收到数据的时刻」当「传感器采样的时刻」**

第一版你会自然地用 `self.get_clock().now()` 打时间戳，这没错——但它标的是**到达时刻**，
包含了串口传输 + 缓冲 + 调度延迟。等你要做多传感器融合或离线回放时，这个误差会变成系统性错位。
正规做法：**MCU 侧带时戳一起发**（v2 协议里给 `stamp` 留个字段），或者接受「到达时刻 = 采样时刻」并**明确写下这个假设**。
写下来就不算错，不写下来就是埋雷。

**陷阱 ③：把 `/dev/ttyUSB0` 写进代码（甚至写进 launch 默认值）**

它会「一直能用」，直到它不能用的那一天——而那天通常是你需要它靠谱的那天。
**所有设备名走参数，参数默认值用 udev 别名。**

### 6.3 止损与时间盒规则（针对考研优先级）

这是整篇文档里唯一「非技术」但最重要的一节。

| 规则 | 内容 |
|---|---|
| **单点 45 分钟规则** | 任何一步卡超过 45 分钟 → 记下现象和已试过的方案，**往下走**，最后统一处理。别让一个坑吃掉整个阶段二 |
| **先软后硬铁律** | Step 1–2 用虚拟串口完成，**必须**在碰板子之前做完。这样出问题时只可能是一端的问题 |
| **一次只改一个变量** | 板子不发数据时，不要同时改波特率、接线、代码。改一处，验一次 |
| **范围红线** | 见第〇节。想加 CRC / DMA / micro-ROS 时，写进「以后再说」清单，不许当场做 |
| **时间预算是硬上限** | 2 个晚上 + 1 个周末。超了就收手，把没做完的写成 TODO，转去 Linux 系统编程——**那才是你自评的最大缺口** |
| **每天留 1 小时给考研** | 阶段二不需要连续攻坚，切成小段反而更适合。数学英语不能因为「今晚在做串口」就断档 |

### 6.4 从阶段二到阶段三的接口（提前对齐，避免返工）

阶段二结束时，你手上应该有：

1. 一个能稳定运行、抗异常、参数化的串口桥
2. **一份 v2 协议文档**（字段名、类型、单位、行尾、波特率）
3. 一段可回放的 bag

阶段三要做的事因此变得很小：**把 v2 协议的字段，打包成一条自定义 `SensorData.msg`**：

```
# SensorData.msg
float32  value
float32  temp
int32    raw_adc
uint32   seq
builtin_interfaces/Time stamp
string   frame_id
```

桥的骨架（线程模型、重连、参数、发布定时器）**一行都不用改**——
只把「`self._latest` 是个 float」换成「`self._latest` 是个四元组」，发布时填进 msg。

**这就是阶段二把协议先定下来的回报：阶段三不再是「重新做一个项目」，而是「给已有的桥换一个出口」。**

---

## 七、下一步

阶段二通关后，按结业清单的安排：

1. **阶段三（约 1 个周末，可选但推荐）**：自定义 `SensorData.msg`，让桥输出结构化消息。
2. **转入主线**：鱼香 ROS → 韦东山《Linux 应用编程》。
   > 你在阶段二里踩到的每一个坑——权限、设备节点、udev、文件描述符、阻塞与非阻塞、线程——
   > **全都是 Linux 系统编程的内容**。阶段二不是绕路，它就是 Linux 系统编程的实战预演。
   > 学韦东山的时候，你会反复想起这几天在串口上撞的墙。那是最好学的时刻。

---

**关联文档**
- `ROS2 21讲结业清单与下一步路线.md`（上游规划）
- `阶段一：软件模拟仿真.md`、`阶段一测试题与答案.md`（前置）
- `07_ROS2自定义接口实战/`（阶段三要用的自定义消息，先扫一眼即可）

**本目录附件**

| 文件 | 说明 | 已验证 |
|---|---|---|
| `参考源码/serial_bridge_node.py` | **阶段二 + 阶段三合并版**：含两处钩子（`EXTRA_PARAMS` / `_create_publisher` / `_pack_msg`），行为向后兼容 | ✅ 语法 + 逻辑实跑通过（2026-09-22）⚠️ **它比纯阶段二版多出钩子**，若看着陌生，那是阶段三的内容；想要纯阶段二版，删掉 `EXTRA_PARAMS` 那段与两个钩子方法即可 |
| `参考源码/serial_protocol.py` | 协议解析纯函数，可独立运行看示例 | ✅ 实跑通过；与 `~/ros_study` 里的版本 `diff` 一致 |
| `参考源码/serial_bridge.launch.py` | launch 模板 | 语法检查通过 |
| `参考源码/99-stm32-bridge.rules` | udev 规则模板（含常见 VID:PID 对照） | ✅ 模板已落地到 `/etc/udev/rules.d/`（CH340 `1a86:7523`） |
| `参考源码/stm32_firmware_snippet.c` | MCU 侧固件片段（MDK / GCC 两种） | 片段，需合进你的 CubeMX 工程；`mv` 单位为**毫伏** |
| `无硬件自测/virtual_serial_sender.py` | 虚拟串口灌数据（含故意使坏） | ✅ 逻辑实跑通过（30 帧：好帧 30 / 坏帧 5）；中文路径下 `import serial_protocol` 实测正常 |
| `无硬件自测/test_serial_protocol.py` | 解析单测，不需要 ROS、不需要串口 | ✅ **16/16 通过**（2026-09-22 复跑） |
| `无硬件自测/test_bridge_logic.py` | 节点逻辑自测，mock 掉串口，**支持 v1 / hardened 双模式** | ✅ `--impl v1` 全绿、`--impl hardened` 5 项断言全绿（2026-09-22） |
| `无硬件自测/verify_step2_e2e.sh` | 一键端到端复查，**支持 `--mode v1\|hardened`** | ⚠️ 沙箱里 `socat` 建不了 pty（`openpty: No such file`），**需在你本机实跑**；判读标准已按两版本分别重写 |

> `virtual_serial_sender.py` 常用参数：
> `--frames 30` 发够 30 帧就退出（方便脚本化测试）、`--v1` 用单字段协议、
> `--no-banner` / `--no-garbage` 关掉故意使坏、`--rate 20` 发送频率。
>
> ⚠️ **`--v1` 只对 v1 版桥是必需的。** 加固版能吃 v2 四字段，加不加都能跑——
> 这正是 Step 7 换 `parse_line()` 的直接回报。

---

## 维护记录

### 2026-09-22 复核（本次修订）

逐项重跑了一遍本目录的源码与脚本，揪出 **3 处与现状脱节**，已修复：

| # | 问题 | 为什么错 | 怎么改的 |
|---|---|---|---|
| 1 | `test_bridge_logic.py` 实测**必失败** | 脚本从 `~/ros_study` 加载 `serial_bridge_node`，而那份代码早已从最小版升级成加固版（定时器发布 / 能吃 v2 / 拦得住超长行），CASES 的期望值还停在 v1 时代 → 发布序列永远为空 | 加 `--impl v1\|hardened\|both`，两套 CASES 两套断言；默认 `hardened` |
| 2 | `verify_step2_e2e.sh` 的判读标准不成立 | 判据照 v1 写（「≈20 Hz」「偶尔一个 inf」「忘加 --v1 → 满屏 inf」）；加固版下频率是自身 `publish_rate`（默认 10 Hz）、压根不会有 inf、也不吃 `--v1` 这一套 | 加 `--mode v1\|hardened`；判读表拆成两套，hardened 那份还顺带解释「为什么不是 20 Hz」 |
| 3 | `参考源码/serial_bridge_node.py` 版本号对不上 Step 2/6 | 它是**阶段二+三合并版**，带 `EXTRA_PARAMS` 与两个钩子，照着 Step 2 学的人会看到没讲过的代码 | 在文档开头加「两个实现版本」对照表，附件表里标注 |

**顺带实测到的三条事实**（写进定稿的依据）：

- 加固版 2 秒 window 内：喂 12 行样本 → **收帧 7 / 坏帧 5**，发布的全是最后一个值（10 帧/2s），超长行**不再产生 inf** ✅
- v1 版同样的输入序列 → 发布 `[1234.0, 12.5, -0.5, 4095.0, 0.0, inf, 2560.0]`，逐项吻合，含那条已知局限 ✅
- `source /opt/ros/humble/setup.bash` 在干净环境（`env -i` + 系统 PATH）下**是生效的**；
  真正的坑是写成 `source xxx \| head`（管道开子 shell，等于白 source），以及 PATH 里若有别的 python3 抢在前面 → 统一写 `/usr/bin/python3` 最稳。

