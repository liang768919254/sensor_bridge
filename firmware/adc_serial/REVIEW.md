# adc_serial 代码审查记录

> 审查日期：2026-09-21 ｜ 工程：`~/stm32_ws/adc_serial`（**唯一权威源**）
> 初版为静态审查（Windows 侧 `E:\stm32_ws\adc_serial`、CubeMX 6.14.1）；
> 2026-09-21 20:59 起在 Linux 侧完成**实机编译 + 烧录验证**，详见第六节。

---

## 总评：工程健康，可以编译

18 个 `.c` + 1 个 `.s` 源文件全部就位，头文件齐全，`Drivers/` 完整（899 个文件）。
**以下三处已由我直接修改**，另外有一处需要你回 CubeMX 改。

---

## 一、我已修正的 3 处

### ① 变量重复定义（`seq` 被定义了两次）

| 位置 | 内容 |
|---|---|
| `USER CODE BEGIN PV`（全局） | `static uint16_t seq = 0;` + `static uint32_t last_led_tick = 0;` |
| `USER CODE BEGIN 2`（局部） | ~~`uint16_t seq = 0;`~~ + ~~`uint32_t last_led = 0;`~~ |

局部变量**遮蔽（shadow）**了全局的，全局那份变成死代码，`last_led` 还完全没被使用
→ 编译会出 `-Wunused-variable` 警告，开了 `-Werror` 直接失败。

**根因**：《Step3 操作手册》§5.2 把变量写在 `BEGIN 2`，而《用户代码填充清单》写在 `BEGIN PV`，
两份都粘了就重复了。**已删掉 `BEGIN 2` 里那份，统一用全局定义。**

### ② 补上 ADC 校准

增加了：

```c
if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
{
  Error_Handler();
}
```

**为什么必须加**：STM32F1 的 HAL **不会自动校准 ADC**。不校准的话读数会有几十 LSB 的固定偏移
——你 `rqt_plot` 出来的曲线会整体抬高或压低，而且**看起来一切正常**，很难发现是校准问题。
必须放在 ADC 初始化之后、第一次 `HAL_ADC_Start` 之前。

### ③ 行尾 `\n` → `\r\n`

```c
printf("%ld\r\n", (long)mv);
```

PC 侧 `readline()` 靠 `\n` 切帧，`strip()` 会吃掉 `\r`，**协议上没区别**。
改成 `\r\n` 是为了让 Windows 串口助手这类工具也能正常换行显示
（miniterm 会自动处理 `\n`，但不是所有工具都这么聪明）。

---

## 二、需要你回 CubeMX 改的 1 处

### ⚠️ ADC 采样时间 `1.5 Cycles` 太短

当前：

```c
sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
```

**问题**：1.5 个 ADC 周期是最短采样时间，只适合**低阻抗信号源**。电位器的分压输出阻抗
通常在几 kΩ 量级，采样电容来不及充饱 → **读数偏低、随转动跳变**。

**改成 `ADC_SAMPLETIME_55CYCLES_5` 或 `ADC_SAMPLETIME_71CYCLES_5`**：

| 采样时间 | 采样耗时 @12MHz | 总转换时间 | 10Hz 够用？ |
|---|---|---|---|
| 1.5 cycles | 0.125 μs | 1.17 μs | 够，但**不准** |
| **55.5 cycles** | **4.6 μs** | **5.7 μs** | ✅ 推荐 |
| 71.5 cycles | 5.96 μs | 7.0 μs | ✅ 更稳 |

> ⚠️ **必须在 CubeMX 里改**：`sConfig.SamplingTime` 位于 `MX_ADC1_Init()` 内部，
> **不在 `USER CODE` 区**，直接改代码会被下次重新生成覆盖。
> 路径：`Analog → ADC1 → Parameter Settings → Rank 1 → Sampling Time`

---

## 三、配得对的地方（重点点名）

| 项 | 检查结果 | 意义 |
|---|---|---|
| **`__HAL_AFIO_REMAP_SWJ_NOJTAG()`**（msp.c 第 77 行） | ✅ 在 | **`Serial Wire` 生效的铁证** —— ST-Link 不会失联 |
| `HSEPredivValue = RCC_HSE_PREDIV_DIV1` | ✅ | 8MHz 直接进 PLL，不是 ÷2 |
| `PLLMUL = RCC_PLL_MUL9` | ✅ | |
| `SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK` | ✅ | 没被切成 HSE |
| `APB1CLKDivider = RCC_HCLK_DIV2` | ✅ | APB1 = 36MHz，正好卡在上限内 |
| `FLASH_LATENCY_2` | ✅ | 72MHz 必须 2 个等待周期 |
| `AdcClockSelection = RCC_ADCPCLK2_DIV6` | ✅ | 72/6 = 12MHz（上限 14MHz） |
| `HSE_VALUE = 8000000U` | ✅ | 和时钟树一致，否则波特率会整体偏 |
| **`syscalls.c` 里的 `_write` 带 `__attribute__((weak))`** | ✅ | 你的强符号能正确覆盖，**链接不会冲突** |
| **`_write` 在 `USER CODE BEGIN 0` 之内** | ✅ | 重新生成代码时不会被冲掉 |
| Makefile `FPU` 变量 | ✅ 留空 | Cortex-M3 无 FPU，正确 |
| `LIBS = -lc -lm -lnosys` | ✅ | `-lnosys` 在 |
| 链接脚本 `64K FLASH / 20K RAM` | ✅ | 和 C8Tx 匹配 |
| ADC 单次转换 / 扫描关闭 / 软件触发 / CH0(PA0) | ✅ | 轮询单次转换配置正确 |
| USART1 `115200 8N1`、PA9 `AF_PP` / PA10 `INPUT` | ✅ | |

---

## 四、两个可选优化（不改也能跑）

1. **CSS（时钟安全系统）还开着** —— `SystemClock_Config()` 末尾有 `HAL_RCC_EnableCSS()`。
   学习阶段建议关掉：HSE 万一不起振，CSS 会自动切到 HSI 继续跑，
   把「晶振问题」掩盖成「程序在跑但时钟不对」，很难查。跑通后再开。
   关闭路径：`System Core → RCC → Clock Configuration 页的 Enable CSS 取消勾选`

2. **ADC 转换超时没检查** ——

```c
HAL_ADC_Start(&hadc1);
if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)   /* ← 判断返回值 */
{
    uint32_t raw = HAL_ADC_GetValue(&hadc1);
    ...
}
```

   现在是直接 `HAL_ADC_PollForConversion` 不看返回值，超时的话会读到上一次的旧值。
   正常情况不会超时，属于 Step 6「工程化加固」再说的范畴。

---

## 五、下一步

```bash
# Ubuntu 侧（工程在 E 盘，NTFS，两系统都能访问）
cd /mnt/e/stm32_ws/adc_serial
make -j$(nproc)
arm-none-eabi-size build/adc_serial.elf     # 看 Flash/RAM 占用

st-flash --reset write build/adc_serial.bin 0x8000000
```

预期：`text` 约 10~12KB Flash、`data+bss` 约 2~3KB RAM（余量很足）。

烧录成功标志：**PC13 LED 开始 0.5 秒闪一次**。
然后别急着开 ROS2，先用 miniterm 确认板子在发数据：

```bash
/usr/bin/python3 -m serial.tools.miniterm /dev/ttyUSB0 115200
```

看到毫伏数字在滚 → 板子 PASS。

---

## 六、状态更新（2026-09-21 20:59，Linux 侧复核）

本记录前五节的待办**已全部完成**，且工程已实际编译 + 烧录验证通过。以下为最新事实：

| 第二节的待办 | 状态 |
|---|---|
| ADC 采样时间改 55.5 / 71.5 cycles | ✅ 已改，现为 `ADC_SAMPLETIME_71CYCLES_5` |
| CSS 建议关掉 | ✅ 已关，`HAL_RCC_EnableCSS()` 已从 `SystemClock_Config()` 移除 |
| `seq` 注释与实际不符 | ✅ 注释已改准：「为 Step 5 预留，v1 协议暂不发」 |

**编译与烧录实测**

| 项 | 值 |
|---|---|
| 编译 | `make -j` 一次通过，**零警告零错误** |
| text / data / bss | 15124 / 112 / 1712（Flash 23.7%、RAM 8.9%） |
| bin | 15236 字节，md5 `42ff1a8e18e3400b05d6c708266457da` |
| 烧录 | `st-flash --reset write build/adc_serial.bin 0x8000000` → verified |
| 环境 | CubeMX 6.18.1（Linux 侧 `~/STM32CubeMX`）、arm-none-eabi-gcc 10.3.1 |

**两处新增说明**

1. **Makefile 是 CubeMX 生成的，20:54 已被重写** —— 不要往里加自定义 target，会被覆盖。
   已另建 `flash.sh`（一键编译+烧录，带 md5 跳过逻辑），放工程根目录，CubeMX 不会碰它。
2. **⚠️ 源码分叉（已定案）**：Windows 侧 `E:\stm32_ws\adc_serial` 与 Linux 侧 `~/stm32_ws/adc_serial`
   两侧都装了 CubeMX、都能「Generate Code」。**2026-09-21 决定：以 Linux 侧为唯一权威源**，
   Windows 侧（停在 14:30）冻结不再编辑。详见工程根目录 `README.md`。
   两侧 `main.c` 的 md5 已确认不一致过（`6e809ff0…` vs `15b3206b…`），分叉是真实发生过的。

**USER CODE 机制已验证**：20:54 那次在 CubeMX 里重新生成代码后，
`USER CODE` 区内的 `_write`、ADC 校准、上电横幅、while 循环**一字未丢**。可放心点 Generate Code。

---

*本记录由 `cubemx-project-review` 技能产出，审查脚本：
`~/.workbuddy/skills/cubemx-project-review/scripts/review_cubemx_project.py`*
