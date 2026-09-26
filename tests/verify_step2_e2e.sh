#!/bin/bash
# verify_step2_e2e.sh —— 串口桥一键端到端复查（省得手开四个终端）
#
# 它自动完成：起 socat → 灌数据 → 起桥节点 → 看一帧 → 测频率 → 全部清理
# 只做「复查」用。第一次学的时候，请照教程老老实实手开四个终端，
# 才能真切体会「一根串口，两个终端，谁在读、谁在写」。
#
# ─────────────────────────────────────────────────────────────
# ⚠️ 两个实现版本，判据完全不同（2026-09-22 修订）
# ─────────────────────────────────────────────────────────────
#   --mode hardened（默认）   Step 6 加固版 · 即 ros_study 现在的 serial_bridge
#       发布走定时器 publish_rate（默认 10Hz）→ 频率【与 sender 速率无关】
#       能吃 v2 四字段 → 不需要 --v1
#       超长行被 max_len 拦住 → 不会再有 inf
#
#   --mode v1                 Step 2 最小闭环版 · 现存放 ~/ros_study/.../serial_bridge_node_v1.py
#       收到一帧发一帧 → 频率应当等于 sender 的 --rate（20Hz）
#       只认单值 → 必须用 --v1，否则满屏 inf + hz ≈ 0.198
#       每 5 秒会蹦出一个 inf（已知局限）
#
# 用法：
#   bash verify_step2_e2e.sh                 # 默认测加固版
#   bash verify_step2_e2e.sh --mode v1       # 测最小闭环版（复现 Step 2 现场）
#   ROS_WS=~/ros_study bash verify_step2_e2e.sh

set -u

MODE=hardened
while [[ $# -gt 0 ]]; do
    case "$1" in
        --mode) MODE="${2:-}"; shift 2 ;;
        --mode=*) MODE="${1#*=}"; shift ;;
        *) shift ;;
    esac
done
if [[ "$MODE" != hardened && "$MODE" != v1 ]]; then
    echo "[ERR] --mode 只接受 hardened 或 v1（收到：$MODE）"
    exit 1
fi

HERE="$(cd "$(dirname "$0")" && pwd)"
WS="${ROS_WS:-$HOME/ros_study}"
PKG_SRC="$WS/src/sensor_bridge/sensor_bridge"
PY=/usr/bin/python3

cleanup() {
    echo
    echo "[清理] 停止 bridge / sender / socat"
    kill "${BR_PID:-}" "${SEND_PID:-}" "${SOCAT_PID:-}" 2>/dev/null
    wait 2>/dev/null
    echo "[清理] 完成（注意 /tmp/ttyV* 软链已随 socat 一起消失）"
}
trap cleanup EXIT INT TERM

# ── 环境 ────────────────────────────────────────────────────
# 若这句之后 ros2 仍找不到命令，把 setup.bash 换成 setup.sh 再试一次
source /opt/ros/humble/setup.bash
if [ -f "$WS/install/setup.bash" ]; then
    source "$WS/install/setup.bash"
else
    echo "[ERR] 找不到 $WS/install/setup.bash，先跑："
    echo "      cd $WS && colcon build --symlink-install --packages-select sensor_bridge"
    exit 1
fi

echo "════════ mode = $MODE ════════"
if [[ "$MODE" == v1 ]]; then
    if [[ ! -f "$PKG_SRC/serial_bridge_node_v1.py" ]]; then
        echo "[ERR] 找不到最小闭环版：$PKG_SRC/serial_bridge_node_v1.py"
        exit 1
    fi
    echo "  被测实现 : serial_bridge_node_v1.py（Step 2 · 收到一帧发一帧）"
    echo "  期望频率 : ≈ sender 的 --rate"
else
    echo "  被测实现 : serial_bridge_node.py（Step 6 加固版 · 定时器发布）"
    echo "  期望频率 : ≈ publish_rate（默认 10Hz，与 sender 速率无关）"
fi
echo

# ── 1. socat：造一对虚拟串口 ────────────────────────────────
echo "[1/5] 起 socat"
socat -d -d pty,raw,echo=0,link=/tmp/ttyV0 pty,raw,echo=0,link=/tmp/ttyV1 \
    >/tmp/step2_socat.log 2>&1 &
SOCAT_PID=$!
sleep 1
if ! ls -l /tmp/ttyV0 /tmp/ttyV1 2>/dev/null; then
    echo "[ERR] 软链没建起来，socat 日志："
    cat /tmp/step2_socat.log
    exit 1
fi

# ── 2. 假 STM32：往 ttyV0 灌数据 ────────────────────────────
#   v1 模式：必须 --v1。sender 默认发 v2 四字段，最小版用 float(text)
#            只认单值 → 正常帧全被丢，只剩每 5 秒那条超长行变 inf → 满屏
#            data: .inf、hz ≈ 0.198 Hz（Step2 笔记第 6 章有完整推演）。
#   加固版  ：可以不加 --v1（它能吃四字段）；加了也没问题，只测单值路径。
echo "[2/5] 起灌数据（假 STM32，20Hz，含故意的坏数据）"
SEND_ARGS=(--port /tmp/ttyV0 --rate 20)
[[ "$MODE" == v1 ]] && SEND_ARGS+=(--v1)
"$PY" "$HERE/virtual_serial_sender.py" "${SEND_ARGS[@]}" \
    >/tmp/step2_sender.log 2>&1 &
SEND_PID=$!
sleep 0.5

# ── 3. 起桥节点 ─────────────────────────────────────────────
echo "[3/5] 起桥节点"
if [[ "$MODE" == v1 ]]; then
    # v1 版没有 ros2 run 入口（setup.py 里 serial_bridge 现在指向加固版），
    # 而且它只用绝对 import（没有 from sensor_bridge.xxx import ...），
    # 所以可以直接把 .py 当脚本跑，不需要 PYTHONPATH
    "$PY" "$PKG_SRC/serial_bridge_node_v1.py" \
        --ros-args -p port:=/tmp/ttyV1 \
        >/tmp/step2_bridge.log 2>&1 &
    BR_PID=$!
else
    ros2 run sensor_bridge serial_bridge --ros-args -p port:=/tmp/ttyV1 \
        >/tmp/step2_bridge.log 2>&1 &
    BR_PID=$!
fi
sleep 3
echo "--- 桥节点日志 ---"
cat /tmp/step2_bridge.log

# ── 4. 节点与话题 ───────────────────────────────────────────
echo "[4/5] 节点列表 / 话题列表"
timeout 10 ros2 node list 2>/dev/null
timeout 10 ros2 topic list 2>/dev/null | grep sensor

# ── 5. 数据 ─────────────────────────────────────────────────
echo "[5/5] 数据：先取一帧，再测 12 秒频率"
timeout 10 ros2 topic echo /sensor/value --once 2>/dev/null

timeout 14 ros2 topic hz /sensor/value 2>/dev/null \
    | grep -E "average rate|min:|max:|std dev" | tail -4

echo
echo "════════ 判读标准（mode=$MODE）════════"
if [[ "$MODE" == v1 ]]; then
    echo "  · average rate ≈ 20 Hz（±5%）          → 频率对，闭环成立"
    echo "  · 单帧 data 落在 0.0 ~ 3300.0 之间      → 数值合理"
    echo "  · 偶尔蹦出一个 inf                      → 已知局限：每 5 秒那条 600 字节超长行没被挡住"
    echo "  · 满屏 inf 且 rate ≈ 0.198 Hz           → 忘加 --v1，正常帧全被丢了"
    echo "  · rate 明显偏低 / std dev 很大          → 有帧被丢弃，查坏数据占比"
else
    echo "  · average rate ≈ publish_rate（默认 10 Hz，±5%）→ 定时器在工作"
    echo "    ⚠️ 别期望 20 Hz：加固版按自己的节奏发，sender 灌多快都不影响它"
    echo "      这正是「发布频率」与「到达频率」解耦的实证"
    echo "  · 单帧 data 落在 0.0 ~ 3300.0 之间      → 数值合理（v2 模式下取的是第 2 个字段 value）"
    echo "  · 一个 inf 都没有                        → 超长行被 raw_line_max=256 挡住了"
    echo "  · 桥日志里『已 Xs 没有新数据，暂停发布』 → 陈旧数据保护生效（停掉 sender 就会看到）"
    echo "  · rate 只有 0.x Hz                      → 多半是 stale_timeout 判成陈旧，查 sender 还活着没"
fi
