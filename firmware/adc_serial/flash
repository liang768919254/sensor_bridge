#!/usr/bin/env bash
# ------------------------------------------------------------------
# flash.sh —— 一键「编译 + 烧录」
#
#   ./flash.sh            编译；仅当机器码与上次烧录的不同才烧
#   ./flash.sh --force    无条件重烧（比如想复位板子）
#   ./flash.sh --build    只编译，不烧录
#
# 为什么不写进 Makefile 的 target：
#   Makefile 是 CubeMX 生成的，每次点「Generate Code」都会被重写，
#   自己加的 target 下次就没了。独立脚本不受影响。
# ------------------------------------------------------------------
set -euo pipefail
cd "$(dirname "$0")"

TARGET=adc_serial
ELF="build/$TARGET.elf"
BIN="build/$TARGET.bin"
STAMP="build/.flashed.md5"

FORCE=0
BUILD_ONLY=0
for arg in "$@"; do
  case "$arg" in
    -f|--force) FORCE=1 ;;
    -b|--build) BUILD_ONLY=1 ;;
    *) echo "未知参数: $arg（用 -f 强制重烧 / -b 只编译）"; exit 1 ;;
  esac
done

echo "▶ 1/3 编译"
make -j"$(nproc)"
echo

echo "▶ 2/3 固件信息"
arm-none-eabi-size "$ELF" | tail -1
NEW_MD5=$(md5sum "$BIN" | awk '{print $1}')
echo "  bin: $(stat -c%s "$BIN") 字节   md5: $NEW_MD5"

if [ "$BUILD_ONLY" -eq 1 ]; then
  echo
  echo "（--build）已跳过烧录。"
  exit 0
fi

# 只改注释/空行时编译器产出完全相同的 bin → 没必要烧
OLD_MD5=$(cat "$STAMP" 2>/dev/null || true)
if [ "$NEW_MD5" = "$OLD_MD5" ] && [ "$FORCE" -eq 0 ]; then
  echo
  echo "⏭  机器码与上次烧录的完全一致，芯片里已经是这份 → 跳过烧录"
  echo "   确实要重烧: ./flash.sh --force"
  exit 0
fi

echo
echo "▶ 3/3 烧录"
st-flash --reset write "$BIN" 0x8000000

echo "$NEW_MD5" > "$STAMP"
echo
echo "✅ 完成 —— 板子应在跑新固件（PC13 每 0.5s 闪 = 程序活着）"
