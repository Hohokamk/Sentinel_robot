#!/usr/bin/env bash
# ============================================================================
#  一键烧录到 STM32F405 并复位运行
#  默认使用 CMSIS-DAP 调试器（与 VisualGDB Debug 配置一致，adapter speed 3000）
#  若用 ST-Link:  OPENOCD_INTERFACE=interface/stlink-v2.cfg bash scripts/flash.sh
# ============================================================================
set -euo pipefail
cd "$(dirname "$0")/.."

ELF="${1:-build/STM32F405.elf}"
: "${OPENOCD_INTERFACE:=interface/cmsis-dap.cfg}"
: "${OPENOCD_SPEED:=3000}"

if ! command -v openocd >/dev/null 2>&1; then
  echo "错误: 找不到 openocd，请先运行  bash scripts/setup_toolchain.sh" >&2
  exit 1
fi
if [ ! -f "$ELF" ]; then
  echo "错误: 找不到 $ELF ，请先编译（Ctrl+Shift+B 或 cmake --build build）" >&2
  exit 1
fi

echo ">> 烧录: $ELF"
echo ">> OpenOCD: $OPENOCD_INTERFACE  @ ${OPENOCD_SPEED}kHz"
exec openocd \
  -f "$OPENOCD_INTERFACE" \
  -c "adapter speed $OPENOCD_SPEED" \
  -f target/stm32f4x.cfg \
  -c "program $ELF verify reset exit"
