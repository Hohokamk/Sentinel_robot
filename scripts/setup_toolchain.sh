#!/usr/bin/env bash
# ============================================================================
#  一键安装 Linux 下的嵌入式开发工具链（Ubuntu）
#  - arm-none-eabi-gcc/gdb    (GNU Arm 工具链)
#  - openocd                  (烧录/调试服务器)
#  - cmake + ninja            (构建系统)
#  - 调试器 udev 规则 + 用户组权限
#
#  用法:  bash scripts/setup_toolchain.sh      (会提示输入 sudo 密码)
# ============================================================================
set -euo pipefail

echo "==> [1/3] apt 安装工具链（gcc-arm-none-eabi / openocd / cmake / ninja）"
# 第三方源（如 Microsoft Edge）缺公钥会导致 apt update 返回错误，
# 但不影响 ubuntu 官方源；这里不中断，仅警告后继续。
if sudo apt update; then
  :
else
  echo "!! 警告: apt update 有源更新失败（通常是有第三方源缺公钥）。"
  echo "   可运行下面的命令修复 Microsoft Edge 源的公钥："
  echo "   curl -fsSL https://packages.microsoft.com/keys/microsoft.asc | sudo gpg --dearmor -o /usr/share/keyrings/microsoft-edge-keyring.gpg"
  echo "   继续安装核心工具链..."
fi
sudo apt install -y gcc-arm-none-eabi openocd cmake ninja-build gdb-multiarch

echo "==> [2/3] 安装调试器 udev 规则（ST-Link / CMSIS-DAP / STM32 串口）"
RULES=/tmp/49-robomaster-probes.rules
cat > "$RULES" << 'RULE'
# --- ST-Link V2 / V2-1 / V3 (STM32 官方调试器) ---
SUBSYSTEM=="usb", ATTR{idVendor}=="0483", ATTR{idProduct}=="3748", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTR{idVendor}=="0483", ATTR{idProduct}=="374b", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTR{idVendor}=="0483", ATTR{idProduct}=="374d", MODE="0666", GROUP="plugdev"
# --- DAPLink / CMSIS-DAP (免驱动，HID；加上更稳) ---
SUBSYSTEM=="usb", ATTR{idVendor}=="0d28", MODE="0666", GROUP="plugdev"
# --- STM32 Virtual COM Port（板载串口，日志/上位机）---
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0666", GROUP="dialout"
RULE
sudo cp "$RULES" /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "==> [3/3] 把当前用户加入 dialout / plugdev 组"
sudo usermod -aG dialout "$USER"
sudo usermod -aG plugdev "$USER"

echo
echo "================ 完成 ================"
echo "请注销并重新登录（让 dialout/plugdev 组生效），然后："
echo "  # 1) 配置并编译"
echo "  cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-toolchain.cmake"
echo "  cmake --build build"
echo "  # 2) 一键烧录"
echo "  bash scripts/flash.sh"
echo "  # 3) 打开 VS Code，按 F5 即可 编译+烧录+单步调试"
