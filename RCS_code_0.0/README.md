# RCS_code_0.0 — STM32F405 电控固件（RoboMaster 哨兵）

基于 **FreeRTOS + STM32 HAL** 的哨兵机器人电控工程。

> 本仓库由 **Windows + Visual Studio + VisualGDB** 工作流迁移而来，现支持
> **Ubuntu Linux + VS Code + Cortex-Debug**：同一套源码，编译 / 一键烧录 / 单步调试体验等价，
> 全部使用免费开源工具链（零成本）。

---

## 1. 目录结构

```
RCS_code_0.0/
├── CMakeLists.txt                 # 构建脚本（取代 VisualGDB 的 .vcxproj）
├── cmake/
│   └── arm-none-eabi-toolchain.cmake   # ARM 交叉编译工具链定义
├── linker/
│   └── STM32F405RGTx_FLASH.ld    # 链接脚本（标准 ST/CubeMX 布局）
├── STM32F405/                     # ★ 你自己的源码（.cpp/.h，与 Windows 版一致）
├── ThirdParty/                    # 第三方库（HAL/CMSIS/FreeRTOS，已内置）
│   ├── STM32F4xx_HAL/             #    STM32F4 HAL 驱动（旧版，含 legacy CAN API）
│   ├── CMSIS/                     #    CMSIS Core + Device（经典布局）
│   ├── FreeRTOS/                  #    FreeRTOS V10.3.1 + CMSIS-RTOS 包装
│   └── SVD/                       #    （可选）STM32F405.svd 外设寄存器描述
├── scripts/
│   ├── setup_toolchain.sh         # 一键安装工具链 + udev 规则（需 sudo 一次）
│   └── flash.sh                   # 一键烧录（OpenOCD）
└── .vscode/                       # VS Code 调试/构建/IntelliSense 配置
    ├── launch.json
    ├── tasks.json
    ├── c_cpp_properties.json
    └── extensions.json
```

## 2. 一次安装工具链（只需一次）

```bash
cd RCS_code_0.0
bash scripts/setup_toolchain.sh     # 输入 sudo 密码，自动装好一切
```

安装内容：
- `gcc-arm-none-eabi`（含 arm-none-eabi-gdb）、`openocd`、`cmake`、`ninja-build`、`gdb-multiarch`
- ST-Link / CMSIS-DAP 的 udev 规则
- 把当前用户加入 `dialout` / `plugdev` 组

> 装完**注销重新登录**一次让用户组生效。然后打开 VS Code 安装推荐扩展
> （右下角弹窗点"Install"或运行 `Extensions: Show Recommended Extensions`）：
> **Cortex-Debug**、**C/C++**、**CMake Tools**。

## 3. 编译

```bash
# 命令行
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug \
      -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-toolchain.cmake
cmake --build build
```
产物：`build/STM32F405.elf`、`build/STM32F405.bin`、`build/compile_commands.json`。

在 VS Code 里直接 **Ctrl+Shift+B** 即可（已配置 `Build (Debug)` 任务）。

## 4. 一键烧录（不调试）

```bash
bash scripts/flash.sh                          # 默认 CMSIS-DAP 调试器
# 用 ST-Link 时:
OPENOCD_INTERFACE=interface/stlink-v2.cfg bash scripts/flash.sh
```
VS Code 里：`Ctrl+Shift+P` → `Run Task` → `Flash (OpenOCD)`。

## 5. 单步调试（F5）

VS Code 按 **F5**：
1. 自动编译（`preLaunchTask`）
2. 启动 OpenOCD（CMSIS-DAP / ST-Link 均可，`launch.json` 里两个配置可选）
3. 烧录固件并复位到 `main` 停止
4. 支持断点、单步、变量监视、寄存器、外设（若有 SVD）、**FreeRTOS 任务列表**（`"rtos": "FreeRTOS"`）

> 与 VisualGDB 的对应关系：
> | VisualGDB | 这里 |
> |---|---|
> | F5 下载并调试 | F5（Cortex-Debug）|
> | Debug 配置 `cmsis-dap.cfg + adapter speed 3000` | `launch.json` 第一个配置 |
> | Release 配置 `stlink-v2.cfg` | `launch.json` 第二个配置 |
> | `AutoDetectRTOS` | `"rtos": "FreeRTOS"` |
> | 一键烧录 | `scripts/flash.sh` |

## 6. 外设寄存器视图（可选）

把 `STM32F405.svd` 放到 `ThirdParty/SVD/STM32F405.svd`（`launch.json` 已引用该路径），
调试时就能看到寄存器名而不是裸地址。SVD 可从 STM32CubeF4 包 / CMSIS 器件包获取。

## 7. 本次迁移做了什么

### 7.1 工具链
- 原 VisualGDB 内置 **GCC 10.3.1 / GDB 10.2.90** → 现用 Ubuntu 的 `gcc-arm-none-eabi`（同代或更新，行为一致）
- 编译/链接参数**完全照搬**原 `.gcc.rsp` / `.link.rsp`：
  `-mcpu=cortex-m4 -mthumb -mfloat-abi=soft -mfpu=fpv4-sp-d16 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti --specs=nano.specs --specs=nosys.specs`

### 7.2 第三方库（ThirdParty）
原工程依赖 VisualGDB 的 STM32 BSP（在 Windows 的 `...\VisualGDB\EmbeddedBSPs\` 目录，不在仓库里）。
迁移时用等价、免费、可获取的官方源替代并内置到 `ThirdParty/`：
- **STM32 HAL**：STM32CubeF4 旧版（含你代码用到的 **legacy bxCAN API**：`pTxMsg/pRxMsg/HAL_CAN_Transmit`）
- **CMSIS**：CMSIS 经典布局（`core_cm4.h`、`stm32f405xx.h`），与你 `stm32f4xx_hal_conf.h`（V1.4.2）配套
- **FreeRTOS**：**V10.3.1**（与你原版相同版本）+ CMSIS-RTOS v1 包装（`cmsis_os.c`）+ `heap_4.c`
- 链接脚本用标准 STM32F405RG（1MB Flash / 128KB RAM / 64KB CCM）

### 7.3 为在 Linux 上编译，对源码做的**最小**修改（都在 `STM32F405/`）
> 这些是 Windows 文件系统"宽容"掩盖的问题，或仓库里本就存在的编译错误（在 Windows 上也编不过）。
> 请逐条核对，若与你的实际意图不符可回退。

| 文件 | 修改 | 原因 |
|---|---|---|
| `RC.h` | `#include "usart.h."` → `"usart.h"` | Windows 会忽略文件名末尾的 `.`，Linux 不会 |
| `motor.h` | `#include "PID.h"` → `"pid.h"` | Windows 文件名大小写不敏感，Linux 敏感 |
| `STM32F405.cpp` | 补 `#include "xuc.h"`；`xuc, Init(...)` → `xuc.Init(...)` | 原代码笔误，`xuc`/`Init` 未声明 |
| `xuc.cpp` | 补全局定义 `XUC xuc;` | `xuc.h` 只有 `extern`，缺定义导致链接失败 |
| `RC.cpp` | 补 `#include "HTmotor.h"` | 用到 `extern DMMOTOR DMmotor[1]` 但没包含该头文件 |
| `control.h` | `enum MODE` 末尾加 `LOCK` | `judgement.cpp` 用到 `CONTROL::MODE::LOCK`，原枚举缺失 |

### 7.4 已知注意事项（非迁移引入，建议后续处理）
- `taskslist.cpp` 访问 `DMmotor[1].Kp`/`.Kd`，但 `DMmotor` 数组长度是 `[1]` —— 越界访问，需按实际电机数量调整数组长度。
- 多处 `xQueueReceive(..., NULL)` 把 `NULL` 当超时时间（应传 `0`/`portMAX_DELAY`），属代码质量问题，不影响编译。
- 编译会有若干 `warning`（未初始化返回、`packed` 属性等），与 Windows 版一致，不阻塞。

## 8. 常见问题

**F5 报找不到 openocd / arm-none-eabi-gdb**：没装工具链或没重登。运行 `bash scripts/setup_toolchain.sh`。

**烧录时 OpenOCD 报 `Error: libusb` / 权限不足**：udev 规则未生效，重插调试器，确认在 `plugdev` 组并重登。

**想用旧版 VisualGDB 的精确 BSP（逐字节一致）**：把 Windows 上
`...\AppData\Local\VisualGDB\EmbeddedBSPs\arm-eabi\com.sysprogs.arm.stm32\`
整个拷到 `ThirdParty/VisualGDB_BSP/`，改 `CMakeLists.txt` 里的路径即可；本工程的
`ThirdParty/` 是等价可编译的替代。
