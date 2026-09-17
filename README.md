# RCS 哨兵电控固件 — 协作说明

> **STM32F405 + FreeRTOS + STM32 HAL** 的电控固件仓库（RoboMaster 哨兵）。
>
> 本工程**一套源码，双平台开发**：Windows 队友用 **Visual Studio + VisualGDB**（保持原有习惯），
> Linux 队友用 **VS Code + Cortex-Debug**。编译 / 一键烧录 / 单步调试体验等价，全部使用免费开源工具链（零成本）。

---

## 1. 团队共识（务必先读）

1. **源码统一 UTF-8、行尾统一**。仓库已做过一次全局 UTF-8 转换，请**不要再把文件另存为 GBK**，
   否则会出现乱码、git 假 diff，污染别人的代码审查。
2. **不提交任何构建产物**：`.o / .dep / .rsp / .elf / .bin / .map / .user / .suo` 等已在
   `.gitignore` 里忽略。看到这些文件没进版本库是**正常**的，别手动 `git add -f` 强推。
3. **每次改动尽量一次提交一件事**，写清提交信息（见 §4 提交规范），方便回溯和 review。
4. 本仓库之外另有 **`RCS_code空框架`** 模板目录（`原版` / `升级版（统一编码和双平台支持）`，位于
   `~/Projects/Robomaster/` 下），是**其他兵种**新建工程用的独立模板，与本仓库无关，
   **不要删除、不要混入本仓库**。

---

## 2. 目录结构

```
Sentinel_robot/                  # ★ 本仓库（git 根目录）
├── CMakeLists.txt               # Linux 构建脚本（Windows 的 VisualGDB 构建不受影响）
├── FreeRTOS.sln                 # Windows: VS + VisualGDB 打开这个文件
├── cmake/
│   └── arm-none-eabi-toolchain.cmake
├── linker/
│   └── STM32F405RGTx_FLASH.ld   # 链接脚本（标准 ST/CubeMX 布局）
├── STM32F405/                   # ★ 你自己的源码（.cpp/.h）——唯一经常改的地方
├── ThirdParty/                  # 第三方库（HAL/CMSIS/FreeRTOS，已内置，一般不用动）
│   └── SVD/                     # （可选）STM32F405.svd 外设寄存器描述
├── scripts/
│   ├── setup_toolchain.sh       # Linux 一键装工具链（需 sudo，只跑一次）
│   └── flash.sh                 # Linux 一键烧录（OpenOCD）
├── .vscode/                     # Linux VS Code 配置（Windows 队友可无视）
├── .gitignore / .gitattributes  # 行尾与忽略策略（两边都生效，别删）
└── README.md                    # 本文件
```

> 注意：`VisualGDB/`、`.visualgdb/`、`.vs/` 是 Windows 时代的历史遗留/缓存目录，
> 已被 git 忽略，**只在你本地存在**，别人 clone 下来不会、也不需要看到。

---

## 3. 环境准备

### 3.1 Windows 队友（VS + VisualGDB）——什么也不用装

- 保持你现有的 Visual Studio + VisualGDB 环境。
- clone 之后直接双击 **`FreeRTOS.sln`** 打开，按原来的方式编译、下载、调试即可。
- 第一次 clone 后如果 VisualGDB 报找不到工程设置，确认 `.vgdbsettings` 是否随 clone 带下来了
  （它在仓库里，正常 clone 会自动带）。

### 3.2 Linux 队友（Ubuntu + VS Code）

**① 一键装工具链**（只需一次，需 sudo）：

```bash
bash scripts/setup_toolchain.sh     # 自动装 gcc-arm-none-eabi / openocd / cmake / ninja / gdb-multiarch
                                    # 并配置 ST-Link / CMSIS-DAP 的 USB 权限
```

装完**注销重新登录**一次，让 `dialout / plugdev` 用户组生效。

> 如果终端打印“Microsoft Edge 源缺公钥”之类的警告，是系统里第三方 apt 源的问题，
> 不影响本工程工具链，可忽略；想顺手修就按脚本提示执行那条 gpg 命令。

**② 打开 VS Code，安装推荐扩展**（`.vscode/extensions.json` 里已声明，可直接在
`Extensions: Show Recommended Extensions` 一键装）：

| 扩展 | 作用 |
|---|---|
| **Cortex-Debug** | 单步调试、外设/寄存器视图、FreeRTOS 任务列表（核心） |
| **C/C++** (cpptools) | 语法高亮、IntelliSense、跳转定义 |
| **CMake Tools** | 配置/构建 CMake 工程、选择工具链 |
| CMake / Arm Assembly | 语法着色辅助 |

---

## 4. 协作流程

### 4.1 首次加入

```bash
git clone git@github.com:Hohokamk/Sentinel_robot.git
cd Sentinel_robot
```

### 4.2 日常开发（推荐）

```bash
git checkout main
git pull                                   # 先同步最新代码
git checkout -b feat/xxx                   # 每个功能/任务开一个分支
# ... 改代码 ...
git add <你改的文件>                        # 只 add 源码，别 add build/ 之类
git commit -m "feat: 新增 xxx 功能"         # 见下面的提交规范
git push -u origin feat/xxx                # 推到远端
# 在 GitHub 上发起 Pull Request → 合并到 main
```

### 4.3 提交信息规范

用**前缀 + 一句话**描述，中文或英文都行，团队保持一致：

- `feat:` 新功能　`fix:` 修 bug　`refactor:` 重构　`docs:` 文档
- `chore:` 杂项（编码转换、清理、CI 等）　`style:` 格式

示例：

```
feat: 云台 PID 参数下发到上位机
fix: taskslist.cpp 中 DMmotor 数组越界
docs: 更新协作说明
```

### 4.4 提交纪律

- **改一行就是一个提交**里只放相关内容，不要一个提交里混着"加功能 + 改格式 + 删文件"。
- 提交前跑一遍编译（见 §5），保证**能编译再提交**。
- 别把别人的改动或无关文件带进自己的提交里。

---

## 5. 构建 / 烧录 / 调试

### 5.1 Windows（VS + VisualGDB）

- **编译 / 下载 / 调试**：全部照旧，和迁移前完全一样，`FreeRTOS.sln` 里选对应配置即可。

### 5.2 Linux（命令行）

```bash
# ① 编译（Debug）
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug \
      -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-toolchain.cmake
cmake --build build
# 产物: build/STM32F405.elf / .bin / .map / compile_commands.json

# ② 一键烧录（不调试）
bash scripts/flash.sh                          # 默认 CMSIS-DAP
OPENOCD_INTERFACE=interface/stlink-v2.cfg bash scripts/flash.sh   # ST-Link 用户
```

### 5.3 Linux（VS Code）

| 操作 | 快捷键 / 命令 |
|---|---|
| 编译 | `Ctrl+Shift+B`（Build 任务，自动先 configure 再 build） |
| 全量重编译 | `Ctrl+Shift+P` → `Run Task` → `Clean + Rebuild` |
| 一键烧录 | `Ctrl+Shift+P` → `Run Task` → `Flash (OpenOCD)` |
| 编译+烧录+单步调试 | 接好 ST-Link / CMSIS-DAP 后直接 **F5** |

**F5 调试说明**：`launch.json` 里有两个配置，都支持断点、单步、变量/寄存器监视、**FreeRTOS 任务列表**：

- `STM32F405 Debug (OpenOCD + ST-Link)` —— **默认**，用 ST-Link V2
- `STM32F405 Debug (OpenOCD + CMSIS-DAP)` —— 用 CMSIS-DAP / DAPLink

VS Code 右下角切换配置，或按 F5 后在下拉框选。调试会先自动编译（`preLaunchTask`），
再烧录并停在 `main`。

---

## 6. 编码 / 行尾约定（跨平台协同的关键）

- **仓库内统一存 LF**，Windows 检出自动转 CRLF、Linux 保持 LF（由 `.gitattributes` 控制）。
- **源文件统一 UTF-8**。Windows 队友请确认 VS 里“高级保存选项”编码为 **UTF-8（带 BOM 或不带都行，
  建议不带）**，别存成 GBK/ANSI。
- 改完代码如果发现 git diff 里出现**大量"整行都被改了"**的情况，多半是行尾/编码被改动了，
  先 `git checkout -- <文件>` 恢复再重来，不要硬提交。

---

## 7. 与迁移前源码的差异（评审时留意）

以下是对原工程源码做过的**最小修改**（记录在此，方便 Windows 队友 review 和 Linux 队友理解）：

| 文件 | 修改 | 原因 |
|---|---|---|
| `RC.h` | `#include "usart.h."` → `"usart.h"` | Windows 会忽略文件名末尾的 `.`，Linux 不会 |
| `motor.h` | `#include "PID.h"` → `"pid.h"` | 大小写问题（Linux 文件名区分大小写） |
| `STM32F405.cpp` | 补 `#include "xuc.h"`；`xuc, Init(...)` → `xuc.Init(...)` | 原代码笔误 |
| `xuc.cpp` | 补全局定义 `XUC xuc;` | 只有 `extern` 声明缺定义，链接失败 |
| `RC.cpp` | 补 `#include "HTmotor.h"` | 用到 `extern DMMOTOR DMmotor[1]` 却没包含头文件 |
| `control.h` | `enum MODE` 末尾补 `LOCK` | `judgement.cpp` 用到该枚举值 |

> 以上在 Windows 上其实也编译不过，属于被 Windows 文件系统"宽容"掩盖的旧问题，不是迁移引入的。

---

## 8. 已知注意事项（建议尽快处理）

- **`taskslist.cpp` 数组越界**：访问 `DMmotor[1].Kp/.Kd`，但 `DMmotor` 数组长度是 `[1]`（只到 `[0]`）。
  属原代码 bug，越界行为未定义，请按实际电机数量修。
- 多处 `xQueueReceive(..., NULL)` 把 `NULL` 当超时参数（应为 `0` 或 `portMAX_DELAY`），是代码质量问题，不影响编译。
- 编译存在若干 warning（未初始化返回、`packed` 等），与 Windows 版一致，不阻塞，但有空建议清一下。

---

## 9. 常见问题（FAQ）

**Q：Linux 上 F5 报"找不到 openocd / arm-none-eabi-gdb"？**
A：工具链没装或没重新登录。运行 `bash scripts/setup_toolchain.sh`，注销重登。
（Ubuntu 24.04 没有 `arm-none-eabi-gdb`，工程已配置用 `gdb-multiarch`，无需额外处理。）

**Q：烧录时报 `Error: libusb` / 权限不足？**
A：udev 规则没生效。重插调试器，确认自己在 `plugdev/dialout` 组，然后注销重登。

**Q：我用 ST-Link 但 F5 烧不进/找不到设备？**
A：确认 `launch.json` 选的是 **ST-Link** 配置（默认即是），且 OpenOCD 用的是
`interface/stlink-v2.cfg`。命令行烧录用 `OPENOCD_INTERFACE=interface/stlink-v2.cfg bash scripts/flash.sh`。

**Q：Windows 队友提交的文件在 Linux 上打开乱码？**
A：多半存成了 GBK。在 VS 里用“文件 → 高级保存选项 → UTF-8”重新保存后再提交。

**Q：改了代码但 git 显示一堆"整文件被改"？**
A：行尾被改了。恢复文件（`git checkout -- <文件>`），确认编辑器设置为不自动改行尾/编码再改。

**Q：想用旧版 VisualGDB 的精确 BSP（逐字节一致）？**
A：把 Windows 上 `...\AppData\Local\VisualGDB\EmbeddedBSPs\arm-eabi\com.sysprogs.arm.stm32\`
整个拷到 `ThirdParty/VisualGDB_BSP/` 并在 `CMakeLists.txt` 改路径即可；
当前 `ThirdParty/` 是等价可编译的免费替代。

---

## 10. 相关工具链版本（Linux）

- 编译器：`gcc-arm-none-eabi`（GCC 13.2.1，`-mcpu=cortex-m4 -mthumb -mfloat-abi=soft`）
- 调试器：`gdb-multiarch`（Ubuntu 24.04 无独立 arm-none-eabi-gdb）
- 烧录/调试服务器：`openocd` 0.12.0
- 第三方库：STM32F4 HAL（旧版，含 legacy bxCAN API）+ CMSIS + FreeRTOS V10.3.1
