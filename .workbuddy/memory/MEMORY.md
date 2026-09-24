# MEMORY.md — Sentinel_robot 项目长期笔记

> RCS 哨兵电控固件。STM32F405RGT6 + FreeRTOS V10.3.1 + STM32F4 HAL（旧版 bxCAN API）。
> 一套源码双平台：Windows 用 VS+VisualGDB（`FreeRTOS.sln`），Linux 用 VS Code + CMake/Ninja + OpenOCD。
> 用户 2026-09-24 明确：**这是当前主要工作区，要作为代码指导老师逐步引导他实现，未获允许不得改动任何代码。**

## 一、四层架构
| 层 | 文件 | 状态 |
|---|---|---|
| 任务调度 | `taskslist.cpp` | 已完成（6 任务） |
| 控制 | `control.cpp` | 接口齐全，**函数体基本为空** |
| 设备模型 | `motor` `HTmotor` `imu` `RC` `judgement` `supercap` `xuc` `Power_read` `pid` `kalman` | 部分可用 |
| 驱动/系统 | `can` `usart` `tim` `gpio` `delay` `sysclk` `CRC` | 已实现，有坑 |

## 二、任务与节拍（FreeRTOS，tick=1000Hz）
| 任务 | 优先级 | 栈 | 周期 | 职责 |
|---|---|---|---|---|
| start_task | 1 | 128 | 一次 | 建任务后自删 |
| ArmTask | 1 | 128 | 100ms | `DMmotorinit()` + `power.Send()` |
| DecodeTask | 3 | 128 | 5ms | `rc.Decode()` + `imu_pantile.Decode()` |
| MotorUpdateTask | 2 | 256 | 2ms | `Motor::Ontimer()` 遍历两个 CAN 总线 + DM 电机 |
| CanTransimtTask | 2 | 256 | 1ms | 三态轮转发送（DM / 0x1ff / 0x200） |
| ControlTask | 2 | 128 | 5ms | `ctrl.chassis.Update()` + `rc.Update()` |

## 三、核心机制（理解电机链的关键）
- **`Motor::Ontimer(idata, odata)` 是双职责函数**：入参 `idata` 解析反馈、出参 `odata` 回填发送缓冲。PID→电流的计算就在它内部（`SPD`/`POS`/`ACE` 分支，目前空）。
- `can.data[12][8]` 按 `StdId - 0x201` 索引；`can.temp_data[16]`：字节 0..7 = 0x200 帧(ID1-4)，8..15 = 0x1ff 帧(ID5-8)。
- 电机装配：`can2_motor` = 底盘 M3508 ×4 (ID1-4)；`can1_motor` = 摩擦轮 M3508 ×2 (ID2,3) + 拨盘 M2006 (ID7) + 云台 M6020 (ID5)，全部标了 `function_type`。
- `ctrl.Init(vector<Motor*>)` 被 main 调两次，按 `function_type` 分派进 chassis/pantile/shooter/supply 数组。
- CAN 波特率 = 42MHz / 6 / (1+2+4) TQ = **1 Mbps**（正确，但采样点 42.9% 偏低）。
- 串口分工：USART1←IMU(CH010,115200)、USART2←遥控器(100000)、UART5←功率计(9600)、USART6←XUC 上位机(115200)、USART3/UART4 空闲。
- `UART` 类 = HAL + DMA 收发 + 空闲中断 + `xQueueOverwriteFromISR` 投递队列，是所有串口外设的地基。

## 四、待实现清单（用户接下来的活）
1. `CONTROL::CHASSIS::Update()` / `Keep_Direction()`（麦轮逆解 + 速度环 + yaw 闭环）
2. `CONTROL::PANTILE::Update()` / `Control_Pantile()` / `Keep_Pantile()`
3. `CONTROL::SHOOTER::Update()`
4. `Motor::Ontimer()` 的 `SPD` / `POS` / `ACE` 分支
5. `RC::OnPC()`、`RC::RC_Control()` 里各 mode 分支
6. 把 `judgement` / `supercap` 接进 main，调 `xuc.Encode()`

## 五、已知坑（尚未修，改动前必须先问用户）
- `CONTROL::Init()` **必崩**：`supply_motor[0]->spinning` 解引用 nullptr；末尾 `pantile_motor[1]->setangle` 同理；supply 分支缺 `break`。
- `HTmotor.h` 声明 `DMmotor[1]` ≠ `STM32F405.cpp` 定义 `DMmotor[2]`；`State_Decode` 用 `idata[ID-1]`，ID=0x09 → 索引 8 越出 `jointidata[6][8]`。
- 所有 `CAN hcan` 参数**按值传**，应改 `CAN&`。
- `motor.cpp` 的 `initial_cnt` 是文件级全局变量，被 8 个电机共享。
- `MotorUpdateTask`/`CanTransimtTask` 把 `xlastWakeTime` 写在 `while` 内 → `vTaskDelayUntil` 退化为 `vTaskDelay`。
- `RC::Decode()` 的 `sizeof(m_frame) < 18` 恒假；`control.h` 的 `mode` 未初始化。
- `usart.h` 在类内初始化 `xQueueCreate`（全局对象构造期分配 RTOS 堆）。
- `imu.cpp` `Check()` 的 HI226 分支无 return（UB）；CH010 的 `crc` 跨帧累积。
- `main()` 里 `HAL_Init()` 排在 `SystemClockConfig()` / `delay.Init()` 之后，顺序应调整。
- `label.h` 的 `ImuQueueHandle` 宏是死代码；`tim.h` 的 `pwm`、`imu.h` 的 `imu_chassis` 只声明未定义。

## 六、协作约定
- 源码 **UTF-8 + LF**，严禁另存为 GBK/ANSI（`.gitattributes` 强制）。
- 不提交构建产物（`.o/.dep/.rsp/.elf/.bin/.map/.user/.suo`）。
- 提交前缀：`feat:` `fix:` `refactor:` `docs:` `chore:` `style:`；一次提交只做一件事。
- `README.md` 的 §7 / §8 是原作者的差异与已知问题记录，可参考但其"数组长度 [1]"的描述与实际定义 `[2]` 不符。
