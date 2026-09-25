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
- **达妙（DM）完全绕开 `Motor` 体系，走独立双数组链**（2026-09-26 核实）：
  | 方向 | 路径 |
  |---|---|
  | 收 | 达妙帧 → `can.cpp:146-151`（按 `Data[0]&0x0F` 匹配 `DMmotor[i].ID`）→ `can2.jointidata[i][8]` → `MotorUpdateTask`(2ms) `DMmotor[i].State_Decode()` → `pos`/`curSpeed`/`torque` |
  | 发 | 控制层写 `DMmotor[i].setPos`/`setSpeed` → `DMmotor_Ontimer(Kp,Kd, can2.jointpdata[i])` 按 `function` 编码（MIT 8 字节位域 / P_S = pos+vel / SPEED = vel+0）→ `CanTransimtTask`(1ms, `%3` 轮转) `DMmotor_transmit(can2)` → `Transmit(ID+offset, jointpdata[slot], 8)`，offset：MIT=0x000 / **P_S=0x100** / **SPEED=0x200** |
  → **控制层唯一要做的事就是写 `setPos`/`setSpeed`**，其余全自动。不走 `Motor::Ontimer`、不走 `temp_data`、不走 0x1ff/0x200 帧；**DM 必须挂 can2**（收发都硬编码 can2）；实际发送周期 = **3ms** 不是 1ms。
  → 本机装配：`DMmotor[0] = (0x09, P_S, Pitch)`、`DMmotor[1] = (0x06, SPEED, Yaw)`。`Kp/Kd` 是死参数（P_S 模式不发），达妙 PID 在上位机调。
  → **上电零缓冲陷阱**：`jointidata` 未收到帧时全 0 → `pos = uint_to_float(0,-4π,4π,16) = -4π`、`curSpeed = -10`。判"有没有真反馈"用 `(jointidata[i][0] & 0x0F) == (DMmotor[i].ID & 0x0F)`（与 ISR 同一条件），或给 `DMMOTOR` 加 `has_feedback`（在 `State_Decode` 里按 `idata[id][0] != 0` 置位，与 `Motor` 的温度判据同构）。
  → **P_S 模式下第二个 float（`setSpeed`）是速度上限**，填 0 可能导致电机不动/报故障；"停"应当靠把 `setPos` 写回当前位置，而不是把 `setSpeed` 清 0。
- 电机装配（`STM32F405.cpp:32-55` 实读）：`can2_motor` = 底盘 M3508 ×4（ID1-4，`SPD/chassis`，`PID(1.5,0.1,0)`）；`can1_motor` = 摩擦轮 M3508 ×2（`[0]/[1]` = ID2/ID3，`SPD/shooter`，`PID(10,0,1.5,0)`）+ 拨盘 M2006（`[2]` = ID7，`SPD/supply`，speed `(20,0,0)` + position `(0.30,0,0)`）+ 云台 M6020（`[3]` = ID5，`POS/pantile`）。`:38-43`/`:49-54` 是原作者留在数组里的注释块，不参与编译。
- ⚠️ **M2006 拨盘盘的 `mode` 疑似漏改（未修）**：`can1_motor[2]` 用的是**两参 PID 构造**，而 `motor.h:31` 该构造的注释写着 `//ACE模式` → 原作者本意走 ACE，`mode` 字段填了 `SPD`。**启用"单发拨弹"只需把 `SPD` 改成 `ACE`**，`pid[position]` 已备好（0.30），无需新增 PID。（两参构造只写 `pid[0]/pid[1]`，不越界；只有三参构造的 `speed2 = 2` 会越界写。）
  ACE 分支（`motor.cpp:100-122`）骨架已在，三个成员 `pd`（单次拨弹）/`spinning`（一秒八发）/`need_curcircle`（待拨格数）专为它设计，目前全空。
  **拨盘走 SPD = 按住连续转；走 ACE 才能一格一格拨。**
- `SHOOTER::Update()` 三职责：① 摩擦轮 `shooter_motor[i]->setspeed = openRub ? shoot_speed : 0`；② 拨弹（SPD 写 `setspeed` / ACE 置 `pd`·`spinning`·`need_curcircle`）；③ `now_bullet_speed` 由摩擦轮 `curspeed` 换算 → "转速达标才允许拨弹" + 上报裁判。
  `control.h:44-56` 字段语义（原作者已定义）：`openRub` 开摩擦轮 / `supply_bullet` 拨弹 / `fraction` 单发 / `fullheat_shoot` 满热量连转 / `auto_shoot` 自动 / `heat_ulimit` 热量超限 / `shoot_speed = 6000`。
- ⚠️ **`maxspeed` 只在 POS 分支被用来钳 `speed_ref`；SPD 分支完全不读它** → SPD 模式下 `setspeed` **没有任何上界**（`CHASSIS::Update()` 自己 clamp 到 2000 正是为此）。`getmax()` 实值（`motor.cpp:200-233`）：M3508 `16384/2000`、M2006 `10000/3000`、M6020 `3000/20`、adjspeed 默认 3000。
- `ctrl.Init(vector<Motor*>)` 被 main 调两次，按 `function_type` 分派进 chassis/pantile/shooter/supply 数组。
- CAN 波特率 = 42MHz / 6 / (1+2+4) TQ = **1 Mbps**（正确，但采样点 42.9% 偏低）。
- 串口分工：USART1←IMU(CH010,115200)、USART2←遥控器(100000)、UART5←功率计(9600)、USART6←XUC 上位机(115200)、USART3/UART4 空闲。
- `UART` 类 = HAL + DMA 收发 + 空闲中断 + `xQueueOverwriteFromISR` 投递队列，是所有串口外设的地基。

## 四、待实现清单（2026-09-26 更新）
| # | 项 | 状态 |
|---|---|---|
| 4 | `Motor::Ontimer()` 的 `SPD` / `POS` 分支 | ✅ 已完成（POS 含 latch + 串级 + ClampAngle） |
| 1 | `CONTROL::CHASSIS::Update()` | ✅ 已完成并上机验证（45° X 逆解 + maxspeed 限幅 + RESET 归零） |
| 1b | `CONTROL::CHASSIS::Keep_Direction()` | ⬜ 空（只服务 FOLLOW，需 IMU/云台角） |
| 5 | `RC::RC_Control()` 各 mode 分支 | 🔄 进行中（本轮主线） |
| 2 | `PANTILE::Control_Pantile()` | 🔄 进行中（本轮主线） |
| 2b | `PANTILE::Update()` 三路（M6020 / DM Pitch / DM Yaw） | 半成品：目前只有 M6020 的 ClampAngle |
| 2c | `PANTILE::Keep_Pantile()` | ⬜ 空 |
| 3 | `CONTROL::SHOOTER::Update()` | ⬜ 空 |
| 4b | `Motor::Ontimer()` 的 `ACE` 分支 | ⬜ 空（拨盘先走 SPD） |
| 5b | `RC::OnPC()` | ⬜ 空（哨兵需图传/上位机才有意义） |
| 6 | `judgement` / `supercap` / `xuc.Encode()` 接入 main | ⬜ 零调用点 |

## 五、已知坑（尚未修，改动前必须先问用户）
- ✅ **`CONTROL::Init()` 崩溃已修（2026-09-25）**：`supply` 分支改为"先赋值指针、再写成员"+补 `break`；`pantile_motor[PITCH]/[YAW]` 两行加判空；`num1..num4` 提升为成员计数器 `chassis_num/pantile_num/shooter_num/supply_num`（`control.h:11`）。
  ⚠️ **更正过两次**：`supply` 缺 `break` **不是崩溃原因**（fall through 到 `default`，等效 no-op，仅 `-Wimplicit-fallthrough` 警告）。
  **遗留**：① 成员计数器无容量上界检查（传超额电机即静默越界写）；② `pantile_motor[PITCH]` 在本机恒 nullptr（预期状态，见下）。
- **云台构型（用户 2026-09-25 确认）**：**达妙 Pitch(0x09) + 达妙 Yaw(0x06) + M6020(ID5，「小 yaw」)**，共 3 台。
  → `pantile_motor[PANTILE::TYPE::YAW]` 装的是 **M6020**，不是达妙。`pantile_motor[PITCH]` 恒空是**框架复用导致的预期状态**（原作者机型不用达妙做云台），判空跳过即正解，不必"修好"。
  → **达妙不进 `Motor` 体系**（独立类、无继承、不在 can1/can2 数组里），`CONTROL::Init(vector<Motor*>)` 收不到。达妙单独初始化，用户明确**不动继承体系**。
  → `control.h:64` `init_dm()` 待实现（钩子）；`control.h:11` `init_DM` 标志未使用。
- **两套单位体系（极易出隐蔽错误）**：
  | | 大疆（M6020/M3508/M2006） | 达妙 |
  |---|---|---|
  | 位置单位 | 机械编码值 `0~8191` ↔ `0~360°`（`label.h:23-24` 宏） | **弧度**，`P_MIN=-4π`/`P_MAX=4π`（`HTmotor.h:20-21`） |
  | 成员 | `Motor::setangle` / `angle[]` | `DMMOTOR::setPos` |
  | 初值入口 | `ctrl.Init()` | `ctrl.init_dm()`（待实现） |
  `para.initial_pitch=4096`（=180°）/ `initial_yaw=4096`（=180°，2026-09-25 实读 `label.cpp:8`，**早前记忆里的 4900 已过期**）是**大疆编码值**：`initial_yaw` 有主（M6020，`control.cpp:46`），`initial_pitch` 本机无消费者（`control.cpp:45` 因 `pantile_motor[PITCH]` 永为 nullptr 而恒假 → **死代码**）。**两者都不能直接给达妙**——`4096` 若按弧度会被 `LIMIT_MIN_MAX` 钳到 `4π`。`INIT_ANGLE_F/B`（`HTmotor.h:12-14`）是原机型**轮腿**角度，与 Pitch/Yaw 无关。**两行都无钳制**；`label.h:35` 的 `pitch_min/pitch_max`（现填 `0/8192` = 全圈）是预留的限位参数，**目前零引用**。
- **三个「yaw」互不相干**：`PANTILE::TYPE::YAW`（枚举 0，数组下标，指向 M6020）/ `POSITION::Yaw`（枚举 6，达妙标签）/ 物理小 yaw（`can1_motor[3]`）。
  已存在使用者 `RC.cpp:20` `ctrl.pantile.mark_yaw = (float)ctrl.pantile_motor[PANTILE::YAW]->sum_angle;` → **`mark_yaw` 单位是机械编码累积值（含圈数），不是度**；`mark_pitch` 单位未定义。
- **待用户确认（阻塞云台控制设计）**：两个 yaw（达妙 Yaw + M6020 小 yaw）的**机械拓扑** —— 串联粗调/精调？同轴反装消隙？双独立方位轴？这决定 `PANTILE::Update()` 怎么写。另：`PANTILE::TYPE` 只有 `{YAW,PITCH}` 两值，**装不下 3 轴**。
- ✅ `HTmotor.h` 声明 `DMMOTOR DMmotor[2]` 已与定义一致（原 `[1]` 已修）；`State_Decode`/`DMmotor_transmit` 改用 `&DMmotor[i]==this` **反查槽位**（不再用 `ID-1`），索引口径与 `jointidata/jointpdata` 统一。
- ✅ 所有 `CAN hcan` 参数已改 `CAN&`。
- ⚠️ **`DMMOTOR::GetPosition()` 恒返回 0（未修）**：它 `return angle[now]`，而 `State_Decode` 写的是 `pos`（`HTmotor.cpp:31`），`angle[]` 全程无人写。取达妙当前位置要读 `pos`。
- ⚠️ **`Motor_Start` / `Motor_Stop` / `ZeroPosition` 只有声明没有定义**（`HTmotor.h:69/71/75`，`HTmotor.cpp` 里无实现）→ 调用即链接错误。
- ⚠️ **`can.cpp:144-151` 达妙接收分支不分总线**：`id` 不在 `0x201~0x208` 的帧**无论从 can1 还是 can2 来**都被写进 `can2.jointidata`。DM 挂 can2 所以当前无害，但 can1 上出现非标 ID 会污染达妙反馈。
- `motor.cpp` 的 `initial_cnt` 是文件级全局变量，被 8 个电机共享。
- `MotorUpdateTask`/`CanTransimtTask` 把 `xlastWakeTime` 写在 `while` 内 → `vTaskDelayUntil` 退化为 `vTaskDelay`。
- `RC::Decode()` 的 `sizeof(m_frame) < 18` 恒假；`control.h` 的 `mode` 未初始化。
- `usart.h` 在类内初始化 `xQueueCreate`（全局对象构造期分配 RTOS 堆）。
- `imu.cpp` `Check()` 的 HI226 分支无 return（UB）；CH010 的 `crc` 跨帧累积。
- `main()` 里 `HAL_Init()` 排在 `SystemClockConfig()` / `delay.Init()` 之后，顺序应调整。
- `label.h` 的 `ImuQueueHandle` 宏是死代码；`tim.h` 的 `pwm`、`imu.h` 的 `imu_chassis` 只声明未定义。
- ⚠️ **M6020（`can1_motor[3]`，代码里写 `ID5`）的索引链条要求拨码开关 = 1**：index = `ID5 - ID1` = 4 → 接收要 `can1.data[4]` 被填，唯一可能写入者是 StdId **0x205** = GM6020 拨码 1；发送落点 `temp_data[8][9]` = 0x1FF 帧字节 0-1 = 也是 GM6020 ID1。**若拨码 = 5**：反馈 StdId 0x209 超出 `can.cpp` 的 `id<=0x208` → 落进达妙分支（`angle[now]` 恒 0，且约 12.5% 位置污染 `can2.jointidata[0/1]`），控制帧需 0x2FF 而代码只发 0x1FF → 永不动作。`motor.cpp` 里被注释的 `if(M6020) +=4` 是另一套方案残留，**启用会越界**（`odata[16]`）。
- `sum_angle` 的 0 点 = 上电位置（每次上电不同），**不能用于写死初值/限位**；`angle[now]` 是 GM6020 绝对机械角（0~8191），断电重启不丢失，**可以写死**。
- **M6020 主 yaw 行程（2026-09-25 实测）**：顺时针 = `angle[now]` **减小**；两机械限位 ≈ **2000** 与 **6000**（跨 0/8192 回绕）。行程 **4192** 计数（≈184.2°）→ 行程中点 **center = 8096**、半宽 **span = 2096**（clamp 用「中点 + 半宽」而非 min/max）。⚠️ **行程 > 4096 时不能用 `getdeltaa(L2 - L1)` 反推行程** —— 它取最短路径，返回的是**互补弧**（这里 4000 = 不可达段），会算出错误的中点；必须先知道可行方向再算。
- `ctrl.Init()` 把 `setangle = para.initial_yaw` 写在 `main` 里（`main:100`，即 `STM32F405.cpp:100` 第二次调用，触发 `control.cpp:46`），而此时 CAN 中断（`:82`）早已在收帧 → `MotorUpdateTask` 第一次 `Ontimer` 就能读到真反馈 → **首帧 latch 成立且早于任何 PID 输出**，`initial_yaw` 实际退化为占位符。
- **`setangle` 目前读者为零**（POS 分支空 + `PANTILE::Update()` 空 + `taskslist.cpp:133` 该调用被注释），写者只有 `control.cpp:46` → 该行对硬件零影响；写 POS 后若不加 latch，则 4096 会成为开机目标（云台自动转向编码 4096，无任何钳制）。

## 六、协作约定
- 源码 **UTF-8 + LF**，严禁另存为 GBK/ANSI（`.gitattributes` 强制）。
- 不提交构建产物（`.o/.dep/.rsp/.elf/.bin/.map/.user/.suo`）。
- 提交前缀：`feat:` `fix:` `refactor:` `docs:` `chore:` `style:`；一次提交只做一件事。
- `README.md` 的 §7 / §8 是原作者的差异与已知问题记录，可参考但其"数组长度 [1]"的描述与实际定义 `[2]` 不符。
