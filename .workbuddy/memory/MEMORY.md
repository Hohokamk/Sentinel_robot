# MEMORY.md — Sentinel_robot 项目长期笔记

> RCS 哨兵电控固件。STM32F405RGT6 + FreeRTOS V10.3.1 + STM32F4 HAL（旧版 bxCAN API）。
> 双平台：Windows VS+VisualGDB（`FreeRTOS.sln`）/ Linux VS Code + CMake+Ninja+OpenOCD。
> 用户 2026-09-24 明确：**当前主要工作区，作为代码指导老师逐步引导，未获允许不得改动任何代码**（所有代码由用户亲手敲）。

## 一、四层架构
| 层 | 文件 | 状态 |
|---|---|---|
| 任务调度 | `taskslist.cpp` | ✅ |
| 控制 | `control.cpp` | 底盘 ✅已上机 / 云台 🔄 / 射击 ⬜ |
| 设备模型 | `motor` `HTmotor` `imu` `RC` `judgement` `supercap` `xuc` `Power_read` `pid` `kalman` | 部分可用 |
| 驱动/系统 | `can` `usart` `tim` `gpio` `delay` `sysclk` `CRC` | ✅ 有坑 |

## 二、任务与节拍（tick = 1000Hz）
| 任务 | 优先级 | 栈 | 周期 | 职责 |
|---|---|---|---|---|
| start_task | 1 | 128 | 一次 | 建任务后自删 |
| ArmTask | 1 | 128 | 100ms | `DMmotorinit()` + `power.Send()` |
| DecodeTask | 3 | 128 | 5ms | `rc.Decode()` + `imu_pantile.Decode()` |
| MotorUpdateTask | 2 | 256 | 2ms | `Ontimer` 遍历 can1/can2 + 达妙 `State_Decode`+`DMmotor_Ontimer` |
| CanTransimtTask | 2 | 256 | 1ms | `%3` 轮转：0=达妙 / 1=0x1ff / 2=0x200（达妙实际周期 3ms） |
| ControlTask | 2 | 128 | 5ms | `chassis.Update()` → `pantile.Update()` → `rc.Update()` |

## 三、核心机制
- **`Motor::Ontimer(idata, odata)` 双职责**：`idata` 解析反馈、`odata` 回填发送缓冲。PID→电流在它内部（`ACC/POS/SPD/ACE` 分支）。
- `can.data[12][8]` 按 `StdId - 0x201` 索引；`can.temp_data[16]`：字节 0..7 = 0x200 帧(ID1-4)，8..15 = 0x1ff 帧(ID5-8)。
- **达妙（DM）完全绕开 `Motor` 体系，走独立双数组链**：
  | 方向 | 路径 |
  |---|---|
  | 收 | 达妙帧 → `can.cpp:146-151`（按 `Data[0]&0x0F` 匹配 `DMmotor[i].ID`）→ `can2.jointidata[i]` → `MotorUpdateTask`(2ms) `State_Decode()` → `pos`/`curSpeed`/`torque` |
  | 发 | 控制层写 `setPos`/`setSpeed` → `DMmotor_Ontimer(Kp,Kd, can2.jointpdata[i])` 按 `function` 编码（MIT 8 字节位域 / P_S = pos+vel / SPEED = vel+0）→ `CanTransimtTask` `DMmotor_transmit(can2)` → `Transmit(ID+offset, ...)`，offset：MIT=0x000 / **P_S=0x100** / **SPEED=0x200** |
  → **控制层唯一要做的事就是写 `setPos`/`setSpeed`**。不走 `Motor::Ontimer`/`temp_data`/0x1ff/0x200 帧；**必须挂 can2**（收发硬编码）；`DMmotor[0]=(0x09,P_S,Pitch)`、`[1]=(0x06,SPEED,Yaw)`；`Kp/Kd` 在 P_S 下是死参数（达妙 PID 在上位机调）。
  → **`DMmotor_Ontimer` 会原地钳制**：`LIMIT_MIN_MAX(setPos, ±4π)`、`LIMIT_MIN_MAX(setSpeed, ±10)` —— 钳的是 `setPos` 本身，**不是累加器 `mark_pitch`**。
  → **上电零缓冲陷阱**：`jointidata` 未收到帧时全 0 → `pos = uint_to_float(0,-4π,4π,16) = -4π`、`curSpeed = -10`。判"有没有真反馈"用 `(jointidata[i][0] & 0x0F) == (DMmotor[i].ID & 0x0F)`（与 ISR 同条件）。
- **电机装配（`STM32F405.cpp:32-55`）**：`can2_motor` = 底盘 M3508 ×4（ID1-4，`SPD/chassis`，`PID(1.5,0.1,0)`）；`can1_motor` = 摩擦轮 M3508 ×2（`[0]/[1]`=ID2/ID3，`SPD/shooter`，`PID(10,0,1.5,0)`）+ 拨盘 M2006（`[2]`=ID7，`SPD/supply`，speed `(20,0,0)`+position `(0.30,0,0)`）+ 云台 M6020（`[3]`=ID5，`POS/pantile`）。`:38-43`/`:49-54` 是注释块，不参与编译。
- **底盘**：四轮**全向轮**，左前起顺时针 1→2→3→4（FL/FR/RR/RL）= `chassis_motor[i]` = `can2_motor[i]` = 拨码 i+1，**已上机核实**。45° X 布局下全向轮与麦轮**逆解同构**：`w1=vx−vy−Kωz`、`w2=vx+vy+Kωz`、`w3=vx−vy+Kωz`、`w4=vx+vy−Kωz`；`K` 只影响自转手感（取 1.0）。
- ⚠️ **M2006 拨盘 `mode` 疑似漏改（未修）**：`can1_motor[2]` 用**两参 PID 构造**，而 `motor.h:31` 该构造注释写着 `//ACE模式` → 原作者本意走 ACE。**启用"单发拨弹"只需把 `SPD` 改成 `ACE`**，`pid[position]=0.30` 已备好。ACE 骨架在 `motor.cpp:100-122`，`pd`/`spinning`/`need_curcircle` 全空。**SPD = 按住连转；ACE 才能一格一格拨。**
- `SHOOTER::Update()` 三职责：① 摩擦轮 `setspeed = openRub ? shoot_speed : 0`；② 拨弹；③ `now_bullet_speed` 换算。`control.h` 字段：`openRub`/`supply_bullet`/`fraction`/`fullheat_shoot`/`auto_shoot`/`heat_ulimit`/`shoot_speed=6000`。
- ⚠️ **`maxspeed` 只在 POS 分支被用来钳 `speed_ref`；SPD 分支完全不读** → SPD 下 `setspeed` **无任何上界**（`CHASSIS::Update()` 自己 clamp 到 2000 正是为此）。`getmax()` 实值：M3508 `16384/2000`、M2006 `10000/3000`、M6020 `3000/20`（adjspeed=10）。
- CAN 波特率 = 42MHz/6/(1+2+4) TQ = **1 Mbps**（采样点 42.9% 偏低）。串口：USART1←IMU(115200)、USART2←遥控器(100000)、UART5←功率计(9600)、USART6←XUC(115200)、USART3/4 空闲。`UART` 类 = HAL+DMA+空闲中断+`xQueueOverwriteFromISR`。

- **达妙协议事实（2026-09-26 官方手册核实，上车排查必读）**：
  | 项 | 值 |
  |---|---|
  | POS_VEL 帧 | `0x100+ID`；8 字节 = float `p_des` + float **`v_des` = 限速**（梯形加速的最大速度），均小端 |
  | VEL 帧 | `0x200+ID`；float 速度 + 4 字节 0 |
  | 使能/失能/清错/存零 | 仲裁 ID = `motor_id`；数据 `FF×7 + FC/FD/FB/FE` |
  | 反馈帧 | 仲裁 ID = **MST_ID（默认 0）**；`Data[0] = status[7:4]｜motor_id[3:0]`，`Data[1:2]=POS`、`Data[3:4]=VEL`、`Data[5:6]=T`、`Data[6]=T_mos`、`Data[7]=T_rotor` |
  | status 码 | `0 = Disabled`（上电默认）/ `1 = Enabled` / `8~E` = 过压·欠压·过流·MOS过温·线圈过温·通信丢失·过载 |
  → **问询式反馈**：电机只在收到控制帧后才周期反馈。⇒ **不使能也能收到反馈帧**（`CanTransimtTask` 一直在发控制帧），但电机不出力 —— 这是最安全的"纯接收"测试姿势。
  → 读状态码：`can2.jointidata[0][0] >> 4`（代码 `State_Decode` **没有**解析 status，不会自动暴露）。
  → ⚠️ **V_MAX 映射范围**：达妙 preset 出厂可能是 `±100`，而代码 `V_MIN/V_MAX = ±10` → 若上位机没同步成 ±10，`curSpeed` 解码会差 10 倍（**只影响显示**；P_S 发送侧走 float 直传，不受 V_MIN/V_MAX 影响，因为 `DMmotor_Ontimer` 的 P_S 分支不是位域编码）。

## 四、进度清单（2026-09-26 更新）
| # | 项 | 状态 |
|---|---|---|
| 1 | `CHASSIS::Update()` | ✅ 已上机验证（逆解+限幅+RESET 归零） |
| 4 | `Motor::Ontimer()` `POS` / `SPD` 分支 | ✅ 完成（POS 含 latch+串级+clamp） |
| — | `CONTROL::Init()` 崩溃 / `init_dm()` / `ClampAngle()` / POS 限位参数 | ✅ |
| 5 | `RC::RC_Control()` 各 mode 分支 | 🔄 已写，**有通道冲突待修** |
| 2 | `PANTILE::Control_Pantile()` + `Update()` 三路 | 🔄 已写完（3 个 bug 已修），**待上机验达妙反馈** |
| 2b | `PANTILE::Keep_Pantile()` | ⬜ 空 |
| 1b | `CHASSIS::Keep_Direction()` | ⬜ 空 → FOLLOW 目前 ≡ SEPARATE |
| 3 | `SHOOTER::Update()` | ⬜ 空（`taskslist.cpp:134` 调用被注释） |
| 4b | `Motor::Ontimer()` `ACE` 分支 | ⬜ 空 |
| 5b | `RC::OnPC()` | ⬜ 空（哨兵主控制方式，需图传） |
| 6 | `judgement`/`supercap`/`xuc.Encode()` 接入 main | ⬜ 零调用点 |

## 五、已知坑（未修，改动前必须先问用户）
- ✅ **达妙命令双写入方 已修**（2026-09-26）：RESET 分支改为调 `Control_Pantile(0,0,0)`，让遥控层只写意图；`DMmotor[i].setSpeed` 现在只有 `PANTILE::Update()` 一个写者。
- ✅ **`mark_pitch` windup 已修**：`PANTILE::Update()` 里 `LIMIT_MIN_MAX(mark_pitch, pitch0-0.35f, pitch0+0.35f)`，用**相对 latch 点**的窗口（±20°）——因为达妙零点未知，绝对角度不可用。`pitch0` 在 latch 时一起赋值（`control.h:41`）。
- ✅ **`SPINNING` 通道冲突 已修**：摇杆赋值从 switch 之前挪进各 case。
- ⚠️ **【新】`FOLLOW` / `SPINNING` 忘记读摇杆**（挪进 case 的副作用）：`speedx/speedy` 不再全局赋 0 → 它们保留上一模式的残留值。FOLLOW 摇杆完全失效（车以进入前速度一直跑）；SPINNING 平移卡旧值。修法：FOLLOW 在 `Keep_Direction()` 前补读摇杆，SPINNING 显式写 `speedx = speedy = 0`。
- ⚠️ 通道分配的根本矛盾仍在：`Control_Pantile` 第三参在 FOLLOW/SEPARATE/FIRE 恒传 0（达妙 yaw 无输入）、第一参在 ROTATION 传 0（大疆 yaw 无输入）→ **4 通道装不下「平移 2 轴 + 云台 3 轴」**，方案待用户定。
- `RC.cpp:19-23` `mark_yaw` 赋值已**死代码**（`Control_Pantile` 直接用 `setangle` 当累加器）且**无判空**。
- `FIRE` 里 `openRub = true` 无人消费（`SHOOTER::Update()` 空）；且只在 RESET 清 false → 离开 FIRE 后残留，接好 `SHOOTER::Update()` 后会突然转摩擦轮（定时炸弹）。
- ✅ **`init_dm()` 的 `setPos = 0` 不再是危险项**：达妙 POS_VEL 的第二个 float 是**限速 vel_limit**，而 `DMmotor[0].setSpeed` 在 latch 成功前保持 0（全局零初始化，且 `1.5f` 那句写在 guard 内部）→ **限速 0 ⇒ 位置规划器不产生运动**。所以"反馈完全没进来"时 Pitch 只会保持不动，不会朝电机零位跑。**天然安全阀，别把它改掉。**
- ✅ **达妙 Pitch 已有相对限位**：`pitch0 ± 0.35 rad`（±20°）。**不是机械限位**，是真机标定前的临时窗口；实测零点/行程后换绝对值。
- **云台构型（用户 2026-09-25 确认）**：达妙 Pitch(0x09) + 达妙 Yaw(0x06) + M6020(ID5)。`pantile_motor[YAW]` 装的是 **M6020**；`pantile_motor[PITCH]` 恒 nullptr是**预期状态**（框架复用），判空跳过即正解。**达妙不进 `Motor` 体系**（用户明确不动继承体系）。
  `PANTILE::TYPE` 只有 `{YAW,PITCH}` 两值（装不下 3 轴）；三个「yaw」互不相干：`PANTILE::TYPE::YAW`（下标 0，指向 M6020）/ `POSITION::Yaw`（达妙标签）/ 物理小 yaw。
- **两套单位体系（极易出错）**：大疆 = 机械编码值 `0~8191` ↔ `0~360°`（`Motor::setangle`/`angle[]`）；达妙 = **弧度**（`setPos`，`P_MIN=-4π`）。`para.initial_pitch/initial_yaw = 4096`（=180°）是**大疆编码值**：`initial_yaw` 有主（M6020），`initial_pitch` 本机无消费者（死代码）。`INIT_ANGLE_F/B` 是原机型轮腿角度，与本机无关。
- **M6020 主 yaw 行程（实测）**：顺时针 = `angle[now]` **减小**；两限位 ≈ **2000 / 6000**（跨 0/8192）。行程 4192（≈184.2°）→ `yaw_center=8096`、`yaw_span=1896`（两端各留 200，写死 `label.cpp:13`）。**clamp 用「中点 + 半宽」，不能用 min/max**（跨零点）。⚠️ 行程 > 4096 时**不能用 `getdeltaa(L2-L1)` 反推**——它返回互补弧，会算错中点。
- `sum_angle` 的 0 点 = 上电位置（每次上电不同），**不能写死**；`angle[now]` 是 M6020 绝对机械角，断电不丢，**可写死**。
- ✅ **M6020 拨码必须 = 1**（代码里写 `ID5`）：index = `ID5-ID1` = 4 → 反馈必须 StdId `0x205`，发送落 `temp_data[8][9]` = 0x1FF 帧字节 0-1。**拨 5 会全链路失效**（已澄清并解决）。
- ✅ `CONTROL::Init()` 崩溃已修：`supply` 分支"先赋值指针再写成员"+`break`；`pantile_motor[]` 加判空；`num1..num4` → 成员计数器 `chassis_num/pantile_num/shooter_num/supply_num` + 容量检查。（`supply` 缺 `break` **不是**崩溃原因。）
- ⚠️ **`DMMOTOR::GetPosition()` 恒返回 0**：`return angle[now]`，但 `State_Decode` 写的是 `pos`，`angle[]` 无人写。读达妙当前位置用 `DMmotor[i].pos`。
- ⚠️ **`Motor_Start`/`Motor_Stop`/`ZeroPosition` 只有声明无定义** → 调用即链接错误。
- ⚠️ `can.cpp:144-151` 达妙接收分支**不分总线**：can1 上非 `0x201~0x208` 的帧也会写进 `can2.jointidata`。
- ⚠️ **`Motor::pid[2]` vs `enum{speed=0, position, speed2=2}`**：三参构造的 `memcpy(&pid[2], ...)` 越界写到 `Torque_constant_2006`/`angle[]`。**当前零调用**（只用 1/2 参构造），要用第三套 PID 时须把 `pid[2]` 改 `pid[3]`。
- `motor.cpp` 的 `initial_cnt` 是文件级全局，被 8 个电机共享。
- `RC::Decode()` 的 `sizeof(m_frame) < 18` 恒假；遥控**断连时提前 return，`ch/s[]` 保持旧值 → 车按最后指令继续跑**（待加帧超时→STOP）。
- `usart.h` 在类内初始化 `xQueueCreate`（全局对象构造期分配 RTOS 堆）。
- `imu.cpp` `Check()` 的 HI226 分支无 return（UB）；CH010 的 `crc` 跨帧累积。
- `main()` 里 `HAL_Init()` 排在 `SystemClockConfig()`/`delay.Init()` 之后，顺序应调整。
- `label.h` 的 `ImuQueueHandle` 宏是死代码；`tim.h` 的 `pwm`、`imu.h` 的 `imu_chassis` 只声明未定义。
- `judgement`/`supercap` 未接入 → 射击无热量/弹量保护、底盘无功率控制。

## 六、协作约定
- 源码 **UTF-8 + LF**（`.gitattributes` 强制），严禁 GBK/ANSI。
- 不提交构建产物（`.o/.dep/.rsp/.elf/.bin/.map/.user/.suo`；`STM32F405-Debug.vgdbsettings` 同属 IDE 配置，宜忽略）。
- 提交前缀：`feat:` `fix:` `refactor:` `docs:` `chore:` `style:`；一次提交只做一件事。
- 用户要求：**先走主线实现完整功能，忽略 task 顺序一类低效细节**；**代码全部由用户自己改**。
