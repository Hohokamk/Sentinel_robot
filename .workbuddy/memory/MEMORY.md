# MEMORY.md — Sentinel_robot

> RCS 哨兵电控固件。STM32F405RGT6 + FreeRTOS + STM32F4 HAL。VS+VisualGDB / CMake+Ninja+OpenOCD。
> **用户要求：AI 不写代码，只做代码指导，所有代码由用户亲手敲。**

## 一、架构与节拍（tick 1kHz）
四层：`taskslist`(调度) / `control`(控制) / `motor·HTmotor·imu·RC·judgement`(模型) / `can·usart·tim`(驱动)
ArmTask 100ms `DMmotorinit()`×2+`power.Send()`｜DecodeTask 5ms `rc.Decode`+`imu.Decode`｜MotorUpdateTask 2ms can1/can2 `Ontimer`+达妙解码｜CanTransimtTask 1ms `%3`(0=达妙/1=0x1FF/2=0x200)｜ControlTask 5ms `chassis.Update`→`pantile.Update`→`shooter.Update`→`rc.Update`

## 二、装配（`STM32F405.cpp:32-59`）
- `can2_motor`：底盘 M3508×4（ID1-4，SPD/chassis，`PID(1.5,0.1,0)`）
- `can1_motor`：摩擦轮 M3508×2（[0][1]=ID2/3，SPD/shooter，`PID(10,0,1.5)`）+ 拨盘 M2006（[2]=ID7，**POS**/supply，`PID(20,0,0)`+`PID(0.30,0,0)`）+ M6020（[3]=ID5，POS/pantile）
- `DMmotor[0]=(0x09,P_S,Pitch)` 挂 **can1**；`[1]=(0x06,SPEED,Yaw)` 挂 **can2**；总线表 `taskslist.cpp:13 dm_bus[2]`
- ⚠️ **容量宏**：`CHASSIS=4 / PANTILE=2 / SHOOTER=2 / SUPPLY=1`。数组越界会**静默别名到下一个成员**（`shooter_motor[2]` == `supply_motor[0]`）
- 底盘全向轮 45° X，左前起顺时针 1→2→3→4 = `chassis_motor[i]` = 拨码 i+1。逆解 `w1=-vx-vy+Kωz`、`w2=+vx-vy+Kωz`、`w3=+vx+vy+Kωz`、`w4=-vx+vy+Kωz`
- 云台三轴 = M6020 大 yaw + 达妙 Pitch + 达妙 Yaw；`pantile_motor[PITCH]` 恒 nullptr

## 三、必记机制
- `Motor::Ontimer(idata, odata)`：解析反馈+回填发送缓冲。`can.data[12][8]` 按 `StdId-0x201`；`temp_data[16]` 字节 0-7=0x200 帧、8-15=0x1FF 帧。PID→电流在内部（POS/SPD/ACE）
- 达妙独立于 `Motor` 体系：控制层只写 `setPos`(弧度)/`setSpeed`；`jointidata`/`jointpdata` 是 **CAN 实例成员**（can1/can2 各一份，别写错总线）
- 达妙协议（已核官方手册）：POS_VEL 第二 float = **限速**；**问询式反馈**（收到自己控制帧才回）；`Data[0]=status[7:4]|id[3:0]`，status：0=Disabled / 1=Enabled / 8~E=过压·欠压·过流·MOS过温·线圈过温·通信丢失·过载；使能/失能/清错/存零 = 仲裁 ID `motor_id` + `FF×7+FC/FD/FB/FE`
- 零缓冲陷阱：`jointidata` 全 0 → `pos=-4π`。判"真帧"用 `(jointidata[i][0]&0x0F)==(DMmotor[i].ID&0x0F)`
- 单位：大疆=编码值 0~8191；达妙=弧度。**`maxspeed` 只在 POS 分支生效，SPD 无上界**
- M6020 主 yaw：顺时针 = `angle[now]` 减小；`yaw_center=8096`、`yaw_span=1896`；clamp 用「中点+半宽」。`angle[now]` 可写死、`sum_angle` 不可
- 拨码 ↔ 反馈 ID（两个厂家基准不同）：索引 = `IDn - ID1` = `n-1`；ISR 落点 `data[StdId-0x201]`
  - `C620`(M3508)/`C610`(M2006)：反馈 `0x200+拨码` → **物理拨码 = n**
  - `GM6020`(M6020)：反馈 `0x204+拨码` → **物理拨码 = n-4**（`ID5` 拨 **1**）
  - can1 实配：`ID2`→拨2、`ID3`→拨3、`ID7`→**拨7**、`ID5`→**拨1**
- ⚠️ **`has_feedback` 唯一入口 = `motor.cpp:78`**，原判据 `temperature != 0`（`idata[idx][6]`）。**本机 C610 温度字节恒 0** → 已加兜底 `|| idata[slot][0] || idata[slot][1]`。连带 `temperature>70 → setspeed=0` 的**过温保护对拨盘失效**
- `judgement`/`supercap` 类已实现但**零调用点**；`xuc.Encode()` 未调
- ⚠️ `supply_bullet` 被 RC 每帧清 false 且 `Update()` 不消费 → 死变量。拨弹判据 = `openRub && 弹速达标 && 扳机`
- ⚠️ **POS 分支多圈必须走 `use_sum_angle` + `sum_angle`**（`getdeltaa`+`(int16_t)` 对多圈失效）；latch 时补 `angle[pre]=angle[now]`
- `Position(err, limit)` 第二参钳的是**积分槽**，`Ti=0` 时无效（纯 P）

## 四、发射机构（POS 双环，已落地待上机）
- `motor.h:62 use_sum_angle`｜`motor.cpp:110-124` POS 双路｜`can1_motor[2]` = `Motor(M2006,POS,supply,ID7,PID(20,0,0),PID(0.30,0,0))`｜`main()` 里 `can1_motor[2].use_sum_angle = true;`
- `para.bullet_step=36864`（一发 = `8192×36/N`，N=8）/ `bullet_lead_max=73728`（2 发）
- `SHOOTER::Update()`（`control.cpp:142-191`）六段：摩擦轮 / 弹速 / 迟滞 / 长短拨 / POS 累加 / 超前钳制
- 扳机 = ch[1]（**仅 FIRE 模式**）；RC 只写原值 `trig_raw`，阈值+迟滞全在 Update 内
- 外环 `Kp=0.30` 纯 P → 一发 1620° → `speed_ref=486rpm` → τ≈0.56s → **约 3s 一发，太慢**（要 125ms 需 Kp≈3~6）

## 五、进度（2026-09-26 深夜）
✅ 底盘 / 云台 / POS 串级 / latch+ClampAngle / `CONTROL::Init()` / `init_dm()` / RC 各 mode / 达妙分总线 / `SHOOTER::Update()` 已写
⬜ **上机调试中**：拨盘 POS 双环整定 + 达妙 Pitch 使能排查｜`ACE` 分支｜`Keep_Direction()`｜`Keep_Pantile()`｜`RC::OnPC()`｜judgement/supercap 接入

## 六、活跃陷阱
- ⚠️ **越界别名**：`shooter_motor[2]`（`SHOOTER_MOTOR_NUM=2`）别名 = `supply_motor[0]`（拨盘）→ 弹速计算混入拨盘；同类：任何 `[i]` 超出容量宏都是静默别名
- ⚠️ **`pitch0` latch 竞态**：`PANTILE::Update()` 的 guard 看 ISR 原始 `jointidata[0][0]`，而 `DMmotor[0].pos` 由 `MotorUpdateTask` 的 `State_Decode` 写 → 存在"guard 先成立、`pos` 仍是初值 0"的窗口 → `pitch0=0` → 朝电机零位冲。且 `pitch_ready` **永不复位** → 一旦 latch 错就永久错。修法：guard 改用"已解码"标志
- ⚠️ **`setSpeed=0` 是天然保护**（`PANTILE::Update()` 里 `setSpeed=1.5f` 写在 guard **内部**）→ guard 不成立时 POS_VEL 限速 0，Pitch 只会不动。**不要把它挪到 guard 外**
- ⚠️ 遥控断连 `Decode()` 提前 return → `ch/s[]`/`trig_raw` 保持旧值 → 车按最后指令跑 + 扳机卡在按下**无限连发**
- ⚠️ `ArmTask`(prio 1) 与 `CanTransimtTask`(prio 2) 共用 `hcan.pTxMsg`（`CAN::Transmit` 非线程安全）→ 使能帧可能被打断；代码缺 `0xFB` 清错帧
- ⚠️ 拨盘 `current` 增量式：误差归零后 `current` 冻结在上次值；`lead` **只有单边钳制**（冲过头会反向震荡）
- ⚠️ 拨盘抖动 4 候选：①`sum_angle` 方向反（正反馈撞限位）②纯 P 极限环 ③ch[1] 停 200~300 反复上升沿 ④`allow` 被越界指针污染
- ⚠️ 达妙 Pitch 无机械限位 clamp（固件只钳 ±4π）；`mark_pitch` 有 `pitch0±0.35` 但依赖 latch 正确
- 达妙 Yaw 仅 `SPINNING` 有输入；`ROTATION` 的 `speedz=rc.ch[2]` 未换算；`AUTOAIM`/`default` 不调 `Control_Pantile`
- `can.cpp` 达妙接收分支**不分总线**；`GetPosition()` 恒返 0（读 `.pos`）；`Motor_Start/Stop/ZeroPosition` 无定义；`pid[2]` vs `speed2=2` 越界（零调用）；`main()` 的 `HAL_Init()` 顺序反；`imu.cpp` `Check()` 有分支无 return
- `ControlTask` 用 `vTaskDelay(5)` 非 `vTaskDelayUntil` → `hold_ms` 计时偏长

## 七、协作约定
UTF-8 + LF；不提交构建产物与 `*.vgdbsettings`；提交前缀 `feat:/fix:/refactor:/docs:/chore:`，一次提交只做一件事。**先走主线，忽略 task 顺序类低效细节。**
