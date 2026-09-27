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
- `para.bullet_step=42130.29`（一发 = `8192×36/N`，**N=7**）/ `bullet_lead_max=84260.58`（2 发）；单发与连发**已统一为 `-bullet_step`**
- 射频红线（N=7）：8 发/s 需 2469 rpm（`maxspeed=3000` 的 82%）；纯满速上限 ≈9.7 发/s；单发最短 ≈103ms
- `SHOOTER::Update()`（`control.cpp:142-191`）六段：摩擦轮 / 弹速 / 迟滞 / 长短拨 / POS 累加 / 超前钳制
- 扳机 = ch[1]（**仅 FIRE 模式**）；RC 只写原值 `trig_raw`，阈值+迟滞全在 Update 内
- 外环 `Kp=0.30` 纯 P → 一发 1620° → `speed_ref=486rpm` → τ≈0.56s → **约 3s 一发，太慢**（要 125ms 需 Kp≈3~6）

## 四之二、上位机链路（XUC + RC::OnPC，2026-09-27 主线）

> 哨兵标准控制方式 = **上位机自瞄**，不是遥控器。这条链路优先级高于 Keep_*。

- 硬件：`xuc.Init(&uart3, USART3, 115200)`（`STM32F405.cpp:91`）已初始化；**`xuc.Decode()` / `xuc.Encode()` 零调用点**
- 报文（`xuc.h:10-30`）：他机帧 `RxPacket{header=0xA5; checksum}` 只是占位；实际按**偏移硬解析**：`[1..4]pitch(度)`、`[5..8]yaw`、`[9..12]yaw_diff`、`[13..16]pitch_diff(度)`、`[17..20]distance`、`[21]fireadvice:1`、`[25..28]v_y`
- 上报帧 `TxPacket{0x5A, detect_color:1, reset_tracker:1, reserved:6, roll/pitch/yaw(float), aim_x/y/z, checksum}` + CRC16
- 两条发送路径：`UARTTransmit()`（`usart.cpp:517`，**HAL 阻塞轮询**，timeout 0xffff）/ `DMATransmit()`（`usart.cpp:507`）。两个 `DMmotorinit` 用阻塞版在 ArmTask(prio 1)、`CanTransimtTask`(prio 2) 会抢占 → 最长可卡 100ms 左右
- `judgement.BuffData()` 用了 `m_uart->updateFlag` / `dataDmaNum`，但 `usart.cpp` 里**这两个成员从未被写**（ISR 只用 `xQueueOverwriteFromISR`）→ 恒 false，零调用所以无害

### ⚠️ `xuc.cpp` 三处必修（2026-09-27 实读）
1. **`u8_to_float()` 字节序反了**（`xuc.h:144-148`）：构造了 `ch[4]` 大端副本后**丢弃**，`memcpy(&s, p, 4)` 拿的是原始小端 → 所有 float 是垃圾。正解：直接 `memcpy`（= `xuc.h:150 FR4()`），或删掉 `u8_to_float` 统一用 `FR4`
2. **字段名混用**（`xuc.cpp:33-39`）：成员函数内写 `xuc.xxx = ...` 灌的是**全局对象**，而 `:30-31` 的 `yaw_pre` 走 `this` → 同一函数两套绑定。虽当前只有一个 `xuc` 实例、结果碰巧对，但语义错。**统一去掉 `xuc.` 前缀**
3. **`yaw_spd` 恒 0**（`xuc.cpp:30-31`）：`yaw_pre = yaw;` 后紧接 `yaw_spd = ((yaw - yaw_pre)/0.004)*2π/60;`，两语句间 `yaw` 未被赋值 → 差恒 0。且量纲错（`yaw` 是度，注释却写 rpm）。顺序必须：先 `yaw = ...`（第 34 行那句提到前面）→ 再算差 → 最后 `yaw_pre = yaw`

### ⚠️ `HAL_UART_Transmit` 与 FreeRTOS 的嵌套临界区
HAL 用 `__HAL_LOCK()`（普通变量，非原子）。`OnUARTITHandler` 在**中断上下文**里调 `HAL_UART_IRQHandler(&huart)`，若此时某任务正阻塞在 `HAL_UART_Transmit` 内 → 双方都看到 `HAL_BUSY` → 任务异常退出 / 中断丢失事件。⇒ **别在任务上下文用 `UARTTransmit`**，要么改 `DMATransmit`（纯寄存器操作，非阻塞），要么用 DMATx + TC 中断 + 信号量

---

## 五、进度（2026-09-27 16:10，用户报"主线基本写完"）
✅ 底盘（逆解/限幅/RESET）· 云台三轴（两 yaw 位置 + Pitch 位置，限幅已实测）· POS 串级 + latch/ClampAngle · RC 9 组合 + 7 mode · 达妙分总线 · `SHOOTER::Update()` 六段 · `has_feedback` 兜底 · 单发/连发同号 · `lead` 双边钳制 · **`POS_DEADBAND=3.0f` 位置死区**（`motor.cpp:128-132`）
🔧 **当前主线（用户定序）**：上位机链路 —— ① 修 `xuc.cpp` 三处（字节序 / 字段名 / `yaw_spd`）② `RC::OnPC()`（`xuc.Decode()` + `xuc.Encode()` 调用点 + 自瞄控制律）③ 再写 `Keep_Pantile` / `Keep_Direction`
⬜ **暂不需要**：judgement · supercap · ACE 分支（用户明确"不需要用"）
✅ `now_bullet_speed` 已被 `rpm_avg` 替代（剔除死变量）；`rpm_avg` 仍是 `Update()` 局部量
💡 **`Keep_Direction` 用达妙 yaw**（用户 2026-09-27 定）：① 达妙 yaw **无机械限幅**、② IMU 装在**它控制的那个 yaw 轴上** → 该轴就是"底盘相对云台"的天然参考，不需要底盘 IMU（`imu_chassis` 从未 Init）
⚠️ 但达妙 yaw 是 **SPEED 模式**（`setSpeed` = 角速度），不反馈位置 → 要么改成 P_S，要么用 `imu_pantile.GetAngleYaw()` 积分/直接取值当角度源

**⚠️ 标定值是自洽性检查，不是需要保留的结论**：`dm_pitch_min/max`、`T_ON/T_OFF`、`POS_DEADBAND`、`bullet_step` 都会随机构/装配变。真正不变的是**推导链**（编码换算、阈值选择准则、迟滞必要性），写进记忆的是链不是数。

## 六、活跃陷阱
- ⚠️ **越界别名**：`shooter_motor[2]`（`SHOOTER_MOTOR_NUM=2`）别名 = `supply_motor[0]`（拨盘）→ 弹速计算混入拨盘；同类：任何 `[i]` 超出容量宏都是静默别名
- ⚠️ **`pitch0` latch 竞态**：`PANTILE::Update()` 的 guard 看 ISR 原始 `jointidata[0][0]`，而 `DMmotor[0].pos` 由 `MotorUpdateTask` 的 `State_Decode` 写 → 存在"guard 先成立、`pos` 仍是初值 0"的窗口 → `pitch0=0` → 朝电机零位冲。且 `pitch_ready` **永不复位** → 一旦 latch 错就永久错。修法：guard 改用"已解码"标志
- ⚠️ **`setSpeed=0` 是天然保护**（`PANTILE::Update()` 里 `setSpeed=1.5f` 写在 guard **内部**）→ guard 不成立时 POS_VEL 限速 0，Pitch 只会不动。**不要把它挪到 guard 外**
- ⚠️ 遥控断连 `Decode()` 提前 return → `ch/s[]`/`trig_raw` 保持旧值 → 车按最后指令跑 + 扳机卡在按下**无限连发**
- ⚠️ `ArmTask`(prio 1) 与 `CanTransimtTask`(prio 2) 共用 `hcan.pTxMsg`（`CAN::Transmit` 非线程安全）→ 使能帧可能被打断；代码缺 `0xFB` 清错帧
- ⚠️ 拨盘 `current` 增量式：误差归零后 `current` 冻结在上次值
- ⚠️ 拨盘发抖（2026-09-27）：外环纯 P **无死区** → stick-slip 极限环；已在 `motor.cpp` 加 `POS_DEADBAND = 3.0f`（转子度）。⚠️ 该常量写在 POS 分支内，**M6020 yaw 共用**（直驱 → 输出 3° 死区）
- ⚠️ 达妙 Pitch：`setPos` 只在 guard 成立时更新，否则残留上一帧值 → guard 用 `(jointidata&0x0F)==ID || pitch_ready` 兜底；`para.dm_pitch_min/max` 若框不住 `pos` 会瞬间被拽到边界。**单周期 guard 只能限制不在 guard 内的修改 → 目标值必须靠 `PANTILE::Update()` 自己钳制**
- 达妙 Yaw 仅 `SPINNING`/`ROTATION` 有输入；`ROTATION` 的 `speedz=rc.ch[2]` **未换算**（唯一量纲不一致处）；`AUTOAIM` 不调 `Control_Pantile`
- `can.cpp` 达妙接收分支**不分总线**；`GetPosition()` 恒返 0（读 `.pos`）；`Motor_Start/Stop/ZeroPosition` 无定义；`pid[2]` vs `speed2=2` 越界（零调用）；`main()` 的 `HAL_Init()` 顺序反；`imu.cpp` `Check()` 有分支无 return
- `ControlTask` 用 `vTaskDelay(5)` 非 `vTaskDelayUntil` → `hold_ms` 计时偏长

## 七、协作约定
UTF-8 + LF；不提交构建产物与 `*.vgdbsettings`；提交前缀 `feat:/fix:/refactor:/docs:/chore:`，一次提交只做一件事。**先走主线，忽略 task 顺序类低效细节。**
