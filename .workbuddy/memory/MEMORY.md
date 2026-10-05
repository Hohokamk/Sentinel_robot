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
- **2026-10-05 比赛形态定案（用户）**：按**固定轨迹**走到特定位置 → 打一块**小幅移动的板子**。⇒ 自瞄只需小幅范围，**达妙 yaw 整场不用，要锁住**。M6020 的 ±1896 计数（±83°）**够用**，先前担心的"166° 做不了全周"问题**消失**；也不需要底盘协同 / `Keep_Direction` / IMU 参考系。yaw 自由度收敛为**单级 M6020**

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
- `judgement`/`supercap` 类已实现但**零调用点**（⇒ `xuc.Encode()` 里的 `own_color` 读 `judgement.data...robot_id` 恒 0 → **恒 RED**）
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

### 四之三、导航下行 + Encode 数据源（2026-10-05 22:00）
- **弃用 `RxPacket` 是刻意的**：它只有 `header+checksum`（sizeof=4），从未描述 32B 帧。**收（视觉定义）用偏移硬解析，发（我方定义）用 packed 结构体** —— 不对称是设计不是遗漏
- ⚠️ 队列 item=100B，ISR 拷**整个缓冲快照**且不清零 → 短帧尾部带残字节 ⇒ **CRC 必须在读偏移之前跑**
- **底盘下行**：入口 = `ctrl.chassis.speedx/speedy/speedz`（**单位是 rpm 不是 m/s**，要实测标定 K）。32B 帧已排满、`[22..24]/[29]` 仅 4B ⇒ **必须扩帧（32→44）**；另开一条帧会因队列深度 1 + `xQueueOverwriteFromISR` 互相冲掉。写在 **`RC::OnPC()`**（后跑，天然覆盖 `RC_Control()` 的摇杆值）；⚠️ 掉线早退只清 shooter，**底盘会留残值继续跑**
- **`Encode()` 只搬运、不在里面算**：`aim_x/y/z` 是成员变量、**全工程无人写 = 恒 0**；`detect_color` 靠 judgement（零调用）**恒 RED**；`reset_tracker` 硬编码 0；⚠️ `Encode()` 只在 `mode==AUTOAIM` 时跑（`taskslist.cpp:163`）⇒ **导航模式上报会被禁**
- **帧长实测**（用本机 Arm GCC 10.3.1 编 `TxPacket` 取真实 `sizeof`/`offsetof`）：**上行 28B** = `0`header / `1` flags(3位域) / `2-5` roll / `6-9` pitch / `10-13` yaw / `14-17` aim_x / `18-21` aim_y / `22-25` aim_z / `26-27` CRC；**下行 32B**（`XUC_FRAME_LEN`）= `0`hdr / `1-4`pitch / `5-8`yaw / `9-12`yaw_diff / `13-16`pitch_diff / `17-20`distance / `21`fireadvice / `22-24`保留 / `25-28`v_y / `29`保留 / `30-31`CRC。**收发不等长、header 不同（0x5A vs 0xA5）、CRC 覆盖不同（26B vs 30B）**，但算法参数一致（`CRC_INIT=0xffff` 反射表）。⚠️ `bool` 做位域类型是**实现定义** ⇒ 换编译器布局可能变，**加 `static_assert(sizeof(TxPacket)==28)` 并与视觉逐字节对联**
- 队列 item=100B（`UART_MAX_LEN`），收进来是 **100B 快照**只用前 32；收频率 = `DecodeTask` 5ms 轮询（`xQueueReceive(...,NULL)` = 非阻塞），发频率 = `ArmTask` 100ms = **10Hz**

---

## 五、进度（2026-10-05）
✅ 底盘（逆解/限幅/RESET）· 云台三轴（两 yaw 位置 + Pitch 位置）· POS 串级 + latch/ClampAngle · RC 9 组合 + 7 mode · 达妙分总线 · `SHOOTER::Update()` 六段 · `has_feedback` 兜底 · 单发/连发同号 · `lead` 双边钳制 · `POS_DEADBAND=3.0f` 位置死区 · **Pitch 限幅已实测**
✅ 上位机链路 1/2/3/4a/5/6 已落地：`xuc.Decode` 进 DecodeTask、`FR4` 替换、`rx_count`、`rc.Update()` 提前、`RC::OnPC()`（守卫+超时+pitch 直给+openRub/auto_shoot）、`SHOOTER::Update` 用 `trig_eff`、`ArmTask` 里 `xuc.Encode()`（判 AUTOAIM）；`yaw_spd` 重排 + 周期改 0.005
🔧 **当前主线（2026-10-05 用户定）**：① **锁达妙 yaw** ② 打通自瞄 yaw 通路（M6020）③ 与视觉对接协议对齐
⬜ **暂不需要**：judgement · supercap · ACE 分支 · `Keep_Pantile` / `Keep_Direction`（比赛形态用不上）· 全周自瞄 / 底盘协同
✅ `now_bullet_speed` 已被 `rpm_avg` 替代（剔除死变量）；`rpm_avg` 仍是 `Update()` 局部量

### 五之二、锁达妙 yaw 的方案（2026-10-05）
达妙 `[1]` 是 **SPEED 模式**，但**反馈帧任何模式都返回 pos/vel/torque**（`State_Decode` 无条件解析）⇒ SPEED 下 `pos` 有值。
| 方案 | 做法 | 抗扰 | 回位 | 改动 |
|---|---|---|---|---|
| A 失能 | 停发 `DMmotorinit` + 发一次 `0xFD` | 无 | 无 | 小 |
| B 速度锁 0 | `setSpeed = 0` 恒定（**现状**） | 抗速度不抗位置 | ❌ | 极小 |
| **C 软锁 + 手动微调（用户 2026-10-05 定案）** | 保持 SPEED，`setSpeed = yaw_speed_out + Kp*(pos_lock - pos)` | ✅ | ✅ | 小，零协议改动 |
选 C 的理由：不改电机内部模式（避免 P_S 重配 PID / ±4π / 零位语义），电机仍跑已调好的速度环，外面套 P 即得位置保持。⚠️ `DMmotor_Ontimer` 有 `LIMIT_MIN_MAX(setSpeed,V_MIN,V_MAX)=±10 rad/s`，Kp 别大。
⚠️ 必须"回位"而不能只"速度 0"：**弹丸反冲**会让 yaw 逐发累积偏移 → 视觉给的偏差角失配。
⚠️ `pos_lock` 取第一帧**真反馈**时锁存；**别重犯 `pitch0` 的 latch 竞态**（guard 看 ISR `jointidata` 而 `pos` 由 2ms 任务写）→ 用 `decoded` 标志兜底。`pos` 初值 0，锁错会朝电机零位猛冲 → 过载掉使能。

### 五之三、锁 yaw 的必改点（2026-10-05 21:20 定稿）
0. 🔴 **`DMmotor[1].setSpeed` 双写者**：`Control_Pantile`（`control.cpp:78-79`，含手动项）与 `PANTILE::Update`（`:143`，纯锁）都写它；`ControlTask` 里 `pantile.Update()` 后跑 ⇒ **覆盖**。**必须单一写者**：`Control_Pantile` 只产出 `yaw_speed_out`（删写 setSpeed 那行），执行只在 `PANTILE::Update`：`setSpeed = yaw_speed_out + Kp*(pos_lock-pos)`
0b. 连锁：`yaw_speed_out` 变成"有记忆的意图量" ⇒ AUTOAIM case 必须补 `yaw_speed_out = 0`（否则 ROTATION/SPINNING 残值让达妙 yaw 漂移）
1. `control.cpp:140` 需补 `yaw_speed_out +` 才是 C（见 0，写法已定）
2. ~~`control.cpp:77` `yaw_speed_out = ...` → 删~~ **错误，撤回**：`yaw_speed_out` 就是 C 的**手动微调项**，`control.h:36` 与 `control.cpp:77` 全部**保留**
3. `Control_Pantile` 第三参 `ch_dm_yaw` **保留**（有真实消费者：→ `yaw_speed_out`）；`ROTATION`/`SPINNING` 传的 `rc.ch[3]`/`rc.ch[1]` 也**保留**
4. `control.h:49` `const float yaw_lock_Kp` → **去 `const`**（已完成 ✅）
5. `RC.cpp` AUTOAIM case 的 `ctrl.pantile.yaw_speed_out = 0;` **必须补**（现状缺失 ❌）
6. C 行为：推杆稳态偏差 = `yaw_speed_out / Kp`；**松杆自动弹回 `yaw_lock_pos`**（抗反冲回位）。满杆 `π rad/s`、`Kp=3` ⇒ 偏差上限 ≈1.05 rad ≈ 60°。⚠️ 首次测试若松手**发散**而非回位 ⇒ `setSpeed` 与 `pos` 反号 ⇒ `Kp` 取负
7. **自瞄 yaw 写 M6020（`pantile_motor[YAW]->setangle`），不写达妙**：M6020 通路无写者冲突（`PANTILE::Update`:125-127 只 `ClampAngle` 钳位不覆盖）；达妙 yaw 已锁死，`OnPC` 不碰

### 五之四、视觉对接（自瞄口径建议）
**优先争取"偏差角增量"口径**：视觉给 `yaw_diff`/`pitch_diff` = 云台还需转过的角度（云台系、度）→ 电控 `setangle += k*yaw_diff`、`mark_pitch += k*pitch_diff`（k≈0.5~1.0）。
好处：不需要 IMU / 世界系 / 底盘协同 / 绝对零位；**丢目标 → 增量为 0 → 云台自然停（天然 fail-safe）**。
⚠️ 增量=纯积分：`yaw_diff` 若有系统偏置会**一路转到 ClampAngle 撞限位** → 必须配死区（0.3~0.5°）+ 限幅 + k<1。
⚠️ 要问视觉"目标 off-screen 时能否给粗略左右方向"——固定轨迹的终点朝向若保证板子在 FOV 内，这条可以省。
换算：度 → M6020 计数 ×`8192/360 = 22.756`。

**与视觉必须对齐的接口项**：帧长、CRC16 参数（初值 `0xFFFF`、覆盖 `len-2`）、每个角度的**单位**（现在 `pitch/pitch_diff` 转弧度而 `yaw/yaw_diff` 没转）、**`yaw/yaw_diff` 的参考系与正方向**、`fireadvice` 语义（建议 vs 立即）、**目标丢失时发什么**、`distance` 单位、`v_y` 用不用、发送频率、字节序、上报帧字段（`aim_x/y/z` 恒 0、`detect_color` 因 judgement 零调用恒 RED、上报仅 10Hz）。

**⚠️ 标定值是自洽性检查，不是需要保留的结论**：`dm_pitch_min/max`、`T_ON/T_OFF`、`POS_DEADBAND`、`bullet_step` 都会随机构/装配变。真正不变的是**推导链**（编码换算、阈值选择准则、迟滞必要性），写进记忆的是链不是数。

## 六、活跃陷阱
- ⚠️ **CRC16 有两份重复实现**：全局 `CRC.h/.cpp`（`GetCRC16CheckSum` / `VerifyCRC16CheckSum` 返 `uint8_t` / `AppendCRC16CheckSum`，`CRC_INIT=0xFFFF`，`wCRC_Table`）与 `xuc.h/.cpp` 类内成员版（`getCRC16CheckSum` / `verifyCRC16CheckSum` 返 `uint32_t` / `appendCRC16CheckSum`，`CRC16_INIT`，`CRC_TAB`）。两张 256 项表**逐项相同** ⇒ 纯重复（~1KB flash）。`xuc.cpp` 已 include `CRC.h` ⇒ 保留全局版、删 xuc 版。参数口径 = **CRC-16/MCRF4XX**（反射 0x1021 = 0x8408、init 0xFFFF、低字节在前、覆盖 `len-2`）= RM 官方视觉协议那套
- ⚠️ **越界别名**：`shooter_motor[2]`（`SHOOTER_MOTOR_NUM=2`）别名 = `supply_motor[0]`（拨盘）→ 弹速计算混入拨盘；同类：任何 `[i]` 超出容量宏都是静默别名
- ⚠️ **`pitch0` latch 竞态**：`PANTILE::Update()` 的 guard 看 ISR 原始 `jointidata[0][0]`，而 `DMmotor[0].pos` 由 `MotorUpdateTask` 的 `State_Decode` 写 → 存在"guard 先成立、`pos` 仍是初值 0"的窗口 → `pitch0=0` → 朝电机零位冲。且 `pitch_ready` **永不复位** → 一旦 latch 错就永久错。修法：guard 改用"已解码"标志
- ⚠️ **`setSpeed=0` 是天然保护**（`PANTILE::Update()` 里 `setSpeed=1.5f` 写在 guard **内部**）→ guard 不成立时 POS_VEL 限速 0，Pitch 只会不动。**不要把它挪到 guard 外**
- ⚠️ 遥控断连 `Decode()` 提前 return → `ch/s[]`/`trig_raw` 保持旧值 → 车按最后指令跑 + 扳机卡在按下**无限连发**
- ⚠️ `ArmTask`(prio 1) 与 `CanTransimtTask`(prio 2) 共用 `hcan.pTxMsg`（`CAN::Transmit` 非线程安全）→ 使能帧可能被打断；代码缺 `0xFB` 清错帧
- ⚠️ 拨盘 `current` 增量式：误差归零后 `current` 冻结在上次值
- ⚠️ 拨盘发抖（2026-09-27）：外环纯 P **无死区** → stick-slip 极限环；已在 `motor.cpp` 加 `POS_DEADBAND = 3.0f`（转子度）。⚠️ 该常量写在 POS 分支内，**M6020 yaw 共用**（直驱 → 输出 3° 死区）
- ⚠️ 达妙 Pitch：`setPos` 只在 guard 成立时更新，否则残留上一帧值 → guard 用 `(jointidata&0x0F)==ID || pitch_ready` 兜底；`para.dm_pitch_min/max` 若框不住 `pos` 会瞬间被拽到边界。**单周期 guard 只能限制不在 guard 内的修改 → 目标值必须靠 `PANTILE::Update()` 自己钳制**
- 🔴 **`xuc.Decode()` 零校验**：`verifyCRC16CheckSum()` 写了但**零调用**，判据只有 `m_frame[0]==0xA5` → **一帧噪声就能让云台跳到任意角度**（`mark_pitch = xuc.pitch` 是直接赋值）。现场电磁环境差，必须补。另 `if (sizeof(m_frame) < 18) return;` 是编译期常量（`uint8_t[100]`）→ 死代码
- 🔴 **双上位机通路冲突**：`rc.pc.x/y`（遥控器帧 `m_frame[6..11]` 解出）与 `xuc`（UART3）两套并存，都在写 `chassis.speedx/speedy`；`RC_Control()` 的 AUTOAIM case 用 `pc.*`，`OnPC()` 用 `xuc.*` → **实际是 pc 赢**。要定唯一主人
- 达妙 Yaw 仅 `SPINNING`/`ROTATION` 有输入；`ROTATION` 的 `speedz=rc.ch[2]` **未换算**（唯一量纲不一致处）；`AUTOAIM` 不调 `Control_Pantile`
- ✅ **达妙接收"有意汇流"（2026-10-05 更正，原记为 bug 有误）**：`can.cpp:146-151` 不判 `hcan`，两路的达妙帧**都写进 `can2.jointidata[i]`**（`i` 由 `Data[0]&0x0F` 匹配 `DMmotor[i].ID` 得到）⇒ `MotorUpdateTask` 里 `State_Decode(can2.jointidata)` 对挂 can1/can2 的电机**通用**，这是当前能跑的原因。⚠️ 隐式耦合：若两条总线出现**同 ID** 的达妙电机，二者争抢同一个 `jointidata` 槽
- `GetPosition()` 恒返 0（读 `.pos`，`angle[]` 全工程未写）；`Motor_Start/Stop/ZeroPosition` 无定义；`pid[2]` vs `speed2=2` 越界（零调用）；`main()` 的 `HAL_Init()` 顺序反；`imu.cpp` `Check()` 有分支无 return
- `ControlTask` 用 `vTaskDelay(5)` 非 `vTaskDelayUntil` → `hold_ms` 计时偏长

## 七、协作约定
UTF-8 + LF；不提交构建产物与 `*.vgdbsettings`；提交前缀 `feat:/fix:/refactor:/docs:/chore:`，一次提交只做一件事。**先走主线，忽略 task 顺序类低效细节。**
