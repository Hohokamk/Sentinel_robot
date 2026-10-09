# MEMORY.md — Sentinel_robot

> RCS 哨兵电控固件。STM32F405RGT6 + FreeRTOS + STM32F4 HAL。VS+VisualGDB / CMake+Ninja+OpenOCD。
> **硬约束：AI 不写代码，只做代码指导，所有代码由用户亲手敲。**
> 比赛形态：**全自动、由上位机控制**，固定轨迹走到固定点，打远处**小幅旋转移动**的靶。
> 结构规则：**收（视觉定义）用偏移硬解析；发（我方定义）用 packed 结构体** —— 不对称是设计不是遗漏。

## 一、架构与节拍（tick 1kHz，1 tick = 1ms）
四层：`taskslist`(调度) / `control`(控制) / `motor·HTmotor·imu·RC·judgement`(模型) / `can·usart·tim`(驱动)
ArmTask 100ms `DMmotorinit()`×2 + `power.Send()` + `xuc.Encode()`（判 AUTOAIM）｜DecodeTask 5ms `rc.Decode`+`imu.Decode`+`xuc.Decode`｜MotorUpdateTask 2ms can1/can2 `Ontimer`+达妙解码｜CanTransimtTask 1ms `%3`(0=达妙/1=0x1FF/2=0x200)｜ControlTask 5ms `chassis.Update`→`pantile.Update`→`shooter.Update`→`rc.Update`
优先级 Decode(3) > Control/Motor/CanTx(2) > Arm(1)。⚠️ ControlTask/ArmTask 用 `vTaskDelay` 非 `vTaskDelayUntil` ⇒ 实际周期≈5ms+执行/122ms（上报≈8Hz 非 10Hz）。

## 二、装配（`STM32F405.cpp:32-59`）
- 云台三轴命名（2026-10-06 用户权威订正）：**大 yaw = 达妙 `DMmotor[1]`（360°，SPEED）** + **小 yaw = M6020（ID5，160°，POS）** + **pitch = 达妙 `DMmotor[0]`（P_S）**。`Control_Pantile` 参数序 = **①小yaw(M6020) ②pitch(达妙) ③大yaw(达妙)**。`pantile_motor[PITCH]` 恒 nullptr。**小 yaw + pitch = 自瞄轴**
- `can2_motor`：底盘 M3508×4（ID1-4，SPD/chassis，`PID(1.5,0.1,0)`）
- `can1_motor`：摩擦轮 M3508×2（[0][1]=ID2/3，SPD/shooter，`PID(10,0,1.5)`）+ 拨盘 M2006（[2]=ID7，**POS**/supply，`PID(20,0,0)`+`PID(0.30,0,0)`）+ M6020（[3]=ID5，POS/pantile）
- `DMmotor[0]=(0x09,P_S,Pitch)` 挂 can1；`[1]=(0x06,SPEED,Yaw)` 挂 can2；总线表 `taskslist.cpp:13 dm_bus[2]`
- ⚠️ **容量宏**：`CHASSIS=4 / PANTILE=2 / SHOOTER=2 / SUPPLY=1`。越界**静默别名到下一个成员**（`shooter_motor[2]` == `supply_motor[0]`）
- 全向轮 45° X，左前起顺时针 1→2→3→4 = `chassis_motor[i]` = 拨码 i+1。逆解 `w1=-vx-vy+Kωz`、`w2=+vx-vy+Kωz`、`w3=+vx+vy+Kωz`、`w4=-vx+vy+Kωz`
- 比赛形态推论：**达妙 yaw 整场不用、要锁住**；M6020 的 `yaw_span=1896`（±83°）够用；不需要全周/底盘协同/`Keep_Pantile`

## 三、必记机制
- `Motor::Ontimer(idata, odata)`：解析反馈+回填发送缓冲。`can.data[12][8]` 按 `StdId-0x201`；`temp_data[16]` 字节 0-7=0x200 帧、8-15=0x1FF 帧。PID→电流在内部（POS/SPD/ACE）
- 达妙独立于 `Motor`：控制层只写 `setPos`(弧度)/`setSpeed`；`jointidata`/`jointpdata` 是 **CAN 实例成员**（can1/can2 各一份）
- 达妙协议：**POS_VEL 模式下第二个 float = 限速（不是角速度！）**；**问询式反馈**（收到自己的控制帧才回）；`Data[0]=status[7:4]|id[3:0]`，status：0=Disabled / 1=Enabled / 8~E=过压·欠压·过流·MOS过温·线圈过温·通信丢失·过载；使能/失能/清错/存零 = 仲裁 ID `motor_id` + `FF×7+FC/FD/FB/FE`
- 零缓冲陷阱：`jointidata` 全 0 → `pos=-4π`。判"真帧"用 `(jointidata[i][0]&0x0F)==(DMmotor[i].ID&0x0F)`
- 单位：大疆=编码值 0~8191；达妙=弧度。**`maxspeed` 只在 POS 分支生效，SPD 无上界**
- 拨码 ↔ 反馈 ID：索引 = `IDn - ID1`；ISR 落点 `data[StdId-0x201]`
  - `C620`/`C610`：反馈 `0x200+拨码` → **物理拨码 = n**
  - `GM6020`：反馈 `0x204+拨码` → **物理拨码 = n-4**（`ID5` 拨 **1**）
- ⚠️ **`has_feedback` 唯一入口 = `motor.cpp:78`**，判据 `temperature != 0`。**本机 C610 温度字节恒 0** → 已加兜底 `|| idata[slot][0] || idata[slot][1]`；连带过温保护对拨盘失效
- ⚠️ **POS 分支多圈必须走 `use_sum_angle`**（`getdeltaa`+`(int16_t)` 对多圈失效）；latch 时补 `angle[pre]=angle[now]`
- `Position(err, limit)` 第二参钳的是**积分槽**，`Ti=0` 时无效
- ✅ 达妙接收"有意汇流"：`can.cpp` 不判 `hcan`，两路达妙帧都写 `can2.jointidata[i]`（i 由 `Data[0]&0x0F` 匹配）⇒ `State_Decode` 对两总线通用。⚠️ 两条总线同 ID 会争抢同槽

## 四、发射机构（POS 双环）
- `can1_motor[2]` = `Motor(M2006,POS,supply,ID7,PID(20,0,0),PID(0.30,0,0))`；`use_sum_angle = true`
- `para.bullet_step=42130.29`（一发 = `8192×36/N`，**N=7**）/ `bullet_lead_max=84260.58`；单发与连发**统一 `-bullet_step`**
- `SHOOTER::Update()` 六段：摩擦轮 / 弹速 / 迟滞 / 长短拨 / POS 累加 / 超前钳制
- 扳机 = ch[1]（**仅 FIRE 模式**）；RC 只写 `trig_raw`，阈值+迟滞在 Update 内；**`trig_pre` 单点写入**
- 拨弹判据 = `openRub && 弹速达标 && 扳机`（`supply_bullet` 是死变量）
- 外环 `Kp=0.30` 纯 P → **约 3s 一发，偏慢**（要 125ms 需 Kp≈3~6）

## 五、上位机链路（XUC + RC::OnPC）
- 硬件：`xuc` 挂 **UART4 @ 921600**（PC10 TX/PC11 RX）；`imu_pantile`→UART2（**枪口**）、`imu_chassis`→UART5（身体）
- 收：UART DMA → IDLE 中断 → **队列（item=100B，深度 1，`xQueueOverwriteFromISR`）** → DecodeTask。⚠️ ISR 拷**整个 100B 快照**且不清零 ⇒ 短帧尾部带残字节 ⇒ **CRC 必须在读偏移之前跑**
- 发：ArmTask 100ms `Encode()` → `DMATransmit` = **10Hz**。⚠️ 处理上限 **200 帧/s**（队列深 1 + 5ms 轮询），视觉发太快会丢帧/拼接混合帧
- ⚠️ **别在任务上下文用 `UARTTransmit`（HAL 阻塞）**：`__HAL_LOCK` 与中断里的 `HAL_UART_IRQHandler` 会双 BUSY ⇒ 用 `DMATransmit`
- CRC：**全局版 `CRC.h`**（`VerifyCRC16CheckSum`/`AppendCRC16CheckSum`，`CRC_INIT=0xFFFF`，反射表，**低字节在前，覆盖 `len-2`**）= CRC-16/MCRF4XX（RM 官方视觉协议同款）。类内那份已删
- 帧长（本机 Arm GCC 10.3.1 实测 sizeof/offsetof）：
  - **下行 40B**（`XUC_FRAME_LEN`）：`0`hdr 0xA5 / `1-4`pitch(度) / `5-8`yaw(度) / `9-12`yaw_diff(度) / `13-16`pitch_diff(度) / `17-20`distance / `21`fireadvice:bit0 / `22-24`保留 / `25-28`v_y / `29`保留 / `30-33`speed_x(m/s) / `34-37`speed_y(m/s) / `38-39`CRC
  - **上行 28B**：`0`hdr 0x5A / `1`flags(detect_color:1 + reset_tracker:1 + reserved:6，合占 1 字节) / `2-5`roll(rad) / `6-9`pitch(rad) / `10-13`yaw(rad) / `14-17`aim_x / `18-21`aim_y / `22-25`aim_z / `26-27`CRC
  - **不对称**：长度 40≠28、header 反向（0xA5/0x5A）、**单位不同（下行度 / 上行弧度）**、CRC 覆盖 38B vs 26B
- ⚠️ **`__attribute__((packed))` 是刚需**：丢掉它 `TxPacket` 从 **32 → 36 字节**（float 对齐 + 尾部补齐），且 `roll` 从 offset 2 挪到 **4** ⇒ 整帧错位。判据：`offsetof(TxPacket, roll)` = **2** 说明 packed 生效
- 参数：`para.K_NAV`（m/s → rpm，**须实测标定**）、`para.DV_MAX`（每周期速度增量限）、`para.NAV_MAX`（与摇杆量程 ±4000 对齐）

## 六、当前状态（2026-10-09 复查）
### ✅ 已修
`pos_deadband` 已移到 `task.Init()` 之前｜M6020 `maxspeed` 20→**80**｜达妙 yaw `setSpeed` **单一写者**（`PANTILE::Update` 合并 `yaw_speed_out + Kp*(pos_lock-pos)`）｜删 `RxPacket`/`u8_to_float`/类内 CRC｜**失联保护**（遥控 `last_rc_tick` + 100ms、视觉 `pc_timeout` 250ms）｜**导航下行**已通（`Decode` 读 `[30..37]`、`OnPC` 限目标→限增量→下发、失联清 `prespeedx/prespeedy`）｜`Keep_Direction` 已实现（IMU 差值法）

### ❌ 未做 / 待办
- 🔴 `Encode()` 仍只在 `mode==AUTOAIM` 跑 ⇒ **导航模式上报被禁**
- 🔴 `aim_x/y/z` 恒 0（全工程无人写）；`detect_color` 因 judgement 零调用**恒 RED**；`reset_tracker` 硬编码 0
- 🔴 `judgement` / `supercap` / `power.Init`（被注释）**整块空白**（用户确认比赛不需要）
- ⬜ **回传确认（方案已定，未实现）**：上行 28→**32B**，尾插 `rx_total`(2B, `pd_Rx` 判过即 ++) + `rx_count`(2B, CRC 通过才 ++)，CRC 后移到 `[30..31]`。⚠️ `++rx_total` 必须夹在 `pd_Rx` 判断之后、帧头判断之前；上位机用**无符号差值**读（uint16 在 200Hz 下约 5.5 分钟回绕）
- ⬜ `K_NAV` 实测标定（写死 `speedx=1000` 跑 1s 量位移反推）；`Keep_Direction` 的 `left` 符号待架空验证；导航段 `speedz` 未清零
- 达妙改 P_S（用户决定**先保持 SPEED 现状**）：P_S 丢帧 fail-safe（停原地），SPEED 丢帧会按残值飞车

## 七、活跃陷阱
- ⚠️ `initial_cnt` 是**全局变量**被所有 `Motor` 实例共享 ⇒ `initial_x` 只有前 2 个电机初始化（`distance` 零消费者故当前无害）
- ⚠️ **过温保护对 POS 模式电机失效**（`setspeed=0` 但 POS 分支不读它）
- ⚠️ **达妙故障后无法自恢复**：只发 0xFC 使能、从不发 0xFB 清错；`status` 存了无人读 ⇒ 一次过载即永久失能
- ⚠️ `can.cpp` 达妙 `else` 分支太宽泛（非 0x201~0x208 全收），建议加仲裁 ID 白名单
- ⚠️ 底盘逆解**逐轮钳位会畸变合速度方向**（超量程时跑偏），宜改等比例缩放
- ⚠️ PID `Delta()` 的 0.92/1.08 缩放 ⇒ **等效积分增益 = 1.08×(Ti + 0.08×Kp)**（底盘 Ti 0.1→0.238）
- ⚠️ `initial_yaw=4096` 与 `yaw_center=8096` 不匹配（首次 ClampAngle 钳到 6200，偏 92°）
- ⚠️ `pitch0` latch 竞态：guard 看 ISR 原始 `jointidata` 而 `pos` 由 2ms 任务写 ⇒ 改用 `decoded` 标志；`pitch_ready` 永不复位
- ⚠️ `setSpeed=0` 写在 guard **内部**是天然保护，**不要挪到 guard 外**
- ⚠️ 拨盘 `current` 增量式：误差归零后 `current` 冻结在上次值
- ⚠️ `ArmTask`(1) 与 `CanTransimtTask`(2) 共用 `hcan.pTxMsg`（`CAN::Transmit` 非线程安全）
- ⚠️ `RC.cpp` 的 `DB` 现由 yaw 段隐式复用 pitch 段那行（值同、脆弱）
- 死代码：`Motor::getStatus()`/`StatusIdentifier()` 零消费者｜`mark_yaw`/`sensitivity`/`pitch0`/`spinning`/`need_curcircle`/`adjspeed`｜`CONTROL::PC`/`LOCK` 死枚举｜`xuc.h` 的 `CRC_TAB[256]`（512B flash）｜`GetPosition()` 恒返 0

## 八、协作约定
UTF-8 + LF；不提交构建产物与 `*.vgdbsettings`；提交前缀 `feat:/fix:/refactor:/docs:/chore:`，一次提交只做一件事。**先走主线，忽略 task 顺序类低效细节。**
⚠️ 标定值（`dm_pitch_min/max`、`T_ON/T_OFF`、`POS_DEADBAND`、`bullet_step`、`K_NAV`）会随机构变 —— 要留的是**推导链**，不是数。
