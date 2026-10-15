# MEMORY.md — Sentinel_robot

> RCS 哨兵电控固件。STM32F405RGT6 + FreeRTOS + HAL。VS+VisualGDB / CMake+Ninja+OpenOCD。
> **硬约束：AI 不写代码，只做代码指导，所有代码由用户亲手敲。**
> 比赛形态：**全自动、由上位机控制**，固定轨迹走到固定点，打远处**小幅旋转移动**的靶。
> 结构规则：**收（视觉定义）用偏移硬解析；发（我方定义）用 packed 结构体** —— 不对称是设计不是遗漏。

## 一、架构与节拍（tick 1kHz）
四层：`taskslist`(调度) / `control`(控制) / `motor·HTmotor·imu·RC·judgement`(模型) / `can·usart·tim`(驱动)

| 任务 | 周期 | 内容 |
|---|---|---|
| DecodeTask | 5 ms | `rc.Decode`+`imu.Decode`×2+`xuc.Decode`；`Encode()` 用私有 `tx_div` 分频到 **50 Hz**（AUTO 判） |
| MotorUpdateTask | 2 ms | can1/can2 `Ontimer` + 达妙解码 |
| CanTransimtTask | 1 ms | 私有 `can_div` 的 `%3`（0=达妙/1=0x1FF/2=0x200） |
| ControlTask | 5 ms | `chassis→pantile→shooter→rc.Update` |
| ArmTask | 100 ms | `DMmotorinit()`×2 + `power.Send()` |

优先级 Decode(3) > Control/Motor/CanTx(2) > Arm(1)。
⚠️ 全部用 `vTaskDelay` 非 `vTaskDelayUntil` ⇒ 实际周期 = 名义 + 执行时间。
🔴 **分频计数器一律用任务私有 `static`，绝不用共享全局 tick**（见七节）。

## 二、装配（`STM32F405.cpp:32-59`）
- 云台三轴（2026-10-06 用户权威订正）：**大 yaw = 达妙 `DMmotor[1]`（360°，SPEED）** + **小 yaw = M6020（ID5，160°，POS）** + **pitch = 达妙 `DMmotor[0]`（P_S）**。`Control_Pantile` 参数序 = **①小yaw(M6020) ②pitch(达妙) ③大yaw(达妙)**，`pantile_motor[PITCH]` 恒 nullptr。**小 yaw + pitch = 自瞄轴**
- `can2_motor`：底盘 M3508×4（ID1-4，SPD/chassis，`PID(1.5,0.1,0)`）
- `can1_motor`：摩擦轮 M3508×2（[0][1]=ID2/3，SPD/shooter，`PID(10,0,1.5)`）+ 拨盘 M2006（[2]=ID7，**POS**/supply，速度环 `PID(10,0.03,0.5,0)` + 位置环 `PID(1.0,0.01,0.02,0)`，`use_sum_angle=true`，`pos_deadband=3.0`）+ M6020（[3]=ID5，POS/pantile，`PID(200,0.08,0)`+`PID(5,0,0.5,0.5)`，`maxspeed=80`/`maxcurrent=25000`/`pos_deadband=0.05`）
- ⚠️ **M2006 反馈转速是「转子」口径**（C610 全部反馈减速前）：官方空载 500 rpm 是**输出轴**，×36 ⇒ 转子可达 18000 rpm。`bullet_step=42130` counts = 5.143 转子圈 ⇒ **每 1 发/秒 需 308.6 转子 rpm** ⇒ `maxspeed` 必须按转子口径给（3000 ≈ 9.7 发/s 能力）
- `DMmotor[0]=(0x09,P_S,Pitch)` 挂 can1；`[1]=(0x06,SPEED,Yaw)` 挂 can2；总线表 `taskslist.cpp:13 dm_bus[2]`
- ⚠️ **容量宏**：`CHASSIS=4 / PANTILE=2 / SHOOTER=2 / SUPPLY=1`。越界**静默别名到下一个成员**（`shooter_motor[2]` == `supply_motor[0]`）
- 全向轮 45° X，左前起顺时针 1→2→3→4 = `chassis_motor[i]` = 拨码 i+1。逆解 `w1=-vx-vy+Kωz`、`w2=+vx-vy+Kωz`、`w3=+vx+vy+Kωz`、`w4=-vx+vy+Kωz`
- 比赛形态推论：**达妙 yaw 整场不用、要锁住**；M6020 `yaw_span=1896`（±83°）够用；不需要全周/底盘协同/`Keep_Pantile`

## 三、必记机制
- `Motor::Ontimer(idata, odata)`：解析反馈+回填发送缓冲。`can.data[12][8]` 按 `StdId-0x201`；`temp_data[16]` 字节 0-7=0x200 帧、8-15=0x1FF 帧。PID→电流在内部（POS/SPD/ACE）
- 达妙独立于 `Motor`：控制层只写 `setPos`(弧度)/`setSpeed`；`jointidata`/`jointpdata` 是 **CAN 实例成员**（can1/can2 各一份）
- 达妙协议：**POS_VEL 模式第二个 float = 限速（不是角速度！）**；**问询式反馈**（收到自己的控制帧才回）；`Data[0]=status[7:4]|id[3:0]`，status：0=Disabled / 1=Enabled / 8~E=过压·欠压·过流·MOS过温·线圈过温·通信丢失·过载；使能/失能/清错/存零 = 仲裁 ID `motor_id` + `FF×7+FC/FD/FB/FE`
- 零缓冲陷阱：`jointidata` 全 0 → `pos=-4π`。判"真帧"用 `(jointidata[i][0]&0x0F)==(DMmotor[i].ID&0x0F)`
- 单位：大疆=编码值 0~8191；达妙=弧度。**`maxspeed` 只在 POS 分支生效，SPD 无上界**
- 拨码 ↔ 反馈 ID：索引 = `IDn - ID1`；ISR 落点 `data[StdId-0x201]`。`C620/C610` 反馈 `0x200+拨码` ⇒ 物理拨码 = n；`GM6020` 反馈 `0x204+拨码` ⇒ 物理拨码 = n-4（`ID5` 拨 **1**）
- ⚠️ **`has_feedback` 唯一入口 = `motor.cpp:78`**，判据 `temperature != 0`。**本机 C610 温度字节恒 0** → 已加兜底 `|| idata[slot][0] || idata[slot][1]`；连带过温保护对拨盘失效
- ⚠️ **POS 分支多圈必须走 `use_sum_angle`**（`getdeltaa`+`(int16_t)` 对多圈失效）；latch 时补 `angle[pre]=angle[now]`
- `Position(err, limit)` 第二参钳的是**积分槽**，`Ti=0` 时无效
- ✅ 达妙接收"有意汇流"：`can.cpp` 不判 `hcan`，两路达妙帧都写 `can2.jointidata[i]`（i 由 `Data[0]&0x0F` 匹配）⇒ `State_Decode` 对两总线通用。⚠️ 两条总线同 ID 会争抢同槽

## 四、发射机构（POS 双环）
- ⚠️ **`mechanicalToDegree(a) = a*360/8192` ⇒ 位置环误差单位是「度」**。`bullet_step=42130` ⇒ **一发 = 1851.6°**，位置环 Kp=1.0（rpm/度）⇒ `speed_ref` 起步就 1851 rpm，**必然顶穿 maxspeed** ⇒ 每次拨弹都是"饱和→退饱和→指数逼近"
- ⚠️ **外环时间常数 τ ≈ 1/(6·Kp)**（rpm→°/s 要 ×6）：Kp=1.0 ⇒ τ≈167 ms，**比连发节拍 125 ms 还长** ⇒ 每发收不了尾，误差累积到 `bullet_lead_max`(2 发)
- ⚠️ **`bullet_lead_max = 2×bullet_step`** ⇒ 追不上时 `speed_ref` 恒为 maxspeed ⇒ **指令平直、反馈抖动**（2026-10-11 波形即此形态）
- ⚠️ 内环 `Td=0.5` 且 `alpha=0`：对**整数量化** rpm 反馈做二阶差分，噪声放大 ≈0.54/计数 ⇒ 电流/转速抖（与 M6020 那次同类），建议 Td→0~0.05
- 外环 `Ti=0.01` 基本无效（积分槽钳在 ±1000，贡献 ≤10 rpm）；`Kp=0.30` 纯 P ⇒ 约 3 s 一发（要 125 ms 需 Kp≈3~6）
- `para.bullet_step=42130.29`（一发 = `8192×36/N`，**N=7**）/ `bullet_lead_max=84260.58`；单发与连发**统一 `-bullet_step`**
- `SHOOTER::Update()` 六段：摩擦轮 / 弹速 / 迟滞 / 长短拨 / POS 累加 / 超前钳制
- 扳机 = ch[1]（**仅 FIRE 模式**）；RC 只写 `trig_raw`，阈值+迟滞在 Update 内；`trig_pre` 单点写入
- 拨弹判据 = `openRub && 弹速达标 && 扳机`（`supply_bullet` 是死变量）

## 五、上位机链路（V2 精简版，2026-10-15 落地）
硬件：`xuc` 挂 **UART4 @ 921600**（PC10 TX/PC11 RX）；`imu_pantile`→UART2（枪口，id=1）、`imu_chassis`→UART5（身体，id=2），均 115200/CH010。

### 🔴 CH010 帧结构（官方手册确认，2026-10-15 实证）
```
[0] 0x5A | [1] 0xA5 | [2..3] LEN u16小端 | [4..5] CRC u16小端 | [6..] 数据域
```
**LEN = 数据域长度**（不含帧头/帧类型/LEN/CRC），**帧总长 = len + 6**（实测 len=76 ⇒ 82）。
**算法 = CRC-16/CCITT-FALSE**（poly `0x1021`、init `0x0000`、MSB-first、不反射、无 final xor）。**与 V2 PC 帧的 MCRF4XX 是两个不同算法，别混。**
🔴 **CRC 是【两段拼接】：`buf[0..3]`（帧头+长度）+ `buf[6..6+len]`（数据域），中间的 CRC 字段 `[4..5]` 不参与计算。**
⚠️ 「除 CRC 本身外」= CRC 字段不算进去，**不是**"算进去凑自同步"（两种理解结果完全不同）。⇒ `Check()` 用 `uint16_t* acc` 支持累加器跨调用延续。
⚠️ **查手册要排在「让用户截图 / 写穷举诊断」之前**。中文全文在 `kancloud.cn/hipnuc/test/3006876`（含帧结构表、CRC 代码、0x91 字段表、样例帧），可 `curl` 抓 markdown；`hipnuc.com` 那份英文 PDF 只有硬件规格、无协议。

**0x91 字段表**（偏移相对 `p = rxData+6`）：`0`=tag｜`1`=ID(u32)｜`4`=气压 Pa｜**`8`=时间戳 ms（A 实验基准）**｜`12`=acc XYZ (1G)｜`24`=gyro XYZ (deg/s)｜`36`=mag XYZ (uT)｜`48`=Roll/Pitch/Yaw (deg)｜`60`=四元数 WXYZ。固定 76 字节。
🔴 **早期代码把 gyro 和角度的 roll/pitch 都读反了**，此前记的"对调是为补偿 90° 安装"是**错误结论**（实为无依据的历史错位）。已按手册恢复 `roll@24 / pitch@28 / yaw@32`、`roll@48 / pitch@52 / yaw@56`。**B 实验起点已变，结论要推翻重来。**
⚠️ 磁强度(`p+36`) 在 9 轴模式非零 ⇒ **可作 B 实验判 6/9 轴的现成抓手**。
⚠️ 数据域是「若干子数据包」的拼接，**标签决定类型与长度** ⇒ 硬编码 offset 只在 `AT+EOUT` 配置不变时成立。

**收**：UART DMA → IDLE 中断 → 队列（item=100 B，深度 1，`xQueueOverwriteFromISR`）→ DecodeTask。ISR 拷**整个 100 B 快照**且不清零 ⇒ 短帧尾部带残字节。
**发**：DecodeTask 5 ms + `static tx_div` 的 `%4` ⇒ **50 Hz**；三帧 `0x01`(46)+`0x02`(34)+`0x81`(26)=**106 B/次**，921600 8N1 ⇒ 线上 **1.15 ms**，占空比 **5.75%**。非 AUTO 时 `tx_div=0` ⇒ 切进 AUTO 立刻发一帧。

帧头 12 B：`0-1`magic `A6 6A` / `2`version=2 / `3`type / `4-5`payload_len / `6-7`seq / `8-11`boot_id`，总长 `= 14+N`，**CRC 落末两字节**。

| type | 消息 | 负载 | 方向 | 状态 |
|---|---|---:|---|---|
| `0x01` | GIMBAL_IMU | 32 | 上行 | ✅ 姿态+gyro+time+id+flags |
| `0x02` | CHASSIS_IMU | 20 | 上行 | ✅ 姿态（无 gyro）+time+id+flags |
| `0x10` | AIM_COMMAND | 24 | 下行 | ✅ 只解析不执行（`XUC_ALLOW_CONTROL=0`）|
| `0x11` | NAV_COMMAND | 24 | 下行 | ✅ **已放行**（`XUC_ALLOW_NAV=1`）|
| `0x81` | LINK_STATUS | 12 | 上行 | ✅ |
| 其余 | — | — | — | 未知类型静默跳过 |

**旧 40 B/64 B 定长帧、`TxPacket` packed、`XUC_FRAME_LEN` 全部作废** —— 变长帧 + type 分流后，加字段是"注册新 type"而非"两端同时改帧长"。

- CRC：V2 PC 帧用全局 `CRC.h` 的 **MCRF4XX**（init `0xFFFF`、反射表、低字节在前、覆盖 `len-2`），实测 `MCRF4XX("123456789")=0x6F91` / 线上 `91 6F` ⇒ **CRC 层零改动**。
  🔴 **IMU 的 `0x5A 0xA5` 帧用的是另一个算法**：CRC-16/CCITT-FALSE（init `0x0000`、poly `0x1021`、MSB-first、无 final xor、不反射），见 `imu.cpp::Check`。**两个别混用**。
- **单位已全改 rad**（下行直接给 rad，删 `*π/180`；上行 IMU 是 deg 需 `*π/180`）。⚠️ 改反即全身系统性差 57.3 倍
- ⚠️ **CH010 `0x91` 包原生单位是 deg**（offset 48/52/56），acc = **G**、gyro = **deg/s**，`imu.cpp` 的 `R4()` 直读无换算
- 🔴 **`imu.cpp` roll/pitch 与官方字段表对调**（offset 48→`angle.pitch`、52→`angle.roll`），gyro 也 X↔Y 交换 ⇒ 疑似补偿 90° 安装。**静态实验支持保留但未动态验证，不算 `AXES_VERIFIED`**
- ⚠️ **互换只在 MCU 一处**（`imu.cpp`）⇒ 上位机**不得再换一次**（V2 §5.1）
- `boot_id`：本工程 HAL **无 UID getter** ⇒ `HAL_GetTick()^this^常数` + xorshift32（同次上电恒定、跨上电大概率不同、不持久化）
- 参数：`para.K_NAV`（m/s→rpm，**须实测**，现 159 是估算）、`para.DV_MAX`、`para.NAV_MAX`

### 自瞄语义：绝对目标 + IMU 闭环（不再是增量累加）
`xuc.cpp::HandleAim()` 收到新命令时算一次 `aim_yaw_target = WrapPi(ref.yaw + yaw_error)`；`RC::ApplyAim()` 每周期只做 `err = target − imu_pantile.GetAngle*()` 的 IMU 闭环小步逼近（外环增益 0.25）。**误差归零即自然停**，重复包无害 ⇒ 从根上消掉「同一份 `pitch_diff` 被 200 次/秒 重复累加、目标顶到限位」的 bug。
`RC::OnPC()` 瘦身为「模式守卫 + `ApplyNav()` + `ApplyAim()`」；`RC.h` 删了 `pc_timeout`/`last_rx_count`（改用上位机 `valid_for_ms` 租约）。导航保留**限目标→限增量→下发**三级。

### TIM2 1 MHz µs 时基（V2 唯一时间源）
`timer.UsInit(1000000)` 在 `SystemClockConfig()` 之后调用。⚠️ `UsInit()` **不是 static**（只有 `NowUs`/`UsElapsed`/`UsDiff` 是）⇒ 必须 `timer.UsInit(...)`。
⚠️ **绕开 HAL 寄存器直配**（`HAL_TIM_Base_Init` 经 MspInit 会开 NVIC，1 MHz 下每秒百万次中断 ⇒ 拖死系统）；另已补 `DIER=0` 防残留使能。PSC=83 精确 1 µs；ARR=0xFFFFFFFF 回绕 **71.58 min** ⇒ **只能用无符号差值，不能比大小**。
⚠️ `UsInit` **拒绝非整除频率**（`APB1_TIMER_HZ % tick_hz != 0` 时返回 false），此时 `rx_time_us` 恒 0，上层可据此判断时基未就绪。

🔴 **协议内所有时间量必须同用 `TIM::NowUs()`**。曾出现过两个来源：`sample_time_us` 用 ISR 打的真 µs，而 `age_us` 判样本新鲜度用 `xTaskGetTickCount()*1000` —— **1 ms 时基减 1 µs 时基，差出来的数没有物理意义**（那个 `NowUs()` 函数名极具误导性：tick 本身 1 ms，乘 1000 只放大数值、不增精度）。
⚠️ **`t_us` 不做"frame_time_us 为 0 就退化"兜底**：ISR 到达时刻 vs 任务组帧时刻（Encode 20 ms 周期）混进同一字段，回归时会变成一个**假截距**，把 A 实验的残差 RMS 撑爆。**时基没就绪就如实报 0**。
⚠️ `xuc.cpp` 里另有 4 处 `xTaskGetTickCount()`（半帧超时 / 租期）是**毫秒级判据**，语义本来就对，不要动。

## 六、当前状态（2026-10-15）
### ✅ 已修（近期）
`pos_deadband` 前移｜M6020 `maxspeed` 80/`maxcurrent` 25000/`pos_deadband` 0.05｜M2006 `maxspeed` 3000｜达妙 yaw `setSpeed` 单一写者｜删 `RxPacket`/`u8_to_float`/类内 CRC｜**失联保护**（遥控 100 ms + 视觉租约）｜`Keep_Direction`（IMU 差值法）｜IMU 角速度/加速度/时间戳/`IsValid` getter｜`AUTOAIM→AUTO`｜**协议换 V2 精简版**｜**自瞄改绝对目标+闭环**｜**`Encode()` 10→50 Hz**（挪进 DecodeTask + 私有分频）｜**`sample_time_us` 改读 ISR 真到达时刻**（`GetRxTimeUs()`，新增 `frame_len` 只喂真实长度）｜**usart 补时间戳/帧长/丢帧计数/DMATransmit 在途守卫**｜**CanTransimt 改私有分频**｜**imu CRC 五处已修 + 三分支重构**

### 🔴 CRC 修复的教训（探针实测，非推理）
括号 UB 只是最轻的一个，实际查出五个更严重的问题：
1. **`crc` 原是成员变量，而 `crc = 0` 写在 `Check()`【之后】** ⇒ 每帧从上一帧残值继续累加。实测：第 1 帧过、**第 2~5 帧全丢**。修法：降为**函数内局部/入参**
2. **`Check()` 的 `len` 形参是 `uint8_t`，调用处算出的是 u16** ⇒ 帧长 >255 时**静默截断**，其余数据根本没进 CRC 却当"通过"（len=284 ⇒ 只校验 28 字节）。修法：先算 `uint32_t len_raw` 再钳位
3. **`Decode()` 缺 `if (pd_Rx != pdTRUE) return;`** ⇒ 队列空时 `rxData` 保持旧内容，**拿旧帧重复解析**、反复刷新 `last_rx_tick` ⇒ 样本有效性判据失真
4. `Check()` 缺 `return`（`-Wreturn-type`）
5. 🔴 **CRC 覆盖范围漏了「帧头+长度」4 字节**（实机 `rx_ok=0 / crc_fail 涨 / len_bad=0` 定位，见第五节 CH010 帧结构）⇒ 手册明确是**两段拼接**、CRC 字段跳过不参与计算 ⇒ `Check()` 需 `uint16_t* acc` 延续累加
⚠️ **「CRC 变量是成员」是隐蔽的跨帧污染源** —— 只要清零位置写错就退化成静默丢帧，现象完全不指向根因。
⚠️ **`sizeof(指针别名)` 只有 8 字节** —— 长度上界必须写 `UART_MAX_LEN - 6` 显式常量，不能写 `sizeof(rxData)` 作用在指针上（探针阶段真踩过）。

### ❌ 未做 / 待办
- 🔴 **两个必须上架子量的 blocker**（否则只能"只读不控"）：① **`sample_time_us` 误差界 ≤2 ms**（`system_time` 仅 1 ms 粒度、UART 传输 ≈6.16 ms、IDLE→DecodeTask 0~5 ms 抖动；**线性回归求截距 + 残差 RMS**，理论下限 0.289 ms）；② **IMU 单轴/参考系标定**（拿 `AXES_VERIFIED`；6/9 轴决定 `heading_source`/`reference_frame`，**开电机看 yaw 是否跳基本能一票定死**）。**详细流程见 `2026-10-14.md`。做完改 `XUC_ALLOW_CONTROL=1` 即开许可。**
- ⚠️ `SAMPLE_TIME_VALID` **仍固定 0**（诚实标注）：现在是"到达时刻"不是"采样时刻"，对应 V2 `time_source=2`
- ⬜ 需视觉同步切到 V2 帧格式 + rad 单位（旧协议不能自动混猜）
- ⬜ 待视觉确认：ENU/NWU、6/9 轴、旋转顺序（312/321）、`valid_for_ms`、轴互换分工书面确认
- ⬜ 接收队列仍是「深度 1 + 覆盖写」⇒ 静默丢帧（**现在可观测**：`rx_drop_cnt`），去覆盖化待做
- ⬜ `vTaskDelay` → `vTaskDelayUntil`（DecodeTask 影响上行时序精度，ControlTask 影响 PID 隐含 dt）
- ⬜ `K_NAV=159` 实测标定（写死 `speedx=1000` 跑 1 s 量位移反推）；`Keep_Direction` 的 `left` 符号待架空验证
- 🔴 `judgement` / `supercap` / `power.Init`（被注释）**整块空白**（用户确认比赛不需要）⇒ V2 已不依赖 `detect_color`
- 达妙改 P_S（用户决定**先保持 SPEED 现状**）：P_S 丢帧 fail-safe（停原地），SPEED 丢帧会按残值飞车
- ⬜ `UART::OnUARTITHandler` 的元信息要在**取到帧的同一 if 分支内只读一次**，否则会得到「A 帧字节 + B 帧时间戳」的错配

## 七、活跃陷阱
- 🔴 **编译验证用真实 rsp，不要只编探针**：工程里有 `VisualGDB/Debug/*.gcc.rsp`（含全部 `-I`/`-D`/`-mcpu`/CMSIS 路径），直接 `arm-none-eabi-g++.exe @VisualGDB/Debug/imu.gcc.rsp` 就能编真 TU。⚠️ rsp 已带 `-MD -MP -MF` 和 `-o`，**追加 `-o` 会报 "too many filenames"**，原样跑。`for f in VisualGDB/Debug/*.gcc.rsp; do arm-none-eabi-g++.exe @$f; done` 即全 TU 全编。
  ⇒ **探针只能验逻辑，验不到 include/类型/真实宏**。曾因只编探针而漏掉真实 `imu.cpp` 的 const 编译错误。
- 🔴 **`x * 1000` 不等于把 x 变成高精度**：时间源的真实分辨率 = 底层计数器自增周期。`xTaskGetTickCount()` 是 1 ms，乘 1000 只放大数值。要 µs 必须有 1 MHz 硬件计数器。
- 🔴 **分频计数器绝不用共享全局 tick**：原 `switch ((timer.counter++) % 3)`，`timer.counter` 在**中断（TIM3@1kHz）与任务（1kHz）各 +1 一次** ⇒ 余数步进 `(1000·Δt+1) mod 3`，只有两时钟相位恰好对齐才等于"每 3ms 各发一帧"，相位一漂就变「某帧连发两次、另一帧间隔 6ms」⇒ 底盘电流环周期抖动。已改任务私有 `static uint8_t can_div`，`timer.counter` 成员已删
- ⚠️ **TIM3 现在挂着一个 1 kHz 空中断**（`timer.Init(BASE,TIM3,1000).BaseInit()`），回调已清空。若确认无消费者可整条去掉（改 `STM32F405.cpp`）
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
- ⚠️ **别在任务上下文用 `UARTTransmit`（HAL 阻塞）**：`__HAL_LOCK` 与中断里的 `HAL_UART_IRQHandler` 会双 BUSY ⇒ 用 `DMATransmit`
- ⚠️ `DMATransmit` 判在途**刻意不用 DMA TC 标志位**：`OnDMAITHandler` 只在 `DMA1_Stream4_IRQHandler`（仅 uart4 TX）里被调，靠标志位会让其余 UART 永久 busy。读 `NDTR` 始终准确
- ⚠️ `UartQueueHandler` 是 `UART` **成员**（`usart.h:50`）⇒ 每串口独立队列、无跨口污染 ✓；但 6 个 UART 在 main 前共占 ~700 B 堆
- 死代码：`Motor::getStatus()`/`StatusIdentifier()`｜`mark_yaw`/`sensitivity`/`pitch0`/`spinning`/`need_curcircle`/`adjspeed`｜`CONTROL::PC`/`LOCK` 死枚举｜`GetPosition()` 恒返 0

## 八、协作约定
🔴 **改文件必须先经用户同意**：除已明确授权的文件外，任何 `*.cpp`/`*.h` 改动前都要先问，列出「改哪个、为什么、怎么改」，得到确认再动手。
✅ **每次完成任务后用一句话写进当日日志**（`YYYY-MM-DD.md` 末尾追加，`## <小标题>` + 一句话结论 + 展开），并在回复里给同样的分点总结。
📓 **对话笔记同步到 Obsidian**：`电控笔记/AI/`，格式 = YAML frontmatter（`tags`）+ `# MM.DD <简洁标题>` + **正文前必须有 `**简介**：` 概述** + 分节正文。命名沿用 `M.D <标题>.md`。**代码改动/调参/实验结论都要落进去。**

UTF-8 + LF；不提交构建产物与 `*.vgdbsettings`；提交前缀 `feat:/fix:/refactor:/docs:/chore:`，一次提交只做一件事。先走主线，忽略 task 顺序类低效细节。
⚠️ 标定值（`dm_pitch_min/max`、`T_ON/T_OFF`、`POS_DEADBAND`、`bullet_step`、`K_NAV`）会随机构变 —— **要留的是推导链，不是数**。
