# RCS 哨兵电控固件 · Sentinel Robot

> **STM32F405RGT6 + FreeRTOS + STM32 HAL** 的 RoboMaster 哨兵机器人电控固件。
> 一套源码双平台：Windows 用 **VS + VisualGDB**，Linux 用 **VS Code + Cortex-Debug**，全部免费开源工具链。

**本文档是代码的说明书，不是设计愿望书。** 凡是表格里的数字，都取自源码当前状态；
凡是标注 🔴 的地方，都是已经踩过、且改不掉的坑。新人上手前请先读第一节和第九节。

---

## 目录

| 节 | 内容 |
|---|---|
| 一 | [机器人与比赛形态](#一机器人与比赛形态) |
| 二 | [总体架构与分层](#二总体架构与分层) |
| 三 | [需要控制的部分](#三需要控制的部分按机构分层) |
| 四 | [专业技术](#四代码里用到的专业技术) |
| 五 | [下位机通信系统](#五下位机通信系统swdusartcan) |
| 六 | [上下位机通信（V2）](#六上下位机通信方式v2-精简版) |
| 七 | [构建 / 烧录 / 调试](#七构建--烧录--调试) |
| 八 | [已知问题与待办](#八已知问题与待办) |
| 九 | [给 AI 的规则](#九给-ai-的话) |

---

## 一、机器人与比赛形态

### 1.1 硬件配置

四轮全向轮（麦克纳姆轮）底盘 + 双轴云台 + 弹链发射机构 + 双 IMU。

| 机构 | 电机 | 型号 | 总线 | 拨码/ID | 控制模式 | 控制环 |
|---|---|---|---|---|---|---|
| **底盘** | 4× 轮 | M3508 | CAN2 | 1 / 2 / 3 / 4 | SPD | 速度环 `PID(1.5, 0.1, 0, 0)` |
| **摩擦轮** | 2× 轮 | M3508 | CAN1 | 2 / 3 | SPD | 速度环 `PID(10, 0, 1.5, 0)` |
| **拨弹盘** | 1× | M2006 | CAN1 | 7 | **POS** | 速度环 + 位置环双环 |
| **小 yaw**（自瞄轴） | 1× | M6020 | CAN1 | 5 | **POS** | 位置(编码值) + 速度 |
| **pitch**（自瞄轴） | 1× | 达妙 | CAN1 | `0x09` | **P_S**（位置+速度） | 外部位置环 + 内部速度环 |
| **大 yaw**（锁死） | 1× | 达妙 | CAN2 | `0x06` | **SPEED** | 仅速度指令 + 软件软锁 |
| **枪口 IMU** | — | CH010 | UART2 | — | 115200 | 上报姿态 + 角速度 |
| **身体 IMU** | — | CH010 | UART5 | — | 115200 | 上报姿态（自瞄/导航参考系） |
| **上位机** | — | — | UART4 | — | 921600 | V2 协议 |
| **遥控器** | — | DJI DBUS | UART1 | — | 100000 | 18 字节定长 |

**为什么云台是这样分配的**（2026-10-06 用户权威订正，代码 `control.cpp` 按此实现）：

- **大 yaw = 达妙**（360° 量程，SPEED 模式）—— 比赛中**整场不用，但要锁住**。
  实现方式是「C 软锁 + 手动微调」：记下上电位置 `yaw_lock_pos`，
  每周期下发 `yaw_speed_out（摇杆微调） + Kp·(yaw_lock_pos − pos)`，电机靠自身阻尼被拽在原位。
- **小 yaw = M6020**（160° 量程，POS 模式，拨码 5）—— 真正干活的扫描/自瞄轴。
- **pitch = 达妙**（P_S 模式）—— 另一个自瞄轴。

⚠️ **`Control_Pantile` 的参数顺序是 ①小yaw ②pitch ③大yaw**，
和 `PANTILE_MOTOR_NUM` 宏（= 2）不一致的原因是 `pantile_motor[PITCH]` **恒为 nullptr**，
pitch 走达妙对象单独处理。别按数组下标去猜哪个是 pitch。

**轮子排布**：45° X 形，左前起顺时针 1→2→3→4，等于拨码 1→4。

### 1.2 比赛形态与由此推出的设计约束

赛制是**全自动、由上位机统一控制**：机器人走固定轨迹到固定点，然后攻击远处一个**小幅旋转 + 平移**的靶。

由此推出四条硬约束，直接决定了代码里的多处取舍：

| 约束 | 推论 | 代码里的体现 |
|---|---|---|
| 全自动，无人遥控 | 失联必须**停车**，不能保持最后指令 | `RC_LOST_MS=100` + `NavReady()` 租约到期 |
| 靶只在小范围动 | **不需要全周扫描**、不需要底盘协同云台 | 小 yaw `yaw_span=1707`（±75°）够用 |
| 靶在远处 | 弹道主要由视觉解算，电控只负责**跟随** | 自瞄改为「绝对目标 + IMU 闭环」，不做弹道预测 |
| 大 yaw 派不上用场 | 干脆锁死，省一个自由度 | `yaw_lock_pos` 软锁 |

---

## 二、总体架构与分层

### 2.1 四层结构

代码按**依赖方向严格单向**分层，上层可以调下层，下层**绝不回调**上层：

```
┌──────────────────────────────────────────────────────────┐
│ L4  taskslist.cpp   调度层                                │
│     5 个 FreeRTOS 任务，只决定"什么时候跑什么"            │
│     ❗不含任何控制逻辑                                   │
├──────────────────────────────────────────────────────────┤
│ L3  control.cpp     控制层                                │
│     CONTROL：CHASSIS / PANTILE / SHOOTER 三个机构          │
│     把"意图"（摇杆量、目标角、是否开火）翻译成电机指令       │
├──────────────────────────────────────────────────────────┤
│ L2  模型层：motor / HTmotor / imu / RC / xuc / judgement   │
│     一个电机 = 一个 Motor 对象，内含 PID + 状态 + 反馈解析  │
│     一个传感器 = 一个 IMU / RC / XUC 对象                  │
│     ❗不下发指令，只维护自身状态                          │
├──────────────────────────────────────────────────────────┤
│ L1  驱动层：can / usart / tim / gpio / delay              │
│     只跟 HAL 打交道，DMA 与中断在这里终结                   │
└──────────────────────────────────────────────────────────┘
```

`motor.h` 反过来 `#include "kalman.h"` 和 `"pid.h"`（算法部件），
`label.h` 只放宏和 `PARAMETER` 全局参数 —— 它是**唯一允许被所有层 include 的头**。

### 2.2 五个任务

| 任务 | 周期 | 优先级 | 栈 | 干什么 |
|---|---|---|---|---|
| `DecodeTask` | 5 ms | **3** | 128 | `rc.Decode` + `imu_*.Decode` + `xuc.Decode`；每 4 拍调一次 `xuc.Encode`（50 Hz 上行） |
| `ControlTask` | 5 ms | 2 | 128 | `rc.Update` → `chassis.Update` → `pantile.Update` → `shooter.Update` |
| `MotorUpdateTask` | 2 ms | 2 | 256 | 8 个大疆电机 `Ontimer`（解析反馈 + 回填发送缓冲）+ 2 个达妙 `State_Decode`/`Ontimer` |
| `CanTransimtTask` | 1 ms | 2 | 256 | 三相轮转：达妙控制帧 / 0x1FF / 0x200 |
| `ArmTask` | 100 ms | 1 | 128 | `DMmotorinit()` ×2（含 1 ms 阻塞 delay）+ `power.Send()` |

优先级安排：`Decode(3) > Control = Motor = CanTx(2) > Arm(1)`。
解码最优先是因为它的输入是**别人的命令**，晚一拍就意味着多一拍的控制延迟；
`Arm` 里有两个阻塞调用，绝不能和上报混在一个任务里。

### 2.3 单向数据流

```
     遥控器(DBUS)          上位机(V2)          IMU×2
        │ UART1             │ UART4          UART2/5
        ▼                   ▼                 ▼
    ┌────────────────────────────────────────────────┐
    │ IDLE 中断 → DMA 快照 → 队列(深1) → DecodeTask │  ← 唯一入口
    └────────────────────────────────────────────────┘
                            │ 解析成对象状态
                            ▼
    ┌────────────────────────────────────────────────┐
    │ ControlTask：RC::Update → OnRC / OnPC         │
    │   OnRC = 遥控逻辑 + 失联保护                    │
    │   OnPC = 上位机覆盖（AUTO 模式下）              │
    │   ⇒ 天然覆盖顺序正确，不需要额外插旗标         │
    └────────────────────────────────────────────────┘
                            │ 写 setspeed / setangle
                            ▼
    ┌────────────────────────────────────────────────┐
    │ MotorUpdateTask(2ms)：Ontimer → PID → 回填缓冲   │
    └────────────────────────────────────────────────┘
                            │
                            ▼
              CanTransimtTask → CAN 总线 → 电机
```

**🔴 三条不能违反的铁律：**

1. **中断只做搬运，不做业务。** ISR 里只允许：停 DMA、读 NDTR、打时间戳、丢进队列。
   任何解析、PID、格式化都不许在 ISR 里。
2. **每个发送缓冲只有一个写者。** `can.temp_data` 由 `Ontimer` 写、`CanTransimtTask` 发，
   `hcan.pTxMsg` 被多个任务共用 ⇒ `CAN::Transmit` **不是线程安全的**，别从别的任务调它。
3. **分频计数器必须是任务私有 `static`。** 🔴 见 §4.9。

### 2.4 全局对象清单

`STM32F405.cpp` 里定义了全部单例，改装配关系时**只改这个文件**：

```cpp
Motor  can1_motor[4] / can2_motor[4];   // 8 个大疆，容量宏在 label.h
DMMOTOR DMmotor[2];                     // 达妙：0=pitch(P_S), 1=yaw(SPEED)
CAN    can1, can2;      UART  uart1..uart6;     TIM   timer;
IMU    imu_pantile, imu_chassis;        RC    rc;
CONTROL ctrl;           PARAMETER para;  TASK   task;
Judgement judgement;   POWER power;     LED   led1..led4;
```

⚠️ **`initial_cnt` 是全局变量，被所有 `Motor` 实例共享** ⇒ 只有前 2 个电机完成了
`initial_x` 标定。当前 `distance` 没有消费者，所以无害，但别在它上面加逻辑。

---

## 三、需要控制的部分（按机构分层）

### 3.1 底盘 · CHASSIS

**控制对象**：`chassis_motor[4]`（4× M3508，CAN2，SPD）

**输入来源（三选一，按优先级）**：

| 来源 | 写入者 | 场景 |
|---|---|---|
| 摇杆 | `RC::RC_Control()` | 手操各模式 |
| 上位机导航 | `RC::ApplyNav()` | AUTO 模式下 `0x11 NAV_COMMAND` |
| 双 IMU 差值 | `Keep_Direction()` | FOLLOW 模式，以云台方向为参考系 |

**处理链**：

```
速度指令 (m/s 或摇杆量)
  → ①目标钳位  |v| ≤ NAV_MAX(4000)
  → ②增量限幅  每周期最多 ±DV_MAX(200 rpm)   ← 防暴冲
  → ③下发 speedx / speedy (rpm)
  → ④45° X 逆解
  → ⑤逐轮限幅到 maxspeed
```

**45° X 布局逆解**（`control.cpp: CHASSIS::Update`）：

```
w₁ = -vx - vy + K·ωz      (左前)
w₂ = +vx - vy + K·ωz      (右前)
w₃ = +vx + vy + K·ωz      (右后)
w₄ = -vx + vy + K·ωz      (左后)
```

🔴 **当前是逐轮独立钳位**，超量程时会**畸变合速度方向**（车会跑偏）。
正解是等比例缩放整体，让方向不变。K_NAV（159）也还没实测标定，
`para.K_NAV` 是把 m/s 换算成 rpm 的经验值。

### 3.2 云台 · PANTILE

**控制对象**（三个电机，两种控制哲学）

| 轴 | 对象 | 模式 | 指令字段 | 限幅 |
|---|---|---|---|---|
| 小 yaw | `pantile_motor[YAW]` = M6020 | POS | `setangle`（编码值 0~8192） | `ClampAngle(±yaw_span=1707)` |
| pitch | `DMmotor[0]` | P_S | `setPos`（弧度）+ `setSpeed`（限速） | `dm_pitch_min=-0.55 ~ +0.4` |
| 大 yaw | `DMmotor[1]` | SPEED | `setSpeed`（rad/s） | 无（靠软锁） |

**三种 yaw/pitch 控制方式并存**：

1. **手动（RC）**：`Control_Pantile()` 把摇杆量积分成位置增量。
   摇杆回中时增量自然为 0 ⇒ 目标角保持不动（这就是"软锁"的由来）。
2. **软锁（大 yaw）**：`PANTILE::Update` 里
   `setSpeed = yaw_speed_out + yaw_lock_Kp·(yaw_lock_pos − pos)`，**单一写者**。
   ⚠️ 这行在 guard **内部**，不要挪到外面。
3. **自瞄（PC）**：`RC::ApplyAim()`，见 §6.4。

**latch 机制**：pitch 首次拿到有效反馈时锁存零点
（`pitch_ready` + `pitch0 = DMmotor[0].pos`），之后所有指令都是相对这个零点。
🔴 **`pitch_ready` 永不复位**，所以上电前几秒没反馈时不要下发指令。

### 3.3 发射机构 · SHOOTER

**控制对象**：`shooter_motor[0..1]`（摩擦轮 M3508）+ `supply_motor[0]`（拨盘 M2006）

`SHOOTER::Update()` 六段：

```
① 摩擦轮      openRub ? ±shoot_speed : 0
② 弹速检测    取两轮平均转速 rpm_avg
③ 扳机判决    阈值 300 / 迟滞 200（摇杆是模拟量，必须迟滞防抖）
④ 短/长拨     上升沿 fire_pulse；按住 >300ms 后 fire_cont，每 125ms 一次
⑤ 拨盘累加    setangle += -bullet_step（每次都同一个方向）
⑥ 超前钳制    lead = setangle - sum_angle，钳到 ±bullet_lead_max
```

**拨盘是 POS 双环**（`motor.cpp: Ontimer` 的 POS 分支）：

```
位置外环（每 2ms）                     速度内环（每 2ms）
err = mechanicalToDegree(setangle - sum_angle)   current += pid[speed].Delta(
  ├ 死区 pos_deadband = 3.0（≈0.08°）              speed_ref - curspeed)
  └ 位置环 PID(1.0, 0.01, 0.02, 0)      speed_ref → 钳到 ±maxspeed
       ↓
   speed_ref (rpm)
```

**几个必须记住的换算**：

| 量 | 值 | 来源 |
|---|---|---|
| 减速比 | 36:1 | M2006 |
| `mechanicalToDegree(a) = a×360/8192` | ⇒ **位置环误差单位是「度」** |
| `bullet_step` | `8192×36/7 = 42130.29` | 7 发/转 |
| 一发对应角度 | 1851.6° | `bullet_step` 换算后 |
| `bullet_lead_max` | `2×bullet_step` | 追不上时钉在 2 发处 |

🔴 **外环时间常数 τ ≈ 1/(6·Kp)**（rpm→°/s 要 ×6）。Kp=1.0 ⇒ τ≈167 ms，
**比连发节拍 125 ms 还长** ⇒ 每发都收不了尾，误差持续累积到 `bullet_lead_max`，
表现为「指令平直、反馈抖动」。建议 Kp 提到 3~6。

🔴 **M2006 反馈转速是「转子」口径**（C610 所有反馈都减速前）：
官方空载 500 rpm 是**输出轴**，×36 ⇒ 转子可达 18000 rpm。
每 1 发/秒 需 308.6 转子 rpm，所以 `maxspeed` 必须按转子口径给
（当前 3000 只相当于 9.7 发/s 的能力）。

🔴 **内环 `Td=0.5` 且 `alpha=0`**：对整数量化的 rpm 反馈做二阶差分，
噪声放大 ≈0.54/计数 ⇒ 电流/转速抖。建议 Td → 0~0.05。

### 3.4 控制权归属表

🔴 **同一个电机只能有一个指令写者**，这张表就是裁判：

| 电机 | 写 `setspeed`/`setangle` 的地方 | 别人不许碰 |
|---|---|---|
| 4 底盘轮 | `CHASSIS::Update()` | `RC` / `ApplyNav` 只写 `speedx/y` |
| 2 摩擦轮 | `SHOOTER::Update()` | `RC` / `ApplyAim` 只写 `openRub` |
| 拨盘 | `SHOOTER::Update()` | **任何地方都不许直接写 `setangle`** |
| 小 yaw | `PANTILE::Update()`（限幅）+ `ApplyAim`（步进） | 二者叠加，不是二选一 |
| pitch 达妙 | `PANTILE::Update()`（写 `setPos`） | — |
| 大 yaw 达妙 | `PANTILE::Update()`（**唯一写者**） | 🔴 别在任何别处写 `DMmotor[1].setSpeed` |

---

## 四、代码里用到的专业技术

### 4.1 PID：三个实现，一套语义

`pid.h` 里三个方法，参数统一是 `(Kp, Ti, Td, alpha)`，**注意是时间常数不是离散系数**。

| 方法 | 类型 | 用在哪 | 特点 |
|---|---|---|---|
| `Delta(err)` | **增量式** | 所有速度环 | 输出增量，直接 `current +=` |
| `Position(err, limit)` | 位置式 + 钳位 | 位置外环 | 积分槽钳到 ±limit |
| `Position1(err)` | 位置式 | `supercap.cpp` | 积分槽固定 ±1000 |

```cpp
// 增量式：三个历史误差带 0.92/1.08 缩放
m_error[LLAST] = m_error[LAST] * 0.92f;
m_error[LAST]  = m_error[NOW] * 0.92f;
m_error[NOW]   = error * 1.08f;
return Kp*(NOW-LAST) + Ti*NOW + Td*(NOW - 2*LAST + LLAST);
```

🔴 **这个 0.92/1.08 缩放让等效积分增益变成 `1.08×(Ti + 0.08×Kp)`**。
底盘 `Ti=0.1` 实际是 0.238 —— 调参时按公式算出来的值和实际生效的不一样。

🔴 **`Position(err, limit)` 第二参钳的是积分槽，不是输出。**
`Ti=0` 时钳位完全无效（积分项恒 0）。

### 4.2 串级双环

拨弹盘用的就是标准串级结构（位置外环 → 速度内环）：

```
外环慢（K 小、量纲是度）→ 产生速度给定
内环快（K 大、量纲是 rpm）→ 产生电流
```

**为什么拨盘要双环**：位置外环把"要转多少度"翻译成"转多快"，
内环负责在 2 ms 内把转速追上去。拨出去的速度必须均匀（否则一颗弹的初速不稳），
所以外环不能直接给电流。

### 4.3 卡尔曼滤波（⚠️ 当前未接线）

`kalman.h` 里是一维卡尔曼滤波器，标准五步：

```cpp
X_mid = A * X_last;                    // 预测
P_mid = A * P_last + Q;                // 协方差预测
kg    = P_mid / (P_mid + R);           // 增益
X_now = X_mid + kg * (dat - X_mid);    // 融合
P_now = (1 - kg) * P_mid;
```

`A = B = 0, H = 1` ⇒ 退化成"一阶低通 + 自适应增益"：
**Q 越大越信任测量，R 越大越信任模型**。构造时传 `(Q, R)`。

**⚠️ 现状要说明白**：`motor.h` 里声明了两个实例

```cpp
Kalman kalman{1.f, 40.f};         // 电机速度
Kalman currentKalman{1.f, 40.f};  // 电流
```

但**全工程 grep 不到任何 `.Filter()` 调用** ⇒ 这两个对象目前是**死的**，
声明了但没接线。真要用起来，正确位置是 `motor.cpp: Ontimer()` 里
对 `curspeed` 和 `current` 各滤一次。

**当前的滤波只有 PID 里的 2 点滑动平均**（`PID::Filter`，`FILTER=2`），
以及 CRC 校验 —— 那不是滤波。

### 4.4 姿态解算：模块做，MCU 不做

🔴 **姿态解算不在本工程里。** CH010 模块内部跑自己的融合算法，
通过 UART 输出**已经算好的欧拉角 + 角速度 + 加速度**。

MCU 侧只做三件事：

1. **CRC 校验**（见 §5.2）
2. **有效性判定**（`IMU::MarkSampleValid`）：
   ```cpp
   data_ok = |roll|≤180 && |pitch|≤90 && |yaw|≤180;   // NaN/Inf 比较恒 false，天然拦掉
   last_rx_tick = xTaskGetTickCount();
   if (data_ok) init_done = true;
   ```
3. **角度回绕归一**（`CONTROL::GetDelta`）：把 ±180° 的差值折到 ±180°。

**单位口径**：模块给**度**，`xuc.Encode()` 里用 `Deg2Rad()` 转成**弧度**再上报。
⚠️ 换算只在这一处，两个方向的换算不要重复做。

### 4.5 麦轮逆解与坐标变换

见 §3.1。`Keep_Direction()` 用的是**双 IMU 差值法**：

```cpp
dyaw = ctrl.GetDelta(imu_pantile.GetAngleYaw() - imu_chassis.GetAngleYaw());
// 车体系速度 → 世界系速度
speedx = fwd·cos(dyaw) - left·sin(dyaw);
speedy = fwd·sin(dyaw) + left·cos(dyaw);
```

两个 IMU 都装在机器人上，**差值就是云台相对车体的转角** —— 不需要额外编码器。
⚠️ `left` 符号待架空验证。

### 4.6 工程上真正起作用的几个"小技术"

这些看起来琐碎，但去掉任何一个都会出问题：

| 技术 | 位置 | 不做会怎样 |
|---|---|---|
| **死区** | `pos_deadband` | 云台在目标附近来回抖（粘滑） |
| **迟滞** | 扳机 300/200 | 摇杆在阈值附近 ⇒ 开火状态疯狂翻转 |
| **变化率限幅** | `DV_MAX`、`RateSlope` | 底盘从一个指令切到另一个瞬间暴冲 |
| **抗积分饱和** | `Position(err, limit)`、`bullet_lead_max` | 堵转时积分爆掉，松手瞬间弹飞 |
| **前馈式整数量化** | M2006 位置累加 | 目标角停在"最近能到的位置"，永久欠一个计数 |
| **无符号差值** | `TIM::UsElapsed`、角度回绕 | 计数器回绕瞬间算出负时间/巨大角度 |
| **角度回绕归一** | `getdeltaa`、`GetDelta` | `±4096` 边界跳变 ⇒ 电机猛转 |

### 4.7 增量式电流输出

所有大疆电机都是**增量式**的：

```cpp
current += pid[speed].Delta(speed_ref - curspeed);   // 每次只加一个增量
current = setrange(current, maxcurrent);             // 最后钳位
```

🔴 **副作用：误差归零后 `current` 会冻结在上次的值**，不会自己回零。
这是增量式的固有特性，改动时要意识到"停机"不是"输出 0"。

### 4.8 保护机制

| 保护 | 实现 | 触发后果 |
|---|---|---|
| 遥控失联 | `last_rc_tick` + `RC_LOST_MS=100` | 清**遥控来源**的指令（不碰视觉） |
| 上位机失联 | `valid_for_ms` 租约（1~150 ms，推荐 100） | `AimReady()/NavReady()` 转 false |
| IMU 失效 | `IsValid()` 三个标记 | flags 位清零 + 上行告知上位机 |
| 过温 | `temperature > 70` → `setspeed = 0` | 🔴 **对 POS 模式无效**（POS 分支不读 `setspeed`） |
| 电机掉线 | `StatusIdentifier()`（20 次电流不变） | 状态置 `UNCONNECTED` |

⚠️ **失联保护只清"遥控来源"**。这是刻意的：视觉和遥控是两路独立来源，
清掉全部会让台架调试时推杆挪不了车。

### 4.9 🔴 分频计数器的线程安全

CAN 发送需要三相轮转（达妙 / 0x1FF / 0x200）各占 1 ms。原实现是：

```cpp
// ❌ 已废弃
switch ((timer.counter++) % 3) { ... }
```

`timer.counter` 是**全局变量**，在 **TIM3 中断（1 kHz）和任务（1 kHz）各 +1 一次** ⇒
余数步进是 `(1000·Δt + 1) mod 3`。只有 SysTick 与 TIM3 相位恰好对齐时才等于
"每 3 ms 各发一帧"，相位一漂就变成「某帧连发两次、另一帧间隔 6 ms」——
对底盘电流环就是**控制周期抖动**。

```cpp
// ✅ 现在：任务私有 static，只在本任务递增
static uint8_t can_div = 0;
switch (++can_div % 3) { ... }
```

**这条规则推广到全工程：任何分频/计数都必须是任务私有 `static`，
不许用共享全局变量。** `timer.counter` 成员已删。

---

## 五、下位机通信系统（SWD / USART / CAN）

### 5.1 SWD —— 调试与烧录通道

⚠️ **先纠正一个概念：SWD 不是"通信协议"，它没有帧、没有波特率、没有上层协议。**
它是 ARM 内核的**调试口**（Serial Wire Debug），只有信号定义：

| 信号 | 引脚（F405） | 方向 | 说明 |
|---|---|---|---|
| SWDIO | PA13 | 双向 | 数据 |
| SWCLK | PA14 | 输入 | 时钟 |
| SWO | PB3 | 输出 | 可选，串行输出（trace） |
| nRST | NRST | 输入 | 复位 |

它承担的是**烧录 + 在线调试 + 断点/单步/变量监视 + FreeRTOS 任务列表**。
本工程用它做的事：

- **Windows 侧**：VisualGDB 通过 SWD 烧录和调试，配置在
  `STM32F405-Debug.vgdbsettings`（`.vgdbsettings` 随仓库带下来，clone 后自动可用）。
- **Linux 侧**：OpenOCD 当 SWD 服务器，`gdb-multiarch` 当客户端，
  VS Code 的 Cortex-Debug 扩展驱动，配置在 `.vscode/launch.json`：
  ```json
  "configFiles": ["interface/stlink-v2.cfg", "target/stm32f4x.cfg"],
  "device": "STM32F405RG", "rtos": "FreeRTOS"
  ```
  支持 **ST-Link V2** 和 **CMSIS-DAP** 两个配置，F5 直接调起。

⚠️ **`SWDIO` 兼作 `PA13`，而有些板子把它接到 TJA/SWD 口而不是排针。**
接调试器时先确认板上是哪一路。`adapter speed` 默认 3000 kHz
（保守值，线长或排线质量差时降到 1000）。

### 5.2 USART —— 6 路，全部 DMA + IDLE

**硬件映射**（`usart.cpp` 实际配置，不是注释里那些笔误）

| 对象 | 外设 | TX | RX | 波特率 | DMA RX | DMA TX | NVIC |
|---|---|---|---|---|---|---|---|
| `uart1` | USART1 | PA9 | PA10 | 100000 | DMA2 S2 | DMA2 S7 | 6 |
| `uart2` | USART2 | PA2 | PA3 | 115200 | DMA1 S5 | DMA1 S6 | 6 |
| `uart3` | USART3 | PC10 | PC11 | — | DMA1 S1 | DMA1 S3 | 6 |
| `uart4` | UART4 | **PA0** | **PA1** | 921600 | DMA1 S2 | DMA1 S4 | 6 |
| `uart5` | UART5 | PC12 | PD2 | 115200 | DMA1 S0 | DMA1 S7 | 6 |
| `uart6` | USART6 | PC6 | PC7 | — | DMA2 S1 | DMA2 S6 | 6 |

🔴 **UART4 的引脚是 PA0/PA1，不是 `STM32F405.cpp` 注释里写的 PC10/PC11。**
代码是对的（`GPIO_PIN_0 | GPIO_PIN_1` + `GPIO_AF8_UART4`），注释是历史笔误。

**接收链路（DMA Normal + IDLE 中断）**：

```
硬件：DMA 循环填 m_uartrx[100]  →  一帧结束触发 IDLE 中断
  │
  ▼  OnUARTITHandler()（中断上下文，只做搬运）
  ① __HAL_DMA_DISABLE           ← 必须先停，NDTR 还在变
  ② len = UART_MAX_LEN - NDTR  ← 真实帧长（🔴 NDTR 递减，不是 NDTR 本身）
  ③ rx_time_us = TIM::NowUs()  ← 🔴 1 MHz 真微秒时基
  ④ if 队列非空 ++rx_drop_cnt ← 静默丢帧变可见
  ⑤ xQueueOverwriteFromISR     ← 队列深度 1
  ⑥ 重装 NDTR + ENABLE
  │
  ▼  DecodeTask（任务上下文，做业务）
  xQueueReceive → 解析
```

**三个设计要点：**

1. 🔴 **NDTR 是"剩余未传输字节数"，随接收递减** ⇒ 真实长度 = `UART_MAX_LEN - NDTR`。
   第一版写成 `len = NDTR`，被探针抓出来。
2. 🔴 **DMA 缓冲不清零** ⇒ 短帧尾部挂着上一帧的残字节。
   **消费端必须靠 `rx_len` 才知道有效数据到哪结束**，CRC 必须在读偏移之前跑。
3. 🔴 **`DMATransmit` 有在途守卫**：
   ```cpp
   if (huart.hdmatx->Instance->NDTR != 0u) { ++tx_busy_reject; return false; }
   ```
   上一帧没发完就**拒绝**本次并计数。原实现无脑 DISABLE→重装→ENABLE，
   会把「上一帧后半段」和「本次前半段」拼成一条错乱帧流 ——
   上位机 CRC 必挂，现象是"偶发丢包"，极难查。
   ⚠️ 判据只读寄存器，不依赖 DMA TC 中断（只有 uart4 的 TX 中断接上了）。

**发送链路**：🔴 **不要在任务上下文用 `UARTTransmit`（HAL 阻塞）**。
`__HAL_LOCK` 与中断里的 `HAL_UART_IRQHandler` 会双 BUSY ⇒ 死锁。用 `DMATransmit`。

**时基**：TIM2 寄存器直配做 **1 MHz 自由计数器**，`PSC=83 / ARR=0xFFFFFFFF`。
- 🔴 **绕开 `HAL_TIM_Base_Init()`** —— 它经 MspInit 会开 NVIC，
  1 MHz 下每秒 100 万次中断直接拖死系统。另补 `DIER=0` 防残留使能。
- 回绕周期 `2³² µs ≈ 71.58 min` ⇒ **只能用无符号差值，不能比大小**。
- `UsInit()` **不是 static**，必须 `timer.UsInit(1000000)`。
- 未使能时 `rx_time_us` 恒 0，上层据此识别"时基没就绪"。

### 5.3 CAN —— 2 路 bxCAN，1 Mbps

**硬件配置**（`can.cpp`）：

| 项 | 值 | 说明 |
|---|---|---|
| 预分频 | `Prescaler = 6` | APB1 = 42 MHz ⇒ 7 MHz |
| 位时段 | SJW=1 / BS1=2 / BS2=4 | 1+2+4 = 7 tq ⇒ **1 Mbps** |
| 滤波器 | 掩码模式 32 位，ID/Mask **全 0** | **全通过**，两条总线都收 |
| CAN1 引脚 | **PA11 = RX, PA12 = TX** | 🔴 注释写的 PD0/PD1 是错的 |
| CAN2 引脚 | PB12 = RX, PB13 = TX | — |
| 模式 | ABOM 自动离线 / AWUM 自动唤醒 / NART 禁重传 | 一次只传，不重试 |

**两套协议共存在同一对总线上：**

**① 大疆 RoboMaster 协议（0x1FF / 0x200 / 0x2xx）**

| 方向 | ID | 内容 |
|---|---|---|
| 发送 | `0x200 + (IDn − 1)` | 4 个电机的电流值，各 2 字节 |
| 发送 | `0x1FF` | 电调在线状态位图 |
| 接收 | `0x201 ~ 0x208` | 反馈：机械角、转速、电流、温度 |

**槽位映射**（🔴 拨码和反馈 ID 不是简单线性）：

| 型号 | 反馈 ID | 物理拨码 |
|---|---|---|
| C620（M3508）/ C610（M2006） | `0x200 + 拨码` | = 拨码 n |
| GM6020（M6020） | `0x204 + 拨码` | = 拨码 n − 4 ⇒ **ID5 的物理拨码是 1** |

数组下标统一是 `IDn − ID1`（= `ID − 0x205`），ISR 落点 `data[StdId − 0x201]`。
两个 buffer 各 12 组，`temp_data[16]` 里 byte 0-7 = 0x200 帧、8-15 = 0x1FF 帧。

🔴 **`has_feedback` 的唯一入口是 `motor.cpp:78`**，判据是
`temperature != 0 || idata[0] || idata[1]`。
本机 C610 的温度字节恒 0，所以加了兜底 —— **连带后果是过温保护对拨弹盘失效**。

**② 达妙 MIT 协议（自己实现，`HTmotor.cpp`）**

| 项 | 值 |
|---|---|
| 仲裁 ID | `motor_id`（0x09 / 0x06） |
| 控制帧 ID | `ID + 偏移`，偏移 = MIT `0x000` / P_S `0x100` / SPEED `0x200` |
| 使能/失能/清错/存零 | 8 字节 `FF FF FF FF FF FF FF xx`，末字节 `FC/FD/FB/FE` |
| 反馈 | `Data[0] = status[7:4] \| id[3:0]` |
| 状态字 | 0=Disabled 1=Enabled 8~E = 过压·欠压·过流·MOS过温·线圈过温·通信丢失·过载 |

🔴 **达妙是"问询式"反馈** —— 只有收到自己的控制帧才回。
所以**控制帧必须持续发**，停发就等于失去反馈。

🔴 **达妙反馈的有意"汇流"**：`can.cpp` 的 `else` 分支不判 `hcan`，
两条总线的达妙帧都写进 `can2.jointidata[i]`（`i` 由 `Data[0] & 0x0F` 匹配）。
好处是 `State_Decode` 对两条总线通用；🔴 代价是**两条总线出现相同 ID 会争抢同一槽**。
另外这个 `else` 分支写得太宽泛（任何非 0x201~0x208 的帧都收），建议加仲裁 ID 白名单。

🔴 **达妙故障后无法自恢复**：只发 `0xFC` 使能、从不发 `0xFB` 清错；
`status` 存了但没人读 ⇒ 一次过载就永久失能。

🔴 **零缓冲陷阱**：`jointidata` 全 0 时 `pos` 会被算成 `-4π`。
判断"真帧"要用 `(jointidata[i][0] & 0x0F) == (DMmotor[i].ID & 0x0F)`。

### 5.4 中断优先级全景

`configPRIO_BITS = 4`，`configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5`
（STM32 HAL 的 `xxx_IRQn` 编号越小优先级越高）。

| 优先级 | 中断 | 说明 |
|---|---|---|
| **0** | SysTick | RTOS 节拍（`sysclk.h` 里重写的） |
| 0 | TIM3 | 1 kHz，**回调已空**，可连 `BaseInit()` 一起删 |
| **6** | USART1~6 / DMA RX+TX / CAN1_RX0 / CAN2_RX0 | 传感器与反馈 |
| **7** | CAN1_TX / CAN2_TX | 发送优先级低于接收 ✔ |

⚠️ **HAL 的 `xxx_IRQn` 数字 ≥ 5 的才能调 `xxxFromISR`**，这条正好卡在 6/7 上。
🔴 **任何优先级 ≤ 4 的中断里都不许调 `xxxFromISR` 系列的 API**。

---

## 六、上下位机通信方式（V2 精简版）

> 完整字段表见仓库根目录 **`docs_通信协议.md`**（当前唯一权威版，可直接发视觉）。
> 本节只讲**结构和设计理由**，字段偏移不重复。

### 6.1 两条链路完全独立

| 链路 | 物理层 | 帧格式 | CRC 算法 |
|---|---|---|---|
| 上位机 ⇄ 电控 | UART4 @ 921600 | 变长 + type 分流 | **CRC-16/MCRF4XX** |
| IMU → 电控 | UART2/UART5 @ 115200 | `5A A5` 定长 | **CRC-16/CCITT-FALSE** |

🔴 **两个 CRC 算法的初值不同、覆盖范围不同、字节序不同，绝对不能混用。**
上位机链路的实现在 `CRC.cpp`（`CRC_INIT=0xFFFF`、反射表 `0x8408`、低字节先发）；
IMU 的是逐位 MSB-first、init `0x0000`、poly `0x1021`，写在 `imu.cpp`。

### 6.2 V2 帧结构

```
0-1   magic        A6 6A
2     version      02
3     type         消息类型
4-5   payload_len  负载长度（小端）
6-7   sequence     序号（小端）
8-11  boot_id      会话标识
12..  payload      数据域
末2   CRC16        MCRF4XX，覆盖 [0 .. 12+N-1]，低字节先发
```

**总长 = 14 + N，最大负载 128。**

从定长帧换成变长帧 + type 分流之后，**加字段变成"注册一个新 type"，
而不是"两端同时改帧长"**。`static_assert(XUC_MAX_FRAME * 3 <= XUC_TX_BUF)` 兜底。

### 6.3 五条消息

| type | 名称 | 负载 | 方向 | 频率 |
|---|---|---|---|---|
| `0x01` | GIMBAL_IMU | 32 | 上行 | 50 Hz |
| `0x02` | CHASSIS_IMU | 20 | 上行 | 50 Hz |
| `0x81` | LINK_STATUS | 12 | 上行 | 50 Hz |
| `0x10` | AIM_COMMAND | 24 | 下行 | — |
| `0x11` | NAV_COMMAND | 24 | 下行 | — |

上行三帧拼成一段连续缓冲，**一次 DMA 发出**（106 B/次，线上 1.15 ms，
921600 baud 下占空比 5.75%）。

其余类型（`0x03/0x20/0x21/0x30/0x31/0x80`）**不实现**，收到即静默跳过。

### 6.4 自瞄：绝对目标 + IMU 闭环

🔴 **这是整个 V2 里最重要的一条设计变更，也是 10-11 踩过的最大坑。**

旧写法是拿上位机给的误差**每 5 ms 累加一次**：

```cpp
// ❌ 已废弃
mark_pitch += KP * pitch_diff;          // pitch_diff 是过期的同一份误差
y->setangle += KY * yaw_diff * RAD2CNT;
```

同一份误差每秒被重复累加约 **200 次**，目标一路顶到限位。

现在：**收到命令时算一次绝对目标**，之后每周期只做 IMU 闭环小步逼近：

```
HandleAim():  aim_yaw_target   = WrapPi(ref_sample.yaw + yaw_error)   ← 一次性
ApplyAim():   perr = aim_pitch_target − imu.pitch
              if (|perr| > 死区) mark_pitch += KP · perr               ← 每周期小步
```

**误差归零即自然停**，视觉重复发同一个目标也无害。
误差源用 IMU 而非电机编码器 —— 这样安装偏置不会变成稳态误差。

### 6.5 接受链七步（顺序固定）

```
帧结构 / CRC  →  boot_id  →  新 seq  →  flags / 有限值
              →  参考样本匹配  →  样本有效期  →  本地模式与机构保护
```

**「本地模式与机构保护」放最后**，前置全过才走到。任一步失败都会把
`last_reject` 写进 `0x81` 回传 —— 这是排查下行被拒的第一入口。

**参考样本机制**：MCU 把每次上行发出去的姿态/时间戳存进 48 深度的环形缓存。
上位机下行时把 `epoch + seq + time_us` 原样回传，MCU 查缓存比对 ——
这样"误差"对应的就是**上位机收到那一刻的姿态**，而不是"现在的姿态"。

### 6.6 当前开关状态

| 宏 | 值 | 含义 |
|---|---|---|
| `XUC_ALLOW_CONTROL` | **0** | `0x10` 只走接受链路，不驱动任何执行器 |
| `XUC_ALLOW_NAV` | **1** | 导航已放行（速度指令本身 fail-safe） |

flags 里 `SAMPLE_TIME_VALID` 和 `AXES_VERIFIED` **恒为 0** ——
**这是诚实标注，不是遗漏**。按协议规定，上位机在拿到这两个位之前
不得把本版数据用于基于样本的控制。

放行前必须完成两个实验：

| 实验 | 内容 | 判据 |
|---|---|---|
| **A** | ISR µs 时间戳 vs IMU `system_time` 线性回归 | 残差 RMS ≤ 2 ms（理论下限 0.289 ms） |
| **B** | IMU 单轴 / 参考系 / gyro 符号 / 6 轴还是 9 轴 | 拿 `AXES_VERIFIED` |

B 实验的现成抓手：**磁强度字段（`p+36`），9 轴非零、6 轴恒零**，
比"开电机看 yaw 跳"更直接。

---

## 七、构建 / 烧录 / 调试

### 7.1 团队共识

1. **源码统一 UTF-8、行尾统一 LF**（`.gitattributes` 控制）。Windows 检出转 CRLF，
   Linux 保持 LF。**别把文件另存为 GBK**，会出现乱码和 git 假 diff。
2. **不提交构建产物**：`.o / .dep / .rsp / .elf / .bin / .map / .user / .suo`
   已在 `.gitignore` 里。看到它们没进版本库是**正常**的。
3. **一次提交只做一件事**，信息格式：`feat:` / `fix:` / `refactor:` / `docs:` / `chore:`。
4. **提交前必须能编译。**

### 7.2 目录结构

```
Sentinel_robot/
├── CMakeLists.txt               # Linux 构建
├── FreeRTOS.sln                 # Windows：VS + VisualGDB 打开这个
├── cmake/arm-none-eabi-toolchain.cmake
├── linker/STM32F405RGTx_FLASH.ld
├── STM32F405/                   # ★ 唯一经常改的地方（.cpp/.h）
│   ├── VisualGDB/Debug/*.gcc.rsp # 真实编译参数（编译验证要用，见 §9）
│   └── *-Debug.vgdbsettings     # VisualGDB 调试配置
├── ThirdParty/                  # HAL / CMSIS / FreeRTOS / SVD
├── scripts/setup_toolchain.sh   # Linux 一键装工具链
├── scripts/flash.sh             # Linux 一键烧录（OpenOCD）
├── docs_通信协议.md             # ★ 当前唯一权威协议文档
├── docs_V2协议_电控侧定值表.md   # 已并入上面那份，加了失效标记
├── docs_V2精简版_信息与帧结构.md # 同上
├── docs/_archive_2026-10-15.zip  # 四份旧文档的原文备份
└── README.md                    # 本文件
```

> `VisualGDB/`、`.visualgdb/`、`.vs/` 是本地缓存，已被 git 忽略，
> clone 下来不会也不需要看到。

### 7.3 Linux 命令行

```bash
# ① 一键装工具链（只需一次，需 sudo；装完注销重登让用户组生效）
bash scripts/setup_toolchain.sh

# ② 编译
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug \
      -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-toolchain.cmake
cmake --build build

# ③ 烧录（默认 CMSIS-DAP）
bash scripts/flash.sh
OPENOCD_INTERFACE=interface/stlink-v2.cfg bash scripts/flash.sh   # ST-Link
```

VS Code 里：`Ctrl+Shift+B` 编译、`F5` 编译+烧录+调试（两个配置：ST-Link / CMSIS-DAP），
都支持断点、单步、寄存器视图和 **FreeRTOS 任务列表**。

### 7.4 🔴 编译验证必须用真实 rsp

Windows 侧 VisualGDB 会生成真实编译参数的响应文件：

```bash
cd STM32F405
/c/sysgcc/arm-eabi/bin/arm-none-eabi-g++.exe @VisualGDB/Debug/imu.gcc.rsp
# 全工程 18 个 TU 一遍编：
for f in VisualGDB/Debug/*.gcc.rsp; do arm-none-eabi-g++.exe @$f; done
```

⚠️ **rsp 里已经带了 `-MD -MP -MF` 和 `-o`，追加 `-o` 会报 "too many filenames"**，
原样跑。

🔴 **探针只能验逻辑，验不到 include / 类型 / 真实宏。**
曾因只编探针而漏掉真实 `imu.cpp` 里的 const 编译错误 ——
探针是摘录版逻辑，没有出问题的那个上下文。

---

## 八、已知问题与待办

### 8.1 活跃陷阱（改代码前必读）

| 现象 | 根因 |
|---|---|
| POS 模式电机过温保护失效 | `temperature > 70` 只写 `setspeed`，POS 分支不读它 |
| 拨盘误差归零后仍输出电流 | 增量式 PID 的固有特性，`current` 冻结在上次值 |
| 多圈位置对不上 | POS 分支必须走 `use_sum_angle`，`getdeltaa` + `int16_t` 对多圈无效 |
| 首次上电 pitch 偏 92° | `initial_yaw=4096` 与 `yaw_center=8192` 不匹配 |
| `DMmotor[i].status` 有值但没人读 | 达妙故障后永久失能（只使能不清错） |
| 底盘超量程时跑偏 | 逆解逐轮钳位畸变方向，应改等比例缩放 |
| 视觉发太快会丢帧 | 处理上限 **200 帧/s**（队列深 1 + 5 ms 轮询） |
| IMU 时间戳差几毫秒 | `sample_time_us` 是**到达时刻**不是采样时刻，需 A 实验标定 |

### 8.2 待办

- **A 实验 / B 实验**（见 §6.6）—— 放行自瞄的唯一前提
- **按标签解析 IMU 数据域**：厂商手册明确数据域是「若干子数据包」的拼接，
  **标签决定类型和长度**。当前硬编码 offset 只在 `AT+EOUT` 配置不变时成立。
- **未消费字段**：气压（`p+4`）、ID（`p+1`）、磁强度（`p+36`）、四元数（`p+60`）
- **`K_NAV` 实测标定**：写死 `speedx=1000` 跑 1 s 量位移反推
- **CAN 达妙 `else` 分支加仲裁 ID 白名单**（现在过宽）
- **`judgement` / `supercap` / `power.Init`**（已注释）整块未接
- **TIM3 空中断**：`HAL_TIM_PeriodElapsedCallback` 回调已空，确认无消费者后可连
  `BaseInit()` 一起删

---

## 九、给 AI 的话

> 这一节是**硬规则**。如果你正在读这段，说明你在替这个项目写代码或改代码。
> 违反其中任何一条，比写不出功能更严重。

### 9.1 授权边界（最重要的一条）

1. **默认不许改任何文件。** 你只能给代码指导和方案，**所有代码由用户亲手敲**。
2. **用户明确点名授权的文件才能动**，一次授权只对那一次有效。
3. **动手之前必须先说清楚三件事**：
   - 改哪个文件、哪几行
   - **为什么**要改（根因是什么，不是"这样更好"）
   - **怎么**改（给出完整代码，不是思路）

   得到用户同意之后才允许执行。
4. **授权不包含"顺手把相关的地方一起优化"。** 发现了别的问题，
   **写进待办、告诉用户，等他决定**，不要自作主张扩大改动范围。

### 9.2 改代码时的硬性要求

5. 🔴 **禁止为了让代码编过而删除或扭曲语义。**
   比如把 `const uint8_t*` 改成 `uint8_t*` 来消除编译错误 ——
   那是把只读缓冲伪装成可写，编译器从此不再帮你拦错误。
   **正确的做法是改函数签名的 const**。
6. 🔴 **同一处连续改两次、两次症状完全一样 ⇒ 停下来说"我的新改动可能引入了问题"。**
   这个项目已经因此浪费过一整轮：CRC 覆盖范围错和比较时机错，
   **症状一模一样**（`crc_fail` 涨 / `rx_ok=0` / 有数据但字段读不到），
   互相掩盖，导致在用户报告"还是不对"时又往错误方向走了一次。
7. **收到"改动后出现新症状"时，第一反应必须是"我的新改动引入了什么"**，
   而不是"上一个改动还没修对"。
8. **每个结论要么来自读代码，要么来自实测，不许推理当事实。**
   尤其是协议、寄存器、时序这类外部知识 —— **查手册，不要猜**。
   🔴 查手册要排在「让用户截图」和「写穷举诊断代码」之前。
9. **改了要验证，而且用真实参数验证**（§7.4）。
   自己造数据自己算 CRC 只能证明算法自洽，**证明不了它和真实模块一致**。
10. **发现了 bug 要说清楚是哪一处的**，并解释为什么会产生这个现象。
    **不要只说"修好了"** —— 用户需要知道下次怎么自己认出来。

### 9.3 沟通方式

11. **先给结论，再给理由。** 不铺垫、不捧场。
12. **有漏洞当面指出，不要包装。** 文档和代码不一致、注释和代码不一致、
    死代码、被注释掉的整块功能 —— 这些都要主动说。
13. **承认自己的错误，说清楚性质。**
    区分"我改出来的 bug"和"本来就有的 bug"，不要混过去。
14. **不确定就说不确定**，不要用"可能""建议考虑"糊弄过去；
    要么去查手册 / 去实测，要么明说这里需要上车验证。

### 9.4 项目约定

15. **源码统一 UTF-8 + LF**，改文件时别动行尾。
16. **一次提交一件事**，信息用 `feat:` / `fix:` / `refactor:` / `docs:` / `chore:` 前缀。
17. **对话笔记**写到 Obsidian 的 `电控笔记/AI` 目录，文件名以日期开头
    （如 `10.15 通信协议换版.md`），正文开头先写一段简介。
18. **每次完成实质工作后，用一句话把结论写进项目记忆**
    （`.workbuddy/memory/`），只记有长期价值的东西 ——
    搜索结果、临时路径、工具报错**不要记**。
19. **文档会漂移，发给别人的协议必须与代码重新核对一遍。**
    这个项目里同一天写的三份笔记，频率、CRC 口径、轴映射三处都发生过漂移。

### 9.5 修改前建议先自查

- 这个改动会影响哪些层？跨层改动是不是该拆成两次？
- 有没有共享全局变量被两个上下文写？（分频计数器是重灾区）
- 有没有新的偏移量 / 长度上界可能越界？
  （长度上界必须写显式常量 `UART_MAX_LEN`，**不要写 `sizeof(指针别名)` —— 那是 8 字节**）
- 累加器型 API 是不是可能被误用？（比如"每段各自比较"）
- 有没有零消费者的变量和死函数？（本项目已经清掉一批，别再攒）

---

*最后更新：2026-10-15 ｜ 状态：V2 协议只读联调中，导航已放行，自瞄待 A/B 实验*