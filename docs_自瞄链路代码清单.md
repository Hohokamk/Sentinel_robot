# 自瞄链路（XUC + OnPC）代码清单

> 生成时间：2026-10-05（含 20:40 修订：达妙 Yaw 走 C 口径「软锁 + 手动微调」，撤回上一版"删 yaw_speed_out"的错误建议）
> 用途：对照抄写，**不自动写入源码**。所有代码由本人亲手敲。

---

## 0. 前置（★ = 状态已变，见 0.5）

### 0.1 `RC.h` 加宏 —— ✅ **已完成**（`RC.h:8` `#define TIMEOUT_TICKS 50`）

### 0.2 `RC.cpp` 腾一个 AUTOAIM 入口 —— ✅ **已完成**，走 `RC_STATE(UP, MID)`

```cpp
	case RC_STATE(UP, MID):
		ctrl.mode = CONTROL::AUTOAIM;
		break;
```

（不是本文档原建议的 `DOWN-MID`。**进门方式 = 左拨上 + 右拨中**；`RC_STATE(UP, UP)` 是 ROTATION，别按错。）

### 0.3 【已作废，见 0.5】手动微调项补在了**错误的函数**里

原作者补在了 `PANTILE::Control_Pantile`（`control.cpp:78-79`），而正确写入点是 `PANTILE::Update`（`control.cpp:143`）—— 后者会**无条件覆盖**前者。详见 §0.5。

### 0.4 `control.h:49` 去掉 `const` —— ✅ **已完成**（`float yaw_lock_Kp = 3.0f;`）

### ★ 0.5 🔴 `DMmotor[1].setSpeed` 双写者冲突（**必修，否则 C 口径永远无效**）

同一个字段被两个函数写，**后跑的赢**：

| # | 位置 | 写入内容 |
|---|---|---|
| 1 | `control.cpp:78-79`（`Control_Pantile` 第 3 段） | `yaw_speed_out + Kp*(pos_lock - pos)` ← 含手动项 |
| 2 | `control.cpp:143`（`PANTILE::Update` 第 3 段） | `Kp*(pos_lock - pos)` ← **不含手动项** |

`ControlTask` 顺序：`rc.Update()`（内含 `Control_Pantile`）→ `chassis.Update()` → `pantile.Update()` → `shooter.Update()`
⇒ **`:143` 后跑 ⇒ 覆盖 `:78-79`** ⇒ 手动微调被丢弃，实际仍是纯 A 口径。

**修法：单一写者 + 意图/执行分层。**

`control.cpp:74-79`（`Control_Pantile` 第 3 段）→ 只产出意图，**删掉写 `setSpeed` 那两行**：

```cpp
	// 3. 达妙 Yaw 轴（纯速度模式）：只产出意图，执行在 PANTILE::Update()
	const float max_dm_speed = PI;                       // 最大 180°/s
	yaw_speed_out = (float)ch_dm_yaw / 660.f * max_dm_speed;
```

`control.cpp:140-144`（`PANTILE::Update` 第 3 段）→ 合并锁 + 手动项：

```cpp
	if (DMmotor[1].decoded)
	{
		if (!yaw_lock_ready) { yaw_lock_pos = DMmotor[1].pos; yaw_lock_ready = true; }
		DMmotor[1].setSpeed = yaw_speed_out                                  // 手动微调（rad/s）
		                    + yaw_lock_Kp * (yaw_lock_pos - DMmotor[1].pos); // 软锁回位
	}
```

**连锁必改（否则修完 0.5 会暴露新 bug）**：`yaw_speed_out` 变成"有记忆的意图量"后，从 `ROTATION`/`SPINNING` 切进 `AUTOAIM` 时残值非零 → 达妙 yaw 持续漂移、锁不住。`RC.cpp` 的 AUTOAIM case 必须补：

```cpp
		ctrl.pantile.yaw_speed_out = 0;      // 清掉上一个模式的手动残值
```

行为：推杆给 `yaw_speed_out`，稳态停在新位置 `yaw_lock_pos + yaw_speed_out / yaw_lock_Kp`；
**松杆 → `yaw_speed_out` = 0 → 自动弹回 `yaw_lock_pos`**（抗弹丸反冲的回位能力）。

### 0.4 `control.h:49` 去掉 `const`

```cpp
	// 现在：const float yaw_lock_Kp = 3.0f;
	float yaw_lock_Kp = 3.0f;       // 去 const 才能 live watch 改值调参
```

`const` 成员在 VisualGDB 里只能读不能写，不调它就只能反复重新编译。

---

## 1. `xuc.h`

```cpp
// 【删】第 25-30 行：struct RxPacket 整个定义
//   理由：协议模型错误（实际是固定偏移硬解析，不是 header+2字节校验），零使用

// 【删】第 37 行
	RxPacket RxNuc;

// 【删】第 145-150 行：u8_to_float 整个函数
//   理由：零调用；且构造大端副本后丢弃，memcpy 拷的还是原序 —— 字节序是错的

// 【删】第 100 行里的 yaw_pre, yaw_spd
	float yaw_pre, yaw_spd, k_feedforward = 0.5f;   // yaw_spd 量纲错且零消费

// 【加】文件顶部 include 之后
#define XUC_FRAME_LEN 32        // 与视觉约定的定长帧（含末两字节 CRC）

// 【加】public 区，第 51 行 rx_count 下面
	uint16_t crc_err = 0;           // CRC 校验失败计数（上车 watch）
```

---

## 2. `xuc.cpp` —— `Decode()` 整体替换

```cpp
void XUC::Decode()
{
	// ① 守卫
	if (!m_uart || !queue_handler || !*queue_handler) return;

	// ② 收帧（非阻塞，没数据直接走）
	pd_Rx = xQueueReceive(*queue_handler, m_frame, 0);
	if (pd_Rx != pdTRUE) return;

	// ③ 帧头
	if (m_frame[0] != 0xA5) return;

	// ④ CRC —— 不通过就当这帧不存在
	if (!verifyCRC16CheckSum(m_frame, XUC_FRAME_LEN)) { ++crc_err; return; }

	// ⑤ 解析：四个角统一成弧度（下游 mark_pitch / setangle 都是 rad）
	const float D2R = PI / 180.f;
	pitch      = FR4(m_frame + 1)  * D2R;
	yaw        = FR4(m_frame + 5)  * D2R;
	yaw_diff   = FR4(m_frame + 9)  * D2R;
	pitch_diff = FR4(m_frame + 13) * D2R;
	distance   = FR4(m_frame + 17);
	fireadvice = (m_frame[21] & 0x01) != 0;
	v_y        = FR4(m_frame + 25);

	// ⑥ 走到这里才算一帧有效数据
	++rx_count;
}
```

**注意**：
- `queue_handler` 与 `m_uart->UartQueueHandler` 是同一个东西，统一用 `queue_handler`。
- `++rx_count` 原在 `if (0xA5)` 外面（噪声帧也计数 → `pc_timeout` 永不超时），必须挪到 ④ 之后。
- **转弧度前先实测**：进 AUTOAIM，手举目标左右平移，看 `xuc.yaw_diff` 量级：
  - `±20 ~ ±40` → 视觉给的是**度** → 按上面写
  - `±0.3 ~ ±0.7` → 视觉给的是**弧度** → 删掉所有 `* D2R`

---

## 3. `RC.cpp` —— AUTOAIM case（★ 已实读，需补一行）

磁盘上现状（`RC.cpp:141-147`）—— `pc.*` 已删净、底盘已交回摇杆 ✅：

```cpp
	case CONTROL::AUTOAIM://自动瞄准	
		ctrl.chassis.speedx = rc.ch[1] * 4000.f / 660.f;
		ctrl.chassis.speedy = -1 * rc.ch[0] * 4000.f / 660.f;
		ctrl.chassis.speedz = rc.ch[2];
		//这里还要写射击，给xuc接管
		break;
```

**❌ 缺 `yaw_speed_out = 0`** —— 补上（理由见 §0.5 的"连锁必改"）：

```cpp
		ctrl.pantile.yaw_speed_out = 0;      // C 锁必需：不写会保留手动模式残值 → 达妙 yaw 漂移
```

⚠️ 可选：想在 AUTOAIM 里保留手动微调，把上面那行换成 `ctrl.pantile.Control_Pantile(0, 0, rc.ch[3]);`
（第 1/2 参传 0 ⇒ 不会和 `OnPC()` 抢 M6020 / Pitch）。**不建议第一版这么做**，变量越少越好调。

---

## 4. `RC.cpp` —— `OnPC()` 补 yaw 段 + pitch 改增量（★ 全部待写）

**★ 实读现状**（`RC.cpp:45-46`）：只有 `ctrl.pantile.mark_pitch = xuc.pitch;`（**绝对赋值**，进 AUTOAIM 瞬间会猛窜），**yaw 段完全没有**。

**先分清两个 yaw**（"为什么只写了 pitch"的答案）：

| 对象 | 类型 | 自瞄该写哪个 |
|---|---|---|
| `ctrl.pantile_motor[PANTILE::YAW]` = **M6020** | 位置式 `setangle` | ✅ **写这里** |
| `DMmotor[1]` = **达妙 yaw** | 速度式 `setSpeed` | ❌ 已软锁，`OnPC` 不碰 |

M6020 通路安全：`PANTILE::Update():125-127` 只对它做 `ClampAngle` 钳位、**不覆盖** ⇒ 无写者冲突。

```cpp
void RC::OnPC()
{
	// ① 只在 AUTOAIM 模式接管，其余模式完全不碰
	if (ctrl.mode != CONTROL::AUTOAIM) return;

	// ② 数据新鲜度
	if (xuc.rx_count != last_rx_count) {
		last_rx_count = xuc.rx_count;
		pc_timeout = TIMEOUT_TICKS;
	}
	else if (pc_timeout > 0) --pc_timeout;

	// ③ 失联 → 执行器归安全态
	if (pc_timeout == 0) {
		ctrl.shooter.openRub = false;
		ctrl.shooter.auto_shoot = false;
		return;
	}

	// ④ 云台 Pitch：偏差角增量（视觉给"还需转过的角"）
	{
		const float KP = 0.5f;                 // 0.3 ~ 1.0，<1 留阻尼
		const float DB = 0.3f * PI / 180.f;    // 0.3° 死区
		if (fabsf(xuc.pitch_diff) > DB)
			ctrl.pantile.mark_pitch += KP * xuc.pitch_diff;
	}

	// ⑤ 云台 Yaw（M6020，位置式）：同样走增量
	{
		Motor* y = ctrl.pantile_motor[CONTROL::PANTILE::YAW];
		const float KY = 0.5f;
		const float DB = 0.3f * PI / 180.f;
		const float RAD2CNT = 8192.f / (2.f * PI);   // ≈ 1303.8 计数/弧度
		if (y && y->has_feedback && fabsf(xuc.yaw_diff) > DB)
			y->setangle += KY * xuc.yaw_diff * RAD2CNT;
	}

	// ⑥ 摩擦轮 + 开火
	ctrl.shooter.openRub = true;
	ctrl.shooter.auto_shoot = xuc.fireadvice;
}
```

要点：
- 死区必须有 —— 增量式是纯积分，视觉若有系统偏置会一路撞 `ClampAngle` 限位。
- `KY / KP < 1` —— 留阻尼防发散。
- 限幅不用自己写：`ClampAngle`（`control.cpp:122-124`）钳 `setangle`，`dm_pitch_min/max` 钳 `mark_pitch`。
- AUTOAIM 下 `Control_Pantile` 不被调用（天然隔离），OnPC 独享 `mark_pitch` 与 `pantile_motor[YAW]->setangle`。

---

## 5. `control.cpp` —— `SHOOTER::Update()` 修正

### 5.1 【必删】`:187` 的 `trig_pre = trig;`（**现在还在**）

```cpp
	// ④ 短/长拨状态机
	const uint16_t TICK = 5, T_ARM = 300, T_RATE = 125;
	bool fire_pulse = false, fire_cont = false;

	if (trig_eff && !trig_pre) { fire_pulse = true; hold_ms = 0; rate_ms = T_RATE; }
	else if (trig_eff) { if (hold_ms < 60000) hold_ms += TICK; if (hold_ms >= T_ARM) fire_cont = true; }
	trig_pre = trig_eff;      // ← 只留这一句
```

**为什么必删**：`:173` 已写 `trig_pre = trig_eff`（含 `auto_shoot`），`:187` 又写 `trig_pre = trig`（不含）。
**手动模式下两行值相同、无害**（所以"之前用着挺好"是真的）；一旦 `auto_shoot = true`：

```
帧 N   :173 trig_pre = true → :187 trig_pre = false（后写的赢）
帧 N+1 :trig_eff && !trig_pre 又成立 → 再判一次上升沿
⇒ 每 5ms 触发一次 fire_pulse，hold_ms 被反复清零
⇒ hold_ms >= T_ARM 永不成立 → fire_cont 连发路径废掉
⇒ fire_pulse 每 5ms 加一个 -bullet_step → lead 顶到 -bullet_lead_max(2 发)
⇒ 拨盘满速狂拨，停火后还会再冲约 2 发
```

修法：**只留一处，且必须用 `trig_eff`**。推荐删 `:187`。

### 5.3 【必补】`auto_shoot` 离开 AUTOAIM 要清掉

`RC.cpp:30` 的 `if (ctrl.mode != CONTROL::AUTOAIM) return;` 早退时**不清 `auto_shoot`**；
而 `RC_Control()` 的前置清零只清 `trig_raw / speedz / openRub / supply_bullet`。
⇒ 在 `fireadvice = true` 时切出 AUTOAIM，`auto_shoot` **永久为 true** → 所有模式 `trig_eff = true` → 无限开火。

```cpp
	if (ctrl.mode != CONTROL::AUTOAIM) { ctrl.shooter.auto_shoot = false; return; }
```

### 5.2 删死物

```cpp
	// :153 这两行无人读，删
	float sum = 0.f; int cnt = 0;

	// :177-184 整段注释掉的旧状态机，删
```

---

## 6. 达妙 Yaw：C 口径（软锁 + 手动微调，**非 A 口径**）

**保持现状，只补 §0.3 那一行。** 上一版清单里"删 `yaw_speed_out`"是错的，已撤回。

| 项 | 处置 |
|---|---|
| `control.cpp:74-77`（`max_dm_speed` + `yaw_speed_out = ...`） | **保留**。这是手动微调源 |
| `control.h:36` `float yaw_speed_out = 0.0f;` | **保留** |
| `Control_Pantile` 第三参 `ch_dm_yaw` | **保留**，有真实消费者（→ `yaw_speed_out`） |
| `control.cpp:140` | **加 `yaw_speed_out +`**（见 §0.3） |
| AUTOAIM case 的 `yaw_speed_out = 0` | **保留**（防手动残值把锁带偏） |

### 6.1 行为与判据

| 场景 | 期望 |
|---|---|
| 手推云台后松手 | `DMmotor[1].pos` 自己回到 `yaw_lock_pos` |
| 推 `ch[3]`（对应模式） | yaw 移动，稳态偏差 ≈ `yaw_speed_out / yaw_lock_Kp` |
| 满杆 `ch_dm_yaw` | `yaw_speed_out = π rad/s`，`Kp = 3` ⇒ 稳态偏差 ≈ 1.05 rad ≈ **60°**（这就是手动行程的上下界） |
| 松杆 | 回位到 `yaw_lock_pos` |

⚠️ **`yaw_lock_pos` 永不复位**（只在第一帧 `decoded` 时锁存）。所以手动推到哪、松手都回"上电位置"。
若想让手动位置**存下来**（松手停在新位置而非回原位），需在"松杆那一刻"加 `yaw_lock_pos = DMmotor[1].pos;`。

⚠️ **符号自检**：首次测试手掰开松手，若 `pos` 是**发散**而不是回位 ⇒ `setSpeed` 与 `pos` 反号 ⇒ 把 `yaw_lock_Kp` 取负。

⚠️ `decoded` 只置位、**永不复位**（与 `has_feedback` 同类）。达妙一旦掉反馈，锁仍按陈旧 `pos` 发令；因为 yaw 整场不用，影响低，但记一笔。

### 6.2 订正 `RC.cpp:173` 的错注释（上次绕圈的源头）

```cpp
	ctrl.pantile.Control_Pantile(rc.ch[2], rc.ch[3], rc.ch[1]);
	// 参数序：①大yaw(M6020) ②pitch(达妙) ③小yaw(达妙，C 口径下手动微调)
```

---

## 7. 需要确认的事实

| 项 | 结论 |
|---|---|
| `shoot_speed = 5000` 是否超限 | **没超**。`motor.cpp:174-177` M3508 → `maxspeed = 5000`。但 `allow` 阈值 = `5000×0.8 = 4000`，满速才 5000，**只有 20% 余量** → 实测稳态 `rpm_avg`，够不到就把系数降到 0.7 |
| `xuc.Encode()` 的 `own_color` | 读 `judgement`（零调用）→ 恒 RED → `detect_color` 恒 1。语义要跟视觉确认 |
| `xuc.Encode()` 的 `aim_x/y/z` | 全工程无人写 → 恒 0。视觉用不用？ |
| 上报频率 | 现在 10Hz（`ArmTask` 100ms）。不够就改 `vTaskDelay`，**别低于 20ms** |

---

## 8. 已验证到位（本轮实读）

- `DMMOTOR::decoded` + `status`（`HTmotor.h:65-66`）与 ID 判据（`HTmotor.cpp:30-32`）
- 达妙 Yaw 锁位骨架（`control.cpp:137-141`）+ `control.h:47-49` —— ⚠️ **还差 §0.3 的手动项 + §0.4 去 const**
- `Motor::use_sum_angle` + POS 双路（`motor.cpp:112-132`）、`pos_deadband`（`STM32F405.cpp:113-114`）
- `control.cpp:127` guard 用 `DMmotor[0].decoded`
- `trig_eff`（`control.cpp:165`）
- `rc.Update()` 已提到 `ControlTask` 最前（`taskslist.cpp:134`）
- `xuc.Decode()` 进 `DecodeTask`（`taskslist.cpp:149`）、`xuc.Encode()` 进 `ArmTask` 判 mode（`:163`）
- `RC.h:60-61` 已有 `pc_timeout` / `last_rx_count`
- `yaw_speed_out` 保留（`control.h:36` / `control.cpp:77`）、AUTOAIM case 已写 `yaw_speed_out = 0`（`RC.cpp:145`）

## 9. 死代码清单（留待 yaw 段定型后再清）

- `RC.cpp:210` `if (sizeof(m_frame) < 18) return;` —— `uint8_t[100]` 编译期常量，恒 false
- `xuc.h`：`frame`（`Init` 赋值零读）、`aim_x/y/z`、`autoaim_controller[2]`、`feedforward`、`speed_x/speed_y`、`x/y/z`、`vx/vy/vz`、`track_flag`、`test_navigation`、`rad_pitch/rad_yaw`、`test_game_status`、`test_game_robot_HP`、`count/navi_count/aim_count`、`stm32_fireadvice`、`k_feedforward`、`test_cont`、`pd_Tx`

## 10. 建议动手顺序（★ 2026-10-05 21:20 重排）

```
── 已完成：0.1 TIMEOUT_TICKS · 0.2 AUTOAIM 入口(UP-MID) · 0.4 去 const · xuc 单位统一 · CRC 接入 ──

1. 【修 0.5】达妙 yaw 单一写者
   a. control.cpp:78-79 删掉写 setSpeed 那两行，只留 yaw_speed_out = ...
   b. control.cpp:140-144 改成 setSpeed = yaw_speed_out + Kp*(pos_lock - pos)
2. 【补 §3】AUTOAIM case 加 ctrl.pantile.yaw_speed_out = 0;   ← 与 1b 必须同时
3. 【OnPC】yaw 段（M6020 增量 + 死区） + pitch 改 mark_pitch += KP*pitch_diff
4. 【§5.1】control.cpp:187 删 trig_pre = trig
5. 上车验证：切 AUTOAIM(左拨上+右拨中) → watch xuc.rx_count / xuc.crc_err
6. 测 xuc.yaw_diff 量级 → 确认单位（若视觉直接给弧度，删掉 * D2R）
7. 调参：KY / KP / 死区 / yaw_lock_Kp（判据：手掰开松手能回位、不嗡嗡震）
```

---

## 11. 导航下行 + Encode 数据源（2026-10-05 22:00 新增）

### 11.0 为什么弃用 `RxPacket`（结论，别再补回来）

`xuc.h:26-31` 的 `RxPacket` 只有 `header + checksum` 两个字段（`sizeof` = 4），**从未描述过真实的 32 字节帧**，是抄来的占位。就算 `memcpy(&RxNuc, m_frame, sizeof(RxNuc))` 也只读到前 4 字节。

用结构体映射接收帧还有四笔代价：必须 `packed`、保留字节得瞎编名字、位域跨编译器不确定、强转是 UB。所以接收侧改**偏移硬解析**（`FR4(m_frame + n)`），发送侧继续用 `TxPacket`（我方定义、字段齐全、要 `sizeof` 喂 DMA）。

⚠️ 顺带：队列 item = `UART_MAX_LEN`(100)，ISR 拷的是**整个 100 字节缓冲快照**且不清零 ⇒ 短帧尾部会带上一帧残字节。**这就是 CRC 必须在读偏移之前跑的原因**（现在顺序正确，别调换）。

### 11.1 底盘速度下行：三条路，选扩帧

现状：
- `xuc.h:50` 有 `speed_x / speed_y / prespeedx / prespeedy` —— 死变量，`Decode()` 从不赋值
- 底盘唯一指令入口 = `ctrl.chassis.speedx/speedy/speedz`（`int32_t`，`control.h:24`）
- AUTOAIM case 现在写摇杆值（`RC.cpp:153-155`）

🔴 **前置障碍：32 字节帧已排满**，`[22..24]` + `[29]` 只有 4 字节保留，装不下 3 个 float（12 字节）。

| 路 | 做法 | 判定 |
|---|---|---|
| **扩帧** | 32 → 44（+3 float），改 `XUC_FRAME_LEN` + CRC 覆盖长度 + 视觉同步改 | ✅ **推荐** |
| 单开导航帧 | 用不同 header 分流 | ❌ 队列深度 1 + `xQueueOverwriteFromISR` → 两类帧互相冲掉，各自实际 ≤5Hz 且可能整帧丢 |
| 复用保留区 | 只有 4 字节 | ❌ 装不下 |

### 11.2 单位换算（最容易漏）

`chassis.speedx` **不是 m/s**。`CHASSIS::Update()`（`control.cpp:97-119`）把 `vx` 直接当 `w[i]` 再当 `setspeed`，而 M3508 `setspeed` 单位 = **rpm**，摇杆满杆给 ±4000。

⇒ `chassis.speedx = v_x * K`，K = 电机 rpm / 底盘 m/s。**实测标定**（给 `speedx = 1000`，量底盘实际速度反推），别用 `60/(2πR)*i` 硬算——轮径/打滑/贴地摩擦都会吃掉误差。

⚠️ ±4000 rpm 量程：超了四轮被 `lim` 钳得不一样，**方向会畸变**。

### 11.3 参考系（必须问视觉）

- **世界系**（相对场地）→ 先按 IMU yaw 旋到车体系，再进逆解
- **车体系**（相对车头）→ 直接塞

固定轨迹形态若车头方向基本不变，第一版可先按车体系走，省掉旋转环节。

### 11.4 写在哪：`RC::OnPC()`，**不要**写在 AUTOAIM case 里

`RC::Update()` = `OnRC()`（内含 `RC_Control()` 写摇杆值）→ `OnPC()`（后跑）⇒ **OnPC 天然覆盖**。与 pitch/yaw 段同层，一个 `mode` 守卫管全部。

⚠️ **掉线必须清底盘**：现在 `pc_timeout == 0` 的早退只清了 shooter（`RC.cpp:39-43`），底盘会保留残值继续跑。补：

```cpp
		ctrl.chassis.speedx = 0;
		ctrl.chassis.speedy = 0;
		ctrl.chassis.speedz = 0;
```

### 11.5 Encode 数据源 —— "上位机要的数据在哪写"

`XUC::Encode()`（`xuc.cpp:48-77`）只负责**搬运**，数据源逐字段：

| 字段 | 来源 | 状态 |
|---|---|---|
| `header` | 常量 0x5A | ✓ |
| `detect_color` | `!own_color` ← `judgement…robot_id <= 7` | 🔴 judgement 零调用 → robot_id 恒 0 → **恒 RED** |
| `reset_tracker` | 硬编码 0 | ⚠️ 应在**进 AUTOAIM 的边沿**置 1 发一帧再归 0 |
| `roll/pitch/yaw` | `imu_pantile.GetAngle*()` | ✓ 真值 |
| `aim_x/y/z` | `xuc.aim_x/y/z`（**成员变量**） | 🔴 全工程无人写 → **恒 0** |
| `checksum` | `appendCRC16CheckSum` | ✓ |

**`aim_x/y/z` 是成员变量，不在 Encode 里算**，要在别处赋值。两种口径选一：
- 枪口**世界坐标** → 需要底盘定位（你现在没有里程计）
- 云台指向的**世界系单位向量** → 只用 IMU 欧拉角即可

⚠️ TxPacket 里**已经有 `roll/pitch/yaw`**，视觉大概率自己从这三算弹道 ⇒ **先问视觉"要不要 `aim_x/y/z`"，不要就删**。

🔴 **链路级坑**：`xuc.Encode()` 只在 `ctrl.mode == CONTROL::AUTOAIM` 才跑（`taskslist.cpp:163`）。若导航与自瞄是两个模式，**导航模式下上报被禁** → 判断条件要放宽。
