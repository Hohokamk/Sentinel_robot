# V2 精简版 —— 信息取舍与帧结构

日期：2026-10-14。基线：《上下位机通信协议 V2（2026-10-13）》。
定位：**V2 的子集实现**。帧头、CRC、以及保留消息的字节布局**与 V2 完全一致**（零分歧）；砍掉的只是**不实现的消息类型**。

---

## 一、取舍原则

| 原则 | 说明 |
|---|---|
| **帧布局不动** | `magic/version/type/len/seq/boot_id/CRC` 与 V2 §3 逐字节一致。省几个字节不值当让视觉重写解析器 |
| **砍的是消息类型** | 不实现的消息**静默跳过**（符合 V2 §3「未知类型在长度合法且 CRC 正确时跳过」） |
| **元数据不进协议** | 型号、参考系、轴约定、profile 写进 `README` + 代码注释，不做协议字段 |
| **拒绝原因合并通道** | 不单独做 `ACK`，统一由 `0x81.last_reject` 携带 |

### 为什么保留 `seq` 和 `boot_id`（这两个是"必要的"，别砍）

- **`seq` 必要**：`0x10` 里的 `ref_imu_seq` 要**回填本侧实际发出的 `0x01` 帧的 seq** ⇒ **参考样本匹配机制依赖它**（V2 §7.2 的四元组匹配）。同时它是"丢帧 vs 降频"的唯一判据 —— 100 Hz 提不上去时你一定会查这个。
- **`boot_id` 必要**：MCU 重启后，host 缓存的参考样本和目标**必须作废**。没有它，host 无从判断"为什么我引用的样本你都说找不到"。
- **`version` 必要**：它是**拒绝旧帧**（`A5`/`5A` 单字节头那套）的判据 —— 你上次改帧长时"100% 丢帧且查不出原因"，就是因为旧帧没有自证能力。

---

## 二、保留的消息清单

| type | 名称 | 方向 | 负载 | 总长 | 首版频率 |
|---|---|---|---:|---:|---:|
| `0x01` | GIMBAL_IMU | 下→上 | 32 | 46 | **20 Hz**（稳定后提 100） |
| `0x02` | CHASSIS_IMU | 下→上 | 20 | 34 | **20 Hz** |
| `0x10` | AIM_COMMAND | 上→下 | 24 | 38 | 视觉侧 ≤50 Hz |
| `0x81` | LINK_STATUS | 下→上 | 12 | 26 | **10 Hz**（比 V2 的 1 Hz 高，用于代替 ACK 反馈） |

**带宽核算（20 Hz 上行）**：`46×20 + 34×20 + 26×10 = 1860 B/s ≈ 2.0%`（921600）。下行 50 Hz 约 2.06%。**两个方向都很宽松。**

### 砍掉的消息

| type | 名称 | 砍掉的理由 | 不做的后果 |
|---|---|---|---|
| `0x03` | IMU_INFO | 装的全是元数据（型号/参考系/capabilities/profile_crc32）⇒ 进 `README` | host 无法自动判断轴约定 ⇒ **`AXES_VERIFIED` 恒 0，控制许可默认关闭**（这是想要的保守状态） |
| `0x11` | NAV_COMMAND | 本轮不实现导航 | 收到后静默跳过 |
| `0x20`/`0x21` | SESSION | 两人小队不需要会话状态机 | 用 **"`valid_for_ms` 超时即释放"** 替代会话语义；`host_session` 双方恒填 0 |
| `0x30`/`0x31` | TIME_SYNC | `age` 的精度需求极低 | `age` 里多一个恒定的 **≈0.5 ms**（= 46B @921600 的传输时间）；对 **100 ms** 的 `valid_for_ms` 窗口完全无所谓 |
| `0x80` | ACK | 没有 OPEN/CLOSE 就失去了必须响应的对象 | 拒绝原因改由 `0x81.last_reject` 携带（10 Hz ⇒ 反馈延迟 ≤100 ms） |

---

## 三、帧结构

### 3.1 通用帧头（全部报文，与 V2 §3 一致）

```text
magic(2) A6 6A | version(1) 02 | type(1) | payload_len(2) | seq(2) | boot_id(4) | payload(N) | crc16(2)
```

| 帧偏移 | 字节 | 类型 | 字段 | 首版取值 |
|---:|---:|---|---|---|
| 0 | 2 | bytes | `magic` | `A6 6A` |
| 2 | 1 | uint8 | `version` | `0x02` |
| 3 | 1 | uint8 | `type` | 见上表 |
| 4 | 2 | uint16 | `payload_len` | 仅负载长度（不含帧头/CRC） |
| 6 | 2 | uint16 | `sequence` | 初值 0，每方向每类型各自递增，65535 回绕 |
| 8 | 4 | uint32 | `boot_id` | 每次启动必须改变（RNG ⊕ 上电时刻即可） |
| 12 | N | bytes | `payload` | |
| 12+N | 2 | uint16 | `crc16` | `AppendCRC16CheckSum(buf, 14+N)`，**低字节先发** |

**总长 = 14 + N**。全部小端、IEEE754 binary32、无隐式 padding、**不用位域**（uint8 + 掩码）。
**CRC 算法零改动**：现有 `CRC.cpp` 即 CRC-16/MCRF4XX（init `0xFFFF`、反射 `0x8408`、无最终异或、低字节先发），与 V2 完全一致（已验：`"123456789"` → `0x6F91`，线上 `91 6F`）。

---

### 3.2 `0x01` GIMBAL_IMU —— 负载 32 字节

偏移相对**负载首字节**（帧内偏移 = +12）。

| 负载偏移 | 字节 | 类型 | 字段 | 含义 | 首版值 |
|---:|---:|---|---|---|---|
| 0 | 1 | uint8 | `sensor_id` | 云台 IMU | **1** |
| 1 | 1 | uint8 | `flags` | 见 3.5 | bit0=1, bit1=1, bit2=0, bit3=0 |
| 2 | 2 | uint16 | `imu_epoch` | 姿态参考系代号 | **1**（代码写死轴配置 ⇒ 不需要递增机制） |
| 4 | 4 | uint32 | `sample_time_us` | 采样时刻（MCU 单调 µs） | **ISR 收到完整帧的 µs 时刻** |
| 8 | 4 | float32 | `roll` | rad | `GetAngleRoll()` × π/180 |
| 12 | 4 | float32 | `pitch` | rad | `GetAnglePitch()` × π/180 |
| 16 | 4 | float32 | `yaw` | rad | `GetAngleYaw()` × π/180 |
| 20 | 4 | float32 | `gx` | rad/s | `GetGyroRoll()` × π/180 ⚠️ |
| 24 | 4 | float32 | `gy` | rad/s | `GetGyroPitch()` × π/180 ⚠️ |
| 28 | 4 | float32 | `gz` | rad/s | `GetGyroYaw()` × π/180 ⚠️ |

⚠️ **gyro 的 ×π/180 待 B4 动态验证后再定**（与 `d(euler)/dt` 比对符号与量级）。

---

### 3.3 `0x02` CHASSIS_IMU —— 负载 20 字节

| 负载偏移 | 字节 | 类型 | 字段 | 首版值 |
|---:|---:|---|---|---|
| 0 | 1 | uint8 | `sensor_id` | **2** |
| 1 | 1 | uint8 | `flags` | **bit1 必须为 0**；其余同 3.2 |
| 2 | 2 | uint16 | `imu_epoch` | **1**（独立维护） |
| 4 | 4 | uint32 | `sample_time_us` | **本路独立打戳**，不得用云台时间代替 |
| 8 | 4 | float32 | `roll` | `imu_chassis.GetAngleRoll()` × π/180 |
| 12 | 4 | float32 | `pitch` | 同上 |
| 16 | 4 | float32 | `yaw` | 同上 |

---

### 3.4 `0x10` AIM_COMMAND —— 负载 24 字节（接收侧）

| 负载偏移 | 字节 | 类型 | 字段 | 本版处理 |
|---:|---:|---|---|---|
| 0 | 4 | uint32 | `host_session` | **恒填 0**（无会话） |
| 4 | 2 | uint16 | `ref_imu_epoch` | 必须 = 本侧发出的 epoch（1） |
| 6 | 2 | uint16 | `ref_imu_seq` | 必须匹配**本侧实际发出过**的 `0x01` 帧 seq |
| 8 | 4 | uint32 | `ref_sample_time_us` | 同上帧的 `sample_time_us` |
| 12 | 2 | uint16 | `valid_for_ms` | 接受 1–150，**联调默认 100** |
| 14 | 1 | uint8 | `flags` | bit0 CONTROL_ENABLE / bit1 AIM_VALID / bit2 FRICTION_ENABLE / bit3 FIRE_REQUEST |
| 15 | 1 | uint8 | `reserved` | **0** |
| 16 | 4 | float32 | `yaw_error` | rad |
| 20 | 4 | float32 | `pitch_error` | rad |

**样本匹配四元组**：`(boot_id, ref_imu_epoch, ref_imu_seq, ref_sample_time_us)` 必须同时命中本侧缓存（≥40 项 / ≥200 ms）。

**有效期从样本时刻算**：
```text
age_us = signed_mod_delta(now_us, ref_sample_time_us)
允许 = 0 <= age_us <= valid_for_ms * 1000
```

**接受顺序**（V2 §7.4）：帧结构/CRC → `boot_id` → 新 seq → flags/有限值 → 参考缓存/epoch → 样本有效期 → 本地模式与机构保护。

**目标重建**（V2 §7.3，替换现有每周期 `+=`）：
```text
target_yaw   = wrap_pi(reference.yaw + yaw_error)      // 仅在新包通过全部检查时算一次
target_pitch = reference.pitch + pitch_error
每控制周期：remaining = wrap_pi(target_yaw - current_imu.yaw)   // 闭环，不重复累加
```

> **本版 `AIM_VALID`/`CONTROL_ENABLE` 的许可默认关闭** ⇒ 走 `NOT_ALLOWED`。整条"接受/拒绝"链路照常跑通，只是不动云台。轴标定完成后改一个常量开许可。

---

### 3.5 有效标志 `flags`（0x01 / 0x02 负载偏移 1）

| bit | 名称 | 首版 | 为 1 的条件 |
|---:|---|---|---|
| 0 | `ORIENTATION_VALID` | **1** | 初始化完成 + 最近样本通过校验 + 三角度有限且在范围内 + 未超时 |
| 1 | `GYRO_VALID` | 云台 **1** / 底盘 **0** | 三轴角速度有限、未超量程、与姿态同源 |
| 2 | `SAMPLE_TIME_VALID` | **0** | 待 A 实验达标后置 1 |
| 3 | `AXES_VERIFIED` | **0** | 待单轴实验（B1–B4）后置 1；无 `0x03` ⇒ 由 host 侧配置比对 |
| 4–7 | reserved | **0** | |

**注意**：`SAMPLE_TIME_VALID=0` 或 `AXES_VERIFIED=0` ⇒ 按 V2 §4.3 **host 不得把本版数据用于控制**。这是首版的正常状态。

---

### 3.6 `0x81` LINK_STATUS —— 负载 12 字节

| 负载偏移 | 字节 | 类型 | 字段 | 本版值 |
|---:|---:|---|---|---|
| 0 | 4 | uint32 | `host_session` | **0** |
| 4 | 2 | uint16 | `last_aim_seq` | 最后接受的 AIM seq 的**帧头** seq |
| 6 | 1 | uint8 | `state` | 见下 |
| 7 | 1 | uint8 | `last_reject` | 见下；启动为 0 |
| 8 | 2 | uint16 | `aim_age_ms` | 无接受项/超范围 → **65535** |
| 10 | 2 | uint16 | `reserved` | **0** |

**`state` 位**（bit7 固定 0）：

| bit | 名称 |
|---:|---|
| 0 | `SESSION_OPEN` —— 本版恒 0（无会话） |
| 1 | `AIM_FRESH` |
| 2 | `GIMBAL_ALLOWED` |
| 3 | `FRICTION_ALLOWED` |
| 4 | `FIRE_ALLOWED` |
| 5 | `NAV_FRESH` —— 恒 0 |
| 6 | `NAV_ALLOWED` —— 恒 0 |

**`last_reject` 码表**（编号与 V2 §9 一致，host 侧枚举无需改动）：

| 值 | 含义 | 本版是否产生 |
|---:|---|---|
| 0 | OK（启动值，**不表示"刚接受"**） | — |
| 1 | BAD_SESSION | ❌（无会话） |
| 2 | BUSY | ❌ |
| 3 | BAD_REFERENCE | ✅ |
| 4 | STALE | ✅ |
| 5 | INVALID_VALUE | ✅ |
| 6 | NOT_ALLOWED | ✅（本版 AIM 默认走这条） |
| 7 | UNSUPPORTED | ✅（收到 `0x11`/`0x20` 时） |
| 8 | DUPLICATE_OR_OLD | ✅ |
| 9 | LOCAL_FAULT | ❌ |

**上报**：稳定 **10 Hz**；会话/权限改变时立即发布。**ALLOWED 只表示本地许可，不是摩擦轮实际转速或电机位置反馈。**

---

## 四、P1 落地方式（省掉一个任务）

**首版不新建 TX 任务**：

| 项 | 做法 |
|---|---|
| 上行 `0x01` + `0x02` @ 20 Hz | 把 `Encode()` 从 `ArmTask`(100 ms) 挪进 `DecodeTask`(5 ms)，用**独立计数器分频到 50 ms**（`%10`） |
| `0x81` @ 10 Hz | 同一个 `DecodeTask`，另一个计数器分频到 100 ms（`%20`） |
| DMA | 20 Hz 下 46 B 需 0.5 ms，50 ms 周期**无冲突** ⇒ `DMATransmit` 的完成检查**此时不是刚需**，可在提到 100 Hz 之前补 |

**提到 100 Hz 时再做的事**（P1 不做）：独立 TX 调度任务 + 原子快照 + `DMATransmit` 补"上一帧发完"检查。

**接收侧**：改流式解析（`A6 6A` 查找 / 粘包 / 半帧 100 ms 超时重同步 / 长度预检查），并**把接收队列从"深度 1 覆盖写"改成不丢帧** —— 否则 `CONTROL_ENABLE=0` 这类停止指令可能被覆盖掉。

---

## 五、发视觉的差异声明（可原样转发）

1. **帧头、CRC、`0x01`/`0x02`/`0x10`/`0x81` 的字节布局与 V2 逐字节一致，无需你们改任何解析代码。**
2. 本轮**不实现** `0x03`/`0x11`/`0x20`/`0x21`/`0x30`/`0x31`/`0x80`。发来这些类型按 V2 §3 **静默跳过**（长度合法且 CRC 正确时）。
3. **无 ACK**：命令是否被接受，请看 `0x81` 的 `state` 与 `last_reject`（10 Hz）。
4. `host_session` 双方**恒填 0**；`imu_epoch` 恒填 1。
5. 上行为 **20 Hz（云台）/ 20 Hz（底盘）/ 10 Hz（状态）**；稳定后云台提到 100 Hz，届时另行通知。
6. **首版 `SAMPLE_TIME_VALID=0`、`AXES_VERIFIED=0`** ⇒ 按 V2 §4.3 请勿用于控制；本版是只读联调版。等时间戳标定与单轴实验完成后置位。
7. 单位 **rad**；roll/pitch 轴互换**保留在电控侧**，请勿在你们侧再交换一次（V2 §5.1）。
8. 暂缺 `profile_crc32`：我们已不做 `0x03`，配置以双方各自 `README` 为准记录。
