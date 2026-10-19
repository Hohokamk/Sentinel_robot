#include "xuc.h"
#include "label.h"
#include "imu.h"
#include "CRC.h"
#include "control.h"
#include "tim.h"		// TIM::NowUs() —— V2 协议唯一的微秒时基（TIM2 @1MHz）
#include "math.h"

XUC xuc;

/* ==========================================================================
 * 小端序读写（逐字节装载，规避 STM32 未对齐 float 访问，V2 §10.5）
 * ========================================================================== */
static inline void W1(uint8_t* p, uint8_t v) { p[0] = v; }
static inline void W2(uint8_t* p, uint16_t v)
{
	p[0] = (uint8_t)(v & 0xFFu);
	p[1] = (uint8_t)((v >> 8) & 0xFFu);
}
static inline void W4(uint8_t* p, uint32_t v)
{
	p[0] = (uint8_t)(v & 0xFFu);
	p[1] = (uint8_t)((v >> 8) & 0xFFu);
	p[2] = (uint8_t)((v >> 16) & 0xFFu);
	p[3] = (uint8_t)((v >> 24) & 0xFFu);
}
static inline void WF(uint8_t* p, float v) { memcpy(p, &v, 4); }

static inline uint16_t R2(const uint8_t* p)
{
	return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
static inline uint32_t R4u(const uint8_t* p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
	     | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static inline float RF(const uint8_t* p)
{
	float f = 0.f;
	memcpy(&f, p, 4);
	return f;
}

static inline float Deg2Rad(float d) { return d * (XUC_PI / 180.f); }

/* ==========================================================================
 * 初始化
 * ========================================================================== */
void XUC::Init(UART* huart, USART_TypeDef* Instance, uint32_t BaudRate)
{
	huart->Init(Instance, BaudRate).DMARxInit(nullptr).DMATxInit();
	m_uart = huart;
	queue_handler = &huart->UartQueueHandler;

	/* boot_id：本次启动的身份，V2 §3 要求「每次 MCU 复位/上电后不同」。
	 *
	 * ⚠️ 本工程用的 HAL 版本没有暴露 UID getter（无 HAL_GetUIDw0/1/2，
	 *    CMSIS 也没定义 UID_BASE），所以这里用「上电时刻 + 地址 + xorshift」
	 *    合成一个伪随机值。性质说明：
	 *      · 同一次上电内恒定（Init 只调一次）——满足协议用途
	 *      · 跨上电大概率不同（HAL_GetTick 反映上电时长与启动抖动）
	 *      · 不持久化：断电重启后不保证单调递增，V2 也只要求「不同」
	 * 真正需要「跨复位可辨 + 单调」时，改用备份寄存器（RTC->BKPxR）自增，
	 * 前提是板上有 VBAT —— 待确认硬件后再升级。*/
	uint32_t seed = HAL_GetTick()
	              ^ (uint32_t)(uintptr_t)this
	              ^ 0x9E3779B9u;
	seed ^= seed << 13; seed ^= seed >> 17; seed ^= seed << 5;   /* xorshift32 */
	boot_id = seed ? seed : 0xA5A5A5A5u;

	rx_len = 0;
	pending_ms = 0;
	ref_head = 0;
	rx_total = rx_count = crc_err = resync_cnt = 0;
	last_reject = XUC_OK;
	seq_gimbal = seq_chassis = seq_link = 0;
	aim_seq_init = false;
	nav_seq_init = false;
	nav_last_seq = 0;
	prespeedx = prespeedy = 0.f;
	frame_time_us = 0;
	frame_len = 0;

	RevokeAim();
	nav_accepted = false;
	nav_enable = false;
}

/* ==========================================================================
 * 下行：接收 + 流式解析
 * ========================================================================== */
void XUC::Decode()
{
	if (!m_uart || !queue_handler || !*queue_handler) return;

	/* ① 抽干队列。⚠️ 底层队列是「深度 1 + 覆盖写」（usart.cpp 的 ISR），
	 *    两次调用之间到达的帧会互相覆盖 ⇒ 视觉下发率必须 ≤ DecodeTask
	 *    的轮询率（5 ms ⇒ 200 Hz）。这里用 0 超时循环把当前所有可用快照取干净。
	 *
	 * ⚠️ 每次只取【一帧】的元信息：深度 1 的队列在一轮里若收到多帧，最终留下
	 *    的那一帧才是队列里真实的那个。取多次会把计数重复累加，得不到正确配对。*/
	if (xQueueReceive(*queue_handler, m_frame, 0) == pdTRUE)
	{
		++rx_total;

		/* ---- 本帧的真实元信息（与队列里那一帧严格同拍）----
		 * 队列是深度 1 覆盖写 ⇒ ISR 写的 rx_* 成员描述的正是队列里这一帧。
		 * ⚠️ 必须在取到帧的【同一个分支内】读取，且只读一次：
		 *    读到一半若被 ISR 覆盖，就会拿到「A 帧的字节 + B 帧的时间戳」。*/
		frame_time_us = m_uart->GetRxTimeUs();   // ISR 打的 µs 时刻（TIM2 @1MHz）
		frame_len     = m_uart->GetRxLen();      // = UART_MAX_LEN - NDTR

		AppendRx(m_frame, frame_len);            // 只喂真实长度，不喂整个 100B 缓冲
	}

	/* ② 流式解析 */
	ParseRx();
}

void XUC::AppendRx(const uint8_t* p, uint16_t n)
{
	if (n > XUC_RX_BUF) n = XUC_RX_BUF;

	if ((uint32_t)rx_len + (uint32_t)n > XUC_RX_BUF)
	{
		/* 缓冲溢出 ⇒ 清空解析状态并失效控制新鲜度（V2 §10.3） */
		rx_len = 0;
		pending_ms = 0;
		++resync_cnt;
	}

	memcpy(rx_buf + rx_len, p, n);
	rx_len = (uint16_t)(rx_len + n);
}

bool XUC::PayloadLenOk(uint8_t type, uint16_t plen)
{
	switch (type)
	{
	case XUC_T_GIMBAL_IMU:    return plen == XUC_PL_GIMBAL_IMU;
	case XUC_T_CHASSIS_IMU:   return plen == XUC_PL_CHASSIS_IMU;
	case XUC_T_IMU_INFO:      return plen == XUC_PL_IMU_INFO;
	case XUC_T_AIM_COMMAND:   return plen == XUC_PL_AIM_COMMAND;
	case XUC_T_NAV_COMMAND:   return plen == XUC_PL_NAV_COMMAND;
	case XUC_T_SESSION_OPEN:
	case XUC_T_SESSION_CLOSE: return plen == XUC_PL_SESSION;
	case XUC_T_TIME_SYNC_REQ: return plen == XUC_PL_TIME_SYNC_REQ;
	case XUC_T_TIME_SYNC_RSP: return plen == XUC_PL_TIME_SYNC_RSP;
	case XUC_T_ACK:           return plen == XUC_PL_ACK;
	case XUC_T_LINK_STATUS:   return plen == XUC_PL_LINK_STATUS;
	default:                  return false;    /* 未知类型 */
	}
}

void XUC::ParseRx()
{
	uint16_t i = 0;
	const uint32_t now_ms = (uint32_t)xTaskGetTickCount();

	while ((uint16_t)(i + 1u) < rx_len)
	{
		/* 1) 找 magic（A6 6A） */
		if (rx_buf[i] != XUC_MAGIC0 || rx_buf[i + 1u] != XUC_MAGIC1)
		{
			++i;
			continue;
		}

		/* 2) 帧头不完整 */
		if ((uint16_t)(rx_len - i) < XUC_HDR_LEN)
		{
			if (pending_ms == 0u) pending_ms = now_ms;
			else if (now_ms - pending_ms > XUC_HALF_FRAME_MS)
			{
				pending_ms = 0u;
				++resync_cnt;
				++i;                        /* 候选半帧超时 ⇒ 丢首字节重同步 */
				continue;
			}
			break;                          /* 等更多数据 */
		}

		const uint8_t  ver  = rx_buf[i + 2u];
		const uint8_t  type = rx_buf[i + 3u];
		const uint16_t plen = R2(rx_buf + i + 4u);

		/* 3) 版本 / 长度上限预检查 —— 不通过就把当前字节当噪声丢掉 */
		if (ver != XUC_VERSION || plen > XUC_MAX_PAYLOAD)
		{
			++resync_cnt;
			++i;
			continue;
		}

		const uint16_t total = (uint16_t)(XUC_HDR_LEN + plen + XUC_CRC_LEN);

		/* 4) 整帧未到齐 */
		if ((uint16_t)(rx_len - i) < total)
		{
			if (pending_ms == 0u) pending_ms = now_ms;
			else if (now_ms - pending_ms > XUC_HALF_FRAME_MS)
			{
				pending_ms = 0u;
				++resync_cnt;
				++i;
				continue;
			}
			break;
		}

		/* 5) CRC —— 覆盖 [0 .. 12+N-1]，低字节在 [12+N]（复用全局 CRC.h） */
		if (!VerifyCRC16CheckSum(rx_buf + i, total))
		{
			++crc_err;
			++i;                            /* 向后移一字节重新找头（V2 §10.2） */
			continue;
		}

		pending_ms = 0u;

		/* 6) 类型/长度分发 */
		if (!PayloadLenOk(type, plen))
		{
			/* 已知类型但长度不符 ⇒ 协议不一致；未知类型 ⇒ 静默跳过（V2 §3.1）。
			 * 两者边界都由 CRC 背书，整帧跳过是安全的。 */
			if (type == XUC_T_AIM_COMMAND || type == XUC_T_NAV_COMMAND)
				last_reject = XUC_UNSUPPORTED;
			i = (uint16_t)(i + total);
			continue;
		}

		++rx_count;

		const uint16_t frame_seq = R2(rx_buf + i + 6u);
		const uint32_t f_boot    = R4u(rx_buf + i + 8u);
		const uint8_t* pl        = rx_buf + i + XUC_HDR_LEN;

		switch (type)
		{
		case XUC_T_AIM_COMMAND:  HandleAim(pl, frame_seq, f_boot); break;
		case XUC_T_NAV_COMMAND:  HandleNav(pl, frame_seq, f_boot); break;
		default: break;      /* 已知但本版不处理：0x03 / 0x20 / 0x21 / 0x30 / 0x31 / 0x80 */
		}

		i = (uint16_t)(i + total);
	}

	/* 7) 保留未消费部分（含可能的候选半帧首字节） */
	if (i > 0u)
	{
		rx_len = (uint16_t)(rx_len - i);
		if (rx_len > 0u) memmove(rx_buf, rx_buf + i, rx_len);
	}
}

/* ==========================================================================
 * 0x10 AIM_COMMAND
 * 接受顺序（V2 §7.4）：
 *   帧结构/CRC → boot_id → 新 seq → flags/有限值 → 参考缓存/epoch
 *   → 样本有效期 → 本地模式与机构保护
 * ========================================================================== */
void XUC::HandleAim(const uint8_t* pl, uint16_t frame_seq, uint32_t f_boot)
{
	aim_rx_tick = xTaskGetTickCount();

	/* --- boot_id --- */
	if (f_boot != boot_id) { last_reject = XUC_BAD_SESSION; return; }

	const uint16_t ref_epoch = R2(pl + 4u);
	const uint16_t ref_seq   = R2(pl + 6u);
	const uint32_t ref_time  = R4u(pl + 8u);
	const uint16_t v4ms      = R2(pl + 12u);
	const uint8_t  flags     = pl[14u];
	const float    yaw_err   = RF(pl + 16u);
	const float    pit_err   = RF(pl + 20u);

	/* --- flags / 有限值 --- */
	if ((flags & (uint8_t)~XUC_AIMF_MASK) != 0u) { last_reject = XUC_INVALID_VALUE; return; }
	if (!isfinite(yaw_err) || !isfinite(pit_err)) { last_reject = XUC_INVALID_VALUE; return; }

	/* --- 新序号：0 < (uint16)(new-last) < 32768 --- */
	if (aim_seq_init)
	{
		const uint16_t d = (uint16_t)(frame_seq - aim_last_seq);
		if (d == 0u || d >= 32768u) { last_reject = XUC_DUPLICATE_OR_OLD; return; }
	}

	/* --- CONTROL_ENABLE=0：显式停止指令。
	 *     按 V2 §7.4「不要求有效参考样本，立即撤销许可」，且不能因 IMU 失效而拒绝。*/
	if ((flags & XUC_AIM_CONTROL_EN) == 0u)
	{
		if (flags != 0u || yaw_err != 0.f || pit_err != 0.f)
		{
			last_reject = XUC_INVALID_VALUE;
			return;
		}
		RevokeAim();
		aim_accepted = false;
		aim_seq_init = true;
		aim_last_seq = frame_seq;
		last_reject = XUC_OK;
		return;
	}

	/* --- valid_for_ms 范围（1..150，推荐 100） --- */
	if (v4ms < 1u || v4ms > 150u) { last_reject = XUC_INVALID_VALUE; return; }

	/* --- 参考样本四元组匹配（boot_id 已查，这里查 epoch/seq/time） --- */
	if (ref_epoch != XUC_EPOCH) { last_reject = XUC_BAD_REFERENCE; return; }
	const XUC_RefSample* s = FindRef(ref_epoch, ref_seq, ref_time);
	if (s == nullptr) { last_reject = XUC_BAD_REFERENCE; return; }

	/* --- 样本有效期：从样本时刻算，不从收包时算 ---
	 * ⚠️ 用 TIM::NowUs()（真 1 µs），不是 xTaskGetTickCount()*1000（1 ms 量化）。
	 *    ref_time 是上位机原样回传我们上行帧里的 sample_time_us，
	 *    那个值是 usart.cpp 的 ISR 用 TIM::NowUs() 打的 —— 同一时基才能相减。*/
	const uint32_t now_us = TIM::NowUs();
	const uint32_t age_us = (uint32_t)(now_us - ref_time);
	if (age_us >= 0x80000000u) { last_reject = XUC_STALE; return; }          /* 未来时刻 */
	if (age_us > (uint32_t)v4ms * 1000u) { last_reject = XUC_STALE; return; }

	/* --- 前置全部通过：重建目标（V2 §7.3，只在新命令通过检查时算一次） --- */
	aim_yaw_target   = WrapPi(s->yaw + yaw_err);
	aim_pitch_target = s->pitch + pit_err;
	aim_ref_time_us  = ref_time;
	aim_valid_for_ms = v4ms;
	aim_control_en   = true;
	aim_valid        = (flags & XUC_AIM_VALID) != 0u;
	aim_friction_en  = (flags & XUC_AIM_FRICTION_EN) != 0u;
	aim_fire_req     = (flags & XUC_AIM_FIRE_REQ) != 0u;
	aim_accepted     = true;
	aim_seq_init     = true;
	aim_last_seq     = frame_seq;

	/* --- 本地模式与机构保护（最后一步） --- */
	if (ctrl.mode != CONTROL::AUTO) { last_reject = XUC_NOT_ALLOWED; return; }
#if XUC_ALLOW_CONTROL == 0
	/* 只读联调版：目标已重建但不下发，明确告知上位机「本地不允许」 */
	last_reject = XUC_NOT_ALLOWED;
#else
	last_reject = XUC_OK;
#endif
}

/* ==========================================================================
 * 0x11 NAV_COMMAND
 * ⚠️ 首版降级：不做 ref_mcu_time_us 的时间映射校验（0x30/0x31 未实现），
 *    改用本地租期 —— 收到后 valid_for_ms 内有效，超时即停。
 * ========================================================================== */
void XUC::HandleNav(const uint8_t* pl, uint16_t frame_seq, uint32_t f_boot)
{
	if (f_boot != boot_id) { last_reject = XUC_BAD_SESSION; return; }

	const uint16_t v4ms  = R2(pl + 8u);
	const uint8_t  flags = pl[10u];
	const float    vx    = RF(pl + 12u);
	const float    vy    = RF(pl + 16u);
	const float    wz    = RF(pl + 20u);

	if ((flags & (uint8_t)~XUC_NAVF_MASK) != 0u) { last_reject = XUC_INVALID_VALUE; return; }
	if (!isfinite(vx) || !isfinite(vy) || !isfinite(wz)) { last_reject = XUC_INVALID_VALUE; return; }

	if (nav_seq_init)
	{
		const uint16_t d = (uint16_t)(frame_seq - nav_last_seq);
		if (d == 0u || d >= 32768u) { last_reject = XUC_DUPLICATE_OR_OLD; return; }
	}

	/* --- NAV_ENABLE=0：速度必须全零，立即停（不要求可靠时间，V2 §8） --- */
	if ((flags & XUC_NAV_ENABLE) == 0u)
	{
		if (vx != 0.f || vy != 0.f || wz != 0.f) { last_reject = XUC_INVALID_VALUE; return; }
		nav_accepted = false;
		nav_enable = false;
		nav_vx = nav_vy = nav_wz = 0.f;
		nav_seq_init = true;
		nav_last_seq = frame_seq;
		last_reject = XUC_OK;
		return;
	}

	if (v4ms < 1u || v4ms > 150u) { last_reject = XUC_INVALID_VALUE; return; }
	if (ctrl.mode != CONTROL::AUTO) { last_reject = XUC_NOT_ALLOWED; return; }

	nav_vx = vx;
	nav_vy = vy;
	nav_wz = wz;
	nav_valid_for_ms = v4ms;
	nav_enable = true;
	nav_accepted = true;
	nav_accept_tick = xTaskGetTickCount();
	nav_seq_init = true;
	nav_last_seq = frame_seq;
	last_reject = XUC_OK;
}

/* ==========================================================================
 * 参考样本缓存
 * ========================================================================== */
void XUC::PushRef(uint16_t epoch, uint16_t seq, uint32_t t_us, float yaw, float pitch)
{
	XUC_RefSample& s = ref_cache[ref_head];
	s.boot_id = boot_id;
	s.time_us = t_us;
	s.yaw = yaw;
	s.pitch = pitch;
	s.epoch = epoch;
	s.seq = seq;
	s.valid = 1u;
	ref_head = (uint16_t)((ref_head + 1u) % XUC_REF_CACHE_N);
}

const XUC_RefSample* XUC::FindRef(uint16_t epoch, uint16_t seq, uint32_t t_us) const
{
	for (uint16_t i = 0; i < XUC_REF_CACHE_N; ++i)
	{
		const XUC_RefSample& s = ref_cache[i];
		if (s.valid && s.boot_id == boot_id && s.epoch == epoch
			&& s.seq == seq && s.time_us == t_us)
			return &s;
	}
	return nullptr;
}

/* ==========================================================================
 * 本侧许可
 * ========================================================================== */
void XUC::RevokeAim()
{
	aim_accepted = false;
	aim_control_en = false;
	aim_valid = false;
	aim_friction_en = false;
	aim_fire_req = false;
}

bool XUC::AimReady()
{
	if (!aim_accepted || !aim_control_en) return false;
	const uint32_t age = (uint32_t)(TIM::NowUs() - aim_ref_time_us);   /* 真 µs */
	if (age >= 0x80000000u) return false;
	if (age > (uint32_t)aim_valid_for_ms * 1000u) return false;
#if XUC_ALLOW_CONTROL == 0
	return false;                        /* 只读联调版：本地不放行 */
#else
	return true;
#endif
}

bool XUC::NavReady()
{
	if (!nav_accepted || !nav_enable) return false;
	if ((uint32_t)(xTaskGetTickCount() - nav_accept_tick) > (uint32_t)nav_valid_for_ms)
		return false;
#if XUC_ALLOW_NAV == 0
	return false;
#else
	return true;
#endif
}

uint16_t XUC::AimAgeMs()
{
	if (!aim_accepted) return 0xFFFFu;
	const uint32_t age = (uint32_t)(TIM::NowUs() - aim_ref_time_us);   /* 真 µs */
	if (age >= 0x80000000u) return 0xFFFFu;
	const uint32_t ms = age / 1000u;
	return (ms > 65534u) ? 0xFFFFu : (uint16_t)ms;
}

uint8_t XUC::LinkState()
{
	uint8_t s = 0u;
	if (aim_accepted && AimAgeMs() != 0xFFFFu) s |= XUC_ST_AIM_FRESH;
	if (AimReady())        s |= XUC_ST_GIMBAL_ALLOW;
	if (aim_accepted && aim_friction_en && AimReady()) s |= XUC_ST_FRICTION_ALLOW;
	if (aim_accepted && aim_fire_req && AimReady())    s |= XUC_ST_FIRE_ALLOW;
	if (NavReady())        s |= XUC_ST_NAV_FRESH | XUC_ST_NAV_ALLOW;
	return s;
}

/* ==========================================================================
 * 上行：组帧 + 发送
 * ========================================================================== */
uint16_t XUC::BuildFrame(uint8_t type, const uint8_t* payload, uint16_t plen,
                         uint16_t seq, uint8_t* out)
{
	out[0] = XUC_MAGIC0;
	out[1] = XUC_MAGIC1;
	out[2] = XUC_VERSION;
	out[3] = type;
	W2(out + 4u, plen);
	W2(out + 6u, seq);
	W4(out + 8u, boot_id);
	if (plen > 0u && payload != nullptr) memcpy(out + XUC_HDR_LEN, payload, plen);

	const uint16_t total = (uint16_t)(XUC_HDR_LEN + plen + XUC_CRC_LEN);
	AppendCRC16CheckSum(out, total);     /* 覆盖 [0 .. 12+N-1]，低字节落 [12+N] */
	return total;
}

uint8_t XUC::GimbalFlags()
{
	uint8_t f = 0u;
	if (imu_pantile.IsValid()) f |= XUC_F_ORIENT_VALID;

	const float gx = imu_pantile.GetGyroRoll();
	const float gy = imu_pantile.GetGyroPitch();
	const float gz = imu_pantile.GetGyroYaw();
	if (imu_pantile.IsValid() && isfinite(gx) && isfinite(gy) && isfinite(gz))
		f |= XUC_F_GYRO_VALID;

	/* 🔴 XUC_F_SAMPLE_VALID 仍固定 0 —— 这是【诚实标注】，不是遗漏。
	 * V2 §6.1 的定义：「sample_time_us 是真实采样时刻」且「误差有界 ≤2ms」时
	 * 才能置位。上报的 t_us 现在是【MCU 收到该 IMU 报文的时刻】（ISR 打戳），
	 * 而 CH010 的 system_time 才是采样时刻。二者的映射关系需要做线性回归
	 * 定出「恒定延迟 + 残差 RMS」（A 实验），达标后才可置 1。
	 * 在此之前按 V2 §6.1 声明 time_source = 2（仅到达时刻），
	 * 上位机【不得】把本数据用于基于样本时刻的控制。
	 * ⇒2026-10-18 决定【跳过 A 实验】：时间戳误差源不可修且无害，
	 *    实测也无法改变任何代码。这个 bit 因此【永久】保持 0。*/

	/* ✅ XUC_F_AXES_VERIFIED 置 1 —— 2026-10-18 实测支撑（不是"跑通了所以置1"）：
	 *   · 枪口 IMU：逆时针 yaw 增大、 低头 pitch 增大（用户实测）
	 *   · 底盘 IMU：只能动 yaw、逆时针增大（用户实测）
	 *   · 云台方向：右摇杆上推=抬头、左摇杆右推=右转（用户确认）
	 *   · 底盘方向：遥控器正常（用户确认）
	 * 由「手动路径与自动路径共用同一执行量」反推：mark_pitch/setangle 增大
	 * 分别等价于枪口低头 / 逆时针，与 IMU 正方向【同向】⇒ ApplyAim 的
	 * 两个 += 符号正确。详见 .workbuddy/memory/2026-10-18.md。
	 * ⚠️ 上车后若看到误差发散 ⇒ 实测有误，此位须撤回并给ApplyAim 对应行取负。*/
	f |= XUC_F_AXES_VERIFIED;

	return f;
}

uint8_t XUC::ChassisFlags()
{
	uint8_t f = 0u;
	if (imu_chassis.IsValid()) f |= XUC_F_ORIENT_VALID;
	/* bit1 GYRO_VALID 在底盘帧固定为 0（V2 §4.2） */
	return f;
}

void XUC::Encode()
{
	if (!m_uart) return;

	/* ---- 上报的 sample_time_us ----
	 * ✅ 现在用【ISR 打的到达时刻】（uart4.rx_time_us），不再是 DecodeTask 的
	 *    轮询时刻。差别有多大：DecodeTask 是 5 ms 周期且用 vTaskDelay(5)
	 *    （= 5 ms + 执行时间，不定），所以旧值的抖动是【0~5 ms + 漂移】，
	 *    已经超过 V2 要求的 2 ms 预算；改成 ISR 打戳后抖动只剩 UART 传输期间
	 *    的那点变化（IDLE 中断在最后一个字节到齐时才触发，天然贴近期末）。
	 *
	 * ⚠️ 仍然不是「模块的采样时刻」—— 那是 CH010 包里的 system_time。
	 *    两者关系待 A 实验标定（线性回归求恒定延迟 + 残差 RMS），
	 *    在此之前 flags 里的 SAMPLE_TIME_VALID 保持 0（见 GimbalFlags）。
	 *
	 * ⚠️ TIM2 未使能（main() 里没调 timer.UsInit）时本值恒为 0，
	 *    上位机看到 time_us 恒 0 就知道微秒时基没就绪。
	 *
	 * 🔴 这里【不做】"frame_time_us 为 0 就退化成任务调用时刻"的兜底。
	 *    frame_time_us = ISR 打的到达时刻（真 µs）；任务调用时刻是另一个值，
	 *    两者混进同一个字段会让上位机拿到「有时是到达时刻、有时是组帧时刻」
	 *    的混合语义。A 实验拿这批数据做线性回归求恒定延迟时，
	 *    混进来的组帧时刻会变成一个【假的截距】（Encode 的 20 ms 周期抖动
	 *    正好落在残差里），直接把 RMS 误差撑爆。
	 *    时基没就绪就该如实报 0，让上层识别，而不是悄悄降级。*/
	const uint32_t t_us = frame_time_us;

	uint32_t w = 0u;

	/* ---------------- 0x01 GIMBAL_IMU（负载 32） ---------------- */
	{
		uint8_t pl[XUC_PL_GIMBAL_IMU];
		const float r = Deg2Rad(imu_pantile.GetAngleRoll());
		const float p = Deg2Rad(imu_pantile.GetAnglePitch());
		const float y = Deg2Rad(imu_pantile.GetAngleYaw());

		W1(pl + 0u,  XUC_IMU_ID_GIMBAL);
		W1(pl + 1u,  GimbalFlags());
		W2(pl + 2u,  XUC_EPOCH);
		W4(pl + 4u,  t_us);
		WF(pl + 8u,  r);
		WF(pl + 12u, p);
		WF(pl + 16u, y);
		WF(pl + 20u, Deg2Rad(imu_pantile.GetGyroRoll()));
		WF(pl + 24u, Deg2Rad(imu_pantile.GetGyroPitch()));
		WF(pl + 28u, Deg2Rad(imu_pantile.GetGyroYaw()));

		const uint16_t seq = seq_gimbal++;
		w += BuildFrame(XUC_T_GIMBAL_IMU, pl, XUC_PL_GIMBAL_IMU, seq, tx_buf + w);

		/* 参考样本内容必须与实际发出的帧完全一致（V2 §7.2） */
		PushRef(XUC_EPOCH, seq, t_us, y, p);
	}

	/* ---------------- 0x02 CHASSIS_IMU（负载 20） ---------------- */
	{
		uint8_t pl[XUC_PL_CHASSIS_IMU];
		W1(pl + 0u,  XUC_IMU_ID_CHASSIS);
		W1(pl + 1u,  ChassisFlags());
		W2(pl + 2u,  XUC_EPOCH);
		/* ⚠️ 底盘 IMU 走 UART5，与上位机的 UART4 不是同一条中断 ⇒
		 *    没有对应的"到达上位机侧"的时刻。这里仍用 t_us（上位机视角，
		 *    即"这批数据何时被我们打包"），保持两块时间同源，便于上位机做时序对齐。
		 *    严格做法是另读 imu_chassis 侧的 ISR 戳（待两路都能独立打戳后补）。*/
		W4(pl + 4u,  t_us);
		WF(pl + 8u,  Deg2Rad(imu_chassis.GetAngleRoll()));
		WF(pl + 12u, Deg2Rad(imu_chassis.GetAnglePitch()));
		WF(pl + 16u, Deg2Rad(imu_chassis.GetAngleYaw()));

		const uint16_t seq = seq_chassis++;
		w += BuildFrame(XUC_T_CHASSIS_IMU, pl, XUC_PL_CHASSIS_IMU, seq, tx_buf + w);
	}

	/* ---------------- 0x81 LINK_STATUS（负载 12） ---------------- */
	{
		uint8_t pl[XUC_PL_LINK_STATUS];
		W4(pl + 0u,  0u);                    /* host_session：无会话 ⇒ 0 */
		W2(pl + 4u,  aim_last_seq);
		W1(pl + 6u,  LinkState());
		W1(pl + 7u,  last_reject);
		W2(pl + 8u,  AimAgeMs());
		W2(pl + 10u, 0u);                    /* reserved */

		const uint16_t seq = seq_link++;
		w += BuildFrame(XUC_T_LINK_STATUS, pl, XUC_PL_LINK_STATUS, seq, tx_buf + w);
	}

	/* 三帧拼成一段连续缓冲，一次 DMA 发出（DMATransmit 不可重入） */
	m_uart->DMATransmit(tx_buf, w);
}
