#pragma once

/* ============================================================================
 * XUC —— 上下位机通信（V2 协议 · 精简版 P1）
 *
 * 帧格式（全部小端）：
 *   magic(2) A6 6A | version(1) | type(1) | payload_len(2)
 *   | sequence(2) | boot_id(4) | payload(N) | crc16(2)
 *   总长 = 14 + N，最大负载 128
 *
 * CRC：CRC-16/MCRF4XX，init 0xFFFF，反射多项式 0x8408，无最终异或，
 *      低字节先发，覆盖 [0 .. 12+N-1]。复用全局 CRC.h（算法零改动）。
 *
 * 本版实现的消息：0x01 GIMBAL_IMU / 0x02 CHASSIS_IMU / 0x10 AIM_COMMAND /
 *                 0x11 NAV_COMMAND / 0x81 LINK_STATUS
 * 其余类型（0x03/0x20/0x21/0x30/0x31/0x80）不实现，收到即静默跳过
 * （V2 §3.1：长度合法且 CRC 正确时跳过）。
 *
 * ⚠️ 本版是「只读联调版」：XUC_ALLOW_CONTROL = 0
 *    ⇒ 0x10/0x11 只走「接受/拒绝」链路，不驱动云台与底盘。
 *    ⇒ 上报的 flags 中 SAMPLE_TIME_VALID / AXES_VERIFIED 恒为 0，
 *      按 V2 §4.3 上位机不得把本版数据用于控制。
 *    轴标定（B 实验）与时间戳标定（A 实验）完成后，改下面的宏即可放行。
 * ============================================================================ */

#include "stm32f4xx_hal.h"
#include "usart.h"
#include "FreeRTOS.h"
#include <cstring>
#include <cstddef>
#include <cmath>

#ifndef XUC_PI
#define XUC_PI 3.14159265358979323846f
#endif

/* ---------------------------------------------------------------- 帧常量 */
#define XUC_MAGIC0            0xA6u
#define XUC_MAGIC1            0x6Au
#define XUC_VERSION           0x02u
#define XUC_HDR_LEN           12u          /* magic2+ver1+type1+len2+seq2+boot4 */
#define XUC_CRC_LEN           2u
#define XUC_MAX_PAYLOAD       128u
#define XUC_MAX_FRAME         (XUC_HDR_LEN + XUC_MAX_PAYLOAD + XUC_CRC_LEN)  /* 142 */

/* ------------------------------------------------------------ 消息类型 */
#define XUC_T_GIMBAL_IMU      0x01u
#define XUC_T_CHASSIS_IMU     0x02u
#define XUC_T_IMU_INFO        0x03u        /* 不实现 */
#define XUC_T_AIM_COMMAND     0x10u
#define XUC_T_NAV_COMMAND     0x11u
#define XUC_T_SESSION_OPEN    0x20u        /* 不实现 */
#define XUC_T_SESSION_CLOSE   0x21u        /* 不实现 */
#define XUC_T_TIME_SYNC_REQ   0x30u        /* 不实现 */
#define XUC_T_TIME_SYNC_RSP   0x31u        /* 不实现 */
#define XUC_T_ACK             0x80u        /* 不实现 */
#define XUC_T_LINK_STATUS     0x81u

/* -------------------------------------------------------- 负载长度（N） */
#define XUC_PL_GIMBAL_IMU     32u
#define XUC_PL_CHASSIS_IMU    20u
#define XUC_PL_IMU_INFO       32u
#define XUC_PL_AIM_COMMAND    24u
#define XUC_PL_NAV_COMMAND    24u
#define XUC_PL_SESSION        4u
#define XUC_PL_TIME_SYNC_REQ  0u
#define XUC_PL_TIME_SYNC_RSP  12u
#define XUC_PL_ACK            8u
#define XUC_PL_LINK_STATUS    12u

/* ------------------------------------------- 0x01 / 0x02 负载 flags 位 */
#define XUC_F_ORIENT_VALID    0x01u
#define XUC_F_GYRO_VALID      0x02u        /* 底盘帧固定 0 */
#define XUC_F_SAMPLE_VALID    0x04u        /* 永久 0：A 实验已决定跳过（时间戳误差不可修且无害） */
#define XUC_F_AXES_VERIFIED   0x08u        /* 2026-10-18 实测支撑后置 1 */
#define XUC_F_MASK            0x0Fu

/* ------------------------------------------------- 0x10 负载 flags 位 */
#define XUC_AIM_CONTROL_EN    0x01u
#define XUC_AIM_VALID         0x02u
#define XUC_AIM_FRICTION_EN   0x04u
#define XUC_AIM_FIRE_REQ      0x08u
#define XUC_AIMF_MASK         0x0Fu

/* ------------------------------------------------- 0x11 负载 flags 位 */
#define XUC_NAV_ENABLE        0x01u
#define XUC_NAVF_MASK         0x01u

/* -------------------------------------------------------- 0x81 state 位 */
#define XUC_ST_SESSION_OPEN   0x01u        /* 首版恒 0 */
#define XUC_ST_AIM_FRESH      0x02u
#define XUC_ST_GIMBAL_ALLOW   0x04u
#define XUC_ST_FRICTION_ALLOW 0x08u
#define XUC_ST_FIRE_ALLOW     0x10u
#define XUC_ST_NAV_FRESH      0x20u
#define XUC_ST_NAV_ALLOW      0x40u

/* ------------------------------------ 拒绝原因 / ACK result（V2 §9） */
enum XUC_RESULT
{
	XUC_OK                = 0,
	XUC_BAD_SESSION       = 1,
	XUC_BUSY              = 2,
	XUC_BAD_REFERENCE     = 3,
	XUC_STALE             = 4,
	XUC_INVALID_VALUE     = 5,
	XUC_NOT_ALLOWED       = 6,
	XUC_UNSUPPORTED       = 7,
	XUC_DUPLICATE_OR_OLD  = 8,
	XUC_LOCAL_FAULT       = 9
};

/* ============================================================ 本版开关 */
/* 0 = 只读联调（不发控制）；1 = 允许 0x10/0x11 驱动执行器。
 *
 * 🔴 置 1 的前置条件（2026-10-18 已全部完成，依据见 .workbuddy/memory/2026-10-18.md）：
 *   ✅ 云台方向 —— 用户实测「右摇杆上推=抬头、左摇杆右推=右转」，与 IMU 实测
 *      「逆时针 yaw 增大 / 低头 pitch 增大」反推出的执行侧方向【同向】
 *      ⇒ ApplyAim 里两个 += 的符号正确，无需取负。
 *   ✅ IMU 轴向 —— 用户实测两个 IMU 均逆时针 yaw 增大；枪口低头 pitch 增大；
 *      底盘只能动 yaw ⇒ Z 轴朝上、yaw 绕铅垂轴 ⇒ AXES 可信。
 *   ✅ 底盘方向 —— 用户确认遥控器底盘正常 ⇒ nav_vx+ = 前进、nav_vy+ = 左移。
 *   ❌ A 实验（sample_time_us 误差界 ≤ 2ms）—— 【已决定跳过】。
 *      理由：时间戳的抖动源不可修且无害（晶振偏差、UART 传输时间都是常数偏移，
 *      会被 PushRef/age_us 一起消掉），实测出来也无法改变任何代码。
 *      ⇒ 对应地SAMPLE_VALID 恒为 0，这是【诚实的标注】而非缺陷。
 *      上位机不得基于 sample_time_us 做外推，只能用姿态 + gyro 自己积分。
 *
 * ⚠️ 上车第一次放行的判据：清空云台周围场地，看 ApplyAim 的误差是否单调收敛。
 *    发散 / 顶到 dm_pitch_min|max(-0.55/+0.4 rad) ⇒ 符号反了，回这里给对应行取负。*/
#define XUC_ALLOW_CONTROL     1
/* 底盘导航单独开关：速度指令本身是 fail-safe（丢包/超时即停），
 * 不依赖 IMU 轴标定，只依赖 K_NAV 实测 ⇒ 可以先放行。
 * ⚠️ K_NAV 仍是【估算值】，必须实测标定后才算可用（见 docs_通信协议.md §2.7）。*/
#define XUC_ALLOW_NAV         1
#define XUC_EPOCH             1u           /* 姿态参考系代号，恒 1 */
#define XUC_IMU_ID_GIMBAL     1u
#define XUC_IMU_ID_CHASSIS    2u
#define XUC_REF_CACHE_N       48u          /* 参考样本环形缓存（≥40） */
#define XUC_RX_BUF            512u
#define XUC_TX_BUF            512u
#define XUC_HALF_FRAME_MS     100u         /* 候选半帧超时 ⇒ 丢首字节重同步 */

/* ------------------------------------------------------- 参考样本（V2 §7.2） */
struct XUC_RefSample
{
	uint32_t boot_id;
	uint32_t time_us;
	float    yaw;          /* rad，与已发出的 0x01 完全一致 */
	float    pitch;        /* rad */
	uint16_t epoch;
	uint16_t seq;
	uint8_t  valid;
	uint8_t  pad[3];
};

class XUC
{
public:
	/* ================= 下行解析结果（供 RC::OnPC 消费） ================= */

	/* --- 0x10 AIM_COMMAND --- */
	float    aim_yaw_target   = 0.f;   /* rad，已重建的绝对目标 = ref.yaw + yaw_error */
	float    aim_pitch_target = 0.f;   /* rad */
	uint32_t aim_ref_time_us  = 0;     /* 参考样本的 sample_time_us */
	uint16_t aim_valid_for_ms = 0;
	uint16_t aim_last_seq     = 0;
	bool     aim_accepted     = false; /* 最近一帧通过全部检查 */
	bool     aim_control_en   = false;
	bool     aim_valid        = false;
	bool     aim_friction_en  = false;
	bool     aim_fire_req     = false;
	uint32_t aim_rx_tick      = 0;     /* 最近收到结构正确 0x10 的时刻 */

	/* --- 0x11 NAV_COMMAND --- */
	float    nav_vx = 0.f;             /* m/s，车体前向 */
	float    nav_vy = 0.f;             /* m/s，车体左向 */
	float    nav_wz = 0.f;             /* rad/s，绕 +Z 左转为正（首版未使用） */
	uint16_t nav_valid_for_ms = 0;
	bool     nav_enable   = false;
	bool     nav_accepted = false;     /* 最近一帧通过全部检查 */
	uint32_t nav_accept_tick = 0;

	/* --- 底盘速度平滑（递推量，跨周期保留） --- */
	float    prespeedx = 0.f;          /* 本周期实际允许的 rpm */
	float    prespeedy = 0.f;

	/* ===================== 诊断计数（watch 用） ===================== */
	uint16_t rx_total    = 0;          /* 从队列取到快照的次数 */
	uint16_t rx_count    = 0;          /* 结构+CRC 通过且类型/长度已知的帧数 */
	uint16_t crc_err     = 0;          /* magic 命中但 CRC 失败 */
	uint16_t resync_cnt  = 0;          /* 重同步/溢出次数 */
	uint8_t  last_reject = XUC_OK;     /* 最近拒绝原因（0 = 启动值，不代表刚接受） */
	uint32_t boot_id     = 0;

	/* ============================ 接口 ============================ */
	void Init(UART* huart, USART_TypeDef* Instance, uint32_t BaudRate);
	void Decode();                     /* DecodeTask 调用 */
	void Encode();                     /* 上行组帧 + 发送 */

	/* --- 供 RC::OnPC 查询 --- */
	bool     AimReady();               /* 控制许可：已接受 + 未过期 + 本版允许 */
	bool     NavReady();
	uint16_t AimAgeMs();
	uint8_t  LinkState();
	void     RevokeAim();              /* 撤销自动瞄准/摩擦轮/开火许可 */

	/* --- 工具 --- */
	static float WrapPi(float a)
	{
		while (a >=  XUC_PI) a -= 2.f * XUC_PI;
		while (a <  -XUC_PI) a += 2.f * XUC_PI;
		return a;
	}

private:
	UART*           m_uart = nullptr;
	QueueHandle_t*  queue_handler = nullptr;

	uint8_t  m_frame[UART_MAX_LEN]{};  /* 队列快照落点 */
	uint8_t  rx_buf[XUC_RX_BUF]{};     /* 流式接收缓冲 */
	uint16_t rx_len = 0;
	uint32_t pending_ms = 0;           /* 候选半帧起始时刻（HAL_GetTick） */
	uint8_t  tx_buf[XUC_TX_BUF]{};     /* 上行组帧缓冲（一次 DMA 发全部） */

	uint16_t seq_gimbal = 0;
	uint16_t seq_chassis = 0;
	uint16_t seq_link = 0;

	bool     aim_seq_init = false;
	bool     nav_seq_init = false;
	uint16_t nav_last_seq = 0;

	XUC_RefSample ref_cache[XUC_REF_CACHE_N]{};
	uint16_t ref_head = 0;

	/* 本轮 Decode 收到的这一帧的元信息（取自 UART 对象，与队列内容同拍）。
	 * ⚠️ 由 Decode() 在取到帧的同一分支内写入，且【只写一次】——
	 *    队列是深度 1 覆盖写，读到一半被 ISR 覆盖会得到
	 *    「A 帧的字节 + B 帧的时间戳」这种错配。*/
	uint32_t frame_time_us = 0;      /* ISR 打的 µs 到达时刻（TIM2 @1MHz） */
	uint16_t frame_len     = 0;      /* = UART_MAX_LEN - NDTR，真实字节数 */

	/* --- 内部 --- */
	/* ⚠️ 原来的 `uint32_t NowUs() { return xTaskGetTickCount() * 1000u; }`
	 * 已删除，两个原因：
	 *   ① 编译不过：xuc.h 只 include 了 FreeRTOS.h，没 include task.h，
	 *      xTaskGetTickCount 不可见（task.h 声明这个符号）。
	 *   ② 更根本：它【分辨率是 1 ms，不是 1 µs】。tick 本身就是 1 ms 粒度，
	 *      乘 1000 只是把毫秒换了标签，没有增加任何精度。
	 *      而 V2 的时间误差预算是 2 ms —— 用一个 1 ms 量化的时间源去判
	 *      样本新鲜度，等于凭空吃掉一半预算。
	 *
	 * ⇒ 统一用 TIM::NowUs()（TIM2 @1 MHz，真 1 µs）。
	 *    同一份协议里只能有一个时基：sample_time_us 已经是 usart.cpp 的 ISR
	 *    用 TIM::NowUs() 打的，age_us 再用另一个来源就是两套时基混算。*/
	uint16_t BuildFrame(uint8_t type, const uint8_t* payload, uint16_t plen,
	                    uint16_t seq, uint8_t* out);
	static bool PayloadLenOk(uint8_t type, uint16_t plen);

	void AppendRx(const uint8_t* p, uint16_t n);
	void ParseRx();
	void HandleAim(const uint8_t* pl, uint16_t frame_seq, uint32_t f_boot);
	void HandleNav(const uint8_t* pl, uint16_t frame_seq, uint32_t f_boot);

	void PushRef(uint16_t epoch, uint16_t seq, uint32_t t_us, float yaw, float pitch);
	const XUC_RefSample* FindRef(uint16_t epoch, uint16_t seq, uint32_t t_us) const;

	uint8_t GimbalFlags();
	uint8_t ChassisFlags();
};

/* 帧长上限自检：三帧同发不得超过发送缓冲 */
static_assert(XUC_MAX_FRAME * 3u <= XUC_TX_BUF, "tx_buf too small");

extern XUC xuc;
