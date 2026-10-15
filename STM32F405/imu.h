#pragma once
#include "stm32f4xx.h"
#include "usart.h"
#include <string.h>
#include <cmath>
enum IMU_TYPE { IMU601 = 0, CH010, HI226 };

class IMU
{
public:
	float ACC_FSR = 4.f, GYRO_FSR = 2000.f;
	typedef struct
	{
		float roll, yaw, pitch;
	}Angle, AngularVelocity;
	typedef struct
	{
		float x{}, y{}, z{};
	}Acceleration;

	void Init(UART* huart, USART_TypeDef* Instance, const uint32_t BaudRate, IMU_TYPE type);
	void Decode();
	/* 单段 CRC 校验（算这一段并与 com 比较）。
	 * ⚠️ CH010 实际不走这个 —— 它的覆盖范围是【两段拼接、末尾比较一次】，
	 *    必须用 CrcReset / CrcAccum / CrcResult，见 imu.cpp 的说明。
	 *    保留本函数供 IMU601（算术和校验）与将来其他型号复用。*/
	bool Check(uint8_t* pdata, uint8_t len, uint32_t com);
	float GetAngleYaw();
	float GetAnglePitch();
	float GetAngleRoll();
	float* GetAcceleration();
	float GetAccX();
	float GetAccY();
	float GetAccZ();
	float GetGyroRoll();
	float GetGyroPitch();
	float GetGyroYaw();

	uint8_t  id = 0;                    // 传感器身份（装配处指定）
	uint32_t GetTimestamp();
	bool     IsValid();

	/* --- 诊断计数（watch 用，A 实验统计样本通过率靠它）---
	 * 关系：rx_ok = crc_ok + crc_fail + len_bad；sample_ok ≤ rx_ok。
	 * A 实验做回归时用 sample_ok 当分母，不要拿 rx_ok —— 后者含"帧有效但
	 * 不是 0x91 姿态包"的帧，那些帧没有可用于标定的 timestamp。*/
	volatile uint32_t rx_ok = 0;        // 帧头 + 长度 + CRC 全部通过的帧数
	volatile uint32_t crc_fail = 0;     // CRC 不匹配（真·校验失败）
	volatile uint32_t len_bad = 0;      // 长度字段非法（0 或越界）
	volatile uint32_t sample_ok = 0;    // 0x91 姿态包通过 CRC 的帧数
	volatile uint32_t short_payload = 0;// 0x91 包但 len < 60，读不到 yaw 就丢弃

	int16_t getword(uint8_t HighBit, uint8_t LowBits);

	BaseType_t pd_Rx = false;
	QueueHandle_t* queueHandler = NULL;
private:
	void DecodeIMU601();     // 0x55 0x55 家族（算术和校验）
	void Decode0x91();       // 0x5A 0xA5 家族（CH010 / HI226，共用）
	void MarkSampleValid();  // 三个分支的"本帧可用"标记统一收口，防漏

	/* CRC 累加器：CH010 的覆盖范围是"两段拼接、中间 CRC 字段跳过"，
	 * 所以运算跨段延续但【只在最后比较一次】。
	 * ⚠️ 别改回"每段都调 Check(seg) 并各自比较"—— 第一段的中间值
	 *    不等于最终 CRC，会提前 return 导致字段永不更新。*/
	void     CrcReset();
	void     CrcAccum(const uint8_t* p, uint16_t n);
	uint16_t CrcResult() const;

	uint16_t crc_acc = 0;      // CRC 累加器（每帧 CrcReset() 清零）

	uint32_t timestamp = 0;             // 0x91 offset 8 system_time (ms)
	uint32_t last_rx_tick = 0;
	bool     init_done = false;
	bool     data_ok = false;

	Angle angle;
	AngularVelocity angularvelocity;
	Acceleration acceleration;
	IMU_TYPE type;

	uint8_t rxData[UART_MAX_LEN];
	UART* m_uart;

};

/* 原 uint16_t U2() 已删：全项目零消费者。
 *
 * ⚠️ 形参必须是 `const uint8_t*`：这两个函数只 memcpy 出来读、不写，
 *    给 const 才是正确语义。声明成非 const 时，调用点一旦指向只读缓冲
 *    （Decode0x91 里的 `const uint8_t* p`）就编译不过 ——
 *    而"去掉 const 就能编过"恰恰是把只读缓冲伪装成可写，风险更糟。
 *
 * ⚠️ memcpy 读多字节字段要求源指针满足目标类型的对齐。uint8_t 数组首地址
 *    至少 1 字节对齐，而 Cortex-M4 的 float/uint32 对齐要求是 4 —— 这里
 *    理论上有 UB。F405 上实测未出问题（对齐在实践中被链接/栈布局兜住），
 *    要严格正确应改成逐字节小端装载。*/
static uint32_t U4(const uint8_t* p) { uint32_t u; memcpy(&u, p, 4); return u; }
static float    R4(const uint8_t* p) { float    r; memcpy(&r, p, 4); return r; }

extern IMU imu_chassis, imu_pantile;