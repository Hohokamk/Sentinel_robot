#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "label.h"

void PARAMETER::Init()
{
	pitch_min = 0, pitch_max = 8192, initial_pitch = 4096, initial_yaw = 4096;
	imu_pitch_max = 18, imu_pitch_min = 16;
	ace_speed = 1000, max_speed = 3000, rota_speed = 1000;
	pitch_speed = 2, yaw_speed = 2;
	dm_initial_pitch = 0.f, dm_initial_yaw = 0.f;
	yaw_center = 8192.f, yaw_span = 1707.f;//左右75度大概
	bullet_step = 42130.29f;bullet_lead_max = 84260.58f;
	dm_pitch_min = -0.55f;   
	dm_pitch_max = +0.4f;  
	// ── 底盘导航：视觉速度(m/s) → 电机转子rpm ──
	//换算链： nav_v [m/s] × K_NAV → 转子rpm → ÷GEAR → 轮rpm → ×2πR → m/s
	//   反解得K_NAV = GEAR × 60 / (2π × R)
	//
	//🔴 K_NAV 原值 159 是【量纲错误】，不是"待标定的估算值"：
	//   159 rpm ÷ 19.2 = 8.28 轮rpm → ×2π×0.0475 = 0.0220 m/s
	//   ⇒ 上位机发 1 m/s，机器人只走 2.2 cm/s，差 45 倍。
	//   正确值应为 3859.9（GEAR=19.2032, R=0.0475m）。
	DV_MAX = 200.f;                    // 每周期(5ms)最多变 200rpm
	NAV_MAX = 6000.f;                  // 目标rpm 上限，与底盘 maxspeed 对齐
	K_NAV = 3859.9f;         // 速度增益 m/s → 转子rpm（理论值，仍需实测校正）
	// ⚠️ 理论值只保证"量纲对"，实测必偏：轮胎打滑、地面摩擦、负载、
	//    以及 M3508 实际减速比与标称值的差异，都会带来 10~30% 偏差。
	//    实测方法：上位机固定发 vx=1.0 m/s，秒表量 3 秒实际距离，
	//    K_NAV_new = K_NAV_theory × (1.0 / v_measured)。
	// ⚠️ 若轮径不是 47.5mm（实测直径 D），K_NAV 应按 3859.9 × 47.5/D 缩放。
}


/*
定义任务句柄
*/
TaskHandle_t StartTask_Handler;
TaskHandle_t LedTask_Handler;
TaskHandle_t DecodeTask_Handler;
TaskHandle_t ControlTask_Handler;
TaskHandle_t MotorTask_Handler;
TaskHandle_t CanTxTask_Handler;

