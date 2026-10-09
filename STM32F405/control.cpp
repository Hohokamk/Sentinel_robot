#include "control.h"
#include "tim.h"
#include "judgement.h"
#include "HTmotor.h"
#include "motor.h"

void CONTROL::init_dm()//达妙单独初始化
{
	DMmotor[0].setPos = para.dm_initial_pitch;   // Pitch：P_S 模式，setPos 有效
	DMmotor[1].setSpeed = para.dm_initial_yaw;                  // Yaw：SPEED 模式，只有 setSpeed 有效
	init_DM = 1;                                 // 标志置位，表示已初始化
}
void CONTROL::Init(std::vector<Motor*> motor)//普通电机初始化
{
	for (int i = 0; i < motor.size(); i++)
		//新增了容量检查，因为每个类型的电机的内存是连续的，
		//所以如果不小心多了电机可能会有覆盖问题
	{
		switch (motor[i]->function)
		{
		case(function_type::chassis):
			if (chassis_num < CHASSIS_MOTOR_NUM) chassis_motor[chassis_num++] = motor[i];
			break;
		case(function_type::pantile):
			if (pantile_num < PANTILE_MOTOR_NUM) {
				pantile_motor[motor[i]->ID == ID5 ? PANTILE::TYPE::YAW
					: PANTILE::TYPE::PITCH] = motor[i];
				++pantile_num;
			}
			break;
		case(function_type::shooter):
			if (shooter_num < SHOOTER_MOTOR_NUM) shooter_motor[shooter_num++] = motor[i];
			break;
		case(function_type::supply):
			if (supply_num < SUPPLY_MOTOR_NUM) {
				supply_motor[supply_num] = motor[i];              // 先放进槽
				supply_motor[supply_num]->spinning = false;       // 再通过指针写成员
				supply_motor[supply_num]->need_curcircle = false;
				++supply_num;
			}
			break;//就算是最后一个，也要补上break;
		default:
			break;
		}
	}
	if (pantile_motor[PANTILE::TYPE::PITCH]) pantile_motor[PANTILE::TYPE::PITCH]->setangle = para.initial_pitch;
	if (pantile_motor[PANTILE::TYPE::YAW])   pantile_motor[PANTILE::TYPE::YAW]->setangle = para.initial_yaw;

}
float CONTROL::ClampAngle(float setangle, float center, float span)
{
	float off = (float)Motor::getdeltaa((int16_t)(setangle - center));  // 相对中点的带符号偏移
	off = std::max(std::min(off, span), -span);                        // 钳到 ±半宽
	setangle = center + off;
	if (setangle < 0.f)     setangle += 8192.f;                        // 回绕归一
	if (setangle >= 8192.f) setangle -= 8192.f;
	return setangle;
}

void CONTROL::PANTILE::Control_Pantile(int32_t ch_dji_yaw, int32_t ch_pitch, int32_t ch_dm_yaw)
{
	// 1. 大疆Yaw/位置累加
	if (ctrl.pantile_motor[PANTILE::TYPE::YAW] && ctrl.pantile_motor[PANTILE::TYPE::YAW]->has_feedback)//feedback和他本身不为空
	{
		// 满杆 180°/s，5ms 周期
		const float yaw_rate = 8192.f * 0.5f;
		// 摇杆有推量时累加目标角度；摇杆回中(0)时目标角度自然保持不变
		ctrl.pantile_motor[PANTILE::TYPE::YAW]->setangle += -(float)ch_dji_yaw / 660.f * yaw_rate * 0.005f;
	}

	// 2. 达妙 Pitch 轴（位置模式） 
	mark_pitch += -(float)ch_pitch / 660.f * (PI * 0.5f) * 0.005f;

	// 3. 达妙 Yaw 轴（纯速度模式）
	// 速度模式：摇杆推多少就给多大角速度，传 0 就立刻刹停
	const float max_dm_speed = PI; // 最大 180°/s
	yaw_speed_out = (float)ch_dm_yaw / 660.f * max_dm_speed;
	

}

void CONTROL::PANTILE::Keep_Pantile(float angleKeep, PANTILE::TYPE type,IMU frameOfReference)
{
	
}

void CONTROL::CHASSIS::Keep_Direction()
{


}

void CONTROL::CHASSIS::Update()
{
	// ① 取指令；RESET 模式直接归零，不留残留
	float vx = 0.f, vy = 0.f, wz = 0.f;
	if (ctrl.mode != CONTROL::RESET)
	{
		vx = (float)speedx;
		vy = (float)speedy;
		wz = (float)speedz;
	}

	// ② 45° X 布局逆解（数组下标 = 你的电机编号 - 1）
	const float K = 1.0f;
	float w[4];
	w[0] = -vx - vy + K * wz;   // 1 左前
	w[1] = +vx - vy + K * wz;   // 2 右前
	w[2] = +vx + vy + K * wz;   // 3 右后
	w[3] = -vx + vy + K * wz;   // 4 左后

	// ③ 下发，顺手限到各自 maxspeed
	for (int i = 0; i < CHASSIS_MOTOR_NUM; i++)
	{
		if (!ctrl.chassis_motor[i]) continue;
		float lim = (float)ctrl.chassis_motor[i]->maxspeed;
		ctrl.chassis_motor[i]->setspeed = (int32_t)std::max(std::min(w[i], lim), -lim);
	}
}

void CONTROL::PANTILE::Update()
{
	// 1. 6020yaw
	if (ctrl.pantile_motor[PANTILE::TYPE::YAW])
		ctrl.pantile_motor[PANTILE::TYPE::YAW]->setangle =
		ClampAngle(ctrl.pantile_motor[PANTILE::TYPE::YAW]->setangle, para.yaw_center, para.yaw_span);// 钳到 ±半宽

	// 2. 达妙 Pitch 保护与更新
	if (DMmotor[0].decoded || pitch_ready)     // pitch 那一段里面不用动
	{
		if (!pitch_ready) { mark_pitch = DMmotor[0].pos;pitch0 = mark_pitch; pitch_ready = true; }
		LIMIT_MIN_MAX(mark_pitch, para.dm_pitch_min, para.dm_pitch_max);
		DMmotor[0].setPos = mark_pitch;
		DMmotor[0].setSpeed = 1.5f;
	}

	// 3. 达妙 Yaw 下发速度指令
	// 如果上层传 0，这里就会下发 0 rad/s，电机依靠自身的阻尼和速度环稳稳刹住
	if (DMmotor[1].decoded)
	{
		if (!yaw_lock_ready) { yaw_lock_pos = DMmotor[1].pos; yaw_lock_ready = true; }
		DMmotor[1].setSpeed = yaw_speed_out                                  // 手动微调（rad/s）
			+ yaw_lock_Kp * (yaw_lock_pos - DMmotor[1].pos); // 软锁回位
	}

}
void CONTROL::SHOOTER::Update()
{
	// ① 摩擦轮开始转
	
	if (ctrl.shooter_motor[0])ctrl.shooter_motor[0]->setspeed = openRub ? shoot_speed : 0;
	if (ctrl.shooter_motor[1])ctrl.shooter_motor[1]->setspeed = openRub ? -shoot_speed : 0;


	// ② 摩擦轮转速达标检测点（③ 的判据，先算）
	float sum = 0.f; int cnt = 0;
	
	const float v0 = ctrl.shooter_motor[0] ? fabsf((float)ctrl.shooter_motor[0]->curspeed) : 0.f;
	const float v1 = ctrl.shooter_motor[1] ? fabsf((float)ctrl.shooter_motor[1]->curspeed) : 0.f;
	const float rpm_avg = 0.5f * (v0 + v1);//两个轮子的平均转速
	

	// ③ 扳机：死区阈值 + 迟滞（因为摇杆是模拟信号，要转换成数字信号）
	const int16_t T_ON = 300, T_OFF = 200;
	if (!trig && trig_raw > T_ON) trig = true;
	if (trig && trig_raw < T_OFF) trig = false;

	const bool trig_eff = trig || ctrl.shooter.auto_shoot;   // 自瞄开火和手动扳机等效

	// ④ 短/长拨状态机
	const uint16_t TICK = 5, T_ARM = 300, T_RATE = 125;
	bool fire_pulse = false, fire_cont = false;

	if (trig_eff && !trig_pre) { fire_pulse = true; hold_ms = 0; rate_ms = T_RATE; }
	else if (trig_eff) { if (hold_ms < 60000) hold_ms += TICK; if (hold_ms >= T_ARM) fire_cont = true; }
	trig_pre = trig_eff;//把下面的替换了



	//if (trig && !trig_pre) { 
	//	fire_pulse = true; 
	//	hold_ms = 0; 
	//	rate_ms = T_RATE; }//上升沿，fire_pulse为脉冲，只有在5毫秒内为true,脉冲触发
	//else if (trig) { 
	//	if (hold_ms < 60000) hold_ms += TICK; //开始计时，为连发做准备
	//	if (hold_ms >= T_ARM) fire_cont = true;//时间大于300，连发
	//}


	//trig_pre = trig;//上升沿结束

	// ⑤ 拨盘：POS 双环，累加 setangle
	Motor* d = ctrl.supply_motor[0];//拨弹轮
	if (!d || !d->has_feedback || !d->angle_latched) return;   // 未就绪 → 别碰目标

	const bool allow = openRub && rpm_avg > (float)shoot_speed * 0.8f;//确保摩擦轮已经转起来了，才允许拨盘转动，这里以达到80%转速为准


	if (fire_pulse && allow) d->setangle += -para.bullet_step;//单发 这里的步长是以7发为准，8182*36/7得到的
	if (fire_cont && allow)//连发
	{
		if (rate_ms >= T_RATE) { rate_ms = 0; d->setangle += -para.bullet_step; }
		else rate_ms += TICK;//每125ms拨一次
	}
	else rate_ms = 0;

	// ⑥ 超前钳制：防 windup
	const float lead = d->setangle - (float)d->sum_angle;
	if (lead > para.bullet_lead_max)
		d->setangle = (float)d->sum_angle + para.bullet_lead_max;
	if (lead < -para.bullet_lead_max)
		d->setangle = (float)d->sum_angle - para.bullet_lead_max;
}

float CONTROL::CHASSIS::Ramp(float setval, float curval, uint32_t RampSlope)
{

	if ((setval - curval) >= 0)
	{
		curval += RampSlope;
		curval = std::min(curval, setval);
	}
	else
	{
		curval -= RampSlope;
		curval = std::max(curval, setval);
	}

	return curval;
}

float CONTROL::GetDelta(float delta)
{
	if (delta <= -180.f)
	{
		delta += 360.f;
	}

	if (delta > 180.f)
	{
		delta -= 360.f;
	}
	return delta;
}

int16_t CONTROL::Setrange(const int16_t original, const int16_t range)
{
	return fmaxf(fminf(range, original), -range);
}

