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



void CONTROL::PANTILE::Control_Pantile(int32_t ch_yaw, int32_t ch_pitch)
{
	
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
	w[0] = vx - vy - K * wz;   // 1 左前
	w[1] = vx + vy + K * wz;   // 2 右前
	w[2] = vx - vy + K * wz;   // 3 右后
	w[3] = vx + vy - K * wz;   // 4 左后

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

	if (ctrl.pantile_motor[PANTILE::TYPE::YAW])
		ctrl.pantile_motor[PANTILE::TYPE::YAW]->setangle =
		ClampAngle(ctrl.pantile_motor[PANTILE::TYPE::YAW]->setangle, para.yaw_center, para.yaw_span);

}

void CONTROL::SHOOTER::Update()
{
	
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

