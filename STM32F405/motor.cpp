#include "motor.h"
#include "gpio.h"
#include "HTmotor.h"
#include "imu.h"
#define DEG_TO_RAD 0.017453292f  // π / 180
Motor::Motor(const motor_type type, const motor_mode mode, const function_type function, const uint32_t id, PID _speed, PID _position, PID _speed2)
	: ID(id)
	, type(type)
	, mode(mode)
{
	getmax(type);
	memcpy(&pid[speed], &_speed, sizeof(PID));
	memcpy(&pid[position], &_position, sizeof(PID));
	memcpy(&pid[speed2], &_speed2, sizeof(PID));
	this->function = function;
}


Motor::Motor(const motor_type type, const motor_mode mode, const function_type function, const uint32_t id, PID _speed, PID _position)
	: ID(id)
	, type(type)
	, mode(mode)
{
	getmax(type);
	memcpy(&pid[speed], &_speed, sizeof(PID));
	memcpy(&pid[position], &_position, sizeof(PID));
	this->function = function;
}

Motor::Motor(const motor_type type, const motor_mode mode, const function_type function, const uint32_t id, PID _speed)
	: ID(id)
	, type(type)
	, mode(mode)
{
	getmax(type);
	memcpy(&pid[speed], &_speed, sizeof(PID));
	this->function = function;
}

void Motor::StatusIdentifier(int32_t torque_current)
{
	if (torque_current == old_torque_current)
		disconnectCount++;
	else
		disconnectCount = 0;

	if (disconnectCount >= disconnectMax)
	{
		disconnectCount = disconnectMax;
		if (old_torque_current == 0)
			m_status = UNCONNECTED;
		else
			m_status = DISCONNECTED;
	}
	else
		m_status = FINE;

	old_torque_current = torque_current;
}
uint8_t Motor::getStatus()const
{
	return (uint8_t)m_status;
}
void Motor::Ontimer(uint8_t idata[][8], uint8_t* odata)//idate: receive;odate: trainsmit;RC
{
	uint32_t trainsmit_or_receive_ID = this->ID - ID1;

	//----------------------------------------------------------------
	/*if (this->type == M6020)
	{
		trainsmit_or_receive_ID += 4;//神秘注释，好像是另外一套偏移id的方案
	}*/
	//----------------------------------------------------------------
	this->torque_current = getword(idata[trainsmit_or_receive_ID][4], idata[trainsmit_or_receive_ID][5]);
	this->StatusIdentifier(this->torque_current);
	this->angle[now] = getword(idata[trainsmit_or_receive_ID][0], idata[trainsmit_or_receive_ID][1]);
	this->temperature = idata[trainsmit_or_receive_ID][6];
	if (this->temperature != 0) this->has_feedback = true;   // 用温度帧来判断是不是第一次调用！！！！真帧才带温度；上电零缓冲不会置位
	//Get currrent speed

	motor_status = 0;
	if (temperature > 70) {
		setspeed = 0;
	}

	if (type == EC60)
	{
		curspeed = static_cast<float>(getdeltaa(angle[now] - angle[pre])) / T / 8192.f * 60.f;
	}
	else {
		curspeed = getword(idata[trainsmit_or_receive_ID][2], idata[trainsmit_or_receive_ID][3]);
	}
	//----------------------------------------------------------------
	/*if (this->type == M6020)
	{
		trainsmit_or_receive_ID -= 4;
	}*/
	//----------------------------------------------------------------
	//20220121--hz
	if (mode == ACE)
	{
		if (spinning)
		{
		}
		else {
			if (need_curcircle > 0)
			{
			}
			else if (need_curcircle <= 0)
			{
			}
		}
		if (setspeed == 0 && curspeed == 0)
		{
			motor_status = 1;
			motor_angle_status = angle[0];
		}
		if (motor_status == 1 && fabs(motor_angle_status - angle[0]) < 50)
		{
			current = 0;
		}
	}
	else if (mode == POS)
	{
		if (!has_feedback) current = 0;                              // 从没收到过真反馈 → 绝不出力
		else
		{
			if (!angle_latched)                       // 只执行一次
			{
				setangle = angle[now];                // 把目标钉在当前位置
				angle_latched = true;
			}
			float pos_error = (float)getdeltaa((int16_t)(setangle - angle[now]));  // 编码值，已取最短路径
			pos_error = mechanicalToDegree(pos_error);                             // 换算成「度」

			float speed_ref = pid[position].Position(pos_error, 1000.f);           // 外环：位置式（绝对输出）
			speed_ref = std::max(std::min(speed_ref, (float)maxspeed), -(float)maxspeed);

			current += pid[speed].Delta(speed_ref - curspeed);                     // 内环：增量式（Δ 累加）

			//float speed_ref = -0.5f;                   // 测试下层：固定目标转速，纯验证用
			//current += (int32_t)pid[speed].Delta(speed_ref - curspeed);
		}
	}
	else if (mode == SPD)
	{
		current += pid[speed].Delta(setspeed - curspeed);
	}
	recorded_the_Laps();
	GetDistanceFromMechanicalAngle();
	angle[pre] = angle[now];
	current = setrange(current, maxcurrent);
	odata[trainsmit_or_receive_ID * 2] = (current & 0xff00) >> 8;//高八位
	odata[trainsmit_or_receive_ID * 2 + 1] = current & 0x00ff;
}
void Motor::recorded_the_Laps() {
	int16_t delta = angle[now] - angle[pre];
	// 处理回绕：顺时针
	if (delta > 8192 / 2)
		delta -= 8192;
	// 处理回绕：逆时针
	else if (delta < -8192 / 2)
		delta += 8192;

	sum_angle+= delta;
//	round_count = total_count / encoder_resolution;
}

uint8_t initial_cnt=0;
void Motor::GetDistanceFromMechanicalAngle() {
	if (initial_cnt<5)
	initial_cnt++;
	distance=(6.2831853f/ 8192.0f)*sum_angle * (WHEEL_RADIUS_MM / GEAR_RATIO)-initial_x;  // 单位：mm

	if(initial_cnt<3)
	initial_x = distance;
}

void Motor::getmax(const type_t type)
{
	adjspeed = 3000;
	switch (type)
	{
	case M3508:
		maxcurrent = 16384;
		maxspeed = 2000;
		break;
	case M3510:
		maxcurrent = 13000;
		maxspeed = 9000;
		break;
	case M2310:
		maxcurrent = 13000;
		maxspeed = 9000;
		adjspeed = 1000;
		break;
	case EC60:
		maxcurrent = 5000;
		maxspeed = 300;
		break;
	case M6623:
		maxcurrent = 5000;
		maxspeed = 300;
		break;
	case M6020:
		maxcurrent = 3000;
		maxspeed = 20;
		adjspeed = 10;
		break;
	case M2006:
		maxcurrent = 10000;
		adjspeed = 1000;
		maxspeed = 3000;
		break;
	default:;
	}
}

int16_t Motor::getdeltaa(int16_t diff)
{
	if (diff <= -4096)
		diff += 8192;
	else if (diff > 4096)
		diff -= 8192;
	return diff;
}

int16_t Motor::getword(const uint8_t high, const uint8_t low)
{
	const int16_t word = high;
	return (word << 8) + low;
}

int32_t Motor::setrange(const int32_t original, const int32_t range)
{
	return std::max(std::min(range, original), -range);
}

