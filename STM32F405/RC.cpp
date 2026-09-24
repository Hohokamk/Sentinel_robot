#include "label.h"
#include "HTmotor.h"
#include "RC.h"
#include "control.h"

void RC::Init(UART* huart, USART_TypeDef* Instance, const uint32_t BaudRate)
{
	huart->Init(Instance, BaudRate).DMARxInit(nullptr);
	m_uart = huart;
	queueHandler = &huart->UartQueueHandler;
}

void RC::OnRC()
{
	RC_CheckState();
	RC_Control();

	if (Shift_mode())
	{
		ctrl.pantile.mark_yaw = (float)ctrl.pantile_motor[CONTROL::PANTILE::YAW]->sum_angle;
		
	}

}

void RC::OnPC()
{
	;
}

void RC::Update()
{
	OnRC();
	OnPC();
}

void RC::RC_CheckState() {

	switch (RC_STATE(rc.s[0], rc.s[1]))
	{
	case RC_STATE(UP, UP):
		ctrl.mode = CONTROL::ROTATION;
		break;

	case RC_STATE(UP, MID):
		ctrl.mode = CONTROL::RESET;
		break;

	case RC_STATE(UP, DOWN):
		ctrl.mode = CONTROL::SPINNING;
		break;

	case RC_STATE(MID, UP):
		ctrl.mode = CONTROL::FOLLOW;
		break;

	case RC_STATE(MID, MID):
		ctrl.mode = CONTROL::RESET;
		break;

	case RC_STATE(MID, DOWN):
		ctrl.mode = CONTROL::SEPARATE;
		break;

	case RC_STATE(DOWN, UP):
		ctrl.mode = CONTROL::FIRE;
		break;

	case RC_STATE(DOWN, MID):
		ctrl.mode = CONTROL::RESET;
		break;

	case RC_STATE(DOWN, DOWN):
		ctrl.mode = CONTROL::STOP;
		break;

	default:
		break;
	}

}

void RC::RC_Control() {
	if (ctrl.mode != CONTROL::RESET)
	{

		ctrl.chassis.speedx = rc.ch[3] * 4000.f / 660.f;
		ctrl.chassis.speedy = -1 * rc.ch[2] * 4000.f / 660.f;
		ctrl.chassis.speedz = 0;

		//ctrl.chassis.Keep_Direction();

		switch (ctrl.mode)
		{
		case CONTROL::ROTATION://小陀螺
			ctrl.chassis.speedx = rc.ch[1] * para.max_speed / 660.f;
			ctrl.chassis.speedy = rc.ch[0] * para.max_speed / 660.f;
			ctrl.chassis.speedz = 1000;
			break;

		case CONTROL::FOLLOW://正方向为云台方向，跟随云台视角
			//先要读取云台的角度，根据云台的方向计算出具体的speedx和speedy
			ctrl.chassis.Keep_Direction();
			break;

		case CONTROL::SEPARATE://底盘与云台分离，底盘不受云台影响
			ctrl.chassis.speedx = rc.ch[1] * para.max_speed / 660.f;
			ctrl.chassis.speedy = rc.ch[0] * para.max_speed / 660.f;
			break;

		case CONTROL::AUTOAIM://自动瞄准	
			ctrl.chassis.speedx = rc.ch[1] * para.max_speed / 660.f;
			ctrl.chassis.speedy = rc.ch[0] * para.max_speed / 660.f;
			break;

		case CONTROL::FIRE://射击
			ctrl.chassis.speedx = rc.ch[1] * para.max_speed / 660.f;
			ctrl.chassis.speedy = rc.ch[0] * para.max_speed / 660.f;
			break;

		case CONTROL::STOP://停止
			ctrl.chassis.speedx = 0;
			ctrl.chassis.speedy = 0;
			ctrl.chassis.speedz = 0;
			break;

		case CONTROL::SPINNING://旋转
			ctrl.chassis.speedz = rc.ch[0] * para.max_speed / 660.f;
			break;

		default:
			ctrl.chassis.speedx = 0;
			ctrl.chassis.speedy = 0;
			ctrl.chassis.speedz = 0;
			break;
		}
	}
	else {
		can1_motor[0].setspeed = 0;
		can1_motor[1].setspeed = 0;
		can1_motor[2].setspeed = 0;
		can1_motor[3].setspeed = 0;
		
		can2_motor[0].setspeed = 0;
		can2_motor[1].setspeed = 0;
		can2_motor[2].setspeed = 0;
		can2_motor[3].setspeed = 0;
		
		DMmotor[0].setSpeed = 0;
		DMmotor[1].setSpeed = 0;
	}
}

void RC::Decode()
{
	if (queueHandler == NULL || *queueHandler == NULL) {
		return;  // 或者报错
	}
	else {
		pd_Rx = xQueueReceive(*queueHandler, m_frame, NULL);
	}

	if (sizeof(m_frame) < 18) return;
	if ((m_frame[0] | m_frame[1] | m_frame[2] | m_frame[3] | m_frame[4] | m_frame[5]) == 0)return;

	rc.ch[0] = ((m_frame[0] | m_frame[1] << 8) & 0x07FF) - 1024;
	rc.ch[1] = ((m_frame[1] >> 3 | m_frame[2] << 5) & 0x07FF) - 1024;
	rc.ch[2] = ((m_frame[2] >> 6 | m_frame[3] << 2 | m_frame[4] << 10) & 0x07FF) - 1024;
	rc.ch[3] = ((m_frame[4] >> 1 | m_frame[5] << 7) & 0x07FF) - 1024;
	if (rc.ch[0] <= 8 && rc.ch[0] >= -8)rc.ch[0] = 0;
	if (rc.ch[1] <= 8 && rc.ch[1] >= -8)rc.ch[1] = 0;
	if (rc.ch[2] <= 8 && rc.ch[2] >= -8)rc.ch[2] = 0;
	if (rc.ch[3] <= 8 && rc.ch[3] >= -8)rc.ch[3] = 0;

	pre_rc.s[0] = rc.s[0];
	pre_rc.s[1] = rc.s[1];

	rc.s[0] = ((m_frame[5] >> 4) & 0x0C) >> 2;
	rc.s[1] = ((m_frame[5] >> 4) & 0x03);

	pc.x = m_frame[6] | (m_frame[7] << 8);
	pc.y = m_frame[8] | (m_frame[9] << 8);
	pc.z = m_frame[10] | (m_frame[11] << 8);
	pc.press_l = m_frame[12];
	pc.press_r = m_frame[13];

	pc.key_h = m_frame[15];//按键的高位部分R F G Z X C 
	pc.key_l = m_frame[14];//按键的低8位 W S A D SHIFT CTRL Q E

}

bool RC::Shift_mode()
{
	if (rc.s[0] != pre_rc.s[0] || rc.s[1] != pre_rc.s[1])
	{
		return true;
	}
	return false;
}