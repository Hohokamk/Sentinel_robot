#include "label.h"
#include "HTmotor.h"
#include "motor.h"
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
		ctrl.pantile.mark_yaw = (float)ctrl.pantile_motor[CONTROL::PANTILE::YAW]->angle[now];
		
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

		
		ctrl.chassis.speedz = 0;
		ctrl.shooter.openRub = false;
		ctrl.shooter.supply_bullet = false;

		//ctrl.chassis.Keep_Direction();

		switch (ctrl.mode)
		{
		case CONTROL::ROTATION://小陀螺
			ctrl.chassis.speedx = rc.ch[1] * 4000.f / 660.f;
			ctrl.chassis.speedy = -1 * rc.ch[0] * 4000.f / 660.f;
			ctrl.chassis.speedz = rc.ch[2];//para.rota_speed;             // 不硬编码 1000
			ctrl.pantile.Control_Pantile(0, 0, rc.ch[3]);//(0, rc.ch[3],rc.ch[2]);
			break;

		case CONTROL::FOLLOW://正方向为云台方向，跟随云台视角
			ctrl.chassis.speedx = 0;
			ctrl.chassis.speedy = 0;
			//先要读取云台的角度，根据云台的方向计算出具体的speedx和speedy
			ctrl.chassis.Keep_Direction();//重点是写这个按照云台的角度来计算出speedx和speedy，以云台的角度为参考系
			//其实就是重新计算speedx和speedy，把之前的覆盖
			ctrl.pantile.Control_Pantile(rc.ch[2], rc.ch[3],0);
			break;

		case CONTROL::SEPARATE://底盘与云台分离，底盘不受云台影响
			ctrl.chassis.speedx = rc.ch[1] * 4000.f / 660.f;
			ctrl.chassis.speedy = -1 * rc.ch[0] * 4000.f / 660.f;
			ctrl.pantile.Control_Pantile(rc.ch[2], rc.ch[3],0);
			break;

		case CONTROL::AUTOAIM://自动瞄准	
			ctrl.chassis.speedx = pc.x * para.max_speed / 660.f;
			ctrl.chassis.speedy = -pc.y * para.max_speed / 660.f;//这是写给pc的接口，rc的不用管
			//这里还要写射击，给pc接管
			break;

		case CONTROL::FIRE://射击
			ctrl.chassis.speedx = rc.ch[1] * 4000.f / 660.f;
			ctrl.chassis.speedy = -1 * rc.ch[0] * 4000.f / 660.f;
			ctrl.pantile.Control_Pantile(rc.ch[2], rc.ch[3],0);
			ctrl.shooter.openRub = true;                       // 意图交给 SHOOTER::Update()
			//这里要重新分配ch[0]和ch[1]的值，作为射击的控制，射击的时候底盘可以不动
			break;

		case CONTROL::STOP://停止
			ctrl.chassis.speedx = 0;
			ctrl.chassis.speedy = 0;
			ctrl.chassis.speedz = 0;
			break;

		case CONTROL::SPINNING://超级雷霆大转盘
			ctrl.chassis.speedz = rc.ch[0] * para.max_speed / 660.f; //底盘旋转
			ctrl.pantile.Control_Pantile(rc.ch[2], rc.ch[3], rc.ch[1]);         // 大yaw，小yaw，pitch
			break;

		default:
			ctrl.chassis.speedx = 0;
			ctrl.chassis.speedy = 0;
			ctrl.chassis.speedz = 0;
			break;
		}
	}
	else {
		ctrl.chassis.speedx = 0;
		ctrl.chassis.speedy = 0;
		ctrl.chassis.speedz = 0;//这里不能写setspeed,那个只能在电机的update里写
		
		if (ctrl.pantile_motor[CONTROL::PANTILE::TYPE::YAW])
			ctrl.pantile_motor[CONTROL::PANTILE::TYPE::YAW]->setangle =
			ctrl.pantile_motor[CONTROL::PANTILE::TYPE::YAW]->angle[now];
		
		ctrl.shooter.openRub = false;
		ctrl.shooter.supply_bullet = false;

		ctrl.pantile.Control_Pantile(0, 0, 0);
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