  /*
   *__/\\\_______/\\\__/\\\\____________/\\\\__/\\\________/\\\______________/\\\\\\\\\____________/\\\\\\\\\_____/\\\\\\\\\\\___
   * _\///\\\___/\\\/__\/\\\\\\________/\\\\\\_\/\\\_______\/\\\____________/\\\///////\\\_______/\\\////////____/\\\/////////\\\_
   *  ___\///\\\\\\/____\/\\\//\\\____/\\\//\\\_\/\\\_______\/\\\___________\/\\\_____\/\\\_____/\\\/____________\//\\\______\///__
   *   _____\//\\\\______\/\\\\///\\\/\\\/_\/\\\_\/\\\_______\/\\\___________\/\\\\\\\\\\\/_____/\\\_______________\////\\\_________
   *    ______\/\\\\______\/\\\__\///\\\/___\/\\\_\/\\\_______\/\\\___________\/\\\//////\\\____\/\\\__________________\////\\\______
   *     ______/\\\\\\_____\/\\\____\///_____\/\\\_\/\\\_______\/\\\___________\/\\\____\//\\\___\//\\\____________________\////\\\___
   *      ____/\\\////\\\___\/\\\_____________\/\\\_\//\\\______/\\\____________\/\\\_____\//\\\___\///\\\___________/\\\______\//\\\__
   *       __/\\\/___\///\\\_\/\\\_____________\/\\\__\///\\\\\\\\\/_____________\/\\\______\//\\\____\////\\\\\\\\\_\///\\\\\\\\\\\/___
   *        _\///_______\///__\///_____________\///_____\/////////_______________\///________\///________\/////////____\///////////_____
  */

#include <stm32f4xx_hal.h>
#include <../CMSIS_RTOS/cmsis_os.h>
#include "can.h"
#include "usart.h"
#include "taskslist.h"
#include "tim.h"
#include "sysclk.h"
#include "delay.h"
#include "imu.h"
#include "motor.h"
#include "RC.h"
#include "control.h"
#include "judgement.h"
#include "led.h"
#include "HTmotor.h"
#include "Power_read.h"
#include "xuc.h"

Motor can1_motor[CAN1_MOTOR_NUM] = {
	Motor(M3508,SPD,chassis, ID1, PID(10.f, 0.0f, 1.5f,0.f)),
	Motor(M2006,SPD,chassis, ID2, PID(10.f, 0.0f, 1.5f,0.f)),
	Motor(M6020,POS,pantile, ID3, PID(40.f, 0.0f, 1.5f,0.f),PID(0.8f, 0.005f, 15.0f,0.f)),
	Motor(M6020,POS,pantile, ID4, PID(40.f, 0.0f, 1.5f,0.f),PID(0.8f, 0.005f, 15.0f,0.f)),
	Motor(M6020,POS,pantile, ID6, PID(40.f, 0.0f, 1.5f,0.f),PID(0.8f, 0.005f, 15.0f,0.f)),
	Motor(M6020,SPD,chassis, ID8, PID(10.f, 0.0f, 1.5f,0.f))
};
Motor can2_motor[CAN2_MOTOR_NUM] = {
	Motor(M3508,SPD,chassis, ID1, PID(10.f, 0.0f, 1.5f,0.f)),
	Motor(M2006,SPD,chassis, ID2, PID(10.f, 0.0f, 1.5f,0.f)),
	Motor(M6020,POS,pantile, ID3, PID(40.f, 0.0f, 1.5f,0.f),PID(0.8f, 0.005f, 15.0f,0.f)),
	Motor(M6020,POS,pantile, ID4, PID(40.f, 0.0f, 1.5f,0.f),PID(0.8f, 0.005f, 15.0f,0.f)),
	Motor(M6020,POS,pantile, ID7, PID(40.f, 0.0f, 1.5f,0.f),PID(0.8f, 0.005f, 15.0f,0.f)),
	Motor(M6020,SPD,chassis, ID8, PID(10.f, 0.0f, 1.5f,0.f))
};
DMMOTOR DMmotor[1] = {
	DMMOTOR(0x01, P_S, L_F),
};


CAN can1, can2;
UART uart1, uart2, uart3, uart4, uart5, uart6;
TIM  timer;
IMU imu_pantile;
DELAY delay;
RC rc;
POWER power;
LED led1, led2, led3, led4;
TASK task;
CONTROL ctrl;
Judgement judgement;
PARAMETER para;


int main(void)
{
	SystemClockConfig();
	delay.Init(168);
	HAL_Init();

	can1.Init(CAN1);
	can2.Init(CAN2);
	timer.Init(BASE, TIM3, 1000).BaseInit();

	imu_pantile.Init(&uart1, USART1, 115200, CH010);
	rc.Init(&uart2, USART2, 100000);
	power.Init(&uart5,UART5,9600);
	xuc.Init(&uart6, USART6, 115200);

	para.Init();

	ctrl.Init(std::vector<Motor*>{
		&can2_motor[0],
			& can2_motor[1],
			& can2_motor[2],
			& can2_motor[3],
			& can2_motor[4],
			& can2_motor[5]
	});
	ctrl.Init(std::vector<Motor*>{
		&can1_motor[0],
			& can1_motor[1],
			& can1_motor[2],
			& can1_motor[3],
			& can1_motor[4],
			& can1_motor[5]
	});

	task.Init();
	for (;;)
		;
}





