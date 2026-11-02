#include "dc_motor.h"


void DC_Motor_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(DC_MOTOR_PORT_RCC,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin=DC_MOTOR_PIN1;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(DC_MOTOR_PORT,&GPIO_InitStructure);
	GPIO_ResetBits(DC_MOTOR_PORT,DC_MOTOR_PIN1);
	
	GPIO_InitStructure.GPIO_Pin=DC_MOTOR_PIN2;
	GPIO_Init(DC_MOTOR_PORT,&GPIO_InitStructure);
	
	GPIO_ResetBits(DC_MOTOR_PORT,DC_MOTOR_PIN2);
}
