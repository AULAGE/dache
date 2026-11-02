#ifndef _dc_motor_H
#define _dc_motor_H

#include "system.h"


#define DC_MOTOR_PORT		GPIOB
#define DC_MOTOR_PIN1		GPIO_Pin_8
#define DC_MOTOR_PIN2		GPIO_Pin_9
#define DC_MOTOR_PORT_RCC	RCC_APB2Periph_GPIOB

#define DC_MOTOR1	PBout(8)
#define DC_MOTOR2	PBout(9)


void DC_Motor_Init(void);

#endif
