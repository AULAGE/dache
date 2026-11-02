#ifndef __SERVO_H
#define	__SERVO_H

#include "system.h"


// SERVO¶æ»ú GPIOºê¶¨Òå

#define	SERVO_CLK											RCC_APB2Periph_GPIOB

#define SERVO1_GPIO_PIN 							GPIO_Pin_0
#define SERVO_GPIO_PORT  							GPIOB

#define	SERVO_CLK											RCC_APB2Periph_GPIOB

#define SERVO2_GPIO_PIN 							GPIO_Pin_1
#define SERVO_GPIO_PORT  							GPIOB

void Servo_Init(void);
void SetAngle_DOWN(float Angle);
void SetAngle_UP(float Angle);

#endif
