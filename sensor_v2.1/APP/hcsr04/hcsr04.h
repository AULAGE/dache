#ifndef _hcsr04_H
#define _hcsr04_H

#include "system.h"


#define	Trig_RCC	RCC_APB2Periph_GPIOB
#define Trig_PORT	GPIOA
#define Trig_PIN	GPIO_Pin_1	

#define	Echo_RCC	RCC_APB2Periph_GPIOB
#define Echo_PORT	GPIOB
#define Echo_PIN	GPIO_Pin_0

#define Trig  PAout(1)
#define Echo  PBin(0)


void HC_Init(void);

void TIM4_Int_Init(void);

void HC_Start(void);

float HC_GetDistance(void);

float CAT(float *date,u8 len,u8 tip);	//一般情况使用

float kalman_get(u8 turn);				//先均值滤波再卡尔曼滤波，高精使用



#endif
