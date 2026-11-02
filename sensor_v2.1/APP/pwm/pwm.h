#ifndef _pwm_H
#define _pwm_H

#include "system.h"

//×óÂÖ  AIN
#define L_IN1_PORT			GPIOA
#define L_IN1_PIN				GPIO_Pin_3
#define L_IN1_PORT_RCC	RCC_APB2Periph_GPIOA

#define L_IN2_PORT			GPIOA
#define L_IN2_PIN				GPIO_Pin_4
#define L_IN2_PORT_RCC	RCC_APB2Periph_GPIOA

//ÓÒÂÖ  BIN
#define R_IN1_PORT			GPIOA
#define R_IN1_PIN				GPIO_Pin_5
#define R_IN1_PORT_RCC	RCC_APB2Periph_GPIOA

#define R_IN2_PORT			GPIOA
#define R_IN2_PIN				GPIO_Pin_6
#define R_IN2_PORT_RCC	RCC_APB2Periph_GPIOA

#define L_IN1	PAout(3)
#define L_IN2	PAout(4)
#define R_IN1	PAout(5)
#define R_IN2	PAout(6)


void PWMA_Init(u16 per,u16 psc);		//PWMA(×óÂÖ)
void PWMB_Init(u16 per,u16 psc);		//PWMB(ÓÒÂÖ)
void Wheel_Init(void);
void stop(void);
void Left_Speed(int speed);
void Right_Speed(int speed);
void Motor_Init(u16 per,u16 psc);


#endif
