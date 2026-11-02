#ifndef _BSP_TCRT5000_H
#define _BSP_TCRT5000_H

#include "system.h"


#define HWL_PORT		GPIOB
#define HWL_PIN			GPIO_Pin_15
#define HWL_PORT_RCC	RCC_APB2Periph_GPIOB

#define HWM_PORT		GPIOB
#define HWM_PIN			GPIO_Pin_14
#define HWM_PORT_RCC	RCC_APB2Periph_GPIOB

#define HWR_PORT		GPIOB
#define HWR_PIN			GPIO_Pin_13
#define HWR_PORT_RCC	RCC_APB2Periph_GPIOB

#define Tracking_L0		1//GPIO_ReadInputDataBit(HWL_PORT,HWL_PIN)//GPIOA
#define Tracking_M0		1//GPIO_ReadInputDataBit(HWM_PORT,HWM_PIN)
#define Tracking_R0		1//GPIO_ReadInputDataBit(HWR_PORT,HWR_PIN)

void TRC5000_Init(void);

#endif
