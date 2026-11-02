#ifndef _leddz_H
#define _leddz_H

#include "system.h"

#define RCLK_PORT 			GPIOB
#define RCLK_PIN 			GPIO_Pin_5
#define RCLK_PORT_RCC		RCC_APB2Periph_GPIOB

#define SRCLK_PORT 			GPIOB  
#define SRCLK_PIN 			GPIO_Pin_3
#define SRCLK_PORT_RCC		RCC_APB2Periph_GPIOB

#define SER_PORT 			GPIOB  
#define SER_PIN 			GPIO_Pin_4
#define SER_PORT_RCC		RCC_APB2Periph_GPIOB


#define LEDDZ_COL_PORT 			GPIOA  
#define LEDDZ_COL_PIN 			GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7
#define LEDDZ_COL_PORT_RCC		RCC_APB2Periph_GPIOA

#define SER		PBout(4)
#define RCLK	PBout(5)
#define SRCLK	PBout(3)


void LEDDZ_Init(void);
void LEDDZ_Row_Write_data(u8 dat);
void LEDDZ_COL_Write_Data(u8 dat);

#endif
