#ifndef __SENSOR_H
#define __SENSOR_H

#include "system.h"

#define FLAME_AO_PIN    GPIO_Pin_1
#define FLAME_AO_PORT   GPIOA
#define FLAME_AO_CLK    RCC_APB2Periph_GPIOA
#define FLAME_ADC_CH    ADC_Channel_1

#define FLAME_DO_PIN    GPIO_Pin_5
#define FLAME_DO_PORT   GPIOB
#define FLAME_DO_CLK    RCC_APB2Periph_GPIOB

#define MQ2_AO_PIN      GPIO_Pin_2
#define MQ2_AO_PORT     GPIOA
#define MQ2_AO_CLK      RCC_APB2Periph_GPIOA
#define MQ2_ADC_CH      ADC_Channel_2

#define MQ2_DO_PIN      GPIO_Pin_12
#define MQ2_DO_PORT     GPIOB
#define MQ2_DO_CLK      RCC_APB2Periph_GPIOB

#define LIGHT_AO_PIN    GPIO_Pin_3
#define LIGHT_AO_PORT   GPIOA
#define LIGHT_AO_CLK    RCC_APB2Periph_GPIOA
#define LIGHT_ADC_CH    ADC_Channel_3

#define LIGHT_DO_PIN    GPIO_Pin_13
#define LIGHT_DO_PORT   GPIOB
#define LIGHT_DO_CLK    RCC_APB2Periph_GPIOB

#define SENSOR_READ_TIMES  10

void    Sensor_Init(void);
u16     Flame_ReadAO(void);
u8      Flame_ReadDO(void);
u16     MQ2_ReadAO(void);
u8      MQ2_ReadDO(void);
float   MQ2_GetPPM(void);
float   MQ2_Calibrate(void);
u16     Light_ReadAO(void);
u8      Light_ReadDO(void);

#endif
